/* Full-resolution rosbag replay UI. GNSS is displayed only for offline comparison. */
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const UI = {
    bag:$('bag-select'), meta:$('bag-meta'), clock:$('clock'), speed:$('speed-value'),
    front:$('front-value'), rear:$('rear-value'), frontAge:$('front-age'), rearAge:$('rear-age'),
    cmd:$('cmd-value'), gnss:$('gnss-value'), gnssRover:$('gnss-rover'), delta:$('delta-value'), distance:$('distance-value'),
    rawSpeed:$('raw-speed-value'),modelAccel:$('model-accel-value'),sensorWeights:$('sensor-weights'),
    estimatorStatus:$('estimator-status'),sceneStatus:$('scene-status'),diagnostic:$('diagnostic'), map:$('map-canvas'), mapEmpty:$('map-empty'), mapReset:$('map-reset'), mapReadout:$('map-readout'),
    play:$('play-pause'), back:$('skip-back'), forward:$('skip-forward'), rate:$('rate'),
    scrub:$('scrubber'), scrubTime:$('scrub-time'), endTime:$('end-time'), duration:$('duration-label'),
    load:$('load-status'), speedChart:$('speed-chart'), cmdChart:$('cmd-chart'),
    readout:$('chart-readout'), events:$('event-list'), eventCount:$('event-count'),
  };
  const COLORS = {estimate:'#ffe075',front:'#3dcff4',rear:'#ffae58',master:'#5bd598',rover:'#c2a1ff',cmd:'#c7d8e7',grid:'#294257',text:'#9ab0c5',cursor:'#eaf7ff'};
  const DEFAULT = {bag:'30618_33bec73f',t:101};
  const query = new URLSearchParams(location.search);
  const initial = {bag:query.get('bag') || DEFAULT.bag,t:Number.isFinite(Number(query.get('t'))) && query.has('t') ? Number(query.get('t')) : DEFAULT.t};
  const cache = new Map();
  let catalog = [], data = null, scene = null, duration = 0, currentTime = 0;
  let playing = false, speedFactor = 1, windowSeconds = 60, lastFrame = 0, lastPaint = 0;
  let requestSeq = 0, odo = [], chartDomain = [0,60], hoveredTime = null;
  let mapBounds = null, mapZoom = 1, mapPan = {x:0,y:0}, mapDrag = null;
  let chartDirty = true, mapDirty = true;

  function fmtTime(t, decimal=true) {
    if (!Number.isFinite(t)) return '—';
    const neg=t<0?'-':'';t=Math.abs(t);
    const m=Math.floor(t/60), s=t-m*60;
    return neg+String(m).padStart(2,'0')+':'+(decimal?s.toFixed(1).padStart(4,'0'):String(Math.floor(s)).padStart(2,'0'));
  }
  const num=(v,d=2)=>Number.isFinite(v)?v.toFixed(d):'—';
  const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
  function lowerBound(points,t){let lo=0,hi=points.length;while(lo<hi){const m=(lo+hi)>>1;if(points[m][0]<t)lo=m+1;else hi=m}return lo}
  function sampleAt(points,t,maxAge=.35){
    if(!points || !points.length)return null;
    const i=lowerBound(points,t+1e-9)-1;
    if(i<0)return null;
    const age=t-points[i][0];
    return age>=-1e-5 && age<=maxAge ? {v:points[i][1],age,t:points[i][0],i} : null;
  }
  function routeAt(points,t){if(!points?.length)return null;const i=lowerBound(points,t+1e-9)-1;return i>=0&&t-points[i][0]<=1?points[i]:null}
  function setStatus(message,error=false){UI.load.textContent=message;UI.load.style.color=error?'#ff9ca0':''}
  function updateUrl(){if(!data)return;const p=new URLSearchParams();p.set('bag',data.id);p.set('t',currentTime.toFixed(1));history.replaceState(null,'',location.pathname+'?'+p)}

  function estimateWheel(t,series){
    const front=sampleAt(series.front,t,.3),rear=sampleAt(series.rear,t,.3);
    const velocity=front&&rear?(front.v+rear.v)/2:front?front.v:rear?rear.v:null;
    return {front,rear,velocity};
  }
  function buildOdom(bag,end){
    const result=[[0,0]];let dist=0,prev=0;
    for(let t=.1;t<=end+.05;t+=.1){
      const at=Math.min(t,end),v=estimateWheel(at,bag.series).velocity;
      if(v!==null)dist+=Math.max(0,v)*(at-prev);
      result.push([at,dist]);prev=at;
    }
    return result;
  }
  function odomAt(t){if(!odo.length)return 0;const i=clamp(lowerBound(odo,t),0,odo.length-1);return odo[i][1]}
  function estimateAt(t){
    const series=data?.estimate?.series;
    if(!series?.length)return null;
    const i=lowerBound(series,t+1e-9)-1;
    if(i<0||t-series[i][0]>.5)return null;
    const p=series[i];
    return {velocity:p[1],distance:p[2],modelAccel:p[3],frontWeight:p[4],rearWeight:p[5],
      frontSlip:Boolean(p[6]),rearSlip:Boolean(p[7]),modelOnly:Boolean(p[8]),tableUsed:Boolean(p[9]),
      pose:p.length>=13?[p[10],p[11],p[12]]:null};
  }

  function canvasSize(canvas){
    const r=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);
    const w=Math.max(1,Math.round(r.width)),h=Math.max(1,Math.round(r.height));
    const pw=Math.round(w*dpr),ph=Math.round(h*dpr);
    if(canvas.width!==pw||canvas.height!==ph){canvas.width=pw;canvas.height=ph}
    const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
    return {ctx,w,h};
  }
  function clearCanvas(ctx,w,h){ctx.clearRect(0,0,w,h);ctx.fillStyle='#0d1e30';ctx.fillRect(0,0,w,h)}
  function countRange(points,lo,hi){return [lowerBound(points,lo),lowerBound(points,hi)+1]}
  function chartRange(){
    if(!data)return [0,60];
    if(windowSeconds===0)return [0,duration];
    let lo=currentTime-windowSeconds/2;
    lo=clamp(lo,0,Math.max(0,duration-windowSeconds));
    return [lo,Math.min(duration,lo+windowSeconds)];
  }
  function drawSeries(ctx,points,lo,hi,x,y,color,gap=.6,step=false,width=1.55){
    if(!points?.length)return;
    const [a,b]=countRange(points,lo,hi),count=Math.max(0,b-a),stride=Math.max(1,Math.floor(count/6000));
    ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;
    let prev=null;
    for(let i=Math.max(0,a-1);i<Math.min(points.length,b+1);i+=stride){
      const p=points[i],px=x(p[0]),py=y(p[1]);
      if(!Number.isFinite(px+py))continue;
      if(!prev||p[0]-prev[0]>gap)ctx.moveTo(px,py);
      else if(step){ctx.lineTo(px,y(prev[1]));ctx.lineTo(px,py)}
      else ctx.lineTo(px,py);
      prev=p;
    }
    ctx.stroke();
  }
  function drawCharts(){
    if(!data)return;
    const speed=canvasSize(UI.speedChart),cmd=canvasSize(UI.cmdChart);
    for(const c of [speed,cmd])clearCanvas(c.ctx,c.w,c.h);
    const [lo,hi]=chartRange();chartDomain=[lo,hi];
    const m={l:48,r:15,t:16,b:25},sw=Math.max(1,speed.w-m.l-m.r),sh=speed.h-m.t-m.b;
    const cx=(t)=>m.l+(t-lo)/(hi-lo||1)*sw;
    let maxV=2;
    for(const k of ['front','rear','master','rover','estimate']){
      const pts=k==='estimate'?(data.estimate?.series||[]):data.series[k];const [a,b]=countRange(pts,lo,hi);
      for(let i=a;i<Math.min(pts.length,b);i++) if(Number.isFinite(pts[i][1]))maxV=Math.max(maxV,pts[i][1]);
    }
    maxV=Math.ceil(maxV*1.1/2)*2;
    const sy=(v)=>m.t+sh-(v/maxV)*sh;
    const sc=speed.ctx;
    sc.strokeStyle=COLORS.grid;sc.fillStyle=COLORS.text;sc.lineWidth=1;sc.font='11px system-ui';
    for(let i=0;i<=4;i++){const y=m.t+sh*i/4;sc.beginPath();sc.moveTo(m.l,y);sc.lineTo(speed.w-m.r,y);sc.stroke();sc.fillText(num(maxV*(1-i/4),0),5,y+4)}
    for(let i=0;i<=5;i++){const x=m.l+sw*i/5;sc.beginPath();sc.moveTo(x,m.t);sc.lineTo(x,m.t+sh);sc.stroke();sc.fillText(fmtTime(lo+(hi-lo)*i/5,false),x-13,speed.h-6)}
    sc.fillStyle='#c9dcea';sc.fillText('Скорость, м/с',m.l+5,12);
    for(const key of ['master','rover','front','rear'])drawSeries(sc,data.series[key],lo,hi,cx,sy,COLORS[key],.6);
    drawSeries(sc,data.estimate?.series||[],lo,hi,cx,sy,COLORS.estimate,.6,false,2.35);
    // Event bands are shaded behind the cursor but remain visually subtle.
    for(const ev of data.events||[]){if(ev.end_s<lo||ev.t_s>hi)continue;
      sc.fillStyle=ev.source==='manual_audit'?'#ffae5826':'#ff6b7112';
      sc.fillRect(clamp(cx(ev.t_s),m.l,m.l+sw),m.t,Math.max(2,Math.min(m.l+sw,cx(ev.end_s))-Math.max(m.l,cx(ev.t_s))),sh);
    }
    const drawCursor=(context,h,top,bottom,time,color,dash=[])=>{if(time===null||time<lo||time>hi)return;context.save();context.strokeStyle=color;context.lineWidth=1;context.setLineDash(dash);context.beginPath();context.moveTo(cx(time),top);context.lineTo(cx(time),h-bottom);context.stroke();context.restore()};
    drawCursor(sc,speed.h,m.t,m.b,currentTime,COLORS.cursor,[4,4]);
    drawCursor(sc,speed.h,m.t,m.b,hoveredTime,COLORS.orange,[2,2]);

    const cc=cmd.ctx,ch=cmd.h,cm={l:m.l,r:m.r,t:12,b:23},cH=ch-cm.t-cm.b;
    const cy=(v)=>cm.t+(15-v)/30*cH;
    cc.strokeStyle=COLORS.grid;cc.fillStyle=COLORS.text;cc.font='11px system-ui';
    for(const v of [-15,0,15]){const y=cy(v);cc.beginPath();cc.moveTo(cm.l,y);cc.lineTo(cmd.w-cm.r,y);cc.stroke();cc.fillText(String(v),8,y+4)}
    cc.fillStyle='#c9dcea';cc.fillText('Ручка',cm.l+5,11);
    drawSeries(cc,data.series.cmd,lo,hi,cx,cy,COLORS.cmd,.3,true);
    drawCursor(cc,ch,cm.t,cm.b,currentTime,COLORS.cursor,[4,4]);
    drawCursor(cc,ch,cm.t,cm.b,hoveredTime,COLORS.orange,[2,2]);
  }
  function quantile(sorted,q){if(!sorted.length)return 0;return sorted[Math.floor((sorted.length-1)*q)]}
  function routeBounds(route,estimate){
    const values=[];
    for(const key of ['master','rover'])for(const p of route[key]||[])if(Number.isFinite(p[1]+p[2]))values.push(p);
    for(const p of estimate?.series||[])if(p.length>=13&&Number.isFinite(p[10]+p[11]))values.push([p[0],p[10],p[11]]);
    if(!values.length)return null;
    const xs=values.map(p=>p[1]).sort((a,b)=>a-b),ys=values.map(p=>p[2]).sort((a,b)=>a-b);
    let xmin=quantile(xs,.01),xmax=quantile(xs,.99),ymin=quantile(ys,.01),ymax=quantile(ys,.99);
    if(xmax-xmin<20){xmin-=10;xmax+=10}if(ymax-ymin<20){ymin-=10;ymax+=10}
    const mx=(xmax-xmin)*.08,my=(ymax-ymin)*.08;
    return {xmin:xmin-mx,xmax:xmax+mx,ymin:ymin-my,ymax:ymax+my};
  }
  function mapTransform(w,h){
    const b=mapBounds,p=18,spanX=b.xmax-b.xmin,spanY=b.ymax-b.ymin;
    const scale=Math.min((w-2*p)/spanX,(h-2*p)/spanY)*mapZoom;
    const midX=(b.xmin+b.xmax)/2,midY=(b.ymin+b.ymax)/2;
    return {x:(v)=>w/2+(v-midX)*scale+mapPan.x,y:(v)=>h/2-(v-midY)*scale+mapPan.y,scale};
  }
  function drawMap(){
    if(!data)return;
    const {ctx,w,h}=canvasSize(UI.map);clearCanvas(ctx,w,h);
    if(!mapBounds){UI.mapEmpty.hidden=false;return}UI.mapEmpty.hidden=true;
    const tf=mapTransform(w,h);
    ctx.strokeStyle='#1e3b52';ctx.lineWidth=1;
    for(let i=1;i<5;i++){ctx.beginPath();ctx.moveTo(i*w/5,0);ctx.lineTo(i*w/5,h);ctx.moveTo(0,i*h/5);ctx.lineTo(w,i*h/5);ctx.stroke()}
    for(const key of ['master','rover']){
      const pts=data.route[key]||[];ctx.beginPath();ctx.strokeStyle=COLORS[key];ctx.globalAlpha=key==='master'?.78:.52;ctx.lineWidth=1.4;
      let prev=null;const stride=Math.max(1,Math.floor(pts.length/4500));
      for(let i=0;i<pts.length;i+=stride){const p=pts[i];const x=tf.x(p[1]),y=tf.y(p[2]);
        if(!prev||p[0]-prev[0]>1.1||Math.hypot(p[1]-prev[1],p[2]-prev[2])>45)ctx.moveTo(x,y);
        else ctx.lineTo(x,y);prev=p;
      }
      ctx.stroke();ctx.globalAlpha=1;
      const now=routeAt(pts,currentTime);
      if(now){const x=tf.x(now[1]),y=tf.y(now[2]);ctx.fillStyle=COLORS[key];ctx.strokeStyle='#06101c';ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,key==='master'?5:4,0,Math.PI*2);ctx.fill();ctx.stroke()}
    }
    const coreSeries=data.estimate?.series||[];
    if(data.estimate?.map_pose?.available&&coreSeries.length){
      ctx.beginPath();ctx.strokeStyle=COLORS.estimate;ctx.globalAlpha=.78;ctx.lineWidth=2;
      let prev=null;const stride=Math.max(1,Math.floor(coreSeries.length/4500));
      for(let i=0;i<coreSeries.length;i+=stride){const p=coreSeries[i];
        if(p.length<13)continue;
        const x=tf.x(p[10]),y=tf.y(p[11]);
        if(!prev||p[0]-prev[0]>1.1||Math.hypot(p[10]-prev[10],p[11]-prev[11])>45)ctx.moveTo(x,y);
        else ctx.lineTo(x,y);
        prev=p;
      }
      ctx.stroke();ctx.globalAlpha=1;
      const pose=estimateAt(currentTime)?.pose;
      if(pose){ctx.fillStyle=COLORS.estimate;ctx.strokeStyle='#111a29';ctx.lineWidth=2.5;
        ctx.beginPath();ctx.arc(tf.x(pose[0]),tf.y(pose[1]),6,0,Math.PI*2);ctx.fill();ctx.stroke()}
    }
    ctx.fillStyle='#91aabd';ctx.font='11px system-ui';ctx.fillText('E →',10,h-10);ctx.save();ctx.translate(12,50);ctx.rotate(-Math.PI/2);ctx.fillText('N ↑',0,0);ctx.restore();
  }
  function computeDiagnostic(s){
    const {front,rear,master,rover,cmd}=s;
    const delta=front&&rear?front.v-rear.v:null;
    const ref=master&&rover&&master.v<25&&rover.v<25&&Math.abs(master.v-rover.v)<.3?(master.v+rover.v)/2:null;
    let kind='ok',message='Колёса согласованы',slipFront=false,slipRear=false;
    if(!front&&!rear){kind='bad';message='Нет свежих данных обеих тележек'}
    else if(!front||!rear){kind='warn';message=`Нет свежих данных ${front?'задней':'передней'} тележки`}
    else if(ref!==null){
      const fErr=front.v-ref,rErr=rear.v-ref;
      if(Math.abs(fErr)>.6&&Math.abs(rErr)<.3){kind='bad';slipFront=true;message=`Передняя расходится с обоими GNSS на ${num(fErr)} м/с`}
      else if(Math.abs(rErr)>.6&&Math.abs(fErr)<.3){kind='bad';slipRear=true;message=`Задняя расходится с обоими GNSS на ${num(rErr)} м/с`}
      else if(Math.abs(fErr)>.7&&Math.abs(rErr)>.7){kind='bad';slipFront=slipRear=true;message='Обе тележки расходятся с согласованными GNSS'}
      else if(Math.abs(delta)>.5){kind='warn';slipFront=slipRear=true;message=`Расхождение тележек ${num(Math.abs(delta))} м/с`}
    }else if(Math.abs(delta||0)>.5){kind='warn';slipFront=slipRear=true;message=`Расхождение тележек ${num(Math.abs(delta))} м/с; источник неизвестен`}
    if(master&&rover&&Math.abs(master.v-rover.v)>1){kind='warn';message=`GNSS-приёмники расходятся на ${num(Math.abs(master.v-rover.v))} м/с`;slipFront=slipRear=false}
    if(front&&rear&&front.v>1&&rear.v>1&&master&&master.v<.1&&rover&&rover.v>1){kind='bad';message='GNSS master замирает; колёса и rover показывают движение';slipFront=slipRear=false}
    if(rover&&rover.v>25&&(s.velocity??0)<20){kind='bad';message=`Выброс GNSS rover: ${num(rover.v)} м/с`;slipFront=slipRear=false}
    if(master&&master.v>25&&(s.velocity??0)<20){kind='bad';message=`Выброс GNSS master: ${num(master.v)} м/с`;slipFront=slipRear=false}
    if(cmd&&cmd.v===-15&&front&&rear&&s.accel>.2){kind='warn';message='Скорость растёт при ручке −15: причина по этим каналам неясна'}
    return {kind,message,delta,slipFront,slipRear,reference:ref};
  }
  function stateAt(t){
    const wheel=estimateWheel(t,data.series),series=data.series;
    const master=sampleAt(series.master,t,.35),rover=sampleAt(series.rover,t,.35);
    const cmd=sampleAt(series.cmd,t,.5);
    const past=estimateWheel(Math.max(0,t-.6),series).velocity;
    const accel=wheel.velocity!==null&&past!==null?(wheel.velocity-past)/.6:0;
    return {...wheel,master,rover,cmd,accel};
  }
  function updateSnapshot(){
    if(!data)return;
    const s=stateAt(currentTime),d=computeDiagnostic(s),core=estimateAt(currentTime);
    const distance=core?.distance??odomAt(currentTime);
    UI.clock.textContent=fmtTime(currentTime);UI.scrubTime.textContent=fmtTime(currentTime);
    UI.speed.textContent=num(core?.velocity);UI.front.textContent=num(s.front?.v);UI.rear.textContent=num(s.rear?.v);
    UI.frontAge.textContent=s.front?`${num(s.front.age,2)} с назад`:'нет данных';
    UI.rearAge.textContent=s.rear?`${num(s.rear.age,2)} с назад`:'нет данных';
    UI.cmd.textContent=s.cmd?String(s.cmd.v):'—';UI.gnss.textContent=num(s.master?.v);
    UI.gnssRover.textContent=`rover ${num(s.rover?.v)} · сравнение`;
    UI.delta.textContent=num(d.delta===null?null:Math.abs(d.delta));
    UI.distance.textContent=core?(distance>=1000?`${num(distance/1000,2)} км`:`${num(distance,0)} м`):'—';
    UI.rawSpeed.textContent=num(s.velocity);UI.modelAccel.textContent=num(core?.modelAccel,3);
    UI.sensorWeights.textContent=core?`${num(core.frontWeight,2)} / ${num(core.rearWeight,2)}`:'—';
    if(core){
      const flags=[];
      if(core.frontSlip)flags.push('передняя отвергнута');
      if(core.rearSlip)flags.push('задняя отвергнута');
      if(core.modelOnly)flags.push('только модель');
      if(!flags.length)flags.push('датчики приняты');
      if(core.tableUsed)flags.push('таблица тяги');
      UI.estimatorStatus.className='diagnostic'+(core.frontSlip||core.rearSlip?' bad':core.modelOnly?' warn':'');
      UI.estimatorStatus.textContent=`C++: ${flags.join(' · ')}`;
      UI.sceneStatus.className='scene-status'+(core.frontSlip||core.rearSlip?' bad':core.modelOnly?' warn':'');
      UI.sceneStatus.textContent=core.frontSlip||core.rearSlip?`C++ · ${core.frontSlip?'передняя ':''}${core.rearSlip?'задняя ':''}недостоверна`:core.modelOnly?'C++ · только модель':'C++ · датчики приняты';
    }else{
      UI.estimatorStatus.className='diagnostic warn';
      UI.estimatorStatus.textContent=data.estimateError||'Ожидание выхода C++-оценивателя';
      UI.sceneStatus.className='scene-status warn';
      UI.sceneStatus.textContent='C++ · нет оценки';
    }
    UI.diagnostic.className='diagnostic'+(d.kind==='ok'?'':' '+d.kind);
    UI.diagnostic.textContent=`Визуальная проверка: ${d.message}`;
    const masterFix=routeAt(data.route.master,currentTime),roverFix=routeAt(data.route.rover,currentTime);
    if(masterFix||roverFix||core?.pose){
      const parts=[];
      if(core?.pose)parts.push(`base_link оценка E ${num(core.pose[0],0)} · N ${num(core.pose[1],0)} м`);
      if(masterFix)parts.push(`master E ${num(masterFix[1],0)} · N ${num(masterFix[2],0)} м`);
      if(roverFix)parts.push(`rover E ${num(roverFix[1],0)} · N ${num(roverFix[2],0)} м`);
      if(masterFix&&roverFix)parts.push(`антенны ${num(Math.hypot(masterFix[1]-roverFix[1],masterFix[2]-roverFix[2]),1)} м`);
      UI.mapReadout.textContent=parts.join('   |   ');
    }else UI.mapReadout.textContent=data.estimate?.map_pose?.reason||'Свежих GNSS-координат нет';
    UI.scrub.value=String(currentTime);
    if(scene)scene.setState({t:currentTime,velocity:core?.velocity??s.velocity??0,
      front:s.front?.v??null,rear:s.rear?.v??null,command:s.cmd?.v??0,distance,
      slipFront:core?.frontSlip??d.slipFront,slipRear:core?.rearSlip??d.slipRear,heading:0});
  }
  function seek(t,keepPlaying=false){
    currentTime=clamp(Number(t)||0,0,duration);if(!keepPlaying)playing=false;
    UI.play.textContent=playing?'Ⅱ':'▶';UI.play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести');
    chartDirty=mapDirty=true;updateSnapshot();drawCharts();drawMap();updateUrl();
  }
  function addEvents(events){
    UI.events.replaceChildren();
    const sorted=[...events].sort((a,b)=>a.t_s-b.t_s);
    const n=sorted.length,word=n%10===1&&n%100!==11?'закладка':n%10>=2&&n%10<=4&&(n%100<12||n%100>14)?'закладки':'закладок';
    UI.eventCount.textContent=`${n} ${word}`;
    if(!sorted.length){UI.events.textContent='Автоматические аномалии не выделены.';return}
    for(const ev of sorted.slice(0,250)){
      const btn=document.createElement('button');btn.className='event';btn.type='button';
      const stamp=document.createElement('span');stamp.className='event-time';
      const source={inputs_only:'входы',gnss_audit:'GNSS',manual_audit:'проверено',estimator:'C++'}[ev.source]||'отметка';
      stamp.textContent=`${fmtTime(ev.t_s)}${ev.end_s-ev.t_s>1?' · '+num(ev.end_s-ev.t_s,1)+' с':''} · ${source}`;
      const name=document.createElement('span');name.className='event-name';name.textContent=ev.label||ev.type;
      btn.append(stamp,name);btn.addEventListener('click',()=>seek(Math.max(0,ev.t_s-2)));
      UI.events.appendChild(btn);
    }
    if(sorted.length>250){const extra=document.createElement('span');extra.className='muted';extra.textContent=`Показаны первые 250 из ${sorted.length}. Остальные участки доступны на временной шкале.`;UI.events.appendChild(extra)}
  }
  async function loadBag(id,t=0){
    const seq=++requestSeq;playing=false;UI.play.textContent='▶';UI.bag.value=id;
    setStatus(`Читаю полный rosbag ${id}…`);
    try{
      let bag=cache.get(id);
      if(!bag){const response=await fetch('/api/bag/'+encodeURIComponent(id));if(!response.ok)throw new Error(`HTTP ${response.status}`);bag=await response.json();cache.set(id,bag);if(cache.size>3)cache.delete(cache.keys().next().value)}
      if(seq!==requestSeq)return;
      data=bag;duration=Math.max(bag.duration_s,...Object.values(bag.series).map(a=>a.length?a[a.length-1][0]:0));
      odo=buildOdom(bag,duration);mapBounds=routeBounds(bag.route,bag.estimate);mapZoom=1;mapPan={x:0,y:0};
      UI.meta.textContent=`Трамвай ${bag.vehicle} · ${fmtTime(duration,false)} · ${bag.counts.gnss_total?'GNSS есть':'GNSS нет'}`;
      UI.duration.textContent=fmtTime(duration);UI.endTime.textContent=fmtTime(duration,false);
      UI.scrub.max=String(duration);UI.scrub.step='0.01';
      addEvents([...(bag.events||[]),...(bag.estimate?.events||[])]);
      setStatus(`Загружено: ${bag.series.front.length.toLocaleString('ru-RU')} передних, ${bag.series.rear.length.toLocaleString('ru-RU')} задних, ${bag.series.cmd.length.toLocaleString('ru-RU')} команд`);
      seek(t);
      if(!bag.estimate){
        setStatus(`Replay C++-оценивателя по входам ${id}…`);
        try{
          const response=await fetch('/api/estimate/'+encodeURIComponent(id));
          if(!response.ok)throw new Error(`HTTP ${response.status}`);
          const estimate=await response.json();
          if(seq!==requestSeq)return;
          if(estimate.origin_header_ns!==bag.origin_header_ns)throw new Error('Время начала рядов не совпадает');
          bag.estimate=estimate;bag.estimateError=null;
          mapBounds=routeBounds(bag.route,estimate);
          addEvents([...(bag.events||[]),...(estimate.events||[])]);
          seek(currentTime);
          setStatus(`C++ replay: ${estimate.series.length.toLocaleString('ru-RU')} выходов · ${estimate.table_active?'таблица тяги включена':'аналитическая тяга'}`);
        }catch(error){
          if(seq!==requestSeq)return;
          bag.estimateError=`C++ replay недоступен: ${error.message}`;
          updateSnapshot();setStatus(bag.estimateError,true);
        }
      }else setStatus(`C++ replay: ${bag.estimate.series.length.toLocaleString('ru-RU')} выходов`);
    }catch(error){if(seq!==requestSeq)return;data=null;setStatus(`Не удалось открыть ${id}: ${error.message}. Запустите py -3.12 analysis/simulator/serve_simulator.py`,true)}
  }
  async function loadCatalog(){
    try{const response=await fetch('/api/catalog');if(!response.ok)throw new Error(`HTTP ${response.status}`);
      catalog=(await response.json()).bags;
      UI.bag.replaceChildren();for(const item of catalog){const o=document.createElement('option');o.value=item.id;o.textContent=`${item.id} · ${(item.duration_s/60).toFixed(1)} мин${item.has_gnss?'':' · без GNSS'}`;UI.bag.appendChild(o)}
      const id=catalog.some(x=>x.id===initial.bag)?initial.bag:catalog[0].id;await loadBag(id,initial.t);
    }catch(error){setStatus(`Сервер данных недоступен: ${error.message}. Запустите py -3.12 analysis/simulator/serve_simulator.py`,true);UI.bag.innerHTML='<option>Нет подключения</option>'}
  }
  function refreshHover(t){
    if(!data)return;
    const s=stateAt(t),core=estimateAt(t);
    UI.readout.textContent=`${fmtTime(t)} · оценка ${num(core?.velocity)} · передняя ${num(s.front?.v)} · задняя ${num(s.rear?.v)} · GNSS master ${num(s.master?.v)} · rover ${num(s.rover?.v)} м/с · путь C++ ${num(core?.distance,1)} м · ручка ${s.cmd?.v??'—'}`;
  }
  function chartPointer(event,click=false){
    if(!data)return;
    const r=event.currentTarget.getBoundingClientRect();const x=clamp(event.clientX-r.left,48,r.width-15);
    const time=chartDomain[0]+(x-48)/(r.width-63)*(chartDomain[1]-chartDomain[0]);
    if(click)seek(time);else{hoveredTime=time;chartDirty=true;refreshHover(time)}
  }
  function tick(now){
    if(!lastFrame)lastFrame=now;const dt=Math.min(.2,(now-lastFrame)/1000);lastFrame=now;
    if(data&&playing){currentTime+=dt*speedFactor;if(currentTime>=duration){currentTime=duration;playing=false;UI.play.textContent='▶'}chartDirty=mapDirty=true;updateSnapshot()}
    if(data&&(now-lastPaint>80||chartDirty||mapDirty)){
      if(chartDirty)drawCharts();if(mapDirty)drawMap();lastPaint=now;chartDirty=mapDirty=false;
    }
    requestAnimationFrame(tick);
  }
  UI.bag.addEventListener('change',()=>loadBag(UI.bag.value,0));
  UI.play.addEventListener('click',()=>{if(!data)return;if(currentTime>=duration)seek(0);playing=!playing;UI.play.textContent=playing?'Ⅱ':'▶';UI.play.setAttribute('aria-label',playing?'Пауза':'Воспроизвести')});
  UI.back.addEventListener('click',()=>seek(currentTime-10));UI.forward.addEventListener('click',()=>seek(currentTime+10));
  UI.scrub.addEventListener('input',()=>seek(Number(UI.scrub.value)));
  UI.rate.addEventListener('change',()=>speedFactor=Number(UI.rate.value)||1);
  for(const b of document.querySelectorAll('[data-window]'))b.addEventListener('click',()=>{
    windowSeconds=Number(b.dataset.window);document.querySelectorAll('[data-window]').forEach(x=>x.classList.toggle('active',x===b));chartDirty=true;drawCharts();
  });
  for(const b of document.querySelectorAll('.preset'))b.addEventListener('click',()=>loadBag(b.dataset.bag,Number(b.dataset.t)));
  for(const c of [UI.speedChart,UI.cmdChart]){
    c.addEventListener('pointermove',e=>chartPointer(e));c.addEventListener('pointerdown',e=>chartPointer(e,true));
    c.addEventListener('pointerleave',()=>{hoveredTime=null;chartDirty=true;UI.readout.textContent='Нажмите на график, чтобы перейти к моменту записи.'});
  }
  UI.mapReset.addEventListener('click',()=>{mapZoom=1;mapPan={x:0,y:0};mapDirty=true});
  UI.map.addEventListener('wheel',e=>{if(!data||!mapBounds)return;e.preventDefault();mapZoom=clamp(mapZoom*(e.deltaY<0?1.17:.85),1,25);mapDirty=true},{passive:false});
  UI.map.addEventListener('pointerdown',e=>{mapDrag={x:e.clientX,y:e.clientY,px:mapPan.x,py:mapPan.y};UI.map.setPointerCapture(e.pointerId)});
  UI.map.addEventListener('pointermove',e=>{if(!mapDrag)return;mapPan={x:mapDrag.px+e.clientX-mapDrag.x,y:mapDrag.py+e.clientY-mapDrag.y};mapDirty=true});
  UI.map.addEventListener('pointerup',()=>mapDrag=null);UI.map.addEventListener('pointercancel',()=>mapDrag=null);
  window.addEventListener('resize',()=>{chartDirty=mapDirty=true;scene?.resize()});
  document.addEventListener('keydown',e=>{if(e.target instanceof HTMLInputElement||e.target instanceof HTMLSelectElement)return;
    if(e.code==='Space'){e.preventDefault();UI.play.click()}else if(e.code==='ArrowLeft')seek(currentTime-1);else if(e.code==='ArrowRight')seek(currentTime+1)});
  if(window.TramScene){try{scene=window.TramScene.create($('scene-canvas'))}catch(error){setStatus(`3D-сцена недоступна: ${error.message}`,true)}}
  loadCatalog();requestAnimationFrame(tick);
})();
