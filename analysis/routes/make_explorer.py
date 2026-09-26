"""Generate a self-contained interactive 2D route explorer (no ROS needed)."""
import json
from html import escape
import numpy as np
import plotly.graph_objects as go
from analyze_gnss import OUT


def main():
    arr=np.load(OUT/'sampled_tracks.npz')
    summary={s['bag']:s for s in json.loads((OUT/'summaries.json').read_text())}
    bags=sorted(arr.files)
    fig=go.Figure()
    allx=[];ally=[]
    for bag in bags:
        p=arr[bag]
        allx.extend(p[::4,0].tolist()+[None]);ally.extend(p[::4,1].tolist()+[None])
    fig.add_trace(go.Scattergl(x=allx,y=ally,mode='lines',line=dict(color='rgba(75,90,110,.09)',width=1),
                               name='Все траектории',hoverinfo='skip',showlegend=True))
    for i,bag in enumerate(bags):
        p=arr[bag];s=summary[bag]
        direction='Восток → запад' if s['master_path'].get('end_enu',[0])[0]<s['master_path'].get('start_enu',[0])[0] else 'Запад → восток'
        custom=np.column_stack([p[:,2]])
        fig.add_trace(go.Scattergl(x=p[:,0],y=p[:,1],customdata=custom,mode='lines',
                                   line=dict(color='#1463b8' if s['vehicle']=='30618' else '#d35d27',width=3),
                                   name=f'{bag} · {direction}',visible=(i==0),
                                   hovertemplate='East %{x:.1f} м<br>North %{y:.1f} м<br>Up %{customdata[0]:.1f} м<extra>'+bag+'</extra>'))
    fig.update_layout(template='plotly_white',height=700,autosize=True,
                      xaxis=dict(title='East, м',range=[-4900,200],scaleanchor='y',scaleratio=1),
                      yaxis=dict(title='North, м',range=[-1800,200]),
                      margin=dict(t=20,l=65,r=20,b=55))
    plot=fig.to_html(include_plotlyjs=True,full_html=False,div_id='route_plot',
                     default_width='100%',default_height='700px',config={'responsive':True})
    options='\n'.join(f'<option value="{i}">{escape(bag)}</option>' for i,bag in enumerate(bags))
    metadata=[]
    for bag in bags:
        s=summary[bag]; start=s['master_path']['start_enu'][0]; end=s['master_path']['end_enu'][0]
        metadata.append(f"Трамвай {s['vehicle']} · {'восток → запад' if end<start else 'запад → восток'} · {s['topics']['mf']['count']:,} GNSS fix".replace(',', ' '))
    html=f'''<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GNSS master — просмотр маршрутов</title>
<style>
*{{box-sizing:border-box}}
html,body{{margin:0;max-width:100%;overflow-x:hidden;background:#f8fafc;color:#142238;font-family:system-ui,-apple-system,Segoe UI,sans-serif}}
main{{width:100%;max-width:1600px;margin:0 auto;padding:18px clamp(12px,2vw,28px)}}
h1{{font-size:clamp(22px,2.4vw,30px);line-height:1.2;margin:0 0 8px}}
p{{margin:0 0 15px;color:#526173;line-height:1.4}}
.toolbar{{display:flex;flex-wrap:wrap;align-items:center;gap:8px 14px;margin-bottom:14px}}
label{{font-weight:600}}
select{{font:inherit;max-width:100%;min-width:180px;padding:7px 10px;border:1px solid #aab8c9;border-radius:6px;background:#fff}}
#run-meta{{color:#526173}}
.chart{{width:100%;min-width:0;max-width:100%;overflow:hidden;border:1px solid #dde4ed;border-radius:8px;background:#fff}}
#route_plot{{width:100%!important;max-width:100%!important}}
@media(max-width:600px){{main{{padding:12px}}.chart{{border:0}}#run-meta{{flex-basis:100%}}}}
</style></head><body><main>
<h1>GNSS master: траектории трамваев</h1>
<p>Фон показывает все прогоны. Начало локальной ENU: 55.810367065° N, 37.462266845° E, 168.3794 м.</p>
<div class="toolbar"><label for="bag-select">Прогон</label><select id="bag-select">{options}</select><span id="run-meta"></span></div>
<div class="chart">{plot}</div>
</main><script>
const runMetadata={json.dumps(metadata,ensure_ascii=False)};
const selector=document.getElementById('bag-select');
const info=document.getElementById('run-meta');
function selectRun(){{
  const selected=Number(selector.value);
  const visibility=Array.from({{length:runMetadata.length+1}},(_,i)=>i===0||i===selected+1);
  Plotly.restyle('route_plot',{{visible:visibility}});
  info.textContent=runMetadata[selected];
}}
selector.addEventListener('change',selectRun);
info.textContent=runMetadata[0];
</script></body></html>'''
    (OUT/'map_explorer.html').write_text(html,encoding='utf-8')
    print('Created',OUT/'map_explorer.html')


if __name__=='__main__':main()
