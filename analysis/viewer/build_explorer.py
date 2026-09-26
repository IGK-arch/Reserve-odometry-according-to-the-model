"""Build a standalone, fully offline Plotly explorer for all 122 ROS 2 bags.

Run from the repository root with ``py -3.12 analysis/viewer/build_explorer.py``.
The generated HTML contains a decimated overview of every bag; use
``make_plots.py BAG_ID`` for full-resolution diagnostic PNGs.
"""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from plotly.offline import get_plotlyjs
from pyproj import Transformer

from rosbag_cdr import messages


ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "analysis" / "catalog" / "bags.csv"
DATA = ROOT / "dataset" / "data"
DEST = ROOT / "analysis" / "viewer" / "dataset_explorer.html"
TOPICS = {
    "/vehicle/front_bogie_velocity",
    "/vehicle/rear_bogie_velocity",
    "/vehicle/driver_position_cmd",
    "/sensing/gnss/master/fix",
    "/sensing/gnss/master/vel",
    "/sensing/gnss/rover/fix",
    "/sensing/gnss/rover/vel",
}
ECEF = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)
LAT0, LON0, H0 = 55.805, 37.425, 170.0
X0, Y0, Z0 = ECEF.transform(LON0, LAT0, H0)
PHI, LAM = np.deg2rad(LAT0), np.deg2rad(LON0)


def minmax_sample(t: list[float], y: list[float], max_points: int = 900,
                  gap_threshold: float | None = None) -> list[list[float | None]]:
    if not t:
        return []
    if len(t) <= max_points:
        chosen = set(range(len(t)))
    else:
        buckets = max(1, max_points // 4)
        edges = np.linspace(0, len(t), buckets + 1, dtype=int)
        chosen: set[int] = set()
        for lo, hi in zip(edges[:-1], edges[1:]):
            if hi <= lo:
                continue
            sl = y[lo:hi]
            chosen.update((lo, hi - 1, lo + int(np.argmin(sl)), lo + int(np.argmax(sl))))
    result: list[list[float | None]] = [[round(t[i], 2), round(y[i], 3)] for i in sorted(chosen)]
    if gap_threshold is not None:
        result.extend([[round((t[i - 1] + t[i]) / 2, 2), None]
                       for i in range(1, len(t)) if t[i] - t[i - 1] > gap_threshold])
        result.sort(key=lambda p: p[0])
    return result


def route_sample(items: list[dict], max_points: int = 450) -> list[list[float]]:
    if not items:
        return []
    step = max(1, len(items) // max_points)
    selected = items[::step]
    if selected[-1] is not items[-1]:
        selected.append(items[-1])
    lon = np.array([m["longitude"] for m in selected])
    lat = np.array([m["latitude"] for m in selected])
    h = np.array([m["altitude"] for m in selected])
    x, y, z = ECEF.transform(lon, lat, h)
    dx, dy, dz = x - X0, y - Y0, z - Z0
    east = -np.sin(LAM) * dx + np.cos(LAM) * dy
    north = -np.sin(PHI) * np.cos(LAM) * dx - np.sin(PHI) * np.sin(LAM) * dy + np.cos(PHI) * dz
    up = np.cos(PHI) * np.cos(LAM) * dx + np.cos(PHI) * np.sin(LAM) * dy + np.sin(PHI) * dz
    return [[round(e, 1), round(n, 1), round(u, 1)] for e, n, u in zip(east, north, up)]


def extract(bag_id: str, meta: dict) -> dict:
    raw: dict[str, list[dict]] = defaultdict(list)
    for topic, _, msg in messages(DATA / bag_id, TOPICS):
        raw[topic].append(msg)
    vehicle_stamps = [m["stamp_ns"] for name in raw if name.startswith("/vehicle/") for m in raw[name]]
    origin_ns = min(vehicle_stamps) if vehicle_stamps else 0
    out = {
        "id": bag_id,
        "vehicle": meta["vehicle"],
        "duration": round(float(meta["duration_s"]), 2),
        "gnss": bool(int(meta["gnss_count"])),
        "counts": {"front": int(meta["front_count"]), "rear": int(meta["rear_count"]),
                   "cmd": int(meta["cmd_count"]), "gnss": int(meta["gnss_count"])},
        "series": {},
        "route": {},
    }
    for receiver in ("master", "rover"):
        topic = f"/sensing/gnss/{receiver}/fix"
        out["route"][receiver] = route_sample(raw.get(topic, []))
    for key, topic in (
        ("front", "/vehicle/front_bogie_velocity"),
        ("rear", "/vehicle/rear_bogie_velocity"),
        ("cmd", "/vehicle/driver_position_cmd"),
        ("master", "/sensing/gnss/master/vel"),
        ("rover", "/sensing/gnss/rover/vel"),
    ):
        items = sorted(raw.get(topic, []), key=lambda m: m["stamp_ns"])
        t = [(m["stamp_ns"] - origin_ns) / 1e9 for m in items]
        if key in ("front", "rear"):
            y = [m["velocity_raw"] / 3.6 for m in items]
        elif key == "cmd":
            y = [m["position"] for m in items]
        else:
            y = [float(np.linalg.norm(m["linear"])) for m in items]
        out["series"][key] = minmax_sample(t, y, 1200 if key != "cmd" else 850,
                                            0.25 if key == "cmd" else 0.6)
    return out


def main() -> None:
    with CATALOG.open(newline="", encoding="utf-8") as f:
        catalog = list(csv.DictReader(f))
    runs: dict[str, dict] = {}
    for idx, meta in enumerate(catalog, 1):
        bag_id = meta["bag"]
        runs[bag_id] = extract(bag_id, meta)
        if idx % 10 == 0 or idx == len(catalog):
            print(f"{idx}/{len(catalog)}", flush=True)
    data = json.dumps(runs, ensure_ascii=False, separators=(",", ":"))
    js = get_plotlyjs()
    html = f"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Трамвайные rosbag: офлайн-обзор</title>
<style>
body{{font-family:system-ui,-apple-system,Segoe UI,sans-serif;margin:0;padding:20px;background:#f7f9fc;color:#182132}}
main{{max-width:1500px;margin:auto}}h1{{font-size:22px;margin:0 0 6px}}p{{margin:6px 0 15px;color:#475569}}
label{{font-weight:600;margin-right:8px}}select{{font:inherit;max-width:100%;padding:5px}}
.bar{{display:flex;flex-wrap:wrap;align-items:center;gap:10px;margin-bottom:10px}}
.details{{font-size:14px;color:#475569}}.plots{{display:grid;grid-template-columns:43% 57%;gap:14px}}
.plot{{min-width:0;background:white;border:1px solid #dae1ec;border-radius:8px;overflow:hidden}}
@media(max-width:900px){{.plots{{grid-template-columns:1fr}}}}
small{{color:#64748b}}
</style><script>{js}</script></head><body><main>
<h1>Телеметрия трамваев: все 122 ROS 2 bag</h1>
<p>Выберите прогон. Графики содержат прореженные точки с сохранением локальных минимумов и максимумов; для полной детализации используйте <code>make_plots.py BAG_ID</code>.</p>
<div class="bar"><label for="bag">Прогон</label><select id="bag"></select><span id="details" class="details"></span></div>
<div class="plots"><div id="route" class="plot"></div><div id="signals" class="plot"></div></div>
<p><small>Скорости тележек переведены из фактических км/ч в м/с. GNSS speed — норма вектора TwistStamped. Маршрут — ENU от 55.805°, 37.425°, 170 м; оси не являются системой координат автоматического судьи.</small></p>
</main><script>
const runs={data};
const names=Object.keys(runs).sort();
const sel=document.getElementById('bag');
for(const id of names){{let o=document.createElement('option');o.value=id;o.textContent=id+' · '+(runs[id].duration/60).toFixed(1)+' мин'+(runs[id].gnss?'':' · без GNSS');sel.append(o)}}
sel.value=runs['30618_0e41eac3']?'30618_0e41eac3':names[0];
function line(points,name,color,yaxis='y'){{return {{x:points.map(p=>p[0]/60),y:points.map(p=>p[1]),name,mode:'lines',line:{{color,width:1.4}},yaxis,xaxis:yaxis==='y2'?'x2':'x',connectgaps:false,hovertemplate:'%{{x:.2f}} мин · %{{y:.3f}}<extra>'+name+'</extra>'}}}}
function route(points,name,color){{return {{x:points.map(p=>p[0]),y:points.map(p=>p[1]),name,mode:'lines',line:{{color,width:1.5}},hovertemplate:'E %{{x:.1f}} м · N %{{y:.1f}} м<extra>'+name+'</extra>'}}}}
function draw(){{
 const b=runs[sel.value],s=b.series;
 document.getElementById('details').textContent=`Трамвай ${{b.vehicle}} · front ${{b.counts.front.toLocaleString()}} · rear ${{b.counts.rear.toLocaleString()}} · cmd ${{b.counts.cmd.toLocaleString()}} · GNSS ${{b.counts.gnss.toLocaleString()}}`;
 const rt=[];if(b.route.master.length)rt.push(route(b.route.master,'GNSS master','#2563eb'));if(b.route.rover.length)rt.push(route(b.route.rover,'GNSS rover','#ea580c'));
 Plotly.react('route',rt,{{title:{{text:rt.length?'Траектория GNSS':'GNSS-позиции отсутствуют',font:{{size:16}}}},xaxis:{{title:'East, м',showgrid:true}},yaxis:{{title:'North, м',scaleanchor:'x',scaleratio:1,showgrid:true}},height:520,margin:{{l:65,r:25,t:50,b:60}},paper_bgcolor:'white',plot_bgcolor:'white',legend:{{orientation:'h',y:-0.2}}}},{{responsive:true,displaylogo:false}});
 const traces=[];
 if(s.master.length)traces.push(line(s.master,'GNSS master','#16a34a'));
 if(s.rover.length)traces.push(line(s.rover,'GNSS rover','#a855f7'));
 traces.push(line(s.front,'Передняя /3,6','#2563eb'),line(s.rear,'Задняя /3,6','#ea580c'));
 traces.push(line(s.cmd,'Ручка','#64748b','y2'));
 Plotly.react('signals',traces,{{title:{{text:'Скорость и положение ручки',font:{{size:16}}}},xaxis:{{showgrid:true,showticklabels:false,anchor:'y'}},xaxis2:{{title:'Время от первого входа, мин',showgrid:true,matches:'x',anchor:'y2'}},yaxis:{{title:'Скорость, м/с',showgrid:true,domain:[0.31,1]}},yaxis2:{{title:'Позиция ручки',range:[-16,16],domain:[0,0.18],showgrid:true}},height:520,margin:{{l:65,r:25,t:50,b:65}},paper_bgcolor:'white',plot_bgcolor:'white',legend:{{orientation:'h',y:-0.22}}}},{{responsive:true,displaylogo:false}});
}}
sel.addEventListener('change',draw);draw();
</script></body></html>"""
    DEST.write_text(html, encoding="utf-8")
    print(f"Wrote {DEST} ({DEST.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
