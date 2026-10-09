"""S4 mass tool: click mass points onto the ATLAS v1 candidate geometry in the browser.

  .venv/bin/python scripts/atlas_v1_mass_tool.py            # then open http://127.0.0.1:8099

Shows the candidate's three surfaces (new upper skin, kept lower skin, kept canopy) from the Blender working copy's
meshes (`meshes_mm.npz`, CAD frame, mm) in a three.js view. Click a surface to drop a mass point, type a name and
grams, drag the list to edit, delete or move points (re-pick), toggle surfaces, save. The page shows the total mass
and the CG of the points placed so far (point masses only; inertia is S5's job).

Output: `masses.json` next to the candidate (`--out`), schema from brain/HANDOFF.md S4:
  {"schema": 1, "candidate": ..., "frames": {...}, "updated": iso, "items": [
     {"name", "grams", "pos_cad_m", "pos_frd_m", "picked_on_body", "timestamp"} ]}

Frames: CAD is the STEP's (x right, y nose, z up, mm; stored here in metres, origin as in the STEP).
FRD = R @ CAD_m with R = [[0,-1,0],[-1,0,0],[0,0,-1]] (the NOSE is CAD -Y, the long slender end, per the user on
2026-10-05; Agent B's "nose +Y" was wrong) and a zero translation (declared choice). The S5 CG step and the airframe builder subtract the
CG themselves.
"""
import argparse
import json
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAND_DIR = ROOT / "results/handoff/T20261005-atlas-v1-foil-references/from_A/v002"
R_CAD_TO_FRD = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]   # nose is CAD -Y (user, 2026-10-05 evening): x_frd = -y_cad, y_frd = -x_cad
BODIES = [  # npz key, display name, colour
    ("candidate_upper_skin", "ATLAS_V1_CAND_V002_UPPER_SKIN", "#4f8fdc"),
    ("source_lower_skin", "SOURCE_SHELL_2_LOWER_SKIN", "#8d9aa8"),
    ("source_canopy", "SOURCE_SHELL_0_CANOPY", "#d9b84a"),
]

lock = threading.Lock()
BOM_DEFAULT = Path.home() / "Downloads/ATLAS_Sourcing_BOM_7.xlsx"


def load_bom(xlsx_path):
    """Airborne lines of the BOM sheet: numeric unit weight (not 'ground'), amount > 0 or unknown; ALTERNATE lines
    (amount 0) drop out by themselves. Returns {"source", "sha256", "parts": [{row, subsystem, name, qty, unit_g,
    total_g, ordered, note}]}."""
    import hashlib
    import openpyxl
    import warnings
    xlsx_path = Path(xlsx_path)
    if not xlsx_path.exists():
        return {"source": str(xlsx_path), "sha256": None, "parts": [], "error": "BOM file not found"}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        wb = openpyxl.load_workbook(xlsx_path, data_only=True, read_only=True)
    ws = wb["BOM"] if "BOM" in wb.sheetnames else wb[wb.sheetnames[0]]
    header, parts, subsystem = None, [], ""
    for i, row in enumerate(ws.iter_rows(values_only=True)):
        if header is None:
            if row and row[0] == "Name":
                header = {str(c).strip(): j for j, c in enumerate(row) if c is not None}
            continue
        name = row[header["Name"]] if header["Name"] < len(row) else None
        if not name:
            continue
        name = str(name).strip()
        amount = row[header.get("Amount", 3)]
        if amount is None and row[header.get("Part price", 6)] == "Subtotal":
            subsystem = name.split("  ")[-1].strip().title() if "  " in name else name
            continue
        unit = row[header.get("Unit weight (kg)", 8)]
        try:
            unit_g = float(unit) * 1000.0
        except (TypeError, ValueError):
            continue  # 'ground' equipment or no weight: not on the aircraft
        try:
            qty = int(float(amount)) if amount is not None else None
        except (TypeError, ValueError):
            qty = None
        if qty == 0:
            continue
        parts.append({
            "row": i + 1, "subsystem": subsystem, "name": name, "qty": qty, "unit_g": round(unit_g, 1),
            "total_g": round(unit_g * qty, 1) if qty else None,
            "ordered": str(row[header.get("Ordered", 1)] or "").upper() == "Y",
            "note": str(row[header.get("Note", 10)] or "")[:300],
        })
    return {"source": str(xlsx_path), "sha256": hashlib.sha256(xlsx_path.read_bytes()).hexdigest(), "parts": parts}


def load_meshes(npz_path):
    z = np.load(npz_path)
    out = []
    for key, name, colour in BODIES:
        v = (np.asarray(z[key + "__v"], float) / 1000.0).round(4)
        f = np.asarray(z[key + "__f"], np.int64)
        out.append({"name": name, "colour": colour, "vertices": v.ravel().tolist(), "faces": f.ravel().tolist()})
    return {"units": "m", "frame": "CAD (x right, y nose, z up)", "bodies": out}


def now_iso():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def frd(p_cad_m):
    r = np.array(R_CAD_TO_FRD, float)
    return (r @ np.asarray(p_cad_m, float)).round(4).tolist()


def empty_doc(args):
    return {
        "schema": 1,
        "candidate": str(Path(args.npz).resolve().relative_to(ROOT)),
        "frames": {
            "pos_cad_m": "STEP frame, x right, y TAIL (the nose is -Y), z up, metres, STEP origin",
            "pos_frd_m": "x forward, y right, z down, metres; FRD = R @ CAD, origin translation 0 (declared)",
            "R_cad_to_frd": R_CAD_TO_FRD,
        },
        "updated": None,
        "items": [],
    }


def read_doc(args):
    p = Path(args.out)
    if p.exists():
        return json.loads(p.read_text())
    return empty_doc(args)


def write_doc(args, items):
    doc = read_doc(args)
    clean = []
    for it in items:
        pos = [float(x) for x in it["pos_cad_m"]]
        clean.append({
            "name": str(it.get("name", "")).strip() or "unnamed",
            "grams": float(it.get("grams", 0) or 0),
            "bom_row": it.get("bom_row"),
            "bom_name": it.get("bom_name"),
            "subsystem": it.get("subsystem"),
            "pos_cad_m": [round(x, 4) for x in pos],
            "pos_frd_m": frd(pos),
            "picked_on_body": it.get("picked_on_body", ""),
            "timestamp": it.get("timestamp") or now_iso(),
        })
    doc["items"] = clean
    doc["updated"] = now_iso()
    p = Path(args.out)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    tmp.replace(p)
    return doc


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>ATLAS v1 mass tool</title>
<style>
:root{--bg:#14171c;--panel:#1e232b;--line:#2f3741;--fg:#e6e9ee;--mute:#9aa4b1;--acc:#4f8fdc;--warn:#e0a53a;--bad:#d9534f;--ok:#4caf7a}
*{box-sizing:border-box} html,body{margin:0;height:100%;overflow:hidden;overscroll-behavior:none;background:var(--bg);color:var(--fg);font:14px/1.4 -apple-system,Segoe UI,Helvetica,Arial,sans-serif}
#app{display:grid;grid-template-columns:1fr 440px;height:100%}
#view{position:relative;min-width:0;overflow:hidden} canvas{display:block;touch-action:none;cursor:crosshair}
#views{position:absolute;right:12px;top:10px;display:flex;gap:4px} #views button{background:rgba(30,35,43,.85);color:var(--fg);border:1px solid var(--line);border-radius:5px;padding:3px 9px;font-size:12px;cursor:pointer} #views button:hover{border-color:var(--mute)}
#side{border-left:1px solid var(--line);background:var(--panel);display:flex;flex-direction:column;min-height:0;height:100%;overflow:hidden}
.sec{flex:none}
#side h1{font-size:15px;margin:0;padding:12px 14px;border-bottom:1px solid var(--line)}
#side h1 small{color:var(--mute);font-weight:normal;margin-left:6px}
.sec{padding:10px 14px;border-bottom:1px solid var(--line)}
.row{display:flex;gap:8px;align-items:center;margin:4px 0;flex-wrap:wrap}
label.t{display:flex;align-items:center;gap:6px;color:var(--mute);font-size:13px}
.sw{display:inline-block;width:10px;height:10px;border-radius:2px}
#list{flex:1 1 0;overflow:auto;padding:6px 8px;min-height:70px}
#pal{flex:1.6 1 0;overflow:auto;padding:4px 8px;min-height:90px;border-bottom:1px solid var(--line)}
#pal .grp{color:var(--mute);font-size:11px;text-transform:uppercase;letter-spacing:.04em;margin:8px 4px 2px}
.part{display:grid;grid-template-columns:1fr 64px 44px;gap:6px;align-items:center;padding:3px 6px;border-radius:5px;border:1px solid transparent;cursor:pointer;font-size:13px}
.part:hover{background:#262c36} .part.sel{border-color:var(--acc);background:#223044}
.part .pn{overflow:hidden;text-overflow:ellipsis;white-space:nowrap} .part .pw{color:var(--mute);text-align:right;font-variant-numeric:tabular-nums}
.part .pq{text-align:right;font-variant-numeric:tabular-nums;color:var(--mute)} .part.done .pq{color:var(--ok)} .part.over .pq{color:var(--warn)}
#search{width:100%;background:#10131a;color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:4px 8px;font-size:13px}
#cur{font-size:13px} #cur input{width:80px;background:#10131a;color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:3px 6px;text-align:right}
#cur button{background:#2b323c;color:var(--fg);border:0;border-radius:4px;padding:3px 8px;cursor:pointer;font-size:12px}
.item{display:grid;grid-template-columns:22px 1fr 70px 26px 26px;gap:6px;align-items:center;padding:4px 6px;border-radius:6px;border:1px solid transparent}
.item:hover{background:#262c36} .item.sel{border-color:var(--acc)}
.item input{width:100%;background:#10131a;color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:3px 6px;font-size:13px}
.item input.g{text-align:right}
.item .n{font-size:11px;color:var(--mute);text-align:center}
.item button{background:none;border:1px solid var(--line);color:var(--mute);border-radius:4px;cursor:pointer;height:24px;padding:0;font-size:12px}
.item button:hover{color:var(--fg);border-color:var(--mute)}
.item button.mv.on{color:var(--warn);border-color:var(--warn)}
#tot{font-size:13px;color:var(--mute)} #tot b{color:var(--fg)}
button.big{background:var(--acc);color:#fff;border:0;border-radius:6px;padding:8px 14px;font-size:14px;cursor:pointer}
button.big.alt{background:#2b323c;color:var(--fg)}
button.big:disabled{opacity:.5;cursor:default}
#msg{font-size:12px;color:var(--mute);min-height:16px}
#hint{position:absolute;left:12px;bottom:10px;font-size:12px;color:var(--mute);background:rgba(20,23,28,.75);padding:4px 8px;border-radius:6px}
#coord{position:absolute;left:12px;top:10px;font-size:12px;color:var(--mute);background:rgba(20,23,28,.75);padding:4px 8px;border-radius:6px;font-variant-numeric:tabular-nums}
.lbl{color:#fff;font-size:11px;background:rgba(0,0,0,.6);padding:1px 5px;border-radius:4px;white-space:nowrap;pointer-events:none}
.lbl.cg{background:rgba(224,165,58,.85);color:#111}
kbd{border:1px solid var(--line);border-radius:3px;padding:0 4px;font-size:11px}
</style>
<script type="importmap">{"imports":{
 "three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
 "three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body><div id="app">
<div id="view"><div id="coord">loading geometry</div><div id="views"><button data-v="iso">Iso</button><button data-v="top">Top</button><button data-v="front">Front</button><button data-v="side">Side</button><button data-v="below">Below</button></div><div id="hint">click: add a point. drag: orbit. right drag or shift+drag: pan. wheel: zoom. double click: centre the view there. <kbd>Esc</kbd> cancels a move</div></div>
<div id="side">
 <h1>ATLAS v1 mass points <small>S4, candidate v002</small></h1>
 <div class="sec" id="bodies"></div>
 <div class="sec"><div class="row" style="justify-content:space-between"><span>BOM parts <span id="bomsum" style="color:var(--mute);font-size:12px"></span></span><input id="search" placeholder="filter parts" style="width:160px"></div>
  <div class="row" id="cur">no part selected: a click places a free point</div>
  <div class="row"><label class="t"><input type="checkbox" id="mirror"> mirror the next point to the other side (x -> -x)</label></div></div>
 <div id="pal"></div>
 <div class="sec" style="padding:6px 14px"><div class="row" style="justify-content:space-between"><span>Placed</span><span id="tot"></span></div></div>
 <div id="list"></div>
 <div class="sec"><div class="row"><button class="big" id="save">Save masses.json</button><button class="big alt" id="reload">Reload</button><button class="big alt" id="fit">Fit view</button></div><div id="msg"></div></div>
</div></div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {CSS2DRenderer, CSS2DObject} from 'three/addons/renderers/CSS2DRenderer.js';

const view=document.getElementById('view'), listEl=document.getElementById('list'), msg=document.getElementById('msg');
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x14171c);
const camera=new THREE.PerspectiveCamera(45,1,0.01,100);
const renderer=new THREE.WebGLRenderer({antialias:true}); view.appendChild(renderer.domElement);
const labels=new CSS2DRenderer(); labels.domElement.style.cssText='position:absolute;top:0;left:0;pointer-events:none'; view.appendChild(labels.domElement);
const controls=new OrbitControls(camera,renderer.domElement); controls.mouseButtons={LEFT:THREE.MOUSE.ROTATE,MIDDLE:THREE.MOUSE.DOLLY,RIGHT:THREE.MOUSE.PAN};
controls.enableDamping=true; controls.dampingFactor=0.12; controls.rotateSpeed=0.7; controls.zoomSpeed=0.7; controls.panSpeed=0.8; controls.screenSpacePanning=true; controls.minDistance=0.2; controls.maxDistance=40; controls.zoomToCursor=true;
renderer.domElement.addEventListener('contextmenu',e=>e.preventDefault());
renderer.domElement.addEventListener('pointerdown',e=>{if(e.button===0&&e.shiftKey){controls.mouseButtons.LEFT=THREE.MOUSE.PAN;}},{capture:true});
window.addEventListener('pointerup',()=>{controls.mouseButtons.LEFT=THREE.MOUSE.ROTATE;},{capture:true});
// CAD frame: x right, y nose, z up.  Three's camera up = CAD z.
camera.up.set(0,0,1);
scene.add(new THREE.HemisphereLight(0xffffff,0x334455,1.1)); const dl=new THREE.DirectionalLight(0xffffff,1.2); dl.position.set(2,-3,5); scene.add(dl);
const dl2=new THREE.DirectionalLight(0xffffff,.5); dl2.position.set(-3,2,-2); scene.add(dl2);
const meshes=[], pointsGroup=new THREE.Group(); scene.add(pointsGroup);
let items=[], sel=-1, moving=-1, dirty=false, centre=new THREE.Vector3();
let bom={parts:[]}, curPart=null, curGrams=0;
const palEl=document.getElementById('pal'), curEl=document.getElementById('cur');
function placedCount(p){return items.filter(i=>i.bom_row===p.row).length;}
function renderPalette(){
 const q=document.getElementById('search').value.trim().toLowerCase(); palEl.innerHTML=''; let grp='';
 let placedG=0, bomG=0; for(const p of bom.parts){bomG+=p.total_g||0;}
 for(const p of bom.parts){
  if(q && !(p.name.toLowerCase().includes(q)||p.subsystem.toLowerCase().includes(q))) continue;
  if(p.subsystem!==grp){grp=p.subsystem; const g=document.createElement('div'); g.className='grp'; g.textContent=grp; palEl.appendChild(g);}
  const n=placedCount(p); const d=document.createElement('div');
  d.className='part'+(curPart&&curPart.row===p.row?' sel':'')+(p.qty!=null&&n>=p.qty?(n>p.qty?' over':' done'):'');
  d.title=(p.note||'')+(p.ordered?' [ordered]':''); d.innerHTML=`<span class="pn">${esc(p.name)}</span><span class="pw">${p.unit_g} g</span><span class="pq">${n} / ${p.qty==null?'?':p.qty}</span>`;
  d.onclick=()=>{selectPart(curPart&&curPart.row===p.row?null:p);}; palEl.appendChild(d);
 }
 for(const it of items) placedG+=+it.grams||0;
 document.getElementById('bomsum').textContent=`${bom.parts.length} lines, ${(bomG/1000).toFixed(1)} kg airborne`;
}
function selectPart(p){curPart=p; curGrams=p?p.unit_g:0; renderPalette(); renderCur();}
function renderCur(){
 if(!curPart){curEl.textContent='no part selected: a click places a free point (name it yourself)'; return;}
 const n=placedCount(curPart);
 curEl.innerHTML=`<b>${esc(curPart.name)}</b> &nbsp; grams per click <input id="cg" type="number" min="0" step="1" value="${curGrams}"> `+
  (curPart.total_g?`<button id="ball" title="one point carrying the whole line (${curPart.qty} x ${curPart.unit_g} g)">all ${(curPart.total_g/1000).toFixed(2)} kg</button> `:'')+
  `<button id="cunit">unit</button> <span style="color:var(--mute)">${n} placed of ${curPart.qty==null?'?':curPart.qty}; click the model</span>`;
 curEl.querySelector('#cg').oninput=e=>{curGrams=+e.target.value||0;};
 const b=curEl.querySelector('#ball'); if(b) b.onclick=()=>{curGrams=curPart.total_g; curEl.querySelector('#cg').value=curGrams;};
 curEl.querySelector('#cunit').onclick=()=>{curGrams=curPart.unit_g; curEl.querySelector('#cg').value=curGrams;};
}
document.getElementById('search').oninput=renderPalette;
async function loadBom(){bom=await (await fetch('/bom')).json(); if(bom.error) msg.textContent='BOM: '+bom.error; renderPalette();}
const ray=new THREE.Raycaster(); const mouse=new THREE.Vector2(); let downPos=null;

function resize(){const w=view.clientWidth,h=view.clientHeight; renderer.setSize(w,h); labels.setSize(w,h); camera.aspect=w/h; camera.updateProjectionMatrix();}
window.addEventListener('resize',resize); new ResizeObserver(resize).observe(view);
function animate(){requestAnimationFrame(animate); controls.update(); renderer.render(scene,camera); labels.render(scene,camera);}

async function loadMesh(){
 const m=await (await fetch('/mesh')).json(); const box=new THREE.Box3(); const bodiesEl=document.getElementById('bodies');
 for(const b of m.bodies){
  const g=new THREE.BufferGeometry(); g.setAttribute('position',new THREE.Float32BufferAttribute(b.vertices,3)); g.setIndex(b.faces); g.computeVertexNormals();
  const mat=new THREE.MeshStandardMaterial({color:b.colour,side:THREE.DoubleSide,roughness:.75,metalness:.05,transparent:true,opacity:.92});
  const mesh=new THREE.Mesh(g,mat); mesh.name=b.name; scene.add(mesh); meshes.push(mesh); g.computeBoundingBox(); box.union(g.boundingBox);
  const lab=document.createElement('label'); lab.className='t'; lab.innerHTML=`<input type="checkbox" checked><span class="sw" style="background:${b.colour}"></span>${b.name}`;
  lab.querySelector('input').onchange=e=>{mesh.visible=e.target.checked}; const row=document.createElement('div'); row.className='row'; row.appendChild(lab); bodiesEl.appendChild(row);
 }
 box.getCenter(centre); const size=box.getSize(new THREE.Vector3()).length();
 const axes=new THREE.AxesHelper(0.5); axes.position.copy(new THREE.Vector3(box.max.x+0.3,box.max.y,box.min.z)); scene.add(axes);
 const grid=new THREE.GridHelper(6,24,0x2a313b,0x222830); grid.rotation.x=Math.PI/2; grid.position.set(centre.x,centre.y,box.min.z-0.05); scene.add(grid);
 fitView(size); document.getElementById('coord').textContent='CAD frame: x right, y aft (nose is -y), z up (m)';
}
let viewSize=4.5;
function fitView(size){size=size||viewSize; viewSize=size; setView('iso');}
function setView(v){const d=viewSize; const c=centre; const o={iso:[.55,-.75,.45],top:[0,-0.001,1.1],front:[0,-1.2,0.001],side:[1.2,0,0.001],below:[.55,-.75,-.45]}[v]||[.55,-.75,.45];
 camera.position.set(c.x+d*o[0],c.y+d*o[1],c.z+d*o[2]); controls.target.copy(c); controls.update();}
document.getElementById('fit').onclick=()=>fitView();
document.querySelectorAll('#views button').forEach(b=>b.onclick=()=>setView(b.dataset.v));
renderer.domElement.addEventListener('dblclick',e=>{clearTimeout(addTimer); const h=pick(e); if(!h) return; const d=camera.position.clone().sub(controls.target); controls.target.copy(h.point); camera.position.copy(h.point).add(d); controls.update();});

function pick(ev){
 const r=renderer.domElement.getBoundingClientRect(); mouse.x=((ev.clientX-r.left)/r.width)*2-1; mouse.y=-((ev.clientY-r.top)/r.height)*2+1;
 ray.setFromCamera(mouse,camera); const hits=ray.intersectObjects(meshes.filter(m=>m.visible),false); return hits[0]||null;
}
renderer.domElement.addEventListener('pointerdown',e=>{downPos=[e.clientX,e.clientY,e.button]});
renderer.domElement.addEventListener('pointermove',e=>{const h=pick(e); const c=document.getElementById('coord');
 c.textContent=h?`${h.object.name}  x ${h.point.x.toFixed(3)}  y ${h.point.y.toFixed(3)}  z ${h.point.z.toFixed(3)} m (CAD)`:'CAD frame: x right, y aft (nose is -y), z up (m)';});
let addTimer=null;
renderer.domElement.addEventListener('pointerup',e=>{
 if(!downPos||downPos[2]!==0) return; const moved=Math.hypot(e.clientX-downPos[0],e.clientY-downPos[1]); downPos=null; if(moved>4) return;
 const h=pick(e); if(!h) return; const p=[+h.point.x.toFixed(4),+h.point.y.toFixed(4),+h.point.z.toFixed(4)];
 clearTimeout(addTimer); addTimer=setTimeout(()=>addPoint(h,p),260);
});
function addPoint(h,p){
 if(moving>=0){items[moving].pos_cad_m=p; items[moving].picked_on_body=h.object.name; items[moving].timestamp=new Date().toISOString(); moving=-1; dirty=true; render(); return;}
 const it={name:'',grams:0,pos_cad_m:p,picked_on_body:h.object.name,timestamp:new Date().toISOString()};
 if(curPart){const n=placedCount(curPart); it.bom_row=curPart.row; it.bom_name=curPart.name; it.subsystem=curPart.subsystem; it.grams=curGrams;
  it.name=shortName(curPart.name)+' #'+(n+1);}
 items.push(it); sel=items.length-1;
 if(document.getElementById('mirror').checked && Math.abs(p[0])>0.005){const m={...it,pos_cad_m:[-p[0],p[1],p[2]]}; if(curPart) m.name=shortName(curPart.name)+' #'+(placedCount(curPart)+1); else m.name=''; items.push(m);}
 dirty=true; render(); renderPalette(); renderCur(); if(!curPart){const inp=listEl.querySelector('.item.sel input.nm'); if(inp) inp.focus();}
}
window.addEventListener('keydown',e=>{if(e.key==='Escape'){moving=-1;render();}});

function renderMarkers(){
 pointsGroup.traverse(o=>{if(o.isCSS2DObject) o.element.remove();}); pointsGroup.clear(); let tot=0, cx=0,cy=0,cz=0;
 items.forEach((it,i)=>{
  const g=+it.grams||0; tot+=g; cx+=g*it.pos_cad_m[0]; cy+=g*it.pos_cad_m[1]; cz+=g*it.pos_cad_m[2];
  const rad=0.02+0.03*Math.cbrt(Math.max(g,1)/1000);
  const s=new THREE.Mesh(new THREE.SphereGeometry(rad,20,14),new THREE.MeshStandardMaterial({color:i===sel?0xffffff:(i===moving?0xe0a53a:0xe85d4a),emissive:i===sel?0x4f8fdc:0x000000}));
  s.position.set(...it.pos_cad_m); pointsGroup.add(s);
  const d=document.createElement('div'); d.className='lbl'; d.textContent=`${i+1} ${it.name||'?'} ${g} g`; const l=new CSS2DObject(d); l.position.set(0,0,rad*1.8); s.add(l);
 });
 if(tot>0){const cg=new THREE.Mesh(new THREE.OctahedronGeometry(0.06),new THREE.MeshBasicMaterial({color:0xe0a53a,wireframe:true})); cg.position.set(cx/tot,cy/tot,cz/tot); pointsGroup.add(cg);
  const d=document.createElement('div'); d.className='lbl cg'; d.textContent='CG'; const l=new CSS2DObject(d); l.position.set(0,0,0.1); cg.add(l);}
 refreshTotals();
 document.getElementById('hint').textContent=moving>=0?`moving point ${moving+1}: click its new spot (Esc cancels)`:'click a surface: add a mass point. drag: orbit. right drag: pan. wheel: zoom.';
}
function render(){
 renderMarkers(); listEl.innerHTML='';
 items.forEach((it,i)=>{
  const row=document.createElement('div'); row.className='item'+(i===sel?' sel':''); if(it.bom_name) row.title=it.bom_name;
  row.innerHTML=`<span class="n">${i+1}</span><input class="nm" placeholder="name" value="${esc(it.name)}"><input class="g" type="number" min="0" step="1" value="${it.grams}"><button class="mv${i===moving?' on':''}" title="move: click a new spot on the model">&#8982;</button><button class="del" title="delete">&times;</button>`;
  row.onclick=()=>{if(sel!==i){listEl.querySelector('.item.sel')?.classList.remove('sel');sel=i;row.classList.add('sel');renderMarkers();}};
  row.querySelector('.nm').oninput=e=>{it.name=e.target.value;dirty=true;renderMarkers();}
  row.querySelector('.g').oninput=e=>{it.grams=+e.target.value||0;dirty=true;renderMarkers();}
  row.querySelector('.mv').onclick=e=>{e.stopPropagation();moving=(moving===i?-1:i);render();}
  row.querySelector('.del').onclick=e=>{e.stopPropagation();items.splice(i,1);if(sel>=items.length)sel=items.length-1;moving=-1;dirty=true;render();renderPalette();renderCur();}
  listEl.appendChild(row);
 });
}
function refreshTotals(){let tot=0,cx=0,cy=0,cz=0; for(const it of items){const g=+it.grams||0;tot+=g;cx+=g*it.pos_cad_m[0];cy+=g*it.pos_cad_m[1];cz+=g*it.pos_cad_m[2];}
 const t=document.getElementById('tot'); t.innerHTML=`${items.length} pts, <b>${(tot/1000).toFixed(3)} kg</b>`+(tot>0?`, CG cad ${(cx/tot).toFixed(3)}, ${(cy/tot).toFixed(3)}, ${(cz/tot).toFixed(3)}`:'')+(dirty?' <span style="color:var(--warn)">unsaved</span>':'');}
function shortName(n){return n.split(',')[0].split(' (')[0].trim().slice(0,40);}
function esc(s){return String(s||'').replace(/&/g,'&amp;').replace(/"/g,'&quot;').replace(/</g,'&lt;');}

async function load(){const d=await (await fetch('/masses')).json(); items=d.items||[]; sel=-1; moving=-1; dirty=false; render(); renderPalette(); renderCur(); msg.textContent=d.updated?`loaded ${items.length} points, saved ${d.updated}`:'no masses.json yet';}
document.getElementById('reload').onclick=load;
document.getElementById('save').onclick=async()=>{
 const empty=items.filter(i=>!i.name.trim()||!(+i.grams>0)).length;
 const r=await fetch('/masses',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({items})});
 const d=await r.json(); items=d.items; dirty=false; render(); msg.textContent=`saved ${items.length} points (${(items.reduce((a,i)=>a+i.grams,0)/1000).toFixed(3)} kg) at ${d.updated}`+(empty?`; ${empty} point(s) still unnamed or 0 g`:'');
};
window.addEventListener('beforeunload',e=>{if(dirty){e.preventDefault();e.returnValue='';}});
resize(); animate(); loadBom(); loadMesh().then(load).catch(e=>{msg.textContent='error: '+e; document.getElementById('coord').textContent='failed to load geometry: '+e;});
</script></body></html>
"""


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--npz", default=str(CAND_DIR / "meshes_mm.npz"), help="meshes from the Blender working copy (CAD frame, mm)")
    p.add_argument("--out", default=str(CAND_DIR / "masses.json"))
    p.add_argument("--http", type=int, default=8099)
    p.add_argument("--bom", default=str(BOM_DEFAULT), help="sourcing BOM .xlsx (sheet BOM); re-read on every page load")
    args = p.parse_args()
    mesh_json = json.dumps(load_meshes(args.npz)).encode()
    bom0 = load_bom(args.bom)
    print(f"BOM {args.bom}: {len(bom0['parts'])} airborne lines" + (f" ({bom0['error']})" if bom0.get("error") else ""))
    (Path(args.out).parent / "bom_airborne.json").write_text(json.dumps(bom0, indent=1))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype="application/json", code=200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(PAGE.encode(), "text/html; charset=utf-8")
            elif self.path == "/mesh":
                self._send(mesh_json)
            elif self.path == "/masses":
                with lock:
                    self._send(json.dumps(read_doc(args)).encode())
            elif self.path == "/bom":
                self._send(json.dumps(load_bom(args.bom)).encode())
            else:
                self._send(b"not found", "text/plain", 404)

        def do_POST(self):
            if self.path != "/masses":
                return self._send(b"not found", "text/plain", 404)
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
            with lock:
                doc = write_doc(args, body.get("items", []))
            self._send(json.dumps(doc).encode())

    print(f"mass tool on http://127.0.0.1:{args.http}  geometry {args.npz}  out {args.out}")
    ThreadingHTTPServer(("127.0.0.1", args.http), Handler).serve_forever()


if __name__ == "__main__":
    main()
