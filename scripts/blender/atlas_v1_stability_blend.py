"""ATLAS v1 stability review scene for Blender: the unchanged source skins (locked), the user's M001 mass points, the
M002 additions (flight packs and BMS as true-size boxes), the CG of each mass set, the neutral point from the Cruise
Test, the elevon strips and the differential-thrust engines of the lateral controller. CAD axes (x right, y aft
with the NOSE at -Y, z up), Blender metres = CAD mm / 1000, CAD origin kept.

  /Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
      --python scripts/blender/atlas_v1_stability_blend.py -- --out results/atlas_v1/m002 \
      --masses M001=<masses.json> --masses M002=<masses_m002.json> --airframe M001=airframes/atlas_v1_m001.json \
      --airframe M002=airframes/atlas_v1_m002.json --cruise <cruise json> [--render]
"""
import argparse, json, math, sys
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True); ap.add_argument("--npz", default="results/handoff/T20261005-atlas-v1-foil-references/from_A/v002/meshes_mm.npz")
ap.add_argument("--masses", action="append", default=[]); ap.add_argument("--airframe", action="append", default=[])
ap.add_argument("--cruise", default=None); ap.add_argument("--render", action="store_true"); ap.add_argument("--name", default="atlas_v1_m002_stability")
a = ap.parse_args(argv)
OUT = Path(a.out).resolve(); OUT.mkdir(parents=True, exist_ok=True)
MM = 0.001
R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float); Q = np.array([0.0, 0.0, 5.0])      # FRD = R (CAD - Q); R is its own inverse


def cad(frd):
    return (R @ np.asarray(frd, float) + Q).tolist()


bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"; scene.unit_settings.length_unit = "METERS"
scene["frame"] = "CAD axes: +X right, +Y aft (nose at -Y), +Z up; metres = CAD mm / 1000; CAD origin kept (not the CG)"


def collection(name, locked=False, hide=False):
    c = bpy.data.collections.new(name); scene.collection.children.link(c); c.hide_select = locked
    if hide:
        c.hide_viewport = True; c.hide_render = True
    return c


def material(name, rgba):
    m = bpy.data.materials.new(name); m.diffuse_color = rgba; m.blend_method = "BLEND" if rgba[3] < 1 else "OPAQUE"
    return m


def add_mesh(coll, name, v, f, rgba):
    me = bpy.data.meshes.new(name); me.from_pydata(v.tolist(), [], f.tolist()); me.validate()
    ob = bpy.data.objects.new(name, me); ob.color = rgba; ob.data.materials.append(material(name, rgba)); coll.objects.link(ob)
    return ob


def sphere(coll, name, pos, radius, rgba):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=pos, segments=24, ring_count=12)
    ob = bpy.context.active_object; ob.name = name; ob.color = rgba; ob.data.materials.append(material(name, rgba))
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    coll.objects.link(ob)
    return ob


def box(coll, name, pos, dims, rgba):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=pos)
    ob = bpy.context.active_object; ob.name = name; ob.scale = dims; ob.color = rgba; ob.data.materials.append(material(name, rgba))
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    coll.objects.link(ob)
    return ob


def label(coll, text, pos, size=0.12, rgba=(1, 1, 1, 1)):
    cu = bpy.data.curves.new(text[:40], "FONT"); cu.body = text; cu.size = size; cu.align_x = "CENTER"
    ob = bpy.data.objects.new("label " + text[:40], cu); ob.location = pos; ob.color = rgba
    ob.rotation_euler = (math.radians(90), 0, 0)          # readable from the front (looking along +y)
    ob.data.materials.append(material("label " + text[:20], rgba)); coll.objects.link(ob)
    return ob


def arrow(coll, name, start, direction, length, rgba, radius=0.02):
    d = Vector(direction).normalized()
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=length, location=(Vector(start) + d * length / 2))
    ob = bpy.context.active_object; ob.name = name; ob.rotation_mode = "QUATERNION"; ob.rotation_quaternion = d.to_track_quat("Z", "Y")
    ob.color = rgba; ob.data.materials.append(material(name, rgba))
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    coll.objects.link(ob)
    bpy.ops.mesh.primitive_cone_add(radius1=radius * 3, depth=radius * 6, location=(Vector(start) + d * (length + radius * 3)))
    tip = bpy.context.active_object; tip.name = name + " tip"; tip.rotation_mode = "QUATERNION"; tip.rotation_quaternion = d.to_track_quat("Z", "Y")
    tip.color = rgba; tip.data.materials.append(material(name + " tip", rgba))
    for c in list(tip.users_collection):
        c.objects.unlink(tip)
    coll.objects.link(tip)


# 1. source skins, unchanged and locked
data = np.load(a.npz)
src = collection("SOURCE atlas_v1.stp (unchanged, locked)", locked=True)
SKIN = {"source_canopy": (0.95, 0.75, 0.2, 0.45), "source_upper_skin": (0.35, 0.55, 0.95, 0.35), "source_lower_skin": (0.55, 0.4, 0.85, 0.35)}
for key, rgba in SKIN.items():
    add_mesh(src, key, data[f"{key}__v"] * MM, data[f"{key}__f"], rgba)

# 2. mass points per set (M001 = the user's clicks; M002 adds packs and BMS)
GROUP = {"fan": (0.2, 0.5, 1.0, 1), "ESC": (0.6, 0.6, 0.6, 1), "battery": (0.2, 0.85, 0.3, 1), "BMS": (0.9, 0.6, 0.1, 1), "other": (0.95, 0.9, 0.3, 1)}
seen = set()
sets = {}
for spec in a.masses:
    tag, path = spec.split("=", 1)
    doc = json.loads(Path(path).read_text()); sets[tag] = doc
    coll = collection(f"MASSES {tag} ({len(doc['items'])} points, {sum(i['grams'] for i in doc['items']) / 1000:.1f} kg)")
    for it in doc["items"]:
        key = (it["name"], tuple(it["pos_cad_m"]))
        if key in seen:
            continue
        seen.add(key)
        n = it["name"]
        g = "fan" if "fan" in n.lower() else "ESC" if "ESC" in n else "BMS" if "BMS" in n else "battery" if "batter" in n.lower() else "other"
        if it.get("dims_m"):
            box(coll, f"{tag} {n}", it["pos_cad_m"], it["dims_m"], GROUP[g])
        else:
            sphere(coll, f"{tag} {n}", it["pos_cad_m"], 0.02 + 0.03 * (it["grams"] / 4300) ** (1 / 3), GROUP[g])

# 3. CG per airframe, neutral point, elevons, controller engines
mark = collection("CG and neutral point")
ctl = collection("CONTROLS (elevons, differential thrust)")
CGC = {"M001": (1.0, 0.15, 0.15, 1), "M002": (0.1, 0.9, 0.2, 1)}
afs = {}
for spec in a.airframe:
    tag, path = spec.split("=", 1)
    af = json.loads(Path(path).read_text()); afs[tag] = af
    c = cad(af["mass"]["cg"])
    sphere(mark, f"CG {tag} ({af['mass']['mass']:.1f} kg)", c, 0.07, CGC.get(tag, (1, 1, 1, 1)))
    label(mark, f"CG {tag}: {af['mass']['mass']:.0f} kg, {c[1] + 3.24:.2f} m behind the nose tip", (c[0], c[1], c[2] + 0.45 + 0.22 * len(afs)), rgba=CGC.get(tag, (1, 1, 1, 1)))
last = sorted(afs)[-1] if afs else None
if last:
    af = afs[last]
    for w in af["wings"]:
        if not w.get("elevon"):
            continue
        s = w["side"]; sw, dh = math.radians(w["sweep_deg"]), math.radians(w["dihedral_deg"])
        le_r = np.array(w["pos"], float)
        le_t = le_r + np.array([-w["span"] * math.tan(sw), s * w["span"] * math.cos(dh), -w["span"] * math.sin(dh)])
        cf = w["elevon"]["chord_fraction"]
        te_r, te_t = le_r - [w["root_chord"], 0, 0], le_t - [w["tip_chord"], 0, 0]
        hr, ht = te_r + [cf * w["root_chord"], 0, 0], te_t + [cf * w["tip_chord"], 0, 0]
        v = np.array([cad(p) for p in (hr, ht, te_t, te_r)])
        add_mesh(ctl, f"elevon {w['name']} ({cf * 100:.0f} % chord, +-{w['elevon']['max_deg']} deg)", v, np.array([[0, 1, 2, 3]]), (1.0, 0.3, 0.1, 0.9))
    for r in af["rotors"]:
        if abs(r["pos"][1]) > 0.3:       # wing fans: the heading hold splits the forward thrust between the two sides
            p = cad(r["pos"]); arrow(ctl, f"thrust {r['name']} (differential for heading)", p, (0, -1, 0), 0.35, (1.0, 0.5, 0.0, 1))
    # split drag rudders at the outer elevons' mid-span trailing edge (Northrop US2412646A), drawn as an opened clamshell
    for w in af["wings"]:
        if not w.get("elevon") or abs(w["pos"][1]) < 1.0:
            continue
        s_ = w["side"]; sw, dh = math.radians(w["sweep_deg"]), math.radians(w["dihedral_deg"])
        le = np.array(w["pos"], float) + 0.5 * np.array([-w["span"] * math.tan(sw), s_ * w["span"] * math.cos(dh), -w["span"] * math.sin(dh)])
        te = le - [0.5 * (w["root_chord"] + w["tip_chord"]), 0, 0]
        c = cad(te)
        for k, dz in enumerate((0.06, -0.06)):
            box(ctl, f"drag rudder {'LR'[s_ > 0]} {'upper' if k == 0 else 'lower'} (opens differentially for heading)", (c[0], c[1] + 0.08, c[2] + dz), (0.5, 0.16, 0.012), (1.0, 0.1, 0.1, 1))
        label(ctl, "split drag rudder", (c[0], c[1] + 0.25, c[2] + 0.25), size=0.1, rgba=(1.0, 0.3, 0.3, 1))
if a.cruise:
    cr = json.loads(Path(a.cruise).read_text()); sw = cr["sweep"]; tr = sw.get("trim") or {}
    npx = sw.get("neutral_point_x")
    if npx is not None:
        c = cad([npx, 0.0, cr["sweep"]["cg"][2]])
        sphere(mark, f"neutral point ({cr['speed_kmh']:.0f} km/h)", c, 0.06, (0.9, 0.2, 0.9, 1))
        label(mark, f"neutral point, static margin {100 * (sw.get('static_margin') or 0):.0f} % at {cr['speed_kmh']:.0f} km/h", (c[0], c[1], c[2] + 0.3), rgba=(0.9, 0.2, 0.9, 1))
    scene["cruise_summary"] = (f"{cr['speed_kmh']:.0f} km/h: alpha {tr.get('alpha_deg', 0):.1f} deg, elevon {tr.get('elevon_deg', 0):+.1f} deg, "
                               f"L/D {tr.get('L_over_D') or 0:.1f}, SM {100 * (sw.get('static_margin') or 0):.0f} %; six axes: {(cr.get('flight6') or {}).get('verdict', {}).get('label', '')}")

# 4. legend
leg = collection("LEGEND")
y0 = 1.2
for k, (g, rgba) in enumerate(GROUP.items()):
    sphere(leg, f"legend {g}", (-2.6, y0, 5.6 - 0.18 * k), 0.05, rgba); label(leg, g, (-2.3, y0, 5.57 - 0.18 * k), size=0.1, rgba=rgba)
label(leg, "red: CG M001 (46 kg, user's parts). green: CG M002 (+18 packs, +9 BMS). magenta: neutral point", (0, 1.4, 6.1), size=0.12)
label(leg, "orange strips: elevons (roll hold + pitch trim). orange arrows: wing fans, differential thrust for heading", (0, 1.4, 5.92), size=0.12)
label(leg, "red clamshells: split drag rudders at the tips (Northrop US2412646A) for heading, no fins", (0, 1.4, 5.74), size=0.12, rgba=(1.0, 0.5, 0.5, 1))
label(leg, "skins unchanged (locked): no fins, no shape change; moving parts only in the outer trailing edges", (0, 1.4, 5.56), size=0.12, rgba=(0.6, 1.0, 0.6, 1))

# 5. camera, shading, renders
v = data["source_lower_skin__v"] * MM
mins, maxs = v.min(0), v.max(0); centre = (mins + maxs) / 2
cam = bpy.data.objects.new("review_camera", bpy.data.cameras.new("review_camera")); scene.collection.objects.link(cam)
cam.data.type = "ORTHO"; cam.data.ortho_scale = 1.5 * float(max(maxs - mins)); cam.data.clip_end = 100.0; cam.rotation_mode = "QUATERNION"


def aim(offset):
    cam.location = (centre + np.array(offset) * 10.0).tolist()
    cam.rotation_quaternion = Vector((-np.array(offset)).tolist()).to_track_quat("-Z", "Y")


VIEWS = {"above_front_left": (-0.55, -0.6, 0.55), "front": (0.0, -1.0, 0.0001), "top": (0.0, 0.0001, 1.0), "below_front_left": (-0.55, -0.6, -0.55)}
aim(VIEWS["above_front_left"]); scene.camera = cam
scene.render.engine = "BLENDER_WORKBENCH"; scene.display.shading.color_type = "OBJECT"; scene.display.shading.light = "STUDIO"
scene.display.shading.show_xray = True; scene.display.shading.xray_alpha = 0.6
scene.render.resolution_x, scene.render.resolution_y = 1600, 1000
blend = OUT / f"{a.name}.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(blend)); print("saved", blend)
if a.render:
    for view, off in VIEWS.items():
        aim(off); scene.render.filepath = str(OUT / f"blender_{view}.png"); bpy.ops.render.render(write_still=True); print("rendered", scene.render.filepath)
