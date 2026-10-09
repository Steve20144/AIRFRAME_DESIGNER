"""Blender side of the ATLAS v1 foil candidate (HANDOFF S1/S2): a working .blend with the unchanged source shells,
the lofted candidate and the per-station foil overlays, all in CAD axes (x right, y nose, z up) scaled mm -> m.
Inputs come from scripts/atlas_v1_foil_candidate.py (meshes_mm.npz, overlays_mm.json, fit_metrics.json).

  /Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
      --python scripts/blender/atlas_v1_build_blend.py -- <from_A/version dir> [--render]
"""
import json, sys
from pathlib import Path
import bpy
import numpy as np

argv = sys.argv[sys.argv.index("--") + 1:]
OUT = Path(argv[0]).resolve()
RENDER = "--render" in argv
VERSION = OUT.name
MM = 0.001
COLORS = {"source_canopy": (0.95, 0.75, 0.2, 0.6), "source_upper_skin": (0.35, 0.55, 0.95, 0.35),
          "source_lower_skin": (0.55, 0.4, 0.85, 0.35)}
CANDIDATE_COLOR = (0.85, 0.2, 0.2, 1.0)
REGION_COLORS = {"centerbody": (1.0, 0.3, 0.2, 1), "transition": (0.2, 0.8, 0.3, 1), "outer": (0.2, 0.6, 1.0, 1),
                 "blend": (0.95, 0.9, 0.3, 1)}

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.length_unit = "MILLIMETERS"
metrics = json.loads((OUT / "fit_metrics.json").read_text())
scene["source_file"] = metrics["source"]["file"]
scene["source_sha256"] = metrics["source"]["sha256"]
scene["candidate_version"] = VERSION
scene["candidate_status"] = metrics["status"]
scene["frame"] = "CAD axes: +X right, +Y nose, +Z up; Blender metres = CAD mm / 1000; CAD origin kept (not CG)"


def collection(name: str, locked: bool) -> bpy.types.Collection:
    c = bpy.data.collections.new(name)
    scene.collection.children.link(c)
    c.hide_select = locked
    return c


def add_mesh(coll, name, v, f, color):
    me = bpy.data.meshes.new(name)
    me.from_pydata((v * MM).tolist(), [], f.tolist())
    me.validate()
    ob = bpy.data.objects.new(name, me)
    ob.color = color
    coll.objects.link(ob)
    return ob


data = np.load(OUT / "meshes_mm.npz")
src = collection("SOURCE_atlas_v1_stp (unchanged, locked)", locked=True)
cand = collection(f"CANDIDATE_{VERSION} (review only)", locked=False)
for key in sorted(k.removesuffix("__v") for k in data.files if k.endswith("__v")):
    if key.startswith("source_"):
        add_mesh(src, key, data[f"{key}__v"], data[f"{key}__f"], COLORS[key])
    else:                     # candidate_<part>: v001 lifting_body, v002 upper_skin
        add_mesh(cand, f"cand_{VERSION}_{key.removeprefix('candidate_')}", data[f"{key}__v"], data[f"{key}__f"], CANDIDATE_COLOR)

ov = collection(f"FOIL_OVERLAYS_{VERSION}", locked=False)
for o in json.loads((OUT / "overlays_mm.json").read_text()):
    name = f"foil_{o['region']}_{o['profile'].removesuffix('.dat')}_X{o['cad_x_mm']:+.0f}"
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = 0.002
    sp = cu.splines.new("POLY")
    pts = np.array(o["points_mm"]) * MM
    sp.points.add(len(pts) - 1)
    for p, q in zip(sp.points, pts):
        p.co = (*q, 1.0)
    sp.use_cyclic_u = o.get("closed", True)
    ob = bpy.data.objects.new(name, cu)
    ob.color = REGION_COLORS[o["region"]]
    ob["cad_x_mm"], ob["profile"], ob["region"] = o["cad_x_mm"], o["profile"], o["region"]
    ov.objects.link(ob)

# a camera framing the aircraft from above-front-left, workbench shading by object colour
mins = np.min([data[f"{k}__v"].min(0) for k in ("source_upper_skin", "source_lower_skin")], axis=0) * MM
maxs = np.max([data[f"{k}__v"].max(0) for k in ("source_upper_skin", "source_lower_skin")], axis=0) * MM
centre = (mins + maxs) / 2
cam = bpy.data.objects.new("review_camera", bpy.data.cameras.new("review_camera"))
scene.collection.objects.link(cam)
cam.data.type = "ORTHO"
cam.data.ortho_scale = 1.6 * float(max(maxs - mins))
cam.rotation_mode = "QUATERNION"
from mathutils import Vector


def aim(offset):
    cam.location = (centre + np.array(offset) * 10.0).tolist()
    cam.rotation_quaternion = Vector((-np.array(offset)).tolist()).to_track_quat("-Z", "Y")


VIEWS = {"above_front_left": (-0.55, 0.6, 0.55), "below_front_left": (-0.55, 0.6, -0.55), "top": (0.0, 0.0001, 1.0)}
aim(VIEWS["above_front_left"])
cam.data.clip_end = 100.0
scene.camera = cam
scene.render.engine = "BLENDER_WORKBENCH"
scene.display.shading.color_type = "OBJECT"
scene.display.shading.light = "STUDIO"
scene.render.resolution_x, scene.render.resolution_y = 1600, 1000
scene.render.film_transparent = False

blend = OUT / f"atlas_v1_work_{VERSION}.blend"
bpy.ops.wm.save_as_mainfile(filepath=str(blend))
print("saved", blend)
if RENDER:
    for view, off in VIEWS.items():
        aim(off)
        scene.render.filepath = str(OUT / f"blender_{view}.png")
        bpy.ops.render.render(write_still=True)
        print("rendered", scene.render.filepath)
