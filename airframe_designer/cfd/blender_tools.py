"""Scripts run *inside* Blender (headless) by cfd.geometry: never import this from the app itself.

    Blender --background [FILE.blend] --python blender_tools.py -- <command> <args...>

Commands
  export  OUT.stl [--scale S] [--match SUBSTR,...]      every mesh object of the opened .blend (or those whose name
                                                          contains one of the substrings) -> one STL in world
                                                          coordinates, multiplied by S (default 1.0)
  remesh  IN.stl OUT.stl --voxel V [--in-scale S] [--target-tris N] [--smooth K]
                                                          voxel remesh (closes gaps, gives one watertight skin),
                                                          optional Laplacian smoothing (K passes) and decimation to
                                                          about N triangles; input multiplied by S first
"""
import sys


def _args():
    argv = sys.argv
    return argv[argv.index("--") + 1:] if "--" in argv else []


def _kv(args):
    pos, opts = [], {}
    i = 0
    while i < len(args):
        a = args[i]
        if a.startswith("--"):
            opts[a[2:]] = args[i + 1]
            i += 2
        else:
            pos.append(a)
            i += 1
    return pos, opts


def main():
    import bpy
    args = _args()
    cmd, rest = args[0], args[1:]
    pos, opts = _kv(rest)
    if cmd == "export":
        out = pos[0]
        scale = float(opts.get("scale", 1.0))
        match = [m for m in opts.get("match", "").split(",") if m]
        objs = [o for o in bpy.data.objects if o.type == "MESH" and (not match or any(m.lower() in o.name.lower() for m in match))]
        if not objs:
            print("BLENDER_TOOLS_ERROR no mesh objects" + (f" matching {match}" if match else ""))
            sys.exit(2)
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.hide_set(False)
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.wm.stl_export(filepath=out, export_selected_objects=True, global_scale=scale, apply_modifiers=True,
                              ascii_format=False)
        print("BLENDER_TOOLS_OK exported", len(objs), "objects:", ", ".join(o.name for o in objs))
        return
    if cmd == "remesh":
        src, out = pos[0], pos[1]
        voxel = float(opts["voxel"])
        in_scale = float(opts.get("in-scale", 1.0))
        target = int(opts.get("target-tris", 0))
        smooth = int(opts.get("smooth", 0))
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.ops.wm.stl_import(filepath=src, global_scale=in_scale)
        objs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        if len(objs) > 1:
            bpy.ops.object.join()
        ob = bpy.context.view_layer.objects.active
        # the import scale lands on the object transform: bake it into the vertices, the voxel size is in mesh units
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        ob.data.remesh_voxel_size = voxel
        ob.data.remesh_voxel_adaptivity = 0.0
        ob.data.use_remesh_fix_poles = True
        ob.data.use_remesh_preserve_volume = True
        bpy.ops.object.voxel_remesh()
        n0 = len(ob.data.polygons)
        if smooth > 0:
            m = ob.modifiers.new("smooth", "SMOOTH")
            m.factor = 0.5
            m.iterations = smooth
            bpy.ops.object.modifier_apply(modifier=m.name)
        if target > 0:
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.mesh.select_all(action="SELECT")
            bpy.ops.mesh.quads_convert_to_tris(quad_method="BEAUTY", ngon_method="BEAUTY")
            bpy.ops.object.mode_set(mode="OBJECT")
            ntri = len(ob.data.polygons)
            if ntri > target:
                d = ob.modifiers.new("dec", "DECIMATE")
                d.ratio = target / ntri
                d.use_collapse_triangulate = True
                bpy.ops.object.modifier_apply(modifier=d.name)
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.ops.wm.stl_export(filepath=out, export_selected_objects=True, apply_modifiers=True, ascii_format=False)
        print(f"BLENDER_TOOLS_OK remeshed voxel={voxel} faces_after_remesh={n0} faces_out={len(ob.data.polygons)}")
        return
    print("BLENDER_TOOLS_ERROR unknown command", cmd)
    sys.exit(2)


if __name__ == "__main__":
    main()
