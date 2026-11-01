import bpy, os, sys, json
# Blender 4.4.3: import FBX, perform light collapse decimation, export FBX.
args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if len(args) != 3:
    raise SystemExit("Usage: -- input.fbx output.fbx target_vertices")
src, dst, target_s = args
target = int(target_s)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=src)
meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not meshes: raise RuntimeError("No mesh objects found")
before_v = sum(len(o.data.vertices) for o in meshes)
before_f = sum(len(o.data.polygons) for o in meshes)
if before_v > target:
    ratio = target / before_v
    for o in meshes:
        if len(o.data.vertices) < 100: continue
        m = o.modifiers.new("Femgram_Light_Decimation", "DECIMATE")
        m.decimate_type = "COLLAPSE"
        m.ratio = ratio
        bpy.context.view_layer.objects.active = o
        o.select_set(True)
        bpy.ops.object.modifier_apply(modifier=m.name)
        o.select_set(False)
after_v = sum(len(o.data.vertices) for o in meshes)
after_f = sum(len(o.data.polygons) for o in meshes)
bpy.ops.object.select_all(action="DESELECT")
for o in meshes: o.select_set(True)
bpy.ops.export_scene.fbx(filepath=dst, use_selection=True, object_types={"MESH","ARMATURE","EMPTY"}, use_mesh_modifiers=True, mesh_smooth_type="FACE", use_tspace=True, path_mode="COPY", embed_textures=True, add_leaf_bones=False)
report = {"input":os.path.basename(src),"output":os.path.basename(dst),"target_vertices":target,"vertices_before":before_v,"vertices_after":after_v,"faces_before":before_f,"faces_after":after_f,"reduction_percent":round((1-after_v/before_v)*100,2),"method":"Decimate Collapse","warning":"Inspect facial details and texture alignment before replacing the original."}
with open(os.path.splitext(dst)[0]+"-report.json","w") as f: json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
