import bpy
import json
import os
import sys

def arg_after_separator(name, default):
    argv = sys.argv
    if "--" not in argv:
        return default
    tail = argv[argv.index("--") + 1:]
    for i, value in enumerate(tail[:-1]):
        if value == name:
            return tail[i + 1]
    return default

out_dir = arg_after_separator("--output-dir", "/tmp/femgram-meshy")
os.makedirs(out_dir, exist_ok=True)

scene = bpy.context.scene
objects = []
for obj in bpy.data.objects:
    item = {
        "name": obj.name,
        "type": obj.type,
        "dimensions": [round(float(v), 5) for v in obj.dimensions],
        "location": [round(float(v), 5) for v in obj.location],
        "rotation_euler": [round(float(v), 5) for v in obj.rotation_euler],
        "scale": [round(float(v), 5) for v in obj.scale],
        "parent": obj.parent.name if obj.parent else None,
        "materials": [m.name if m else None for m in getattr(obj.data, "materials", [])],
        "modifiers": [{"name": m.name, "type": m.type} for m in obj.modifiers],
    }
    if obj.type == "MESH":
        item["vertices"] = len(obj.data.vertices)
        item["polygons"] = len(obj.data.polygons)
    if obj.type == "ARMATURE":
        item["bones"] = [b.name for b in obj.data.bones]
    objects.append(item)

report = {
    "blend_file": bpy.data.filepath,
    "blender_version": bpy.app.version_string,
    "scene": scene.name,
    "render_engine": scene.render.engine,
    "camera": scene.camera.name if scene.camera else None,
    "object_count": len(objects),
    "objects": objects,
    "actions": [a.name for a in bpy.data.actions],
    "collections": [c.name for c in bpy.data.collections],
}
report_path = os.path.join(out_dir, "Femgram-Meshy-inspection.json")
with open(report_path, "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)

# Preserve a working copy of the original scene before exporting.
blend_out = os.path.join(out_dir, "Femgram-Meshy.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend_out)

fbx_out = os.path.join(out_dir, "Femgram-Meshy.fbx")
bpy.ops.object.select_all(action="SELECT")
bpy.ops.export_scene.fbx(
    filepath=fbx_out,
    use_selection=True,
    apply_unit_scale=True,
    bake_space_transform=False,
    add_leaf_bones=False,
    use_mesh_modifiers=True,
    mesh_smooth_type="FACE",
    path_mode="AUTO",
)

with open(os.path.join(out_dir, "Femgram-Meshy-summary.txt"), "w", encoding="utf-8") as f:
    f.write("Meshy .blend inspection completed.\n")
    f.write("Objects: %d\n" % len(objects))
    f.write("Scene: %s\n" % scene.name)
    f.write("Camera: %s\n" % (scene.camera.name if scene.camera else "None"))
    f.write("Armatures: %d\n" % sum(1 for o in bpy.data.objects if o.type == "ARMATURE"))
    f.write("Mesh objects: %d\n" % sum(1 for o in bpy.data.objects if o.type == "MESH"))
