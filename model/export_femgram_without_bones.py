import bpy
import json
import sys

src, dst = sys.argv[-2], sys.argv[-1]
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=src, use_anim=False)

meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not meshes:
    raise RuntimeError("No mesh found in source FBX")

for obj in list(bpy.context.scene.objects):
    if obj.type == "ARMATURE":
        bpy.data.objects.remove(obj, do_unlink=True)

for obj in meshes:
    for modifier in list(obj.modifiers):
        if modifier.type == "ARMATURE":
            obj.modifiers.remove(modifier)
    obj.vertex_groups.clear()

vertex_count = sum(len(obj.data.vertices) for obj in meshes)
if vertex_count != 75000:
    raise RuntimeError(f"Expected 75000 vertices, got {vertex_count}")

bpy.ops.object.select_all(action="DESELECT")
for obj in meshes:
    obj.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]

bpy.ops.export_scene.fbx(
    filepath=dst,
    use_selection=True,
    object_types={"MESH"},
    apply_unit_scale=True,
    bake_space_transform=False,
    path_mode="COPY",
    embed_textures=True,
    add_leaf_bones=False,
    bake_anim=False
)
print(json.dumps({
    "mesh_objects": len(meshes),
    "vertices": vertex_count,
    "armatures": sum(1 for obj in bpy.context.scene.objects if obj.type == "ARMATURE"),
    "vertex_groups": sum(len(obj.vertex_groups) for obj in meshes)
}))
