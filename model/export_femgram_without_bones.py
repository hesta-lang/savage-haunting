import bpy
import json
import os
import sys

src, dst = sys.argv[-2], sys.argv[-1]
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)

bpy.ops.import_scene.fbx(filepath=src, use_anim=False)
meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not meshes:
    raise RuntimeError("No se encontró ninguna malla en el FBX de entrada")

# Keep every mesh and its geometry; remove only the skeleton/skin binding.
for obj in list(bpy.context.scene.objects):
    if obj.type == "ARMATURE":
        bpy.data.objects.remove(obj, do_unlink=True)
for obj in meshes:
    for mod in list(obj.modifiers):
        if mod.type == "ARMATURE":
            obj.modifiers.remove(mod)
    obj.vertex_groups.clear()
    obj.select_set(True)
    obj.hide_set(False)

bpy.ops.object.select_all(action="DESELECT")
for obj in meshes:
    obj.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]
verts = sum(len(o.data.vertices) for o in meshes)
if verts != 75000:
    raise RuntimeError(f"Se esperaba conservar 75000 vértices; se encontraron {verts}")

bpy.ops.export_scene.fbx(
    filepath=dst, use_selection=True, object_types={"MESH"},
    apply_unit_scale=True, bake_space_transform=False,
    path_mode="COPY", embed_textures=True, add_leaf_bones=False,
    bake_anim=False
)
print(json.dumps({"mesh_objects": len(meshes), "vertices": verts, "armatures": 0, "vertex_groups": sum(len(o.vertex_groups) for o in meshes)}))
