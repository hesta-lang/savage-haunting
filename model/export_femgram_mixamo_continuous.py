import bpy
import sys
import json
from mathutils import Vector

source_path, output_path = sys.argv[-2], sys.argv[-1]

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=source_path, use_anim=False)

mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]
if not mesh_objects:
    raise RuntimeError("The source FBX contains no mesh.")

bpy.ops.object.select_all(action="DESELECT")
for obj in mesh_objects:
    obj.select_set(True)
bpy.context.view_layer.objects.active = mesh_objects[0]
bpy.ops.object.join()

body = bpy.context.view_layer.objects.active
body.name = "Femgram_Mixamo_Continuous"

bounds = [body.matrix_world @ Vector(corner) for corner in body.bound_box]
height = max(point.z for point in bounds) - min(point.z for point in bounds)
if height <= 0:
    raise RuntimeError("Invalid character dimensions.")

voxel_size = height / 120.0
remesh = body.modifiers.new("Continuous_Body_Surface", "REMESH")
remesh.mode = "VOXEL"
remesh.voxel_size = voxel_size
remesh.use_smooth_shade = True
bpy.ops.object.modifier_apply(modifier=remesh.name)

smooth = body.modifiers.new("Gentle_Surface_Smoothing", "SMOOTH")
smooth.factor = 0.2
smooth.iterations = 2
bpy.ops.object.modifier_apply(modifier=smooth.name)

body.vertex_groups.clear()
for modifier in list(body.modifiers):
    body.modifiers.remove(modifier)

for obj in list(bpy.context.scene.objects):
    if obj != body:
        bpy.data.objects.remove(obj, do_unlink=True)

if len(body.data.vertices) < 1000:
    raise RuntimeError("The rebuilt mesh is unexpectedly small.")

bpy.ops.object.select_all(action="DESELECT")
body.select_set(True)
bpy.context.view_layer.objects.active = body

bpy.ops.export_scene.fbx(
    filepath=output_path,
    use_selection=True,
    object_types={"MESH"},
    apply_unit_scale=True,
    apply_scale_options="FBX_SCALE_ALL",
    path_mode="COPY",
    embed_textures=True,
    add_leaf_bones=False,
    bake_anim=False,
    use_mesh_modifiers=False,
    axis_forward="-Z",
    axis_up="Y"
)

print(json.dumps({
    "mesh_objects": 1,
    "vertices": len(body.data.vertices),
    "polygons": len(body.data.polygons),
    "voxel_size": voxel_size,
    "armatures": 0,
    "vertex_groups": len(body.vertex_groups)
}))
