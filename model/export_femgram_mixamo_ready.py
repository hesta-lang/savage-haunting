import bpy
import json
import sys

src, dst = sys.argv[-2], sys.argv[-1]

# Start from a genuinely empty scene.
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
for datablocks in (bpy.data.meshes, bpy.data.armatures, bpy.data.actions):
    for datablock in list(datablocks):
        if datablock.users == 0:
            datablocks.remove(datablock)

bpy.ops.import_scene.fbx(filepath=src, use_anim=False)

source_meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not source_meshes:
    raise RuntimeError("No mesh found in source FBX")

# Rebuild each mesh as a fresh Blender object/data-block. Preserve geometry,
# polygon material indices, UVs, color attributes, and material assignments.
clean_meshes = []
for index, source in enumerate(source_meshes):
    fresh_data = source.data.copy()
    fresh = bpy.data.objects.new(f"Femgram_Mesh_{index+1}", fresh_data)
    bpy.context.scene.collection.objects.link(fresh)

    fresh.matrix_world = source.matrix_world.copy()
    fresh.parent = None
    fresh.vertex_groups.clear()

    # Remove all modifiers and constraints; no armature or skinning survives.
    for modifier in list(fresh.modifiers):
        fresh.modifiers.remove(modifier)
    for constraint in list(fresh.constraints):
        fresh.constraints.remove(constraint)

    # Preserve material slots from the source mesh.
    for slot in source.material_slots:
        fresh.data.materials.append(slot.material)

    clean_meshes.append(fresh)

# Delete imported objects and all non-mesh scene objects.
for obj in list(bpy.context.scene.objects):
    if obj not in clean_meshes:
        bpy.data.objects.remove(obj, do_unlink=True)

# Drop unused armatures/actions left by the imported FBX.
for datablocks in (bpy.data.armatures, bpy.data.actions):
    for datablock in list(datablocks):
        if datablock.users == 0:
            datablocks.remove(datablock)

vertex_count = sum(len(obj.data.vertices) for obj in clean_meshes)
if vertex_count != 75000:
    raise RuntimeError(f"Expected 75000 vertices, got {vertex_count}")

# Validate there is no deform setup in the clean scene.
armatures = [o for o in bpy.context.scene.objects if o.type == "ARMATURE"]
groups = sum(len(o.vertex_groups) for o in clean_meshes)
if armatures or groups:
    raise RuntimeError(f"Unexpected rig data: armatures={len(armatures)}, vertex_groups={groups}")

bpy.ops.object.select_all(action="DESELECT")
for obj in clean_meshes:
    obj.select_set(True)
bpy.context.view_layer.objects.active = clean_meshes[0]

bpy.ops.export_scene.fbx(
    filepath=dst,
    use_selection=True,
    object_types={"MESH"},
    apply_unit_scale=True,
    apply_scale_options="FBX_SCALE_ALL",
    bake_space_transform=False,
    path_mode="COPY",
    embed_textures=True,
    add_leaf_bones=False,
    bake_anim=False,
    use_mesh_modifiers=False,
    use_triangles=False,
    axis_forward="-Z",
    axis_up="Y",
    version="7400"
)
print(json.dumps({
    "mesh_objects": len(clean_meshes),
    "vertices": vertex_count,
    "armatures": len(armatures),
    "vertex_groups": groups,
    "fresh_mesh_datablocks": True
}))
