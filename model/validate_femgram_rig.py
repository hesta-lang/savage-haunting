import bpy, os, json, math
from mathutils import Vector

src = os.environ["RIGGED_FBX"]
out = os.environ.get("OUTPUT_DIR", "dist/validation")
os.makedirs(out, exist_ok=True)

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=src, automatic_bone_orientation=False)

meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
arms = [o for o in bpy.context.scene.objects if o.type == "ARMATURE"]
if not meshes or not arms:
    raise RuntimeError("FBX must contain at least one mesh and one armature")
mesh = max(meshes, key=lambda o: len(o.data.vertices))
arm = arms[0]
verts = list(mesh.data.vertices)
weighted = sum(bool(v.groups) for v in verts)
bone_names = set(arm.data.bones.keys())
expected = [
    "ValveBiped.Bip01_Pelvis", "ValveBiped.Bip01_Spine",
    "ValveBiped.Bip01_Head", "ValveBiped.Bip01_L_Thigh",
    "ValveBiped.Bip01_R_Thigh", "ValveBiped.Bip01_L_UpperArm",
    "ValveBiped.Bip01_R_UpperArm", "ValveBiped.Bip01_L_Hand",
    "ValveBiped.Bip01_R_Hand"
]
missing = [n for n in expected if n not in bone_names]
arm_mods = [m for m in mesh.modifiers if m.type == "ARMATURE" and m.object == arm]
bounds = [mesh.matrix_world @ Vector(c) for c in mesh.bound_box]
lo = [min(v[i] for v in bounds) for i in range(3)]
hi = [max(v[i] for v in bounds) for i in range(3)]
center = Vector(tuple((lo[i] + hi[i]) / 2 for i in range(3)))
height = hi[2] - lo[2]
width = hi[0] - lo[0]
depth = hi[1] - lo[1]
size = max(height, width, depth)

scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 900
scene.render.resolution_y = 1100
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
scene.render.film_transparent = False
scene.render.resolution_percentage = 100
scene.render.image_settings.compression = 18
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"
scene.view_settings.exposure = -0.7
scene.view_settings.gamma = 1.0
scene.render.image_settings.file_format = "PNG"
scene.render.filepath = os.path.join(out, "Femgram-neutral.png")

# Diagnostic clay material prevents a white/metallic source material from washing out the renders.
clay = bpy.data.materials.new("Femgram_Diagnostic_Clay")
clay.diffuse_color = (0.38, 0.32, 0.42, 1.0)
clay.use_nodes = True
bs = clay.node_tree.nodes.get("Principled BSDF")
bs.inputs["Base Color"].default_value = (0.38, 0.32, 0.42, 1.0)
bs.inputs["Metallic"].default_value = 0.0
bs.inputs["Roughness"].default_value = 0.78
mesh.data.materials.clear()
mesh.data.materials.append(clay)

world = scene.world or bpy.data.worlds.new("FemgramValidationWorld")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
bg.inputs["Color"].default_value = (0.16, 0.17, 0.20, 1.0)
bg.inputs["Strength"].default_value = 0.35
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.eevee.taa_render_samples = 96

def aim(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()

cam_data = bpy.data.cameras.new("FemgramValidationCamera")
cam = bpy.data.objects.new("FemgramValidationCamera", cam_data)
scene.collection.objects.link(cam)
cam.location = center + Vector((0, -size * 3.2, size * 0.06))
aim(cam, center)
cam_data.type = "ORTHO"
cam_data.ortho_scale = height * 1.16
scene.camera = cam

def area(name, loc, power, scale):
    data = bpy.data.lights.new(name, "AREA")
    data.energy = power
    data.shape = "DISK"
    data.size = scale
    obj = bpy.data.objects.new(name, data)
    scene.collection.objects.link(obj)
    obj.location = center + Vector(loc)
    aim(obj, center)

area("Key", (size * 0.9, -size * 1.5, size * 1.0), 650, size * 0.9)
area("Fill", (-size * 1.0, -size * 1.0, size * 0.35), 280, size * 1.0)
area("Rim", (0, size * 1.1, size * 0.8), 420, size * 0.8)

def render(name):
    scene.render.filepath = os.path.join(out, name + ".png")
    bpy.ops.render.render(write_still=True)

def reset_pose():
    for pb in arm.pose.bones:
        pb.rotation_mode = "XYZ"
        pb.rotation_euler = (0.0, 0.0, 0.0)
    bpy.context.view_layer.update()

# Find the local rotation axis that actually moves the skinned mesh most for each joint.
sample = list(range(0, len(verts), max(1, len(verts)//1800)))
def sampled_positions():
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    evaluated = mesh.evaluated_get(deps)
    evmesh = evaluated.to_mesh()
    try:
        return [evaluated.matrix_world @ evmesh.vertices[i].co for i in sample if i < len(evmesh.vertices)]
    finally:
        evaluated.to_mesh_clear()

def choose_axis(bone_name, angle):
    # Preserve earlier joint rotations while testing this joint's local axes.
    saved = {pb.name: pb.rotation_euler.copy() for pb in arm.pose.bones}
    best = (0.0, 0)
    for axis in range(3):
        reset_pose()
        pb = arm.pose.bones.get(bone_name)
        if pb is None:
            continue
        pb.rotation_euler[axis] = angle
        after = sampled_positions()
        count = min(len(neutral_positions), len(after))
        if count:
            rms = math.sqrt(sum((after[i] - neutral_positions[i]).length_squared for i in range(count)) / count)
            if rms > best[0]:
                best = (rms, axis)
    reset_pose()
    for name, rotation in saved.items():
        if name in arm.pose.bones:
            arm.pose.bones[name].rotation_euler = rotation
    bpy.context.view_layer.update()
    return best

reset_pose()
neutral_positions = sampled_positions()
pose_tests = {}
def apply_joint(bone_name, angle):
    if bone_name not in arm.pose.bones:
        pose_tests[bone_name] = {"found": False, "rms_displacement": 0.0}
        return
    rms, axis = choose_axis(bone_name, angle)
    arm.pose.bones[bone_name].rotation_euler[axis] = angle
    pose_tests[bone_name] = {"found": True, "chosen_local_axis": ["X", "Y", "Z"][axis], "rms_displacement": round(rms, 6)}

# Neutral
reset_pose()
render("Femgram-neutral")

# Stronger, multi-joint arm pose: shoulders lift and elbows bend.
reset_pose()
for side, sign in (("L", 1), ("R", -1)):
    apply_joint("ValveBiped.Bip01_" + side + "_UpperArm", sign * 0.95)
    apply_joint("ValveBiped.Bip01_" + side + "_Forearm", -sign * 1.15)
bpy.context.view_layer.update()
arm_pose_positions = sampled_positions()
arm_rms = math.sqrt(sum((arm_pose_positions[i] - neutral_positions[i]).length_squared for i in range(min(len(arm_pose_positions), len(neutral_positions)))) / min(len(arm_pose_positions), len(neutral_positions)))
render("Femgram-arm-pose")

# Restore, then bend both hips and knees.
reset_pose()
for side, sign in (("L", 1), ("R", -1)):
    apply_joint("ValveBiped.Bip01_" + side + "_Thigh", sign * 0.35)
    apply_joint("ValveBiped.Bip01_" + side + "_Calf", -sign * 0.50)
bpy.context.view_layer.update()
leg_pose_positions = sampled_positions()
leg_rms = math.sqrt(sum((leg_pose_positions[i] - neutral_positions[i]).length_squared for i in range(min(len(leg_pose_positions), len(neutral_positions)))) / min(len(leg_pose_positions), len(neutral_positions)))
render("Femgram-leg-pose")

report = {
    "mesh_objects": len(meshes), "armature_objects": len(arms),
    "vertices": len(verts), "weighted_vertices": weighted,
    "unweighted_vertices": len(verts)-weighted, "bones": len(bone_names),
    "missing_expected_bones": missing, "armature_modifier_linked": bool(arm_mods),
    "vertex_groups": len(mesh.vertex_groups),
    "pose_tests": pose_tests,
    "arm_pose_rms_displacement": round(arm_rms, 6),
    "leg_pose_rms_displacement": round(leg_rms, 6),
    "renders": ["Femgram-neutral.png", "Femgram-arm-pose.png", "Femgram-leg-pose.png"],
    "checks": {
        "mesh_and_armature_present": bool(meshes and arms),
        "all_vertices_weighted": weighted == len(verts),
        "expected_bones_present": not missing,
        "armature_modifier_linked": bool(arm_mods),
        "arm_pose_deforms_mesh": arm_rms > 0.002,
        "leg_pose_deforms_mesh": leg_rms > 0.002
    },
    "warning": "Pose checks verify visible mesh movement, not final Source-engine compatibility. Inspect joint deformation and texture/material conversion before release."
}
report["passed_structural_checks"] = all(report["checks"].values())
with open(os.path.join(out, "Femgram-Rig-Validation.json"), "w") as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
if not report["passed_structural_checks"]:
    raise RuntimeError("One or more rig/deformation checks failed")
