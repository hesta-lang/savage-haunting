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
center = Vector(((lo[0]+hi[0])/2, (lo[1]+hi[1])/2, (lo[2]+hi[2])/2))
size = max(hi[i]-lo[i] for i in range(3))

scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 900
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.film_transparent = False
scene.world.color = (0.055, 0.055, 0.055)
scene.view_settings.view_transform = "Standard"

def aim(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()

cam_data = bpy.data.cameras.new("FemgramValidationCamera")
cam = bpy.data.objects.new("FemgramValidationCamera", cam_data)
scene.collection.objects.link(cam)
cam.location = center + Vector((0, -size*2.8, size*0.08))
aim(cam, center)
cam_data.type = "ORTHO"
cam_data.ortho_scale = size * 1.25
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
area("Key", (size, -size*1.4, size*1.4), 1500, size)
area("Fill", (-size, -size, size*0.5), 900, size*0.8)
area("Rim", (0, size, size), 1200, size)

def render(name):
    scene.render.filepath = os.path.join(out, name + ".png")
    bpy.ops.render.render(write_still=True)

# Neutral pose
bpy.context.view_layer.objects.active = arm
arm.select_set(True)
bpy.ops.object.mode_set(mode="POSE")
for pb in arm.pose.bones:
    pb.rotation_mode = "XYZ"
    pb.rotation_euler = (0, 0, 0)
bpy.ops.object.mode_set(mode="OBJECT")
render("Femgram-neutral")

# Mild arm articulation, then restore
bpy.ops.object.mode_set(mode="POSE")
for side, angle in (("L", -0.35), ("R", 0.35)):
    name = "ValveBiped.Bip01_" + side + "_UpperArm"
    if name in arm.pose.bones:
        arm.pose.bones[name].rotation_euler[1] = angle
bpy.ops.object.mode_set(mode="OBJECT")
render("Femgram-arm-pose")

# Mild leg articulation, then restore
bpy.ops.object.mode_set(mode="POSE")
for side, angle in (("L", 0.20), ("R", -0.20)):
    name = "ValveBiped.Bip01_" + side + "_Thigh"
    if name in arm.pose.bones:
        arm.pose.bones[name].rotation_euler[1] = angle
bpy.ops.object.mode_set(mode="OBJECT")
render("Femgram-leg-pose")

report = {
    "mesh_objects": len(meshes),
    "armature_objects": len(arms),
    "vertices": len(verts),
    "weighted_vertices": weighted,
    "unweighted_vertices": len(verts)-weighted,
    "bones": len(bone_names),
    "missing_expected_bones": missing,
    "armature_modifier_linked": bool(arm_mods),
    "vertex_groups": len(mesh.vertex_groups),
    "renders": ["Femgram-neutral.png", "Femgram-arm-pose.png", "Femgram-leg-pose.png"],
    "checks": {
        "mesh_and_armature_present": bool(meshes and arms),
        "all_vertices_weighted": weighted == len(verts),
        "expected_bones_present": not missing,
        "armature_modifier_linked": bool(arm_mods)
    },
    "warning": "Rendered pose checks are visual diagnostics, not proof of final Garry's Mod/Source compatibility. Inspect elbows, shoulders, hips, fingers and any texture/material loss."
}
report["passed_structural_checks"] = all(report["checks"].values())
with open(os.path.join(out, "Femgram-Rig-Validation.json"), "w") as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
if not report["passed_structural_checks"]:
    raise RuntimeError("One or more structural rig validation checks failed")
