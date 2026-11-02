import bpy, os, json, math, runpy
from mathutils import Vector

ROOT = os.environ.get("GITHUB_WORKSPACE", os.getcwd())
SOURCE = os.environ["SOURCE_FBX"]
OUT = os.environ.get("OUTPUT_DIR", "dist/knee-deformation")
os.makedirs(OUT, exist_ok=True)

# Build the exact current production rig from the original 75K source.
os.environ["SOURCE_FBX"] = SOURCE
os.environ["OUTPUT_DIR"] = os.path.join(OUT, "_rig_build")
runpy.run_path(os.path.join(ROOT, "model", "rig_femgram_75k.py"), run_name="__main__")

scene = bpy.context.scene
arm = next((o for o in scene.objects if o.type == "ARMATURE"), None)
mesh = max((o for o in scene.objects if o.type == "MESH"), key=lambda o: len(o.data.vertices))
if arm is None or len(mesh.data.vertices) != 75000:
    raise RuntimeError("Expected the 75K mesh and generated ValveBiped armature.")
for side in ("L", "R"):
    name = "ValveBiped.Bip01_" + side + "_Calf"
    if arm.pose.bones.get(name) is None:
        raise RuntimeError("Missing pose bone: " + name)

scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 1400
scene.render.resolution_y = 1400
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.film_transparent = False
scene.world.color = (0.025, 0.025, 0.025)
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "Medium High Contrast"

# Ensure the armature is evaluated in pose mode and the mesh modifier
# actually consumes the named vertex groups.
arm.data.pose_position = "POSE"
arm_modifiers = [m for m in mesh.modifiers if m.type == "ARMATURE"]
if not arm_modifiers:
    raise RuntimeError("The 75K mesh has no Armature modifier.")
for modifier in arm_modifiers:
    modifier.object = arm
    modifier.use_vertex_groups = True
    modifier.use_bone_envelopes = False
    modifier.show_viewport = True
    modifier.show_render = True

def reset_pose():
    for pb in arm.pose.bones:
        pb.rotation_mode = "XYZ"
        pb.rotation_euler = (0.0, 0.0, 0.0)
        # Rotation limits are animator controls; mute them during this
        # controlled deformation test so they cannot clamp the test pose.
        for constraint in pb.constraints:
            constraint.mute = True
    scene.frame_set(scene.frame_current)
    bpy.context.view_layer.update()

def evaluated_positions():
    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated = mesh.evaluated_get(depsgraph)
    data = evaluated.to_mesh()
    matrix = evaluated.matrix_world.copy()
    coords = [matrix @ v.co for v in data.vertices]
    evaluated.to_mesh_clear()
    return coords

reset_pose()
neutral = evaluated_positions()
if len(neutral) != len(mesh.data.vertices):
    raise RuntimeError("Evaluated mesh vertex count changed unexpectedly.")

# Camera looks across the sagittal plane, centered tightly on the tested knee.
def render_knee(side, angle):
    reset_pose()
    bone_name = "ValveBiped.Bip01_" + side + "_Calf"
    pb = arm.pose.bones[bone_name]
    signed_angle = -angle if side == "L" else angle
    rest_matrix = pb.matrix.copy()
    pb.rotation_euler = (math.radians(signed_angle), 0.0, 0.0)
    scene.frame_set(scene.frame_current)
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    posed_bone_matrix = arm.evaluated_get(depsgraph).pose.bones[bone_name].matrix.copy()
    bone_matrix_delta = max(abs(posed_bone_matrix[i][j] - rest_matrix[i][j]) for i in range(4) for j in range(4))
    if bone_matrix_delta < 1e-6:
        raise RuntimeError("Pose did not change the calf bone matrix; check constraints/rotation axis: " + bone_name)

    arm_eval = arm.evaluated_get(depsgraph)
    calf = arm_eval.data.bones[bone_name]
    knee = arm_eval.matrix_world @ calf.head_local
    ankle = arm_eval.matrix_world @ calf.tail_local
    center = (knee + ankle) * 0.5
    span = max((ankle-knee).length * 2.8, 0.25)

    cam_data = bpy.data.cameras.new("KneeTestCamera")
    cam = bpy.data.objects.new("KneeTestCamera", cam_data)
    scene.collection.objects.link(cam)
    # L is on +X, R on -X. Looking from the outside keeps the tested leg visible.
    cam.location = center + Vector((span * (1 if side == "L" else -1), 0, 0))
    cam.rotation_euler = (center-cam.location).to_track_quat("-Z", "Y").to_euler()
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = span
    scene.camera = cam

    path = os.path.join(OUT, "Femgram-Knee-" + side + "-" + str(angle) + "deg.png")
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam, do_unlink=True)

    posed = evaluated_positions()
    displacements = [(b-a).length for a,b in zip(neutral, posed)]
    finite = all(math.isfinite(c) for p in posed for c in p)
    ordered = sorted(displacements)
    p95 = ordered[min(len(ordered)-1, int(len(ordered)*0.95))]
    moved = sum(d > 1e-5 for d in displacements)
    result = {
        "side": "left" if side == "L" else "right",
        "angle_degrees": angle,
        "signed_local_x_degrees": signed_angle,
        "bone_matrix_delta": round(bone_matrix_delta, 6),
        "finite_coordinates": finite,
        "vertices": len(posed),
        "vertices_moved": moved,
        "max_displacement": round(max(displacements), 6),
        "p95_displacement": round(p95, 6),
        "render": os.path.basename(path)
    }
    bpy.data.objects.remove(cam, do_unlink=True) if bpy.data.objects.get("KneeTestCamera") else None
    if not finite or moved == 0:
        raise RuntimeError("Knee test failed: invalid coordinates or no mesh deformation: " + str(result))
    return result

results = []
for side in ("L", "R"):
    for angle in (45, 90):
        results.append(render_knee(side, angle))

report = {
    "test": "ValveBiped knee flexion deformation",
    "source_vertices": len(mesh.data.vertices),
    "test_angles_degrees": [45, 90],
    "rotation_axis": "local X",
    "rotation_sign": {"left": "negative", "right": "positive"},
    "results": results,
    "limitations": [
        "This is a Blender armature deformation test, not a Source-engine runtime test.",
        "Displacement values are mesh-space distances in the source model's units.",
        "Visual review of the four knee renders is still required to judge pinching, intersections, and silhouette quality."
    ]
}
with open(os.path.join(OUT, "Femgram-Knee-Deformation-Report.json"), "w") as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
