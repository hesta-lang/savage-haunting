import bpy, os, json, math, bmesh
from collections import defaultdict
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

# Geometry and skinning diagnostics. These are measurements, not automatic
# proof of visual quality; thresholds are deliberately reported separately.
bm = bmesh.new()
bm.from_mesh(mesh.data)
bm.verts.ensure_lookup_table()
bm.edges.ensure_lookup_table()
bm.faces.ensure_lookup_table()

# Connected components of the surface, using mesh-edge connectivity.
parent = list(range(len(bm.verts)))
def find_root(i):
    while parent[i] != i:
        parent[i] = parent[parent[i]]
        i = parent[i]
    return i
def union(a, b):
    ra, rb = find_root(a), find_root(b)
    if ra != rb:
        parent[rb] = ra
for edge in bm.edges:
    if len(edge.verts) == 2:
        union(edge.verts[0].index, edge.verts[1].index)
components = defaultdict(int)
for vert in bm.verts:
    components[find_root(vert.index)] += 1

boundary_edges = sum(1 for e in bm.edges if len(e.link_faces) == 1)
wire_edges = sum(1 for e in bm.edges if len(e.link_faces) == 0)
nonmanifold_edges = sum(1 for e in bm.edges if not e.is_manifold)
degenerate_faces = sum(1 for face in bm.faces if face.calc_area() < 1e-10)
zero_length_edges = sum(1 for e in bm.edges if e.calc_length() < 1e-8)
inconsistent_normal_pairs = 0
for edge in bm.edges:
    if len(edge.link_faces) == 2:
        if edge.link_faces[0].normal.dot(edge.link_faces[1].normal) < -0.25:
            inconsistent_normal_pairs += 1

# Per-vertex normalized group weights; compare adjacent vertices for sharp
# changes in skinning. The L1 metric is bounded by 2 for normalized weights.
group_names = [g.name for g in mesh.vertex_groups]
group_index_to_name = {g.index: g.name for g in mesh.vertex_groups}
vertex_weights = []
for vert in verts:
    weights = {group_index_to_name[g.group]: float(g.weight)
               for g in vert.groups if g.group in group_index_to_name and g.weight > 1e-8}
    total = sum(weights.values())
    if total > 1e-8:
        weights = {k: v / total for k, v in weights.items()}
    vertex_weights.append(weights)

def weight_distance(a, b):
    keys = set(a) | set(b)
    return sum(abs(a.get(k, 0.0) - b.get(k, 0.0)) for k in keys)

edge_diffs = [weight_distance(vertex_weights[e.verts[0].index],
                              vertex_weights[e.verts[1].index]) for e in bm.edges]
edge_diffs_sorted = sorted(edge_diffs)
def percentile(values, p):
    if not values:
        return 0.0
    return values[min(len(values)-1, int(round((len(values)-1) * p)))]

# Body-zone summaries based on normalized coordinates. These bins are
# diagnostic approximations; they do not decide which vertices are anatomy.
def norm_x(v):
    return (mesh.matrix_world @ v.co).x
def norm_z(v):
    return (mesh.matrix_world @ v.co).z
def nz(v):
    return (norm_z(v) - lo[2]) / max(height, 1e-8)
def nx(v):
    return ((norm_x(v) - (lo[0]+hi[0])*0.5) / max(width, 1e-8))
def ny(v):
    return ((mesh.matrix_world @ v.co).y - (lo[1] + hi[1]) * 0.5) / max(depth, 1e-8)

# Finer spatial bins reveal which joint zone or surface carries unexpected
# influence. These are approximate geometric regions, not semantic labels.
regions = {
    "left_upper_thigh": lambda v: 0.29 <= nz(v) < 0.43 and nx(v) > 0.025,
    "left_knee": lambda v: 0.20 <= nz(v) < 0.29 and nx(v) > 0.025,
    "left_calf": lambda v: 0.075 <= nz(v) < 0.20 and nx(v) > 0.025,
    "left_ankle_foot": lambda v: nz(v) < 0.075 and nx(v) > 0.025,
    "right_upper_thigh": lambda v: 0.29 <= nz(v) < 0.43 and nx(v) < -0.025,
    "right_knee": lambda v: 0.20 <= nz(v) < 0.29 and nx(v) < -0.025,
    "right_calf": lambda v: 0.075 <= nz(v) < 0.20 and nx(v) < -0.025,
    "right_ankle_foot": lambda v: nz(v) < 0.075 and nx(v) < -0.025,
    "left_front_leg": lambda v: nz(v) < 0.43 and nx(v) > 0.025 and ny(v) < -0.12,
    "left_rear_leg": lambda v: nz(v) < 0.43 and nx(v) > 0.025 and ny(v) > 0.12,
    "right_front_leg": lambda v: nz(v) < 0.43 and nx(v) < -0.025 and ny(v) < -0.12,
    "right_rear_leg": lambda v: nz(v) < 0.43 and nx(v) < -0.025 and ny(v) > 0.12,
    "lower_body": lambda v: nz(v) < 0.48,
    "torso": lambda v: 0.38 <= nz(v) <= 0.72,
    "upper_body": lambda v: nz(v) > 0.68
}
# Adaptive front/rear classification: global Y thresholds failed because the
# character's leg depth occupies only a small fraction of the whole-body bounds.
# For each side and anatomical height band, derive front/back cutoffs from that
# band's own Y distribution. Front is the negative-Y direction used by the camera.
adaptive_surface_regions = {}
adaptive_surface_counts = {}
height_bands = {
    "upper_thigh": (0.29, 0.43),
    "knee": (0.20, 0.29),
    "calf": (0.075, 0.20),
    "ankle_foot": (0.0, 0.075)
}
for side_name, side_test in (
    ("left", lambda v: nx(v) > 0.025),
    ("right", lambda v: nx(v) < -0.025)
):
    for band_name, (z_min, z_max) in height_bands.items():
        candidates = [v for v in verts
                      if side_test(v) and z_min <= nz(v) < z_max]
        depths = sorted((mesh.matrix_world @ v.co).y for v in candidates)
        if len(depths) < 12:
            adaptive_surface_counts[side_name + "_" + band_name] = {
                "vertices": len(depths), "status": "insufficient_samples"
            }
            continue
        # Use 30th/70th percentiles so both surfaces remain well populated
        # even when the leg is rotated or its depth is nonuniform.
        front_cut = depths[int(0.30 * (len(depths) - 1))]
        rear_cut = depths[int(0.70 * (len(depths) - 1))]
        front_key = side_name + "_" + band_name + "_front"
        rear_key = side_name + "_" + band_name + "_rear"
        adaptive_surface_regions[front_key] = (
            lambda v, test=side_test, low=z_min, high=z_max, cut=front_cut:
                test(v) and low <= nz(v) < high
                and (mesh.matrix_world @ v.co).y <= cut
        )
        adaptive_surface_regions[rear_key] = (
            lambda v, test=side_test, low=z_min, high=z_max, cut=rear_cut:
                test(v) and low <= nz(v) < high
                and (mesh.matrix_world @ v.co).y >= cut
        )
        adaptive_surface_counts[side_name + "_" + band_name] = {
            "vertices": len(depths),
            "front_cut_y": round(front_cut, 6),
            "rear_cut_y": round(rear_cut, 6),
            "expected_front_samples": sum(1 for y in depths if y <= front_cut),
            "expected_rear_samples": sum(1 for y in depths if y >= rear_cut),
            "status": "ok"
        }

focus_groups = [n for n in group_names if any(
    token in n.lower() for token in ("pelvis", "thigh", "calf", "foot"))]
regions.update(adaptive_surface_regions)
region_weight_summary = {}
for region, predicate in regions.items():
    indices = [v.index for v in verts if predicate(v)]
    totals = defaultdict(float)
    active = defaultdict(int)
    for idx in indices:
        for name, value in vertex_weights[idx].items():
            totals[name] += value
            if value > 0.05:
                active[name] += 1
    denom = max(1, len(indices))
    region_weight_summary[region] = {
        "vertices": len(indices),
        "mean_weight_by_leg_group": {
            name: round(totals[name] / denom, 5) for name in focus_groups
        },
        "vertices_with_influence_over_0_05": {
            name: active[name] for name in focus_groups
        }
    }

geometry_diagnostics = {
    "vertices": len(bm.verts),
    "edges": len(bm.edges),
    "faces": len(bm.faces),
    "connected_components": len(components),
    "component_vertex_counts_largest_first": sorted(components.values(), reverse=True)[:20],
    "boundary_edges": boundary_edges,
    "wire_edges": wire_edges,
    "nonmanifold_edges": nonmanifold_edges,
    "degenerate_faces": degenerate_faces,
    "zero_length_edges": zero_length_edges,
    "adjacent_face_normal_pairs_opposed": inconsistent_normal_pairs,
    "weight_edge_discontinuity": {
        "edge_samples": len(edge_diffs),
        "mean_l1": round(sum(edge_diffs) / max(1, len(edge_diffs)), 6),
        "p50_l1": round(percentile(edge_diffs_sorted, 0.50), 6),
        "p95_l1": round(percentile(edge_diffs_sorted, 0.95), 6),
        "p99_l1": round(percentile(edge_diffs_sorted, 0.99), 6),
        "edges_l1_over_0_75": sum(1 for d in edge_diffs if d > 0.75),
        "edges_l1_over_1_25": sum(1 for d in edge_diffs if d > 1.25)
    },
    "adaptive_surface_sample_counts": adaptive_surface_counts,
    "regional_skinning_weights": region_weight_summary
}
bm.free()

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

def set_camera_view(view):
    if view == "front":
        cam.location = center + Vector((0, -size * 3.2, size * 0.06))
    elif view == "rear":
        cam.location = center + Vector((0, size * 3.2, size * 0.06))
    elif view == "left":
        cam.location = center + Vector((-size * 3.2, 0, size * 0.06))
    aim(cam, center)
    bpy.context.view_layer.update()

def render(name, view="front"):
    set_camera_view(view)
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
render("Femgram-neutral", "front")
render("Femgram-neutral-rear", "rear")
render("Femgram-neutral-side", "left")

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
render("Femgram-leg-pose-front", "front")
render("Femgram-leg-pose-rear", "rear")
render("Femgram-leg-pose-side", "left")


# Bone-placement audit: export the actual rest-pose endpoints and render a visible
# skeleton overlay. This is separate from skinning/pose tests.
bone_audit = {}
for bone_name in sorted(bone_names):
    bone = arm.data.bones[bone_name]
    head = arm.matrix_world @ bone.head_local
    tail = arm.matrix_world @ bone.tail_local
    bone_audit[bone_name] = {
        "head_world": [round(float(v), 6) for v in head],
        "tail_world": [round(float(v), 6) for v in tail],
        "length": round((tail-head).length, 6),
        "parent": bone.parent.name if bone.parent else None
    }

def make_overlay_material(name, color):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1.0)
    mat.use_nodes = True
    node = mat.node_tree.nodes.get("Principled BSDF")
    node.inputs["Base Color"].default_value = (*color, 1.0)
    node.inputs["Emission Color"].default_value = (*color, 1.0)
    node.inputs["Emission Strength"].default_value = 1.2
    node.inputs["Roughness"].default_value = 0.35
    return mat

bone_mat = make_overlay_material("RigOverlay_Bones", (0.08, 0.85, 1.0))
joint_mat = make_overlay_material("RigOverlay_Joints", (1.0, 0.35, 0.06))
overlay_objects = []
def add_bone_rod(name, start, end):
    delta = end-start
    length = delta.length
    if length < 1e-6:
        return
    bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=max(size*0.0035, 0.002), depth=length, location=(start+end)*0.5)
    obj = bpy.context.object
    obj.name = "Overlay_" + name
    obj.rotation_euler = delta.to_track_quat("Z", "Y").to_euler()
    obj.data.materials.append(bone_mat)
    overlay_objects.append(obj)

joint_points = {}
for bone_name, values in bone_audit.items():
    if any(token in bone_name for token in ("_Thigh", "_Calf", "_Foot")):
        head = Vector(values["head_world"])
        tail = Vector(values["tail_world"])
        add_bone_rod(bone_name, head, tail)
        for label, point in (("head", head), ("tail", tail)):
            key = tuple(round(float(v), 5) for v in point)
            joint_points[key] = point
for point in joint_points.values():
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=max(size*0.008, 0.004), location=point)
    obj = bpy.context.object
    obj.name = "Overlay_Joint"
    obj.data.materials.append(joint_mat)
    overlay_objects.append(obj)

# Hide the overlay objects from the neutral/pose meshes above; render explicit
# rest-pose skeleton overlays now, then retain all measurements in JSON.
reset_pose()
render("Femgram-bones-front", "front")
render("Femgram-bones-side", "left")
for obj in overlay_objects:
    obj.hide_render = True


report = {
    "mesh_objects": len(meshes), "armature_objects": len(arms),
    "vertices": len(verts), "weighted_vertices": weighted,
    "unweighted_vertices": len(verts)-weighted, "bones": len(bone_names),
    "missing_expected_bones": missing, "armature_modifier_linked": bool(arm_mods),
    "vertex_groups": len(mesh.vertex_groups),
    "geometry_diagnostics": geometry_diagnostics,
    "pose_tests": pose_tests,
    "arm_pose_rms_displacement": round(arm_rms, 6),
    "leg_pose_rms_displacement": round(leg_rms, 6),
    "bone_rest_pose_world": bone_audit,
    "renders": ["Femgram-bones-front.png", "Femgram-bones-side.png", "Femgram-neutral.png", "Femgram-neutral-rear.png", "Femgram-neutral-side.png", "Femgram-arm-pose.png", "Femgram-leg-pose-front.png", "Femgram-leg-pose-rear.png", "Femgram-leg-pose-side.png"],
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
