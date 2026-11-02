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

# Measure the actual foot/ankle mesh geometry per side before changing bone placement.
# The lower 7.5% height band is the existing ankle/foot diagnostic zone.
foot_geometry_measurements = {}
for side_name, side_test, prefix in (
    ("left", lambda v: nx(v) > 0.025, "L"),
    ("right", lambda v: nx(v) < -0.025, "R")
):
    foot_vertices = [v for v in verts if side_test(v) and nz(v) < 0.075]
    points = [mesh.matrix_world @ v.co for v in foot_vertices]
    if points:
        foot_lo = [min(p[i] for p in points) for i in range(3)]
        foot_hi = [max(p[i] for p in points) for i in range(3)]
        foot_center = [sum(p[i] for p in points) / len(points) for i in range(3)]
        foot_geometry_measurements[side_name] = {
            "vertex_count": len(points),
            "bounds_world": {
                "min_x_y_z": [round(float(x), 6) for x in foot_lo],
                "max_x_y_z": [round(float(x), 6) for x in foot_hi]
            },
            "dimensions_x_y_z": [round(float(foot_hi[i]-foot_lo[i]), 6) for i in range(3)],
            "centroid_x_y_z": [round(float(x), 6) for x in foot_center],
            "toe_direction": "negative_Y (front-facing model axis)",
            "ankle_bone_head_world": [round(float(x), 6) for x in (arm.matrix_world @ arm.data.bones["ValveBiped.Bip01_" + prefix + "_Foot"].head_local)],
            "foot_bone_tail_world": [round(float(x), 6) for x in (arm.matrix_world @ arm.data.bones["ValveBiped.Bip01_" + prefix + "_Foot"].tail_local)]
        }
if "left" in foot_geometry_measurements and "right" in foot_geometry_measurements:
    lc = foot_geometry_measurements["left"]["centroid_x_y_z"]
    rc = foot_geometry_measurements["right"]["centroid_x_y_z"]
    foot_geometry_measurements["pair"] = {
        "centroid_separation_x": round(abs(lc[0] - rc[0]), 6),
        "centroid_separation_xyz": round(math.sqrt(sum((lc[i]-rc[i])**2 for i in range(3))), 6),
        "left_right_centroid_delta_xyz": [round(lc[i]-rc[i], 6) for i in range(3)]
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
# Fast validation renders: the rig metrics still use the full 75K mesh,
# while preview images use fewer pixels and samples to avoid spending most
# of the workflow time on redundant presentation renders.
scene.render.resolution_x = 600
scene.render.resolution_y = 800
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
scene.eevee.taa_render_samples = 20

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
def sampled_positions(indices=None):
    bpy.context.view_layer.update()
    deps = bpy.context.evaluated_depsgraph_get()
    evaluated = mesh.evaluated_get(deps)
    evmesh = evaluated.to_mesh()
    try:
        selected = sample if indices is None else indices
        return [evaluated.matrix_world @ evmesh.vertices[i].co
                for i in selected if i < len(evmesh.vertices)]
    finally:
        evaluated.to_mesh_clear()

def choose_axis(bone_name, angle):
    # The rig generator aligns the leg-bone roll so local X is the intended
    # anatomical flexion axis. Never select an axis by maximum RMS: that can
    # choose twist and makes the test depend on small numerical differences.
    group = mesh.vertex_groups.get(bone_name)
    indices = []
    if group is not None:
        indices = [v.index for v in verts
                   if any(g.group == group.index and g.weight > 0.05 for g in v.groups)]
    stride = max(1, len(indices) // 1800)
    indices = indices[::stride] if indices else sample
    saved = {pb.name: pb.rotation_euler.copy() for pb in arm.pose.bones}
    base_positions = sampled_positions(indices)
    axis_measurements = {}
    for axis in range(3):
        reset_pose()
        for name, rotation in saved.items():
            if name in arm.pose.bones:
                arm.pose.bones[name].rotation_euler = rotation
        pb = arm.pose.bones.get(bone_name)
        if pb is None:
            continue
        pb.rotation_euler[axis] = angle
        after = sampled_positions(indices)
        count = min(len(base_positions), len(after))
        rms = math.sqrt(sum((after[i] - base_positions[i]).length_squared
                            for i in range(count)) / max(1, count)) if count else 0.0
        axis_measurements["XYZ"[axis]] = round(rms, 8)
    reset_pose()
    for name, rotation in saved.items():
        if name in arm.pose.bones:
            arm.pose.bones[name].rotation_euler = rotation
    bpy.context.view_layer.update()
    # X is the explicit hinge axis authored by rig_femgram_75k.py.
    return (axis_measurements.get("X", 0.0), 0), len(indices), axis_measurements

reset_pose()
neutral_positions = sampled_positions()
pose_tests = {}
def apply_joint(bone_name, angle):
    if bone_name not in arm.pose.bones:
        result = {"found": False, "deformation_detected": False,
                  "rms_displacement": 0.0, "reason": "missing_pose_bone"}
        pose_tests[bone_name] = result
        return result
    pb = arm.pose.bones[bone_name]
    before_matrix = pb.matrix.copy()
    (rms, axis), sample_count, axis_measurements = choose_axis(bone_name, angle)
    if axis < 0 or rms <= 0.0001:
        bpy.context.view_layer.update()
        result = {
            "found": True, "deformation_detected": False,
            "chosen_local_axis": None, "rms_displacement": round(rms, 8),
            "axis_rms_displacement": axis_measurements,
            "influenced_sample_vertices": sample_count,
            "bone_matrix_changed": False,
            "reason": "weighted_vertices_did_not_move_on_any_local_axis"
        }
        pose_tests[bone_name] = result
        return result
    pb.rotation_euler[axis] = angle
    bpy.context.view_layer.update()
    matrix_delta = sum(abs(pb.matrix[r][c] - before_matrix[r][c])
                       for r in range(4) for c in range(4))
    result = {
        "found": True, "deformation_detected": True,
        "chosen_local_axis": ["X", "Y", "Z"][axis],
        "rms_displacement": round(rms, 8),
        "axis_rms_displacement": axis_measurements,
        "influenced_sample_vertices": sample_count,
        "bone_matrix_changed": matrix_delta > 1e-5,
        "bone_matrix_delta_l1": round(float(matrix_delta), 8)
    }
    pose_tests[bone_name] = result
    return result

# Test every leg joint independently from a neutral pose. This prevents a
# moving parent from hiding a child joint that is not deforming the mesh.
isolated_joint_tests = {}
for side_name in ("L", "R"):
    for part, test_angle in (("Thigh", 0.35), ("Calf", -0.50), ("Foot", 0.30)):
        reset_pose()
        full_name = "ValveBiped.Bip01_" + side_name + "_" + part
        isolated_joint_tests[full_name] = apply_joint(full_name, test_angle)
        reset_pose()

# Per-vertex knee audit: distinguish group membership from actual deformation
# and mesh-topology connectivity. Each knee vertex is tested against isolated
# thigh and calf rotations, then linked edges/neighbors and component membership
# are recorded from the imported FBX.
knee_vertex_audit = {}
component_sizes = defaultdict(int)
for root in parent:
    component_sizes[find_root(root)] += 1
all_indices = list(range(len(verts)))
for side_name, side_test, prefix in (
    ("left", lambda v: nx(v) > 0.025, "L"),
    ("right", lambda v: nx(v) < -0.025, "R")
):
    knee_indices = [v.index for v in verts if side_test(v) and 0.20 <= nz(v) < 0.29]
    side_report = {
        "vertex_count": len(knee_indices),
        "joint_tests": {},
        "vertices": []
    }
    # Store exact membership and topology for every vertex in this spatial knee band.
    vertex_records = {}
    for idx in knee_indices:
        v = verts[idx]
        weights = vertex_weights[idx]
        root = find_root(idx)
        linked = [e for e in bm.edges if any(ev.index == idx for ev in e.verts)] if False else []
        # Build neighbors from mesh edges without relying on surface normals.
        vertex_records[idx] = {
            "index": idx,
            "position_world": [round(float(x), 6) for x in (mesh.matrix_world @ v.co)],
            "weight_sum": round(sum(weights.values()), 8),
            "weights": {name: round(float(value), 8) for name, value in sorted(weights.items())},
            "has_matching_thigh_group": "ValveBiped.Bip01_" + prefix + "_Thigh" in weights,
            "has_matching_calf_group": "ValveBiped.Bip01_" + prefix + "_Calf" in weights,
            "has_matching_foot_group": "ValveBiped.Bip01_" + prefix + "_Foot" in weights,
            "connected_component_root": root,
            "connected_component_size": component_sizes[root],
            "neighbor_count": 0,
            "neighbor_indices": []
        }
    adjacency = defaultdict(list)
    for edge in mesh.data.edges:
        a, b = edge.vertices
        if a in vertex_records:
            adjacency[a].append(b)
        if b in vertex_records:
            adjacency[b].append(a)
    for idx, record in vertex_records.items():
        record["neighbor_count"] = len(adjacency[idx])
        record["neighbor_indices"] = adjacency[idx][:12]
    # Test each individual knee vertex against an isolated rotation of its own
    # thigh and calf; displacement is measured after Armature evaluation.
    reset_pose()
    base = sampled_positions(all_indices)
    for part, angle in (("Thigh", 0.35), ("Calf", -0.50)):
        full_name = "ValveBiped.Bip01_" + prefix + "_" + part
        (best_rms, axis), _, axis_metrics = choose_axis(full_name, angle)
        reset_pose()
        pb = arm.pose.bones.get(full_name)
        if pb is None or axis < 0:
            moved = [0.0] * len(verts)
            chosen_axis = None
        else:
            pb.rotation_euler[axis] = angle
            bpy.context.view_layer.update()
            posed = sampled_positions(all_indices)
            moved = [(posed[i] - base[i]).length for i in range(min(len(base), len(posed)))]
            chosen_axis = "XYZ"[axis]
        immobile = []
        moved_count = 0
        for idx in knee_indices:
            displacement = moved[idx] if idx < len(moved) else 0.0
            vertex_records[idx][part.lower() + "_displacement"] = round(float(displacement), 8)
            if displacement > 0.0001:
                moved_count += 1
            else:
                immobile.append(idx)
        side_report["joint_tests"][part] = {
            "chosen_local_axis": chosen_axis,
            "axis_rms_displacement": axis_metrics,
            "vertices_moved_over_0_0001": moved_count,
            "vertices_immobile": len(immobile),
            "immobile_vertex_indices": immobile[:500]
        }
        reset_pose()
    side_report["vertices"] = list(vertex_records.values())
    knee_vertex_audit[side_name] = side_report

# Neutral
reset_pose()
render("Femgram-neutral", "front")

# Stronger, multi-joint arm pose: shoulders lift and elbows bend.
reset_pose()
for side, sign in (("L", 1), ("R", -1)):
    apply_joint("ValveBiped.Bip01_" + side + "_UpperArm", sign * 0.95)
    apply_joint("ValveBiped.Bip01_" + side + "_Forearm", -sign * 1.15)
bpy.context.view_layer.update()
arm_pose_positions = sampled_positions()
arm_rms = math.sqrt(sum((arm_pose_positions[i] - neutral_positions[i]).length_squared for i in range(min(len(arm_pose_positions), len(neutral_positions)))) / min(len(arm_pose_positions), len(neutral_positions)))
# Arm deformation remains covered by numeric pose tests; skip its extra render.

# Leg deformation tests are isolated per side. The previous simultaneous
# opposite-direction test made the knees overlap in projection and obscured
# whether a crease came from the weights or from the deliberately crossed pose.
leg_pose_tests = {}
def run_leg_pose(side, hip_angle, knee_angle, label):
    reset_pose()
    apply_joint("ValveBiped.Bip01_" + side + "_Thigh", hip_angle)
    apply_joint("ValveBiped.Bip01_" + side + "_Calf", knee_angle)
    bpy.context.view_layer.update()
    posed = sampled_positions()
    count = min(len(posed), len(neutral_positions))
    rms = math.sqrt(sum((posed[i] - neutral_positions[i]).length_squared
                        for i in range(count)) / max(1, count))
    # Keep visual renders only for the isolated knee tests, which are the
    # critical deformation target. Combined poses remain measured numerically.
    if label.endswith("knee-only"):
        render("Femgram-leg-pose-" + label + "-front", "front")
        render("Femgram-leg-pose-" + label + "-side", "left")
    leg_pose_tests[label] = {
        "side": side, "hip_angle_radians": hip_angle,
        "knee_angle_radians": knee_angle, "rms_displacement": round(rms, 6)
    }
    return rms

left_leg_rms = run_leg_pose("L", 0.30, -0.55, "left")
right_leg_rms = run_leg_pose("R", -0.30, 0.55, "right")
left_knee_rms = run_leg_pose("L", 0.0, -0.65, "left-knee-only")
right_knee_rms = run_leg_pose("R", 0.0, 0.65, "right-knee-only")
leg_rms = max(left_leg_rms, right_leg_rms, left_knee_rms, right_knee_rms)
# Keep legacy names as aliases of the isolated left-leg combined pose.
reset_pose()
apply_joint("ValveBiped.Bip01_L_Thigh", 0.30)
apply_joint("ValveBiped.Bip01_L_Calf", -0.55)
bpy.context.view_layer.update()
# Legacy combined-pose preview renders omitted; the isolated knee renders above
# provide the visual check without duplicating the same deformation views.


# Bone-placement audit: export the actual rest-pose endpoints and render a visible
# skeleton overlay. This is separate from skinning/pose tests.
bone_audit = {}
for bone_name in sorted(bone_names):
    bone = arm.data.bones[bone_name]
    head = arm.matrix_world @ bone.head_local
    tail = arm.matrix_world @ bone.tail_local
    basis = bone.matrix_local.to_3x3()
    world_basis = arm.matrix_world.to_3x3() @ basis
    bone_audit[bone_name] = {
        "head_world": [round(float(v), 6) for v in head],
        "tail_world": [round(float(v), 6) for v in tail],
        "length": round((tail-head).length, 6),
        "parent": bone.parent.name if bone.parent else None,
        "local_axes_armature": {
            "X": [round(float(v), 6) for v in (basis @ Vector((1, 0, 0)))],
            "Y": [round(float(v), 6) for v in (basis @ Vector((0, 1, 0)))],
            "Z": [round(float(v), 6) for v in (basis @ Vector((0, 0, 1)))]
        },
        "local_axes_world": {
            "X": [round(float(v), 6) for v in (world_basis @ Vector((1, 0, 0)))],
            "Y": [round(float(v), 6) for v in (world_basis @ Vector((0, 1, 0)))],
            "Z": [round(float(v), 6) for v in (world_basis @ Vector((0, 0, 1)))]
        }
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
    "knee_vertex_audit": knee_vertex_audit,
    "foot_geometry_measurements": foot_geometry_measurements,
    "pose_tests": pose_tests,
    "isolated_leg_joint_tests": isolated_joint_tests,
    "arm_pose_rms_displacement": round(arm_rms, 6),
    "leg_pose_rms_displacement": round(leg_rms, 6),
    "leg_pose_tests": leg_pose_tests,
    "bone_rest_pose_world": bone_audit,
    "renders": ["Femgram-bones-front.png", "Femgram-bones-side.png", "Femgram-neutral.png", "Femgram-leg-pose-left-knee-only-front.png", "Femgram-leg-pose-left-knee-only-side.png", "Femgram-leg-pose-right-knee-only-front.png", "Femgram-leg-pose-right-knee-only-side.png"],
    "checks": {
        "mesh_and_armature_present": bool(meshes and arms),
        "all_vertices_weighted": weighted == len(verts),
        "expected_bones_present": not missing,
        "armature_modifier_linked": bool(arm_mods),
        "arm_pose_deforms_mesh": arm_rms > 0.002,
        "leg_pose_deforms_mesh": leg_rms > 0.002,
        "all_six_leg_joints_deform": len(isolated_joint_tests) == 6 and all(
            test.get("deformation_detected", False)
            for test in isolated_joint_tests.values()
        )
    },
    "warning": "Pose checks verify visible mesh movement, not final Source-engine compatibility. Inspect joint deformation and texture/material conversion before release."
}
report["passed_structural_checks"] = all(report["checks"].values())
with open(os.path.join(out, "Femgram-Rig-Validation.json"), "w") as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
if not report["passed_structural_checks"]:
    raise RuntimeError("One or more rig/deformation checks failed")
