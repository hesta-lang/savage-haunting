import bpy, os, sys, json, re

# Conservative weighted decimation: preserve likely detail meshes and small
# accessory meshes; distribute the remaining reduction across large body meshes.
args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if len(args) != 3:
    raise SystemExit("Usage: -- input.fbx output.fbx target_vertices")
src, dst, target_s = args
target = int(target_s)
if target < 1000:
    raise SystemExit("Target must be at least 1000 vertices")

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=src)
meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not meshes:
    raise RuntimeError("No mesh objects found")

before_v = sum(len(o.data.vertices) for o in meshes)
before_f = sum(len(o.data.polygons) for o in meshes)
if target >= before_v:
    print(f"Target {target} is not below source count {before_v}; exporting without decimation.")

# Names commonly used for facial and silhouette-critical components.
protected_pattern = re.compile(
    r"(face|head|eye|eyebrow|brow|mouth|nose|ear|hair|bang|fringe|"
    r"hand|finger|tooth|tongue|lash|pupil|iris)",
    re.IGNORECASE,
)
protected = []
reducible = []
for obj in meshes:
    count = len(obj.data.vertices)
    if protected_pattern.search(obj.name) or count <= 1000:
        protected.append(obj)
    else:
        reducible.append(obj)

protected_v = sum(len(o.data.vertices) for o in protected)
reducible_v = sum(len(o.data.vertices) for o in reducible)
if target < protected_v:
    raise RuntimeError(
        f"Target {target} is below protected geometry count {protected_v}; "
        "refusing to decimate protected face/detail meshes."
    )

if before_v > target and reducible_v:
    ratio = max(0.0, min(1.0, (target - protected_v) / reducible_v))
    for obj in reducible:
        if len(obj.data.vertices) < 100:
            continue
        mod = obj.modifiers.new("Femgram_Conservative_Decimation", "DECIMATE")
        mod.decimate_type = "COLLAPSE"
        mod.ratio = ratio
        bpy.context.view_layer.objects.active = obj
        obj.select_set(True)
        bpy.ops.object.modifier_apply(modifier=mod.name)
        obj.select_set(False)

after_v = sum(len(o.data.vertices) for o in meshes)
after_f = sum(len(o.data.polygons) for o in meshes)

bpy.ops.object.select_all(action="DESELECT")
for obj in meshes:
    obj.select_set(True)
bpy.ops.export_scene.fbx(
    filepath=dst,
    use_selection=True,
    object_types={"MESH", "ARMATURE", "EMPTY"},
    use_mesh_modifiers=True,
    mesh_smooth_type="FACE",
    use_tspace=True,
    path_mode="COPY",
    embed_textures=True,
    add_leaf_bones=False,
)

report = {
    "input": os.path.basename(src),
    "output": os.path.basename(dst),
    "target_vertices": target,
    "vertices_before": before_v,
    "vertices_after": after_v,
    "faces_before": before_f,
    "faces_after": after_f,
    "reduction_percent": round((1 - after_v / before_v) * 100, 2),
    "method": "Conservative Decimate Collapse",
    "protected_objects": [o.name for o in protected],
    "protected_vertices_before": protected_v,
    "reducible_objects": [o.name for o in reducible],
    "warning": "The target is approximate. Inspect close-up renders, UVs, facial geometry and rig deformation before replacing the source."
}
with open(os.path.splitext(dst)[0] + "-report.json", "w") as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
