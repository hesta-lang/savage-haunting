import bpy, json, os, sys
from collections import Counter

# Import the optimized 75K FBX into a clean scene, then inspect its actual rig.
fbx_path = os.environ.get("SOURCE_FBX")
if not fbx_path or not os.path.isfile(fbx_path):
    raise RuntimeError("SOURCE_FBX does not point to an existing FBX: " + str(fbx_path))

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=fbx_path, automatic_bone_orientation=False)

scene = bpy.context.scene
armatures = []
for arm in [o for o in scene.objects if o.type == "ARMATURE"]:
    bones = []
    for b in arm.data.bones:
        bones.append({
            "name": b.name,
            "parent": b.parent.name if b.parent else None,
            "head": [round(v, 5) for v in b.head_local],
            "tail": [round(v, 5) for v in b.tail_local],
            "deform": bool(b.use_deform)
        })
    armatures.append({"object": arm.name, "bones": bones})

meshes = []
for obj in [o for o in scene.objects if o.type == "MESH"]:
    groups = {g.index: g.name for g in obj.vertex_groups}
    counts = Counter()
    weighted = 0
    unweighted = 0
    for v in obj.data.vertices:
        assigned = [g for g in v.groups if g.weight > 0.00001]
        if assigned:
            weighted += 1
            for g in assigned:
                counts[groups.get(g.group, str(g.group))] += 1
        else:
            unweighted += 1
    meshes.append({
        "name": obj.name,
        "vertices": len(obj.data.vertices),
        "polygons": len(obj.data.polygons),
        "dimensions": [round(x, 5) for x in obj.dimensions],
        "vertex_groups": [g.name for g in obj.vertex_groups],
        "weighted_vertices": weighted,
        "unweighted_vertices": unweighted,
        "weight_assignments_by_group": dict(counts),
        "armature_modifiers": [m.object.name if m.object else None for m in obj.modifiers if m.type == "ARMATURE"],
        "materials": [m.name if m else None for m in obj.data.materials],
        "uv_layers": [u.name for u in obj.data.uv_layers]
    })

report = {
    "source_fbx": os.path.abspath(fbx_path),
    "scene": scene.name,
    "armatures": armatures,
    "mesh_count": len(meshes),
    "meshes": meshes,
    "summary": {
        "armature_count": len(armatures),
        "total_bones": sum(len(a["bones"]) for a in armatures),
        "mesh_objects_with_vertex_groups": sum(bool(m["vertex_groups"]) for m in meshes),
        "mesh_objects_with_armature_modifier": sum(bool(m["armature_modifiers"]) for m in meshes),
        "total_vertices": sum(m["vertices"] for m in meshes),
        "total_weighted_vertices": sum(m["weighted_vertices"] for m in meshes),
        "total_unweighted_vertices": sum(m["unweighted_vertices"] for m in meshes)
    }
}
out = os.environ.get("REPORT_PATH", "dist/femgram-75k-rig-inspection.json")
os.makedirs(os.path.dirname(out), exist_ok=True)
with open(out, "w", encoding="utf-8") as f:
    json.dump(report, f, indent=2, ensure_ascii=False)
print("RIG_INSPECTION_REPORT=" + os.path.abspath(out))
print(json.dumps(report["summary"], indent=2))
