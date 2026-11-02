import bpy, os, json, runpy, math
from mathutils import Vector
from mathutils import Quaternion

ROOT = os.environ.get("GITHUB_WORKSPACE", os.getcwd())
SOURCE = os.environ["SOURCE_FBX"]
OUT = os.environ.get("OUTPUT_DIR", "dist/vertex-groups")
os.makedirs(OUT, exist_ok=True)

# Reuse the existing, reviewed anatomical weighting algorithm. It creates the
# ValveBiped vertex groups; this script then removes the temporary armature.
os.environ["SOURCE_FBX"] = SOURCE
os.environ["OUTPUT_DIR"] = os.path.join(OUT, "_temporary_rig")
runpy.run_path(os.path.join(ROOT, "model", "rig_femgram_75k.py"), run_name="__main__")

meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not meshes:
    raise RuntimeError("The weighting script did not produce a mesh.")
mesh = max(meshes, key=lambda o: len(o.data.vertices))
if len(mesh.data.vertices) != 75000:
    raise RuntimeError("Expected 75,000 vertices, got %d" % len(mesh.data.vertices))

# Detach the mesh while preserving its world transform; remove the temporary rig.
world = mesh.matrix_world.copy()
mesh.parent = None
mesh.matrix_world = world
for mod in list(mesh.modifiers):
    if mod.type == "ARMATURE":
        mesh.modifiers.remove(mod)
for obj in list(bpy.context.scene.objects):
    if obj.type == "ARMATURE":
        bpy.data.objects.remove(obj, do_unlink=True)

groups = list(mesh.vertex_groups)
if not groups:
    raise RuntimeError("No vertex groups were generated.")

# Distinct, high-contrast palette. Dominant influence is shown per vertex.
palette = [
 (0.95,0.12,0.12,1),(0.12,0.45,0.98,1),(0.10,0.80,0.35,1),
 (1.00,0.68,0.08,1),(0.72,0.20,0.92,1),(0.05,0.82,0.82,1),
 (0.98,0.35,0.62,1),(0.58,0.72,0.12,1),(0.98,0.52,0.18,1),
 (0.35,0.30,0.95,1),(0.12,0.68,0.52,1),(0.78,0.48,0.24,1),
 (0.55,0.55,0.55,1),(0.95,0.78,0.75,1),(0.30,0.72,0.95,1),
 (0.75,0.85,0.18,1)
]
group_index = {g.index:g.name for g in groups}
group_color = {g.name:palette[i % len(palette)] for i,g in enumerate(groups)}
attr = mesh.data.color_attributes.get("VertexGroupDominantColor")
if attr:
    mesh.data.color_attributes.remove(attr)
attr = mesh.data.color_attributes.new(name="VertexGroupDominantColor", type="FLOAT_COLOR", domain="POINT")

counts = {g.name:0 for g in groups}
unweighted = []
samples = []
for v in mesh.data.vertices:
    influences = [(e.weight, group_index.get(e.group, "")) for e in v.groups]
    if not influences:
        unweighted.append(v.index)
        attr.data[v.index].color = (0.02,0.02,0.02,1)
        continue
    influences.sort(reverse=True)
    weight, dominant = influences[0]
    counts[dominant] += 1
    attr.data[v.index].color = group_color[dominant]
    if len(samples) < 50 and weight < 0.55:
        samples.append({"vertex":v.index,"dominant_group":dominant,"dominant_weight":round(weight,4),
                        "influences":[{"group":n,"weight":round(w,4)} for w,n in influences]})

# Material displays the POINT color attribute with flat lighting.
mat = bpy.data.materials.new("Vertex Groups - Dominant Influence")
mat.diffuse_color = (0.8,0.8,0.8,1)
mat.use_nodes = True
nodes = mat.node_tree.nodes
nodes.clear()
out_node = nodes.new("ShaderNodeOutputMaterial")
em = nodes.new("ShaderNodeEmission")
vc = nodes.new("ShaderNodeVertexColor")
vc.layer_name = attr.name
mat.node_tree.links.new(vc.outputs["Color"], em.inputs["Color"])
mat.node_tree.links.new(em.outputs[0], out_node.inputs["Surface"])
mesh.data.materials.clear()
mesh.data.materials.append(mat)

# Camera and lighting for a clean, full-body front/side diagnostic render.
scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x = 1800
scene.render.resolution_y = 1800
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.film_transparent = False
scene.world.color = (0.035,0.035,0.035)
bpy.context.view_layer.update()
corners = [mesh.matrix_world @ Vector(c) for c in mesh.bound_box]
lo = Vector((min(v.x for v in corners),min(v.y for v in corners),min(v.z for v in corners)))
hi = Vector((max(v.x for v in corners),max(v.y for v in corners),max(v.z for v in corners)))
center = (lo+hi)*0.5
span = max(hi.x-lo.x, hi.y-lo.y, hi.z-lo.z)
def camera_at(name, location):
    data = bpy.data.cameras.new(name)
    cam = bpy.data.objects.new(name,data)
    scene.collection.objects.link(cam)
    cam.location = location
    direction = center-cam.location
    cam.rotation_euler = direction.to_track_quat("-Z","Y").to_euler()
    data.type = "ORTHO"
    data.ortho_scale = span*1.25
    scene.camera = cam
    return cam
def render_view(name, loc):
    cam = camera_at("DiagnosticCamera_"+name, loc)
    scene.render.filepath = os.path.join(OUT,"Femgram-Vertex-Groups-"+name+".png")
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam,do_unlink=True)
render_view("Front", center+Vector((0,-span*2.8,span*0.12)))
render_view("Side", center+Vector((span*2.8,0,span*0.12)))

# CSV legend and numerical QA.
legend = [{"group":g.name,"dominant_vertices":counts[g.name],
           "color":[round(x,3) for x in group_color[g.name]]} for g in groups]
report = {
 "vertices":len(mesh.data.vertices),"polygons":len(mesh.data.polygons),
 "groups":len(groups),"weighted_vertices":len(mesh.data.vertices)-len(unweighted),
 "unweighted_vertices":len(unweighted),"dominant_group_counts":counts,
 "palette_legend":legend,"low_dominance_samples":samples,
 "unweighted_vertex_indices":unweighted[:500],
 "note":"Colors show the strongest (dominant) vertex-group influence per vertex. Mixed weights are listed in low_dominance_samples; visual boundaries are diagnostic, not proof of anatomical correctness."
}
with open(os.path.join(OUT,"Femgram-Vertex-Groups-Report.json"),"w") as f:
    json.dump(report,f,indent=2)
# Export the mesh with groups but without an armature.
bpy.ops.object.select_all(action="DESELECT")
mesh.select_set(True)
bpy.context.view_layer.objects.active = mesh
bpy.ops.export_scene.fbx(filepath=os.path.join(OUT,"Femgram-75K-Vertex-Groups-Only.fbx"),
    use_selection=True,object_types={"MESH"},add_leaf_bones=False,bake_anim=False,
    path_mode="COPY",embed_textures=True,axis_forward="-Z",axis_up="Y")
print(json.dumps({"vertices":report["vertices"],"groups":report["groups"],
 "weighted":report["weighted_vertices"],"unweighted":report["unweighted_vertices"],
 "output":OUT},indent=2))
