import bpy
import json
import os
import sys
from array import array
from statistics import mean

def arg_after_separator(name, default):
    argv = sys.argv
    if "--" not in argv:
        return default
    tail = argv[argv.index("--") + 1:]
    for i, value in enumerate(tail[:-1]):
        if value == name:
            return tail[i + 1]
    return default

out_dir = arg_after_separator("--output-dir", "/tmp/femgram-meshy")
texture_dir = arg_after_separator("--texture-dir", "")
os.makedirs(out_dir, exist_ok=True)

def image_stats(image):
    width, height = image.size
    count = width * height
    raw = array('f', [0.0]) * (count * 4)
    image.pixels.foreach_get(raw)
    stride = max(1, count // 12000)
    samples = []
    for p in range(0, count, stride):
        i = p * 4
        samples.append((raw[i], raw[i + 1], raw[i + 2]))
    if not samples:
        return {"width": width, "height": height, "mean_rgb": [0, 0, 0], "chroma": 0, "normal_score": 0}
    channels = list(zip(*samples))
    avg = [mean(c) for c in channels]
    chroma = mean([max(px) - min(px) for px in samples])
    normal_score = max(0.0, avg[2] - (avg[0] + avg[1]) / 2.0)
    return {
        "width": width, "height": height,
        "mean_rgb": [round(v, 4) for v in avg],
        "chroma": round(chroma, 4),
        "normal_score": round(normal_score, 4),
        "sample_count": len(samples),
    }

texture_records = []
loaded = []
if texture_dir and os.path.isdir(texture_dir):
    for filename in sorted(os.listdir(texture_dir)):
        if not filename.lower().endswith(".png"):
            continue
        path = os.path.join(texture_dir, filename)
        try:
            image = bpy.data.images.load(path, check_existing=True)
            stats = image_stats(image)
            record = {"file": filename, **stats}
            texture_records.append(record)
            loaded.append((image, record))
        except Exception as exc:
            texture_records.append({"file": filename, "error": str(exc)})

# Identify only maps with strong visual signatures. Unknown grayscale maps are
# treated as roughness candidates, never guessed as metallic.
normal_choice = None
for image, record in loaded:
    rgb = record["mean_rgb"]
    if (record["normal_score"] > 0.12 and 0.32 <= rgb[0] <= 0.68
            and 0.32 <= rgb[1] <= 0.68 and abs(rgb[0] - rgb[1]) < 0.18):
        if normal_choice is None or record["normal_score"] > normal_choice[1]["normal_score"]:
            normal_choice = (image, record)

remaining = [(img, rec) for img, rec in loaded if normal_choice is None or img != normal_choice[0]]
base_choice = max(remaining, key=lambda pair: pair[1].get("chroma", 0), default=None)
if base_choice and base_choice[1].get("chroma", 0) < 0.035:
    # Avoid assigning a likely grayscale data map as visible color.
    base_choice = None

rough_choice = None
rough_candidates = []
for image, record in remaining:
    if base_choice and image == base_choice[0]:
        continue
    if record.get("chroma", 1) < 0.055:
        rough_candidates.append((image, record))
if rough_candidates:
    rough_choice = rough_candidates[0]

material = bpy.data.materials.get("Femgram_Meshy_PBR") or bpy.data.materials.new("Femgram_Meshy_PBR")
material.use_nodes = True
nodes = material.node_tree.nodes
links = material.node_tree.links
bsdf = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
if bsdf is None:
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
output = next((n for n in nodes if n.type == "OUTPUT_MATERIAL"), None)
if output is None:
    output = nodes.new("ShaderNodeOutputMaterial")
if not any(link.from_node == bsdf and link.to_node == output for link in links):
    links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])

def add_image_node(image, label, colorspace):
    image.colorspace_settings.name = colorspace
    node = nodes.new("ShaderNodeTexImage")
    node.image = image
    node.label = label
    node.name = label.replace(" ", "_")
    node.interpolation = "Linear"
    return node

assigned = {}
if base_choice:
    node = add_image_node(base_choice[0], "Meshy Base Color", "sRGB")
    links.new(node.outputs["Color"], bsdf.inputs["Base Color"])
    assigned["base_color"] = base_choice[1]["file"]
if normal_choice:
    node = add_image_node(normal_choice[0], "Meshy Normal", "Non-Color")
    normal = nodes.new("ShaderNodeNormalMap")
    links.new(node.outputs["Color"], normal.inputs["Color"])
    links.new(normal.outputs["Normal"], bsdf.inputs["Normal"])
    assigned["normal"] = normal_choice[1]["file"]
if rough_choice:
    node = add_image_node(rough_choice[0], "Meshy Roughness", "Non-Color")
    links.new(node.outputs["Color"], bsdf.inputs["Roughness"])
    assigned["roughness"] = rough_choice[1]["file"]

# Use the material on the source mesh while preserving the original geometry.
mesh_objects = [obj for obj in bpy.data.objects if obj.type == "MESH"]
for obj in mesh_objects:
    if len(obj.data.materials) == 0:
        obj.data.materials.append(material)
    else:
        # Meshy source has no assigned materials in the inspected source;
        # retain any existing slots and append this material for safety.
        if material.name not in [m.name for m in obj.data.materials if m]:
            obj.data.materials.append(material)
    mat_index = list(obj.data.materials).index(material)
    for polygon in obj.data.polygons:
        polygon.material_index = mat_index

# Pack all texture images into the .blend so the project remains portable.
for image, record in loaded:
    try:
        image.pack()
    except Exception:
        pass

scene = bpy.context.scene
objects = []
for obj in bpy.data.objects:
    item = {
        "name": obj.name, "type": obj.type,
        "dimensions": [round(float(v), 5) for v in obj.dimensions],
        "location": [round(float(v), 5) for v in obj.location],
        "materials": [m.name if m else None for m in getattr(obj.data, "materials", [])],
        "modifiers": [{"name": m.name, "type": m.type} for m in obj.modifiers],
    }
    if obj.type == "MESH":
        item["vertices"] = len(obj.data.vertices)
        item["polygons"] = len(obj.data.polygons)
        item["uv_layers"] = [uv.name for uv in obj.data.uv_layers]
    if obj.type == "ARMATURE":
        item["bones"] = [b.name for b in obj.data.bones]
    objects.append(item)

report = {
    "blend_file": bpy.data.filepath,
    "blender_version": bpy.app.version_string,
    "scene": scene.name,
    "object_count": len(objects),
    "objects": objects,
    "textures": texture_records,
    "assigned_texture_maps": assigned,
    "unassigned_texture_files": [r["file"] for r in texture_records if "error" not in r and r["file"] not in assigned.values()],
    "uv_available": any(obj.type == "MESH" and len(obj.data.uv_layers) > 0 for obj in bpy.data.objects),
}
with open(os.path.join(out_dir, "Femgram-Meshy-inspection.json"), "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)

blend_out = os.path.join(out_dir, "Femgram-Meshy-Textured.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend_out)

fbx_out = os.path.join(out_dir, "Femgram-Meshy-Textured.fbx")
bpy.ops.object.select_all(action="SELECT")
bpy.ops.export_scene.fbx(
    filepath=fbx_out, use_selection=True, apply_unit_scale=True,
    bake_space_transform=False, add_leaf_bones=False,
    use_mesh_modifiers=True, mesh_smooth_type="FACE", path_mode="COPY",
    embed_textures=True,
)

with open(os.path.join(out_dir, "Femgram-Meshy-texture-summary.txt"), "w", encoding="utf-8") as f:
    f.write("Femgram Meshy texture processing completed.\n")
    f.write("Mesh objects: %d\n" % len(mesh_objects))
    f.write("UV mapping available: %s\n" % report["uv_available"])
    f.write("Assigned maps: %s\n" % json.dumps(assigned, ensure_ascii=False))
    f.write("Textures found: %d\n" % len(texture_records))
    for rec in texture_records:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


// Create preview renders only; camera and lights are not saved into the model.
// The mesh, face, and texture coordinates are left untouched.
try:
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x = 1200
    scene.render.resolution_y = 1600
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX"

    from mathutils import Vector
    render_meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    corners = [o.matrix_world @ Vector(corner) for o in render_meshes for corner in o.bound_box]
    if not corners:
        raise RuntimeError("No mesh bounds available for preview render")
    mins = Vector((min(v.x for v in corners), min(v.y for v in corners), min(v.z for v in corners)))
    maxs = Vector((max(v.x for v in corners), max(v.y for v in corners), max(v.z for v in corners)))
    center = (mins + maxs) * 0.5
    size = maxs - mins
    radius = max(size.length * 0.5, 1.0)

    world = scene.world or bpy.data.worlds.new("Femgram Preview World")
    scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.055, 0.065, 0.085, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.45

    cam_data = bpy.data.cameras.new("Femgram_Preview_Camera")
    cam = bpy.data.objects.new("Femgram_Preview_Camera", cam_data)
    scene.collection.objects.link(cam)
    cam.data.lens = 52
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = max(size.z * 1.16, size.x * 1.55, 1.0)
    scene.camera = cam

    lights = []
    for name, offset, power, light_size in [
        ("Preview_Key", Vector((1.8, -2.4, 2.2)), 1500, 5.0),
        ("Preview_Fill", Vector((-2.0, -1.0, 0.5)), 900, 4.0),
        ("Preview_Rim", Vector((0.5, 2.0, 1.6)), 1800, 3.0),
    ]:
        data = bpy.data.lights.new(name, "AREA")
        data.energy = power
        data.shape = "DISK"
        data.size = light_size
        obj = bpy.data.objects.new(name, data)
        scene.collection.objects.link(obj)
        obj.location = center + offset * radius
        obj.rotation_euler = (center - obj.location).to_track_quat("-Z", "Y").to_euler()
        lights.append(obj)

    original_filepath = scene.render.filepath
    for view_name, direction in [
        ("Femgram-Render-Front.png", Vector((0, -1, 0))),
        ("Femgram-Render-Reverse.png", Vector((0, 1, 0))),
    ]:
        cam.location = center + direction * radius * 3.2 + Vector((0, 0, size.z * 0.03))
        cam.rotation_euler = (center - cam.location).to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = os.path.join(out_dir, view_name)
        bpy.ops.render.render(write_still=True)
    scene.render.filepath = original_filepath

    scene.camera = None
    bpy.data.objects.remove(cam, do_unlink=True)
    bpy.data.cameras.remove(cam_data)
    for light_obj in lights:
        light_data = light_obj.data
        bpy.data.objects.remove(light_obj, do_unlink=True)
        bpy.data.lights.remove(light_data)
except Exception as exc:
    with open(os.path.join(out_dir, "Femgram-render-error.txt"), "w", encoding="utf-8") as f:
        f.write(str(exc))
