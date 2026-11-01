import bpy, sys, math, os
from mathutils import Vector

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if len(args) != 2:
    raise SystemExit("Usage: -- input.fbx output.png")
src, out = args

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=src)

meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
if not meshes:
    raise RuntimeError("No mesh objects found for preview")

# Compute world-space bounds and frame the complete character.
corners = [o.matrix_world @ Vector(c) for o in meshes for c in o.bound_box]
mins = Vector((min(v.x for v in corners), min(v.y for v in corners), min(v.z for v in corners)))
maxs = Vector((max(v.x for v in corners), max(v.y for v in corners), max(v.z for v in corners)))
center = (mins + maxs) * 0.5
size = maxs - mins
radius = max(size.length * 0.5, 0.5)

bpy.ops.object.camera_add(location=(center.x, center.y - radius * 3.2, center.z + radius * 0.10))
cam = bpy.context.object
direction = center - cam.location
cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
cam.data.type = "ORTHO"
cam.data.ortho_scale = max(size.z * 1.18, size.x * 1.45, 1.0)
bpy.context.scene.camera = cam

def area_light(name, location, energy, size):
    bpy.ops.object.light_add(type="AREA", location=location)
    light = bpy.context.object
    light.name = name
    light.data.energy = energy
    light.data.shape = "DISK"
    light.data.size = size
    light.rotation_euler = (center - light.location).to_track_quat("-Z", "Y").to_euler()

area_light("Key", (center.x - radius, center.y - radius * 1.8, center.z + radius * 1.8), 800, radius * 1.3)
area_light("Fill", (center.x + radius * 1.5, center.y - radius, center.z + radius * 0.6), 250, radius)
area_light("Rim", (center.x, center.y + radius * 1.5, center.z + radius * 1.4), 450, radius)

scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.eevee.taa_render_samples = 64
scene.render.resolution_x = 1200
scene.render.resolution_y = 1200
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.film_transparent = False
scene.world.color = (0.018, 0.018, 0.018)
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"
scene.view_settings.exposure = -0.35
scene.render.filepath = os.path.abspath(out)
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
bpy.ops.render.render(write_still=True)
print("PREVIEW_RENDERED=" + os.path.abspath(out))
