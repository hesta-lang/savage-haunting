import bpy
import math
from mathutils import Vector

# Femgram: blockout based ONLY on the newly approved 2026-10-31 reference.
# Stylized furry rabbit, taller silhouette, thick thighs, slim/medium waist.

def mat(name, color, roughness=0.82):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color, 1.0)
    m.roughness = roughness
    return m

WHITE = mat("Fur - warm white", (0.91, 0.91, 0.89))
BLACK = mat("Glossy near black", (0.006, 0.006, 0.008), 0.16)
GRAY = mat("Sweater charcoal gray", (0.22, 0.22, 0.23))
MIDGRAY = mat("Sweater ribbing", (0.13, 0.13, 0.14))
PALE = mat("Eye highlight", (0.98, 0.98, 0.96))

def uv(name, loc, scale, material, segments=24, rings=16):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=segments, ring_count=rings, location=loc)
    o = bpy.context.object
    o.name = name
    o.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    o.data.materials.append(material)
    for p in o.data.polygons:
        p.use_smooth = True
    return o

def capsule(name, a, b, radius, material, depth_scale=1.0):
    a, b = Vector(a), Vector(b)
    d = b-a
    mid = (a+b)*0.5
    ob = uv(name, mid, (radius, radius*depth_scale, d.length*0.5+radius*0.55), material)
    ob.rotation_mode = 'QUATERNION'
    ob.rotation_quaternion = d.to_track_quat('Z','Y')
    return ob

def curve_tube(name, points, bevel, material):
    c = bpy.data.curves.new(name, 'CURVE')
    c.dimensions = '3D'
    c.bevel_depth = bevel
    c.bevel_resolution = 3
    s = c.splines.new('BEZIER')
    s.bezier_points.add(len(points)-1)
    for p, co in zip(s.bezier_points, points):
        p.co = co
        p.handle_left_type = 'AUTO'
        p.handle_right_type = 'AUTO'
    o = bpy.data.objects.new(name, c)
    bpy.context.collection.objects.link(o)
    o.data.materials.append(material)
    return o

def build():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    # Character height approx 7.2 Blender units; front faces -Y.
    # Head and muzzle
    uv("Head_Furry", (0,0,5.65), (0.92,0.72,0.91), WHITE, 32,24)
    uv("Muzzle", (0,-0.53,5.35), (0.52,0.27,0.31), WHITE)
    uv("Nose", (0,-0.79,5.42), (0.075,0.045,0.05), BLACK,16,12)
    # Both large black eyes are visible in the new reference.
    uv("Eye_L", (-0.42,-0.655,5.78), (0.30,0.105,0.27), BLACK)
    uv("Eye_R", (0.42,-0.655,5.78), (0.30,0.105,0.27), BLACK)
    uv("Eye_glint_L", (-0.51,-0.755,5.91), (0.045,0.025,0.055), PALE,16,12)
    uv("Eye_glint_R", (0.33,-0.755,5.91), (0.045,0.025,0.055), PALE,16,12)

    # Long rabbit ears, slightly splayed, with black inner panels.
    earL = uv("Ear_L", (-0.53,0.02,6.72), (0.34,0.22,0.82), WHITE)
    earL.rotation_euler[1] = -0.22
    innerL = uv("Ear_inner_L", (-0.54,-0.195,6.73), (0.19,0.055,0.58), BLACK)
    innerL.rotation_euler[1] = -0.22
    earR = uv("Ear_R", (0.52,0.02,6.78), (0.34,0.22,0.79), WHITE)
    earR.rotation_euler[1] = 0.18
    innerR = uv("Ear_inner_R", (0.52,-0.195,6.79), (0.19,0.055,0.55), BLACK)
    innerR.rotation_euler[1] = 0.18

    # Fluffy hair cap and side locks / fringe.
    uv("Hair_cap", (0,-0.015,6.13), (0.94,0.76,0.49), WHITE)
    uv("Fringe_main", (0.18,-0.58,5.98), (0.62,0.20,0.48), WHITE)
    uv("Fringe_tip", (0.38,-0.60,5.64), (0.30,0.19,0.34), WHITE)
    # Long flowing hair falling behind and along both sides to hip level.
    uv("Hair_lock_L", (-0.91,-0.10,4.20), (0.30,0.34,1.62), WHITE,32,24)
    uv("Hair_lock_R", (0.91,-0.05,4.18), (0.30,0.34,1.66), WHITE,32,24)
    uv("Back_hair", (0,0.39,4.62), (1.02,0.40,1.62), WHITE,32,24)

    # Neck and sweater torso: taller torso, moderate waist.
    uv("Neck", (0,0,4.77), (0.30,0.30,0.28), WHITE)
    uv("Sweater_body", (0,0.02,4.00), (0.72,0.48,0.88), GRAY)
    uv("Sweater_hem", (0,-0.005,3.34), (0.76,0.49,0.18), MIDGRAY)
    uv("Folded_collar", (0,-0.015,4.48), (0.91,0.43,0.22), MIDGRAY)
    uv("Upper_chest_panel", (0,-0.02,4.63), (0.53,0.40,0.20), GRAY)
    curve_tube("Necklace_chain", [(-0.26,-0.39,4.78),(0,-0.49,4.52),(0.26,-0.39,4.78)], 0.018, PALE)
    uv("Pendant", (0,-0.51,4.43), (0.09,0.045,0.12), PALE,16,12)
    # Ribbed collar/hem hints
    for z in (3.29,3.36,4.67):
        curve_tube("Rib_detail", [(-0.34,-0.39,z),(0,-0.48,z-0.015),(0.34,-0.39,z)], 0.018, GRAY)

    # Sleeves, wrists and paws.
    for side, label in [(-1,"L"),(1,"R")]:
        capsule("Sleeve_"+label, (side*0.55,0,4.45), (side*1.00,-0.02,3.55), 0.31, GRAY, 1.05)
        uv("Cuff_"+label, (side*1.00,-0.02,3.55), (0.29,0.30,0.15), MIDGRAY)
        capsule("Forearm_fur_"+label, (side*1.00,-0.02,3.55), (side*1.12,-0.09,3.28), 0.17, WHITE)
        uv("Rabbit_paw_"+label, (side*1.16,-0.17,3.23), (0.22,0.19,0.15), WHITE)
        # subtle paw digits
        for j in range(2):
            uv("Paw_digit_"+label+str(j), (side*(1.10+j*0.12),-0.31,3.20), (0.075,0.06,0.09), WHITE,16,12)

    # Shorts with clear high-waist silhouette.
    uv("Shorts", (0,0.015,2.94), (0.76,0.48,0.43), BLACK)
    uv("Shorts_waistband", (0,-0.005,3.30), (0.78,0.49,0.14), MIDGRAY)
    # Two light side stripes on front/outer side.
    for side in (-1,1):
        curve_tube("Shorts_side_stripe", [(side*0.60,-0.40,3.17),(side*0.65,-0.43,2.95),(side*0.69,-0.37,2.73)], 0.018, PALE)

    # Thick, rounded thighs; not a heavy/obese body.
    for side, label in [(-1,"L"),(1,"R")]:
        uv("Thigh_"+label, (side*0.40,0.015,2.35), (0.53,0.47,0.86), WHITE,32,24)
        # Digitigrade lower leg: knee -> raised hock -> ankle.
        capsule("Shin_"+label, (side*0.42,0.02,1.82), (side*0.42,0.18,0.83), 0.22, WHITE)
        capsule("Hock_"+label, (side*0.42,0.18,0.83), (side*0.42,0.43,0.55), 0.19, WHITE)
        # Long rabbit foot points forward (-Y).
        uv("Rabbit_foot_"+label, (side*0.42,-0.12,0.28), (0.27,0.57,0.22), WHITE)
        for j in range(3):
            uv("Toe_"+label+str(j), (side*0.42+(j-1)*0.13,-0.57,0.24), (0.095,0.22,0.12), WHITE,16,12)

    # Tail
    uv("Cotton_tail", (0,0.49,3.13), (0.25,0.24,0.27), WHITE)

    # Basic ValveBiped-named skeleton for later weighting/animation work.
    arm_data = bpy.data.armatures.new("Femgram_ValveBiped")
    arm = bpy.data.objects.new("Femgram_ValveBiped", arm_data)
    bpy.context.collection.objects.link(arm)
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bones = [
        ("ValveBiped.Bip01_Pelvis",(0,0,3.0),(0,0,3.45),None),
        ("ValveBiped.Bip01_Spine",(0,0,3.45),(0,0,4.05),"ValveBiped.Bip01_Pelvis"),
        ("ValveBiped.Bip01_Spine1",(0,0,4.05),(0,0,4.48),"ValveBiped.Bip01_Spine"),
        ("ValveBiped.Bip01_Neck1",(0,0,4.48),(0,0,4.95),"ValveBiped.Bip01_Spine1"),
        ("ValveBiped.Bip01_Head",(0,0,4.95),(0,0,5.85),"ValveBiped.Bip01_Neck1"),
    ]
    created = {}
    for side, suf in [(-1,"_L"),(1,"_R")]:
        bones += [
            ("ValveBiped.Bip01_Thigh"+suf,(side*0.40,0,3.0),(side*0.42,0,2.0),"ValveBiped.Bip01_Pelvis"),
            ("ValveBiped.Bip01_Calf"+suf,(side*0.42,0,2.0),(side*0.42,0.18,0.83),"ValveBiped.Bip01_Thigh"+suf),
            ("ValveBiped.Bip01_Foot"+suf,(side*0.42,0.18,0.83),(side*0.42,-0.35,0.25),"ValveBiped.Bip01_Calf"+suf),
            ("ValveBiped.Bip01_Clavicle"+suf,(0,0,4.45),(side*0.58,0,4.45),"ValveBiped.Bip01_Spine1"),
            ("ValveBiped.Bip01_UpperArm"+suf,(side*0.58,0,4.45),(side*0.88,0,3.95),"ValveBiped.Bip01_Spine1"),
            ("ValveBiped.Bip01_Forearm"+suf,(side*0.88,0,3.95),(side*1.08,0,3.45),"ValveBiped.Bip01_UpperArm"+suf),
            ("ValveBiped.Bip01_Hand"+suf,(side*1.08,0,3.45),(side*1.16,-0.12,3.22),"ValveBiped.Bip01_Forearm"+suf),
        ]
    for name, head, tail, parent in bones:
        b = arm_data.edit_bones.new(name)
        b.head, b.tail = head, tail
        if parent:
            b.parent = arm_data.edit_bones.get(parent)
    bpy.ops.object.mode_set(mode='OBJECT')
    arm.show_in_front = True

    # Scene setup
    bpy.ops.object.select_all(action='DESELECT')
    for o in bpy.context.scene.objects:
        if o.type in {'MESH','CURVE','ARMATURE'}:
            o.select_set(True)
    bpy.context.view_layer.objects.active = arm
    bpy.context.scene.render.engine = 'BLENDER_EEVEE'
    bpy.context.scene.render.resolution_x = 900
    bpy.context.scene.render.resolution_y = 1200
    bpy.context.scene.render.resolution_percentage = 100
    bpy.context.scene.world.color = (0.8,0.8,0.8)
    # Front orthographic camera and area lights for preview render.
    bpy.ops.object.camera_add(location=(0,-13,3.5))
    cam = bpy.context.object
    cam.name = "Femgram_Camera"
    cam.data.type = 'ORTHO'
    cam.data.ortho_scale = 8.2
    cam.rotation_euler = (math.radians(90),0,0)
    bpy.context.scene.camera = cam
    bpy.ops.object.light_add(type='AREA', location=(-4,-7,8))
    bpy.context.object.data.energy = 900
    bpy.context.object.data.shape = 'DISK'
    bpy.context.object.data.size = 5
    bpy.ops.object.light_add(type='AREA', location=(4,-4,4))
    bpy.context.object.data.energy = 550
    bpy.context.object.data.size = 4

    bpy.context.scene.render.filepath = "/tmp/Femgram-preview.png"
    bpy.ops.render.render(write_still=True)
    bpy.ops.wm.save_as_mainfile(filepath="/tmp/Femgram.blend")
    bpy.ops.export_scene.fbx(filepath="/tmp/Femgram.fbx", use_selection=False, apply_scale_options='FBX_SCALE_ALL', add_leaf_bones=False, bake_anim=False)

build()
