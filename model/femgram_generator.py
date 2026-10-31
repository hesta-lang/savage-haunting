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


def loft_mesh(name, rings, material, segments=48):
    """Smooth continuous organic surface from elliptical cross-sections.
    Each ring: (z, center_x, center_y, radius_x, radius_y)."""
    verts, faces = [], []
    for z, cx, cy, rx, ry in rings:
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            verts.append((cx + rx * math.cos(a), cy + ry * math.sin(a), z))
    for r in range(len(rings)-1):
        for i in range(segments):
            a = r*segments+i
            b = r*segments+(i+1)%segments
            c = (r+1)*segments+(i+1)%segments
            d = (r+1)*segments+i
            faces.append((a,b,c,d))
    faces.append(tuple(range(segments-1,-1,-1)))
    off=(len(rings)-1)*segments
    faces.append(tuple(off+i for i in range(segments)))
    mesh=bpy.data.meshes.new(name+"_Mesh")
    mesh.from_pydata(verts,[],faces)
    mesh.materials.append(material)
    mesh.update()
    ob=bpy.data.objects.new(name,mesh)
    bpy.context.collection.objects.link(ob)
    for p in mesh.polygons: p.use_smooth=True
    sub=ob.modifiers.new("Organic surface smoothing","SUBSURF")
    sub.levels=2
    sub.render_levels=2
    return ob

def leaf_ear(name, x, z, lean, material, inner=False):
    # A tapered, curved ear shell built from a shaped mesh, not a sphere.
    rows=24; cols=24; verts=[]; faces=[]
    length=1.55 if not inner else 1.12
    width=0.29 if not inner else 0.155
    depth=0.105 if not inner else 0.035
    for j in range(rows+1):
        t=j/rows
        center_x=x+lean*t
        center_z=z+length*t
        w=width*(math.sin(math.pi*(t*0.94+0.03))**0.72)
        for i in range(cols):
            a=2*math.pi*i/cols
            verts.append((center_x+w*math.cos(a), -0.10+depth*math.sin(a), center_z))
    for j in range(rows):
        for i in range(cols):
            a=j*cols+i; b=j*cols+(i+1)%cols
            c=(j+1)*cols+(i+1)%cols; d=(j+1)*cols+i
            faces.append((a,b,c,d))
    me=bpy.data.meshes.new(name+"_Mesh"); me.from_pydata(verts,[],faces); me.materials.append(material); me.update()
    ob=bpy.data.objects.new(name,me); bpy.context.collection.objects.link(ob)
    for p in me.polygons: p.use_smooth=True
    ob.modifiers.new("Ear smoothing","SUBSURF").levels=2
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
    # Head proportions and facial expression follow the supplied front reference.
    loft_mesh("Head_Sculpted", [
        (4.86,0,0.02,0.22,0.25),(4.98,0,0.00,0.42,0.43),
        (5.15,0,0.00,0.62,0.57),(5.42,0,0.02,0.70,0.62),
        (5.70,0,0.04,0.73,0.64),(5.96,0,0.05,0.69,0.60),
        (6.16,0,0.05,0.53,0.49),(6.24,0,0.05,0.12,0.12)
    ], WHITE, 64)
    # Subtle muzzle integrated into lower face; avoid toy-like protruding snout.
    loft_mesh("Muzzle_Sculpted", [
        (5.03,0,-0.36,0.18,0.12),(5.10,0,-0.42,0.34,0.16),
        (5.20,0,-0.43,0.42,0.18),(5.29,0,-0.40,0.30,0.15),
        (5.34,0,-0.36,0.10,0.08)
    ], WHITE, 48)
    uv("Nose", (0,-0.60,5.28), (0.035,0.025,0.025), BLACK,16,12)
    # Half-lidded, wide black eyes with winged eyeliner, not round cartoon eyes.
    uv("Eye_L", (-0.31,-0.558,5.57), (0.255,0.075,0.145), BLACK,32,20)
    uv("Eye_R", (0.31,-0.558,5.57), (0.235,0.075,0.135), BLACK,32,20)
    curve_tube("Eyeliner_L", [(-0.58,-0.60,5.63),(-0.37,-0.63,5.68),(-0.12,-0.60,5.62)], 0.025, BLACK)
    curve_tube("Eyeliner_R", [(0.10,-0.60,5.62),(0.34,-0.63,5.67),(0.57,-0.60,5.64)], 0.022, BLACK)
    curve_tube("Brow_L", [(-0.50,-0.53,5.79),(-0.30,-0.56,5.84),(-0.10,-0.53,5.81)], 0.012, MIDGRAY)
    curve_tube("Brow_R", [(0.10,-0.53,5.81),(0.30,-0.56,5.84),(0.49,-0.53,5.78)], 0.012, MIDGRAY)
    curve_tube("Mouth", [(-0.035,-0.595,5.12),(0,-0.602,5.11),(0.035,-0.595,5.12)], 0.008, MIDGRAY)

    # One upright ear and one folded forward ear, with black inner surfaces.
    leaf_ear("Ear_L", -0.42, 6.03, -0.08, WHITE)
    leaf_ear("Ear_inner_L", -0.42, 6.13, -0.08, BLACK, True)
    leaf_ear("Ear_R", 0.43, 6.02, 0.10, WHITE)
    leaf_ear("Ear_inner_R", 0.43, 6.12, 0.10, BLACK, True)
    # Folded ear tip mesh as a curved tapered flap.
    loft_mesh("Folded_Ear_Tip", [(7.08,0.60,-0.04,0.10,0.10),(7.18,0.66,-0.05,0.22,0.12),(7.29,0.78,-0.04,0.28,0.10),(7.34,0.88,-0.02,0.12,0.07)], WHITE, 40)

    

        # Long layered white hairstyle: crown, swept fringe, side locks and back curtain.
    loft_mesh("Hair_Cap", [(5.78,0,0.08,0.62,0.48),(5.94,0,0.08,0.76,0.55),(6.12,0,0.08,0.73,0.55),(6.27,0,0.07,0.53,0.43),(6.34,0,0.06,0.12,0.12)], WHITE,64)
    loft_mesh("Swept_Fringe", [(5.38,0.24,-0.56,0.20,0.10),(5.52,0.20,-0.59,0.38,0.13),(5.72,0.10,-0.58,0.60,0.16),(5.94,-0.06,-0.49,0.66,0.17),(6.12,-0.12,-0.34,0.38,0.14)], WHITE,64)
    loft_mesh("Hair_Lock_Left", [(2.18,-0.88,-0.20,0.12,0.15),(2.48,-0.94,-0.20,0.23,0.19),(3.05,-0.98,-0.18,0.31,0.22),(3.75,-0.96,-0.16,0.34,0.24),(4.45,-0.88,-0.14,0.33,0.24),(5.12,-0.74,-0.12,0.30,0.22),(5.60,-0.62,-0.08,0.24,0.18)], WHITE,64)
    loft_mesh("Hair_Lock_Right", [(2.18,0.88,-0.20,0.12,0.15),(2.48,0.94,-0.20,0.23,0.19),(3.05,0.98,-0.18,0.31,0.22),(3.75,0.96,-0.16,0.34,0.24),(4.45,0.88,-0.14,0.33,0.24),(5.12,0.74,-0.12,0.30,0.22),(5.60,0.62,-0.08,0.24,0.18)], WHITE,64)
    loft_mesh("Back_Hair", [(2.25,0,0.35,0.10,0.13),(2.65,0,0.35,0.54,0.22),(3.35,0,0.34,0.86,0.27),(4.2,0,0.32,0.90,0.28),(5.0,0,0.28,0.78,0.25),(5.72,0,0.20,0.60,0.20)], WHITE,64)
    curve_tube("Hair_strand_L", [(-0.42,-0.58,6.02),(-0.58,-0.57,5.72),(-0.49,-0.53,5.45)], 0.018, PALE)
    curve_tube("Hair_strand_R", [(0.05,-0.60,6.10),(0.26,-0.59,5.88),(0.43,-0.54,5.62)], 0.014, PALE)

    

        # Anatomical torso with a narrow waist and broad pelvis; clothing follows its shape.
    loft_mesh("Body_Continuous", [
        (2.52,0,0.02,0.42,0.30),(2.72,0,0.02,0.66,0.39),
        (2.96,0,0.02,0.75,0.43),(3.18,0,0.02,0.66,0.40),
        (3.42,0,0.02,0.51,0.34),(3.68,0,0.02,0.48,0.33),
        (3.94,0,0.02,0.55,0.37),(4.18,0,0.02,0.66,0.40),
        (4.36,0,0.02,0.70,0.40),(4.52,0,0.02,0.40,0.30)
    ], WHITE, 64)
    loft_mesh("Neck", [(4.40,0,0.02,0.24,0.22),(4.62,0,0.02,0.25,0.23),(4.80,0,0.02,0.26,0.24)], WHITE)
    # Fitted sweater body with hourglass waist and broad shoulder line.
    loft_mesh("Sweater_Body", [(2.76,0,0.02,0.72,0.44),(2.88,0,0.02,0.78,0.47),(3.12,0,0.02,0.67,0.43),(3.38,0,0.02,0.53,0.37),(3.66,0,0.02,0.50,0.36),(3.92,0,0.02,0.58,0.39),(4.16,0,0.02,0.70,0.43),(4.31,0,0.02,0.70,0.42)], GRAY,64)
    # High folded turtleneck and wide off-shoulder folded collar.
    loft_mesh("Turtleneck", [(4.30,0,0.02,0.36,0.30),(4.38,0,0.02,0.43,0.34),(4.55,0,0.02,0.41,0.33),(4.68,0,0.02,0.34,0.29)], MIDGRAY,56)
    loft_mesh("OffShoulder_Collar", [(4.04,0,0.02,0.73,0.44),(4.12,0,0.02,0.82,0.47),(4.22,0,0.02,0.84,0.48),(4.30,0,0.02,0.76,0.44)], MIDGRAY,64)
    loft_mesh("Sweater_Hem", [(2.68,0,0.02,0.75,0.46),(2.78,0,0.02,0.79,0.47),(2.88,0,0.02,0.77,0.46)], MIDGRAY,64)
    curve_tube("Necklace_chain", [(-0.24,-0.38,4.32),(0,-0.47,4.05),(0.24,-0.38,4.32)], 0.015, PALE)
    uv("Pendant", (0,-0.49,4.00), (0.07,0.035,0.09), PALE,16,12)

    

        # Oversized bell sleeves taper into narrow cuffs; shoulders remain exposed.
    for side, label in [(-1,"L"),(1,"R")]:
        loft_mesh("Sleeve_"+label, [
            (2.70,side*1.00,-0.01,0.18,0.20),(2.82,side*1.08,-0.01,0.28,0.28),
            (3.12,side*1.08,-0.01,0.38,0.34),(3.42,side*0.90,-0.01,0.36,0.34),
            (3.72,side*0.76,-0.01,0.30,0.30),(4.02,side*0.69,-0.01,0.25,0.26),
            (4.18,side*0.67,-0.01,0.22,0.22)
        ], GRAY,56)
        loft_mesh("Cuff_"+label, [(2.68,side*1.00,-0.01,0.20,0.21),(2.78,side*1.00,-0.01,0.21,0.22)], MIDGRAY,40)
        loft_mesh("Hand_"+label, [(2.52,side*1.04,-0.10,0.10,0.11),(2.62,side*1.04,-0.12,0.17,0.15),(2.72,side*1.04,-0.10,0.15,0.14)], WHITE,40)
        for j in range(2):
            uv("Finger_"+label+str(j),(side*(0.99+j*0.10),-0.25,2.56),(0.07,0.07,0.10),WHITE,16,12)

    

        # High-waisted glossy black shorts with curved leg openings.
    loft_mesh("Shorts", [(2.35,0,0.02,0.64,0.40),(2.48,0,0.02,0.78,0.46),(2.68,0,0.02,0.80,0.47),(2.86,0,0.02,0.76,0.45)], BLACK,64)
    loft_mesh("Shorts_Waistband", [(2.78,0,0.02,0.77,0.46),(2.90,0,0.02,0.75,0.45)], MIDGRAY,64)
    # Full white upper thighs bridge the pelvis into the stockings; no floating leg segments.
    for side,label in [(-1,"L"),(1,"R")]:
        loft_mesh("UpperThigh_"+label, [
            (1.40,side*0.40,0.00,0.31,0.28),(1.58,side*0.40,0.00,0.38,0.33),
            (1.78,side*0.40,0.00,0.43,0.36),(1.98,side*0.40,0.00,0.46,0.37),
            (2.18,side*0.40,0.00,0.48,0.38),(2.36,side*0.40,0.00,0.43,0.35),
            (2.50,side*0.40,0.00,0.32,0.28)
        ], WHITE,64)
        # White fur cuff sits over the top edge of each glossy stocking.
        loft_mesh("Thigh_FurBand_"+label, [(1.54,side*0.40,-0.005,0.37,0.32),(1.66,side*0.40,-0.005,0.43,0.36),(1.82,side*0.40,-0.005,0.43,0.36),(1.94,side*0.40,-0.005,0.39,0.33)], WHITE,64)
        loft_mesh("Stocking_"+label, [(0.34,side*0.40,0.02,0.22,0.23),(0.60,side*0.40,0.02,0.24,0.25),(0.95,side*0.40,0.02,0.27,0.27),(1.28,side*0.40,0.02,0.31,0.29),(1.48,side*0.40,0.02,0.34,0.31),(1.60,side*0.40,0.02,0.36,0.32)], BLACK,56)

    

        # Lower legs and feet continue below the thigh-high stockings.
    for side,label in [(-1,"L"),(1,"R")]:
        loft_mesh("LowerLeg_"+label, [(0.34,side*0.40,0.02,0.19,0.20),(0.55,side*0.40,0.02,0.21,0.22),(0.82,side*0.40,0.02,0.20,0.21),(1.05,side*0.40,0.02,0.22,0.22)], WHITE,48)
        loft_mesh("DigitigradeFoot_"+label, [(0.10,side*0.40,-0.28,0.17,0.25),(0.17,side*0.40,-0.23,0.22,0.38),(0.30,side*0.40,-0.13,0.20,0.32),(0.42,side*0.40,0.00,0.14,0.20)], WHITE,40)
        for j in range(3):
            uv("Toe_"+label+str(j),(side*0.40+(j-1)*0.10,-0.50,0.17),(0.06,0.12,0.07),WHITE,16,12)

    

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
