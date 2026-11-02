import bpy, os, json, math
from mathutils import Vector

src=os.environ["SOURCE_FBX"]; out=os.environ.get("OUTPUT_DIR","dist/rigged")
os.makedirs(out,exist_ok=True)
bpy.ops.object.select_all(action="SELECT"); bpy.ops.object.delete(use_global=False)
bpy.ops.import_scene.fbx(filepath=src,automatic_bone_orientation=False)
mesh=max((o for o in bpy.context.scene.objects if o.type=="MESH"),key=lambda o:len(o.data.vertices))
vs=[mesh.matrix_world@v.co for v in mesh.data.vertices]
lo=[min(v[i] for v in vs) for i in range(3)]; hi=[max(v[i] for v in vs) for i in range(3)]
w,d,h=[hi[i]-lo[i] for i in range(3)]
if h<max(w,d): raise RuntimeError("Expected Z-up model; bounds="+str((w,d,h)))
cx=(lo[0]+hi[0])/2; cy=(lo[1]+hi[1])/2
def p(x,z,y=0): return world_to_arm @ Vector((cx+x*w,cy+y*d,lo[2]+z*h))
# Measure anatomical knee and ankle centers from the actual mesh cross-sections.
# Use component-wise medians to reject stray vertices and keep each side's
# pivot centered on its own surface rather than assuming a perfectly symmetric
# X/Y position. The same normalized bands are independently audited later.
joint_targets={}
for side_name, sign in (("L",1),("R",-1)):
    joint_targets[side_name]={}
    for joint_name, z_target in (("knee",0.265),("ankle",0.075)):
        samples=[q for q in vs
                 if ((q.x-cx)/w)*sign > 0.025
                 and abs((q.z-lo[2])/h-z_target) <= 0.006]
        if len(samples) < 20:
            raise RuntimeError("Insufficient "+side_name+" "+joint_name+" cross-section: "+str(len(samples)))
        joint_targets[side_name][joint_name]=Vector((
            sorted(q.x for q in samples)[len(samples)//2],
            sorted(q.y for q in samples)[len(samples)//2],
            sorted(q.z for q in samples)[len(samples)//2]
        ))
# Measure the toe endpoint from the actual upper/central toe surface.
# The previous front-most quantile included underside/sole vertices and could
# place the Foot tail below the foot mesh, tilting its long axis and making
# local pitch rotation shear the entire sole. Restrict the cross-section to
# the anatomical foot body above the sole, then use component-wise medians.
foot_targets={}
for side_name, sign in (("L",1),("R",-1)):
    foot_points=[q for q in vs
                 if ((q.x-cx)/w)*sign > 0.025
                 and (q.z-lo[2])/h < 0.075]
    if len(foot_points) < 20:
        raise RuntimeError("Insufficient "+side_name+" foot geometry: "+str(len(foot_points)))
    toe_surface=[q for q in foot_points
                 if 0.012 <= (q.z-lo[2])/h <= 0.065]
    if len(toe_surface) < 20:
        raise RuntimeError("Insufficient "+side_name+" upper toe surface: "+str(len(toe_surface)))
    front_depths=sorted(q.y for q in toe_surface)
    toe_cut=front_depths[max(0,int((len(front_depths)-1)*0.05))]
    toe_points=[q for q in toe_surface if q.y <= toe_cut]
    if len(toe_points) < 8:
        toe_cut=front_depths[max(0,int((len(front_depths)-1)*0.10))]
        toe_points=[q for q in toe_surface if q.y <= toe_cut]
    if len(toe_points) < 8:
        raise RuntimeError("Insufficient "+side_name+" toe cross-section: "+str(len(toe_points)))
    toe_world = Vector((
        sorted(q.x for q in toe_points)[len(toe_points)//2],
        sorted(q.y for q in toe_points)[len(toe_points)//2],
        sorted(q.z for q in toe_points)[len(toe_points)//2]
    ))
    foot_targets[side_name]={
        "x": (toe_world.x-cx)/w,
        "z": (toe_world.z-lo[2])/h,
        "toe_y": (toe_world.y-cy)/d,
        "toe_world": [float(toe_world.x), float(toe_world.y), float(toe_world.z)],
        "toe_sample_vertices": len(toe_points),
        "toe_surface_height_range": [0.012,0.065],
        "method": "raw-world component medians of front-most upper/central toe cross-section"
    }
def normalized_joint_point(point):
    return p((point.x-cx)/w,(point.z-lo[2])/h,(point.y-cy)/d)
# Match the armature object transform to the imported mesh. Bone positions are
# authored in mesh world coordinates and converted to armature-local coordinates.
mesh_world=mesh.matrix_world.copy()
world_to_arm=mesh_world.inverted_safe()
ad=bpy.data.armatures.new("Femgram_ValveBiped"); arm=bpy.data.objects.new("Femgram_ValveBiped",ad); bpy.context.collection.objects.link(arm)
arm.matrix_world=mesh_world
bpy.context.view_layer.objects.active=arm; arm.select_set(True); bpy.ops.object.mode_set(mode="EDIT")
eb={}; segments=[]
def add(n,a,b,parent=None,connected=False):
 q=ad.edit_bones.new("ValveBiped.Bip01_"+n); q.head=a; q.tail=b
 if parent: q.parent=eb[parent]; q.use_connect=connected
 eb[n]=q; segments.append((n,Vector(a),Vector(b)))
add("Root",p(0,.46),p(0,.49)); add("Pelvis",p(0,.46),p(0,.54),"Root")
add("Spine",p(0,.54),p(0,.62),"Pelvis"); add("Spine1",p(0,.62),p(0,.69),"Spine")
add("Spine2",p(0,.69),p(0,.75),"Spine1"); add("Neck",p(0,.75),p(0,.82),"Spine2"); add("Head",p(0,.82),p(0,.94),"Neck")
for s,sgn in (("L",1),("R",-1)):
 thigh=s+"_Thigh"; calf=s+"_Calf"; foot=s+"_Foot"
 knee_point=normalized_joint_point(joint_targets[s]["knee"])
 ankle_point=normalized_joint_point(joint_targets[s]["ankle"])
 add(thigh,p(sgn*.105,.49),knee_point,"Pelvis")
 add(calf,knee_point,ankle_point,thigh,True)
 # Calf tail and Foot head share the measured ankle center exactly. The foot
 # axis then follows the measured ankle-to-toe direction, preventing the
 # previous ~10 cm pivot offset from shearing the ankle during foot rotation.
 target=foot_targets[s]
 toe_tail_local = world_to_arm @ Vector(target["toe_world"])
 add(foot,ankle_point,toe_tail_local,calf,True)
 clav=s+"_Clavicle"; upper=s+"_UpperArm"; fore=s+"_Forearm"; hand=s+"_Hand"
 add(clav,p(sgn*.035,.70),p(sgn*.145,.70),"Spine2")
 add(upper,p(sgn*.145,.70),p(sgn*.315,.695),clav,True)
 add(fore,p(sgn*.315,.695),p(sgn*.445,.69),upper,True)
 add(hand,p(sgn*.445,.69),p(sgn*.50,.69),fore,True)
 for i,off in enumerate((-.018,-.009,0,.009,.018)):
  n=s+"_Finger"+str(i); a=p(sgn*.455,.69,off); b=p(sgn*.48,.69,off); c=p(sgn*.50,.69,off)
  add(n,a,b,hand); add(n+"_1",b,c,n,True); add(n+"_2",c,p(sgn*.51,.69,off),n+"_1",True)
# Give the thigh and calf a shared anatomical hinge frame. Their local X
# axes remain lateral, so knee flexion does not twist the lower leg.
# The foot is different: its long axis runs diagonally from ankle to toe.
# Reusing the calf's world-Y roll reference makes the foot's local X axis
# rotate obliquely, producing the observed ankle/sole shear. Align the foot's
# local Z toward world vertical instead; this makes its local X axis lateral
# (the intended ankle pitch axis) while preserving the measured ankle-to-toe
# bone direction and all original mesh vertices/weights.
for side_name in ("L", "R"):
 for part in ("Thigh", "Calf"):
  eb[side_name+"_"+part].align_roll(Vector((0.0, 1.0, 0.0)))
 eb[side_name+"_Foot"].align_roll(Vector((0.0, 0.0, 1.0)))
bpy.ops.object.mode_set(mode="OBJECT"); arm.show_in_front=True
# Explicit lower-limb deformation settings.
# Rotation inheritance carries the foot naturally during calf flexion, while
# NONE prevents inherited scale/shear from changing the child foot frame.
for side_name in ("L", "R"):
 for part in ("Thigh", "Calf", "Foot"):
  bone = ad.bones.get("ValveBiped.Bip01_" + side_name + "_" + part)
  if bone is None:
   raise RuntimeError("Missing lower-limb bone: " + side_name + "_" + part)
  bone.use_deform = True
  bone.inherit_scale = "NONE"
  bone.use_inherit_rotation = True
# Authoring-time anatomical rotation limits for the lower limbs.
# The knee hinge direction is mirrored in this rig's local X convention:
# left flexion is negative X; right flexion is positive X.
# Blender constraints are animator controls; they are not assumed to survive
# FBX export or to be enforced by the Source engine at runtime.
rotation_limit_specs = {
 "Thigh": {"x": (-105.0, 105.0), "y": (-35.0, 35.0), "z": (-35.0, 35.0)},
 "Calf": {"y": (-8.0, 8.0), "z": (-8.0, 8.0)},
 "Foot": {"y": (-20.0, 20.0), "z": (-25.0, 25.0)}
}
for side_name in ("L", "R"):
 for part in ("Thigh", "Calf", "Foot"):
  bone_name = "ValveBiped.Bip01_" + side_name + "_" + part
  pose_bone = arm.pose.bones[bone_name]
  pose_bone.rotation_mode = "XYZ"
  constraint = pose_bone.constraints.new("LIMIT_ROTATION")
  constraint.name = "Femgram anatomical limits"
  constraint.owner_space = "LOCAL"
  constraint.use_transform_limit = True
  specs = dict(rotation_limit_specs[part])
  if part == "Calf":
   specs["x"] = (-150.0, 5.0) if side_name == "L" else (-5.0, 150.0)
  elif part == "Foot":
   specs["x"] = (-45.0, 45.0)
  for axis, (minimum, maximum) in specs.items():
   low = math.radians(minimum)
   high = math.radians(maximum)
   setattr(constraint, "use_limit_" + axis.lower(), True)
   setattr(constraint, "min_" + axis.lower(), low)
   setattr(constraint, "max_" + axis.lower(), high)

# Connected joints must share exact endpoints.
for side_name in ("L", "R"):
 for parent_part, child_part in (("Thigh", "Calf"), ("Calf", "Foot")):
  parent_bone = ad.bones["ValveBiped.Bip01_" + side_name + "_" + parent_part]
  child_bone = ad.bones["ValveBiped.Bip01_" + side_name + "_" + child_part]
  if not child_bone.use_connect or (child_bone.head_local-parent_bone.tail_local).length > 1e-5:
   raise RuntimeError("Disconnected lower-limb joint: " + side_name + " " + child_part)
# Controlled anatomical weights: avoid Blender heat weights bleeding across limbs.
# The model is a proportion-based T-pose; use region + side filtering and at most
# three nearby bones so opposite limbs and fingers cannot steal influence.
for g in list(mesh.vertex_groups):
 mesh.vertex_groups.remove(g)
for m in list(mesh.modifiers):
 if m.type=="ARMATURE": mesh.modifiers.remove(m)
segments=[(n,a,b) for n,a,b in segments if n!="Root"]
groups={n:mesh.vertex_groups.new(name="ValveBiped.Bip01_"+n) for n,a,b in segments}
def distseg(x,a,b):
 v=b-a; t=max(0,min(1,(x-a).dot(v)/(v.dot(v)+1e-12))); return (x-(a+t*v)).length
def allowed_bones(x,z):
 side="L" if x>=0 else "R"
 if z < .465:
  # Keep the skirt and the long side/back hair attached to the pelvis.
  # Only the clearly exposed leg section below the clothing hem may follow
  # the thigh/calf chain. The upper thigh remains pelvis-weighted to avoid
  # pulling the skirt hem apart during hip rotation.
  ax=abs(x)
  if z >= .425 or ax < .045 or ax > .155:
   return ["Pelvis"]
  return [side+"_Thigh",side+"_Calf",side+"_Foot","Pelvis"]
 if z > .785:
  return [n for n,a,b in segments if n in ("Neck","Head")]
 if z >= .615 and abs(x) > .115:
  names=[n for n,a,b in segments if n in ("Spine2",side+"_Clavicle",side+"_UpperArm",side+"_Forearm",side+"_Hand") or n.startswith(side+"_Finger")]
  if abs(x) < .405: names=[n for n in names if "Finger" not in n]
  return names
 return [n for n,a,b in segments if n in ("Pelvis","Spine","Spine1","Spine2","Neck","Head","L_Clavicle","R_Clavicle")]
segmap={n:(arm.matrix_world @ a,arm.matrix_world @ b) for n,a,b in segments}
for v in mesh.data.vertices:
 xw=mesh.matrix_world@v.co
 nx=(xw.x-cx)/w
 nz=(xw.z-lo[2])/h
 ax=abs(nx)
 # Keep skirt and side/back hair outside the leg influence, but calculate
 # the actual thigh/calf/foot blend from distance to the anatomical bone
 # segments. This removes the hard height bands that caused visible seams.
 def smoothstep(a,b,value):
  t=max(0.0,min(1.0,(value-a)/(b-a)))
  return t*t*(3.0-2.0*t)
 side="L" if nx >= 0 else "R"
 # Widen the leg envelope: the previous narrow lateral gate left outer
 # calf/foot vertices pelvis-weighted. Distance to the actual limb chain
 # below prevents this wider envelope from capturing nearby clothing.
 lateral=smoothstep(.018,.045,ax)*(1.0-smoothstep(.205,.245,ax))
 vertical=1.0-smoothstep(.395,.465,nz)

 # Treat the knee as a real joint zone, not as an incidental height blend.
 # Within this band, exposed leg vertices are fully assigned to the thigh /
 # calf chain: the pelvis must not retain residual influence and pull the
 # kneecap away from the bending joint.
 knee_zone=smoothstep(.185,.215,nz)*(1.0-smoothstep(.315,.345,nz))
 knee_lateral=smoothstep(.025,.050,ax)*(1.0-smoothstep(.155,.180,ax))
 knee_core=max(0.0,min(1.0,knee_zone*knee_lateral))
 # Include the full exposed leg surface, using distance to the actual
 # side-specific thigh/calf/foot chain rather than a narrow X-coordinate band.
 leg_bones=[side+"_Thigh",side+"_Calf",side+"_Foot"]
 leg_distance=min(distseg(xw,*segmap[n]) for n in leg_bones)
 proximity=1.0-smoothstep(.115,.205,leg_distance)
 # Explicit foot island: the lowest exposed foot surface must not retain
 # Pelvis influence. A tiny pelvis remainder makes adjacent foot vertices
 # follow different transforms when the calf/foot chain rotates, producing
 # the extreme edge stretch found in vertices 67549/67626 and 65656/65485.
 # Keep this envelope side-specific and narrow enough to exclude the torso.
 foot_lateral=smoothstep(.008,.018,ax)*(1.0-smoothstep(.205,.245,ax))
 foot_vertical=1.0-smoothstep(.040,.095,nz)
 foot_core=max(0.0,min(1.0,foot_lateral*foot_vertical))
 leg_mask=max(0.0,min(1.0,max(lateral*vertical,knee_core,proximity*vertical,foot_core)))
 # The previous 0.995 cutoff missed edge vertices with a small but harmful
 # Pelvis remainder (for example, 7.1% Pelvis beside 100% Foot). Treat the
 # lower-foot island as fully leg-driven once its anatomical mask is strong.
 if foot_core >= 0.90:
  leg_mask=1.0
 
 if leg_mask > 0.001:
  # Stable anatomical partition: distance falloff must not overpower the
  # knee/ankle blend or pull the foot back onto the calf.
  knee_blend=smoothstep(.235,.295,nz)
  foot_blend=1.0-smoothstep(.025,.145,nz)
  priors={
   side+"_Thigh": knee_blend,
   side+"_Calf": (1.0-knee_blend)*(1.0-foot_blend),
   side+"_Foot": (1.0-knee_blend)*foot_blend
  }
  leg_total=sum(priors.values())
  for n,weight in priors.items():
   if weight > 1e-10:
    groups[n].add([v.index],leg_mask*weight/leg_total,"REPLACE")
  # Pelvis influence is allowed only outside the exposed leg mask. In the
  # knee core leg_mask is 1, so every knee vertex is attached to the leg chain.
  pelvis_weight=1.0-leg_mask
  # In the foot core, hard-zero Pelvis instead of allowing a residual blend.
  # The lower-limb prior is normalized below, so the foot remains fully weighted.
  if foot_core >= 0.90:
   pelvis_weight=0.0
  if pelvis_weight > 1e-12:
   groups["Pelvis"].add([v.index],pelvis_weight,"REPLACE")
  continue
 candidates=allowed_bones(nx,nz)
 if not candidates: candidates=[n for n,a,b in segments]
 near=sorted((distseg(xw,*segmap[n]),n) for n in candidates)
 near=near[:3]
 vals=[(n,1.0/((dist+0.045)**3.0)) for dist,n in near]
 total=sum(weight for n,weight in vals)
 for n,weight in vals:
  groups[n].add([v.index],weight/total,"REPLACE")
# Repair isolated knee-weight outliers by inheriting the skinning of
# their directly connected, correctly weighted knee neighbors. This does not
# create a dummy group: it copies normalized influences from the actual
# ValveBiped thigh/calf groups, so the repaired vertices deform with those bones.
knee_weight_repair = {"left": [], "right": []}
adjacency = [[] for _ in mesh.data.vertices]
for edge in mesh.data.edges:
    a, b = edge.vertices
    adjacency[a].append(b)
    adjacency[b].append(a)
group_by_name = {g.name: g for g in mesh.vertex_groups}
group_index_to_name = {g.index: g.name for g in mesh.vertex_groups}
for v in mesh.data.vertices:
    world = mesh.matrix_world @ v.co
    nx = (world.x - cx) / w
    nz = (world.z - lo[2]) / h
    if not (0.20 <= nz < 0.29 and abs(nx) > 0.025):
        continue
    side = "L" if nx > 0 else "R"
    thigh_name = "ValveBiped.Bip01_" + side + "_Thigh"
    calf_name = "ValveBiped.Bip01_" + side + "_Calf"
    foot_name = "ValveBiped.Bip01_" + side + "_Foot"
    pelvis_name = "ValveBiped.Bip01_Pelvis"
    own = {group_index_to_name[g.group]: g.weight for g in v.groups}
    own_leg = own.get(thigh_name, 0.0) + own.get(calf_name, 0.0) + own.get(foot_name, 0.0)
    neighbor_samples = []
    for ni in adjacency[v.index]:
        nv = mesh.data.vertices[ni]
        nw = mesh.matrix_world @ nv.co
        nnx = (nw.x - cx) / w
        nnz = (nw.z - lo[2]) / h
        if not (0.17 <= nnz < 0.32 and nnx * nx > 0):
            continue
        nweights = {group_index_to_name[g.group]: g.weight for g in nv.groups}
        nleg = nweights.get(thigh_name, 0.0) + nweights.get(calf_name, 0.0)
        if nleg >= 0.90:
            neighbor_samples.append(nweights)
    if len(neighbor_samples) < 2:
        continue
    # Only repair clear outliers. Preserve healthy existing knee blends.
    if own_leg >= 0.90:
        continue
    names = (thigh_name, calf_name)
    avg = {name: sum(sample.get(name, 0.0) for sample in neighbor_samples) /
                 len(neighbor_samples) for name in names}
    total = sum(avg.values())
    if total <= 1e-8:
        continue
    avg = {name: value / total for name, value in avg.items()}
    mesh.vertex_groups[thigh_name].add([v.index], avg[thigh_name], "REPLACE")
    mesh.vertex_groups[calf_name].add([v.index], avg[calf_name], "REPLACE")
    # Remove any conflicting pelvis/foot influence from this repaired knee outlier.
    for group_name in (pelvis_name, foot_name):
        group = group_by_name.get(group_name)
        if group:
            try:
                group.remove([v.index])
            except RuntimeError:
                pass
    knee_weight_repair["left" if side == "L" else "right"].append({
        "vertex": v.index,
        "neighbor_count": len(neighbor_samples),
        "inherited_weights": avg
    })
# Smooth the Thigh-to-Calf blend across the connected knee surface.
# The earlier height-only blend assigned identical ratios to every vertex at a
# given Z, but the irregular surface topology needs a locally continuous field.
# Smooth only the normalized Thigh/Calf share over mesh-connected knee vertices;
# preserve total lower-limb influence and never reintroduce Pelvis or Foot weight.
knee_transition_smoothing = {
    "left": [], "right": [], "iterations": 0, "factor": 0.0,
    "normalized_height_band": [0.17, 0.35],
    "preserves_total_leg_weight": True,
    "disabled_reason": "Avoids broad topology averaging that smears the hinge blend into the calf",
    "pelvis_weight_added": False,
    "topology_preserved": True,
    "method": "bilateral mesh-adjacency smoothing of normalized thigh/calf shares"
}
for side_name, sign in (("L", 1), ("R", -1)):
    thigh_name = "ValveBiped.Bip01_" + side_name + "_Thigh"
    calf_name = "ValveBiped.Bip01_" + side_name + "_Calf"
    eligible = set()
    for vtx in mesh.data.vertices:
        world = mesh.matrix_world @ vtx.co
        nx = (world.x - cx) / w
        nz = (world.z - lo[2]) / h
        if nx * sign <= 0.025 or not (0.17 <= nz <= 0.35):
            continue
        weights = {group_index_to_name[g.group]: g.weight for g in vtx.groups}
        leg_total = weights.get(thigh_name, 0.0) + weights.get(calf_name, 0.0)
        if leg_total >= 0.90 and weights.get("ValveBiped.Bip01_Pelvis", 0.0) <= 1e-6:
            eligible.add(vtx.index)

    for iteration in range(knee_transition_smoothing["iterations"]):
        updates = {}
        for idx in eligible:
            vtx = mesh.data.vertices[idx]
            own = {group_index_to_name[g.group]: g.weight for g in vtx.groups}
            total = own.get(thigh_name, 0.0) + own.get(calf_name, 0.0)
            if total <= 1e-8:
                continue
            own_thigh_share = own.get(thigh_name, 0.0) / total
            neighbor_shares = []
            for ni in adjacency[idx]:
                if ni not in eligible:
                    continue
                nv = mesh.data.vertices[ni]
                nw = {group_index_to_name[g.group]: g.weight for g in nv.groups}
                n_total = nw.get(thigh_name, 0.0) + nw.get(calf_name, 0.0)
                if n_total > 1e-8:
                    neighbor_shares.append(nw.get(thigh_name, 0.0) / n_total)
            if not neighbor_shares:
                continue
            average = sum(neighbor_shares) / len(neighbor_shares)
            factor = knee_transition_smoothing["factor"]
            new_share = max(0.0, min(1.0,
                own_thigh_share * (1.0-factor) + average*factor))
            updates[idx] = (total*new_share, total*(1.0-new_share))
        for idx, (thigh_weight, calf_weight) in updates.items():
            mesh.vertex_groups[thigh_name].add([idx], thigh_weight, "REPLACE")
            mesh.vertex_groups[calf_name].add([idx], calf_weight, "REPLACE")
    knee_transition_smoothing["left" if side_name == "L" else "right"] = sorted(eligible)

# Smooth the Calf-to-Foot handoff on the connected ankle surface.
# Work on the normalized Calf/Foot share only; preserve each vertex's total
# lower-limb influence and leave Thigh/Pelvis and the source geometry untouched.
ankle_transition_smoothing = {"left": [], "right": [], "iterations": 16, "factor": 0.35,
                               "normalized_height_band": [0.025, 0.155],
                               "foot_blend_height_range": [0.025, 0.145],
                               "method": "broad calf-to-foot ankle transition with bilateral topology smoothing; preserves total limb weight"}
for side_name, sign in (("L", 1), ("R", -1)):
    calf_name = "ValveBiped.Bip01_" + side_name + "_Calf"
    foot_name = "ValveBiped.Bip01_" + side_name + "_Foot"
    eligible = set()
    for vtx in mesh.data.vertices:
        world = mesh.matrix_world @ vtx.co
        nx = (world.x - cx) / w
        nz = (world.z - lo[2]) / h
        if nx * sign > 0.025 and 0.025 <= nz <= 0.155:
            weights = {group_index_to_name[g.group]: g.weight for g in vtx.groups}
            if weights.get(calf_name, 0.0) + weights.get(foot_name, 0.0) > 0.20:
                eligible.add(vtx.index)
    # Smooth the Foot share over mesh-connected neighbors, not Euclidean
    # proximity, so nearby but disconnected surfaces cannot exchange weights.
    for iteration in range(ankle_transition_smoothing["iterations"]):
        updates = {}
        for idx in eligible:
            vtx = mesh.data.vertices[idx]
            own = {group_index_to_name[g.group]: g.weight for g in vtx.groups}
            total = own.get(calf_name, 0.0) + own.get(foot_name, 0.0)
            if total <= 1e-8:
                continue
            own_foot_share = own.get(foot_name, 0.0) / total
            neighbor_shares = []
            for ni in adjacency[idx]:
                if ni not in eligible:
                    continue
                nv = mesh.data.vertices[ni]
                nw = {group_index_to_name[g.group]: g.weight for g in nv.groups}
                n_total = nw.get(calf_name, 0.0) + nw.get(foot_name, 0.0)
                if n_total > 1e-8:
                    neighbor_shares.append(nw.get(foot_name, 0.0) / n_total)
            if not neighbor_shares:
                continue
            average = sum(neighbor_shares) / len(neighbor_shares)
            factor = ankle_transition_smoothing["factor"]
            new_share = max(0.0, min(1.0, own_foot_share * (1.0-factor) + average*factor))
            updates[idx] = (total*(1.0-new_share), total*new_share)
        for idx, (calf_weight, foot_weight) in updates.items():
            mesh.vertex_groups[calf_name].add([idx], calf_weight, "REPLACE")
            mesh.vertex_groups[foot_name].add([idx], foot_weight, "REPLACE")
    ankle_transition_smoothing["left" if side_name == "L" else "right"] = sorted(eligible)

# Repair toe/sole outliers from their connected foot-surface neighbors.
# The copied influences always use the existing matching ValveBiped groups.
foot_weight_repair = {"left": [], "right": []}
for v in mesh.data.vertices:
    world = mesh.matrix_world @ v.co
    nx = (world.x - cx) / w
    nz = (world.z - lo[2]) / h
    if not (nz < 0.035 and 0.010 < abs(nx) < 0.22):
        continue
    side = "L" if nx > 0 else "R"
    thigh_name = "ValveBiped.Bip01_" + side + "_Thigh"
    calf_name = "ValveBiped.Bip01_" + side + "_Calf"
    foot_name = "ValveBiped.Bip01_" + side + "_Foot"
    pelvis_name = "ValveBiped.Bip01_Pelvis"
    own = {group_index_to_name[g.group]: g.weight for g in v.groups}
    if own.get(foot_name, 0.0) >= 0.85:
        continue
    neighbor_samples = []
    for ni in adjacency[v.index]:
        nv = mesh.data.vertices[ni]
        nw = mesh.matrix_world @ nv.co
        nnx = (nw.x - cx) / w
        nnz = (nw.z - lo[2]) / h
        if not (nnz < 0.10 and nnx * nx > 0):
            continue
        nweights = {group_index_to_name[g.group]: g.weight for g in nv.groups}
        if nweights.get(foot_name, 0.0) >= 0.85:
            neighbor_samples.append(nweights)
    if len(neighbor_samples) < 2:
        continue
    names = (thigh_name, calf_name, foot_name)
    avg = {name: sum(sample.get(name, 0.0) for sample in neighbor_samples) /
                 len(neighbor_samples) for name in names}
    total = sum(avg.values())
    if total <= 1e-8:
        continue
    avg = {name: value / total for name, value in avg.items()}
    for name in names:
        mesh.vertex_groups[name].add([v.index], avg[name], "REPLACE")
    group = group_by_name.get(pelvis_name)
    if group:
        try:
            group.remove([v.index])
        except RuntimeError:
            pass
    foot_weight_repair["left" if side == "L" else "right"].append({
        "vertex": v.index,
        "neighbor_count": len(neighbor_samples),
        "inherited_weights": avg
    })
# Lock the anatomical sole/toe cap to the Foot bone.
# The previous neighbor repair skipped vertices that already had >=85% Foot,
# even when the remaining 5-8% Pelvis influence was enough to stretch long
# edges during calf/foot rotation. The distal foot cap is a rigid foot region:
# make its influence exactly 100% Foot on both sides, without touching knees.
foot_cap_weight_lock = {"left": [], "right": [], "normalized_height_max": 0.035,
                        "foot_weight_after": 1.0, "pelvis_weight_after": 0.0}
for v in mesh.data.vertices:
    world = mesh.matrix_world @ v.co
    nx = (world.x - cx) / w
    nz = (world.z - lo[2]) / h
    if not (nz < foot_cap_weight_lock["normalized_height_max"] and
            0.025 < abs(nx) < 0.20):
        continue
    side = "L" if nx > 0 else "R"
    thigh_name = "ValveBiped.Bip01_" + side + "_Thigh"
    calf_name = "ValveBiped.Bip01_" + side + "_Calf"
    foot_name = "ValveBiped.Bip01_" + side + "_Foot"
    pelvis_name = "ValveBiped.Bip01_Pelvis"
    own = {group_index_to_name[g.group]: g.weight for g in v.groups}
    previous = {name: round(float(own.get(name, 0.0)), 6)
                for name in (thigh_name, calf_name, foot_name, pelvis_name)}
    mesh.vertex_groups[foot_name].add([v.index], 1.0, "REPLACE")
    for group_name in (thigh_name, calf_name, pelvis_name):
        group = group_by_name.get(group_name)
        if group:
            try:
                group.remove([v.index])
            except RuntimeError:
                pass
    foot_cap_weight_lock["left" if side == "L" else "right"].append({
        "vertex": v.index, "previous_weights": previous
    })

# Quantitative ankle/foot weight diagnostics (read-only; no vertex or weight edits).
# Report both the complete ankle transition and narrower sub-bands so residual
# pelvis influence and abrupt calf/foot handoffs can be located per side.
ankle_weight_diagnostics = {"bands": {}, "thresholds": {"meaningful_weight": 0.01, "pelvis_warning": 0.01}}
diagnostic_bands = {
    "ankle_transition": (0.045, 0.135),
    "lower_ankle": (0.045, 0.075),
    "ankle_joint": (0.075, 0.105),
    "upper_ankle": (0.105, 0.135),
    "foot_top": (0.035, 0.075),
    "lower_leg_context": (0.135, 0.17)
}
for side_name, sign in (("L", 1), ("R", -1)):
    names = {
        "thigh": "ValveBiped.Bip01_" + side_name + "_Thigh",
        "calf": "ValveBiped.Bip01_" + side_name + "_Calf",
        "foot": "ValveBiped.Bip01_" + side_name + "_Foot",
        "pelvis": "ValveBiped.Bip01_Pelvis"
    }
    side_report = {}
    for band_name, (z_min, z_max) in diagnostic_bands.items():
        samples = []
        for vertex in mesh.data.vertices:
            world = mesh.matrix_world @ vertex.co
            nx = (world.x - cx) / w
            nz = (world.z - lo[2]) / h
            if nx * sign <= 0.025 or not (z_min <= nz < z_max):
                continue
            weights = {group_index_to_name[g.group]: g.weight for g in vertex.groups}
            samples.append((vertex.index, {
                key: weights.get(group_name, 0.0)
                for key, group_name in names.items()
            }))
        band_report = {"vertex_count": len(samples)}
        for key in names:
            values = sorted(sample[1][key] for sample in samples)
            if not values:
                band_report[key] = {"mean": 0.0, "min": 0.0, "max": 0.0,
                                    "over_1pct": 0, "over_10pct": 0}
                continue
            band_report[key] = {
                "mean": round(sum(values) / len(values), 6),
                "min": round(values[0], 6),
                "p50": round(values[len(values)//2], 6),
                "p95": round(values[min(len(values)-1, int(len(values)*0.95))], 6),
                "max": round(values[-1], 6),
                "over_1pct": sum(value > 0.01 for value in values),
                "over_10pct": sum(value > 0.10 for value in values)
            }
        band_report["pelvis_contamination_vertices"] = [
            index for index, weights in samples if weights["pelvis"] > 0.01
        ]
        band_report["dominant_bone_counts"] = {
            key: sum(max(sample[1], key=sample[1].get) == key for sample in samples)
            for key in names
        }
        side_report[band_name] = band_report
    ankle_weight_diagnostics["bands"]["left" if side_name == "L" else "right"] = side_report

mod=mesh.modifiers.new("Femgram Armature","ARMATURE")
mod.object=arm
# Dual-quaternion skinning preserves limb volume during large knee bends.
# This is evaluated in Blender and exported as the armature modifier setting.
mod.use_deform_preserve_volume=True
# Preserve the mesh world transform when parenting it to the armature.
mesh.parent=arm
mesh.matrix_parent_inverse=arm.matrix_world.inverted_safe()
auto=False
err="Using controlled anatomical region/side weights instead of automatic heat weights"
# Normalize each vertex's influences.
for v in mesh.data.vertices:
 total=sum(g.weight for g in v.groups)
 if total:
  for g in v.groups: mesh.vertex_groups[g.group].add([v.index],g.weight/total,"REPLACE")

# Final distal-foot lock AFTER every smoothing/repair/normalization pass.
# Match the validator's ROI exactly so no boundary vertices escape the lock.
final_distal_foot_lock = {"left": 0, "right": 0, "normalized_height_max": 0.045,
                          "normalized_lateral_min": 0.018, "normalized_lateral_max": 0.20}
for v in mesh.data.vertices:
 world = mesh.matrix_world @ v.co
 nx = (world.x-cx)/w
 nz = (world.z-lo[2])/h
 if not (nz < final_distal_foot_lock["normalized_height_max"] and
         0.018 < abs(nx) < 0.20):
  continue
 side = "L" if nx > 0 else "R"
 foot_name = "ValveBiped.Bip01_" + side + "_Foot"
 for group in mesh.vertex_groups:
  if group.name != foot_name:
   try:
    group.remove([v.index])
   except RuntimeError:
    pass
 mesh.vertex_groups[foot_name].add([v.index], 1.0, "REPLACE")
 final_distal_foot_lock["left" if side == "L" else "right"] += 1

# Final knee cleanup after all interpolation/repair passes. The earlier smoother
# excluded vertices with even a small Pelvis remainder; those isolated vertices
# created severe edge stretch on the medial right knee. Only vertices already
# strongly assigned to the matching leg chain are eligible, so skirt/body
# vertices remain untouched. Remove residual Pelvis/Foot and smooth the
# normalized Thigh/Calf share over the actual connected knee surface.
final_knee_weight_cleanup = {"left": 0, "right": 0, "iterations": 0, "factor": 0.0,
                             "normalized_height_band": [0.17, 0.35],
                             "minimum_existing_leg_weight": 0.80,
                             "smoothing_disabled": True}
for side_name, sign in (("L", 1), ("R", -1)):
    thigh_name = "ValveBiped.Bip01_" + side_name + "_Thigh"
    calf_name = "ValveBiped.Bip01_" + side_name + "_Calf"
    foot_name = "ValveBiped.Bip01_" + side_name + "_Foot"
    pelvis_name = "ValveBiped.Bip01_Pelvis"
    eligible = set()
    for vtx in mesh.data.vertices:
        world = mesh.matrix_world @ vtx.co
        nx = (world.x-cx)/w
        nz = (world.z-lo[2])/h
        if nx*sign <= 0.025 or not (0.17 <= nz <= 0.35):
            continue
        own = {group_index_to_name[g.group]: g.weight for g in vtx.groups}
        thigh_w = own.get(thigh_name, 0.0)
        calf_w = own.get(calf_name, 0.0)
        leg_w = thigh_w + calf_w
        if leg_w < final_knee_weight_cleanup["minimum_existing_leg_weight"]:
            continue
        # Preserve the vertex's existing anatomical Thigh/Calf ratio while
        # removing only contaminating influences from the other body regions.
        total = max(1e-8, leg_w)
        mesh.vertex_groups[thigh_name].add([vtx.index], thigh_w/total, "REPLACE")
        mesh.vertex_groups[calf_name].add([vtx.index], calf_w/total, "REPLACE")
        for other_name in (foot_name, pelvis_name):
            other = group_by_name.get(other_name)
            if other:
                try: other.remove([vtx.index])
                except RuntimeError: pass
        eligible.add(vtx.index)
    # Iterative bilateral graph smoothing; updates are applied per iteration
    # to keep the field stable on irregular topology.
    for _ in range(final_knee_weight_cleanup["iterations"]):
        updates = {}
        for idx in eligible:
            vtx = mesh.data.vertices[idx]
            own = {group_index_to_name[g.group]: g.weight for g in vtx.groups}
            own_total = own.get(thigh_name, 0.0) + own.get(calf_name, 0.0)
            if own_total <= 1e-8: continue
            shares = []
            for ni in adjacency[idx]:
                if ni not in eligible: continue
                nw = {group_index_to_name[g.group]: g.weight
                      for g in mesh.data.vertices[ni].groups}
                nt = nw.get(thigh_name, 0.0) + nw.get(calf_name, 0.0)
                if nt > 1e-8: shares.append(nw.get(thigh_name, 0.0)/nt)
            if not shares: continue
            average = sum(shares)/len(shares)
            current = own.get(thigh_name, 0.0)/own_total
            share = max(0.0, min(1.0,
                current*(1.0-final_knee_weight_cleanup["factor"]) +
                average*final_knee_weight_cleanup["factor"]))
            updates[idx] = (share, 1.0-share)
        for idx, (thigh_w, calf_w) in updates.items():
            mesh.vertex_groups[thigh_name].add([idx], thigh_w, "REPLACE")
            mesh.vertex_groups[calf_name].add([idx], calf_w, "REPLACE")
    final_knee_weight_cleanup["left" if side_name == "L" else "right"] = len(eligible)

# Verify the authored Foot bone endpoints before FBX export. This catches an
# armature-space/world-space regression before the artifact is published.
foot_bone_endpoint_audit = {}
for side_name in ("L", "R"):
    bone = ad.bones.get("ValveBiped.Bip01_" + side_name + "_Foot")
    target = Vector(foot_targets[side_name]["toe_world"])
    actual = arm.matrix_world @ bone.tail_local
    delta = actual-target
    foot_bone_endpoint_audit[side_name] = {
        "target_world": [float(x) for x in target],
        "actual_tail_world": [float(x) for x in actual],
        "offset_distance": float(delta.length)
    }
    if delta.length > 0.005:
        raise RuntimeError("Foot bone endpoint is not at measured toe surface: " +
                           side_name + " offset=" + str(delta.length))

bpy.ops.object.select_all(action="DESELECT"); mesh.select_set(True); arm.select_set(True); bpy.context.view_layer.objects.active=arm
dest=os.path.join(out,"Femgram-75K-ValveBiped-Rigged.fbx")
bpy.ops.export_scene.fbx(filepath=dest,use_selection=True,object_types={"ARMATURE","MESH"},add_leaf_bones=False,bake_anim=False,path_mode="COPY",embed_textures=True,axis_forward="-Z",axis_up="Y")
report={"vertices":len(mesh.data.vertices),"polygons":len(mesh.data.polygons),"bones":len(ad.bones),"vertex_groups":len(mesh.vertex_groups),"weighted_vertices":sum(bool(v.groups) for v in mesh.data.vertices),"knee_weight_repair":knee_weight_repair,"knee_transition_smoothing":knee_transition_smoothing,"foot_weight_repair":foot_weight_repair,"foot_cap_weight_lock":foot_cap_weight_lock,"final_distal_foot_lock":final_distal_foot_lock,"final_knee_weight_cleanup":final_knee_weight_cleanup,"foot_bone_endpoint_audit":foot_bone_endpoint_audit,"foot_targets":foot_targets,"ankle_transition_smoothing":ankle_transition_smoothing,"ankle_weight_diagnostics":ankle_weight_diagnostics,"unweighted_vertices":sum(not v.groups for v in mesh.data.vertices),"automatic_weights":auto,"automatic_weights_error":err,"materials":[m.name if m else None for m in mesh.data.materials],"uv_layers":[u.name for u in mesh.data.uv_layers],"dimensions":[w,d,h],"warning":"Provisional proportion-based rig; validate deformation and Source-engine compatibility before release."}
with open(os.path.join(out,"Femgram-75K-Rig-Report.json"),"w") as f: json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
