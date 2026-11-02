import bpy, os, json
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
# Measure each actual foot region before creating bones. The foot tail is placed
# at the mesh toe extremity rather than using an arbitrary whole-body depth.
foot_targets={}
for side_name, sign in (("L",1),("R",-1)):
    foot_points=[q for q in vs
                 if ((q.x-cx)/w)*sign > 0.025
                 and (q.z-lo[2])/h < 0.075]
    if not foot_points:
        raise RuntimeError("Could not locate "+side_name+" foot geometry")
    # Use the foot-region centroid for the bone axis instead of the single
    # most-forward vertex. The latter can be a stray toe/mesh spike and made
    # the foot bone as long as the entire calf in the previous build.
    # Find the toe from a robust front-edge percentile, not the centroid
    # of the whole foot. A few stray vertices must not stretch the bone.
    front_depths=sorted(q.y for q in foot_points)
    toe_cut=front_depths[max(0,int((len(front_depths)-1)*0.08))]
    toe_points=[q for q in foot_points if q.y <= toe_cut]
    foot_targets[side_name]={
        "x": (sum(q.x for q in toe_points)/len(toe_points)-cx)/w,
        "z": (sum(q.z for q in toe_points)/len(toe_points)-lo[2])/h,
        "toe_y": (sum(q.y for q in toe_points)/len(toe_points)-cy)/d,
        "toe_sample_vertices": len(toe_points)
    }
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
 add(thigh,p(sgn*.105,.49),p(sgn*.105,.265),"Pelvis")
 add(calf,p(sgn*.105,.265),p(sgn*.105,.075),thigh,True)
 # The ankle is exactly the calf endpoint. The short foot bone follows the
 # measured ankle-to-toe direction and ends at the actual toe extremity.
 target=foot_targets[s]
 add(foot,p(sgn*.105,.075),p(target["x"],target["z"],target["toe_y"]),calf,True)
 clav=s+"_Clavicle"; upper=s+"_UpperArm"; fore=s+"_Forearm"; hand=s+"_Hand"
 add(clav,p(sgn*.035,.70),p(sgn*.145,.70),"Spine2")
 add(upper,p(sgn*.145,.70),p(sgn*.315,.695),clav,True)
 add(fore,p(sgn*.315,.695),p(sgn*.445,.69),upper,True)
 add(hand,p(sgn*.445,.69),p(sgn*.50,.69),fore,True)
 for i,off in enumerate((-.018,-.009,0,.009,.018)):
  n=s+"_Finger"+str(i); a=p(sgn*.455,.69,off); b=p(sgn*.48,.69,off); c=p(sgn*.50,.69,off)
  add(n,a,b,hand); add(n+"_1",b,c,n,True); add(n+"_2",c,p(sgn*.51,.69,off),n+"_1",True)
bpy.ops.object.mode_set(mode="OBJECT"); arm.show_in_front=True
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
 lateral=smoothstep(.030,.055,ax)*(1.0-smoothstep(.155,.185,ax))
 vertical=1.0-smoothstep(.385,.445,nz)

 # Treat the knee as a real joint zone, not as an incidental height blend.
 # Within this band, exposed leg vertices are fully assigned to the thigh /
 # calf chain: the pelvis must not retain residual influence and pull the
 # kneecap away from the bending joint.
 knee_zone=smoothstep(.185,.215,nz)*(1.0-smoothstep(.315,.345,nz))
 knee_lateral=smoothstep(.025,.050,ax)*(1.0-smoothstep(.155,.180,ax))
 knee_core=max(0.0,min(1.0,knee_zone*knee_lateral))
 leg_mask=max(0.0,min(1.0,max(lateral*vertical,knee_core)))

 if leg_mask > 0.001:
  # The knee pivot is at normalized height .265. Blend only in a narrow
  # anatomical band around that pivot; above it the thigh owns the vertices,
  # below it the calf owns them. At the joint itself both receive 50%.
  # Base anatomical partition: blend smoothly across the knee and ankle.
  # These priors establish which joint owns each height band; distance to the
  # actual bone segment then corrects the distribution for the mesh surface.
  knee_blend=smoothstep(.215,.315,nz)
  foot_blend=1.0-smoothstep(.045,.090,nz)
  priors={
   side+"_Thigh": knee_blend,
   side+"_Calf": (1.0-knee_blend)*(1.0-foot_blend),
   side+"_Foot": (1.0-knee_blend)*foot_blend
  }
  # Proximity correction prevents vertices on the rear/side of the ankle and
  # foot from inheriting weights merely because they share the same Z height.
  # Adjacent bones naturally share influence near the knee and ankle pivots.
  leg_gates={}
  for bone_name, prior in priors.items():
   if prior <= 1e-10:
    continue
   distance=distseg(xw,*segmap[bone_name])
   leg_gates[bone_name]=prior/((distance+0.025)**2.0)
  leg_total=sum(leg_gates.values())
  if leg_total <= 1e-12:
   leg_gates={side+"_Thigh":knee_blend,
              side+"_Calf":(1.0-knee_blend)*(1.0-foot_blend),
              side+"_Foot":(1.0-knee_blend)*foot_blend}
   leg_total=sum(leg_gates.values())
  for n,weight in leg_gates.items():
   groups[n].add([v.index],leg_mask*weight/leg_total,"REPLACE")
  # Pelvis influence is allowed only outside the exposed leg mask. In the
  # knee core leg_mask is 1, so every knee vertex is attached to the leg chain.
  pelvis_weight=1.0-leg_mask
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
mod=mesh.modifiers.new("Femgram Armature","ARMATURE")
mod.object=arm
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
bpy.ops.object.select_all(action="DESELECT"); mesh.select_set(True); arm.select_set(True); bpy.context.view_layer.objects.active=arm
dest=os.path.join(out,"Femgram-75K-ValveBiped-Rigged.fbx")
bpy.ops.export_scene.fbx(filepath=dest,use_selection=True,object_types={"ARMATURE","MESH"},add_leaf_bones=False,bake_anim=False,path_mode="COPY",embed_textures=True,axis_forward="-Z",axis_up="Y")
report={"vertices":len(mesh.data.vertices),"polygons":len(mesh.data.polygons),"bones":len(ad.bones),"vertex_groups":len(mesh.vertex_groups),"weighted_vertices":sum(bool(v.groups) for v in mesh.data.vertices),"unweighted_vertices":sum(not v.groups for v in mesh.data.vertices),"automatic_weights":auto,"automatic_weights_error":err,"materials":[m.name if m else None for m in mesh.data.materials],"uv_layers":[u.name for u in mesh.data.uv_layers],"dimensions":[w,d,h],"warning":"Provisional proportion-based rig; validate deformation and Source-engine compatibility before release."}
with open(os.path.join(out,"Femgram-75K-Rig-Report.json"),"w") as f: json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
