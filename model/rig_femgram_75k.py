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
def p(x,z,y=0): return Vector((cx+x*w,cy+y*d,lo[2]+z*h))
ad=bpy.data.armatures.new("Femgram_ValveBiped"); arm=bpy.data.objects.new("Femgram_ValveBiped",ad); bpy.context.collection.objects.link(arm)
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
 add(foot,p(sgn*.105,.075),p(sgn*.105,.025,-.16),calf,True)
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
 if z < .535:
  return [n for n,a,b in segments if n in ("Pelvis","Spine") or ("_"+n.split("_")[-1] in ("Thigh","Calf","Foot") and n.startswith(side+"_"))]
 if z > .785:
  return [n for n,a,b in segments if n in ("Neck","Head")]
 if z >= .615 and abs(x) > .115:
  names=[n for n,a,b in segments if n in ("Spine2",side+"_Clavicle",side+"_UpperArm",side+"_Forearm",side+"_Hand") or n.startswith(side+"_Finger")]
  if abs(x) < .405: names=[n for n in names if "Finger" not in n]
  return names
 return [n for n,a,b in segments if n in ("Pelvis","Spine","Spine1","Spine2","Neck","Head","L_Clavicle","R_Clavicle")]
segmap={n:(a,b) for n,a,b in segments}
for v in mesh.data.vertices:
 xw=mesh.matrix_world@v.co
 nx=(xw.x-cx)/w
 nz=(xw.z-lo[2])/h
 candidates=allowed_bones(nx,nz)
 if not candidates: candidates=[n for n,a,b in segments]
 near=sorted((distseg(xw,*segmap[n]),n) for n in candidates)[:3]
 vals=[(n,1.0/((dist+0.035)**2.5)) for dist,n in near]
 total=sum(weight for n,weight in vals)
 for n,weight in vals:
  groups[n].add([v.index],weight/total,"REPLACE")
mod=mesh.modifiers.new("Femgram Armature","ARMATURE")
mod.object=arm
mesh.parent=arm
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
