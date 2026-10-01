"""Inspect actual float render vertices and independent UE ground collision."""
import builtins
import math
import time
import struct
import unreal
args=globals().get("ARGS",{})
previous=getattr(builtins,"_planet_surface_probe",None)
if previous and previous.get("status")=="running":
    raise RuntimeError("Surface probe already running")
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE for this editor mesh probe")
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
for cls,label in ((unreal.PlanetTerrain,"Planet_SurfaceRenderProbe"),(unreal.PlanetGroundPatch,"Planet_SurfaceRenderGround")):
    for old in unreal.GameplayStatics.get_all_actors_of_class(world,cls):
        if old.get_actor_label()==label:
            actors.destroy_actor(old)
radius=float(args.get("radius_meters",1000000))
terrain=actors.spawn_actor_from_class(unreal.PlanetTerrain,unreal.Vector(0,0,1e10),transient=True)
terrain.set_actor_label("Planet_SurfaceRenderProbe")
terrain.set_editor_property("follow_player_camera",False)
terrain.set_editor_property("leaf_budget",int(args.get("leaf_budget",32768)))
terrain.set_editor_property("maximum_level",32)
terrain.set_editor_property("seed",42)
terrain.set_editor_property("radius_meters",radius)
terrain.set_editor_property("observer_meters",unreal.Vector(0,0,radius*1.1))
terrain.set_editor_property("surface_material",unreal.load_asset("/Game/Planet/Materials/M_PlanetSurface"))
ground=actors.spawn_actor_from_class(unreal.PlanetGroundPatch,unreal.Vector(),transient=True)
ground.set_actor_label("Planet_SurfaceRenderGround")
ground.set_editor_property("terrain",terrain)
ground.set_editor_property("half_extent_meters",16.0)
ground.set_editor_property("cells_per_side",32)
state={"status":"running","phase":"field","start":time.monotonic(),"handle":None,"terrain":terrain,"ground":ground,
       "old_mesh_frames":0,"snapshots":[],"target":0}
builtins._planet_surface_probe=state

def unit(v):
    return v/v.length()

def dot(a,b):
    return unreal.MathLibrary.dot_vector_vector(a,b)

def cross(a,b):
    return unreal.MathLibrary.cross_vector_vector(a,b)

def request():
    if not terrain.request_generation():
        raise RuntimeError(terrain.generation_message)
    state["target"]=terrain.requested_revision
request()

def finish(error=None):
    state["status"]="failed" if error else "passed"
    if state["handle"] is not None:
        unreal.unregister_slate_post_tick_callback(state["handle"])
        state["handle"]=None
    state["result"]={"status":state["status"],"error":error,"seconds":time.monotonic()-state["start"],
        "radius_meters":radius,"coast":bool(args.get("coast",False)),"river_reach_index":args.get("river_reach_index"),"river_fraction":args.get("river_fraction",0.5),"old_mesh_frames":state["old_mesh_frames"],"snapshots":state["snapshots"],
        "physics_world":"Editor","finite_mesh_samples_only":True,"gpu_position_conversion_emulated":True,"gpu_readback":False,"walking_rendered_terrain_validated":False,
        "morph_validated":False,"saved_terrain_assets":False,"inadequate_budget_rejected":state.get("budget_rejected",False)}
    state.pop("terrain",None)
    state.pop("ground",None)

def inspect():
    mesh=terrain.get_visible_mesh()
    if mesh is None or not mesh.is_mesh_section_visible(0) or terrain.visible_surface_radius_meters!=24:
        raise RuntimeError("No published mandatory surface mesh")
    vertices,indices,normals,uvs,tangents=unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh,0)
    if len(vertices)!=terrain.visible_vertex_count or len(indices)!=3*terrain.visible_triangle_count:
        raise RuntimeError("Published mesh counters differ from actual buffers")
    material=mesh.get_material(0)
    if not isinstance(material,unreal.MaterialInstanceDynamic):
        raise RuntimeError("Published terrain lacks its own texture frame material")
    vectors={name:material.get_vector_parameter_value(name) for name in
             ("TextureOrigin","TextureAxisX","TextureAxisY","TextureAxisZ")}
    axes=[unreal.Vector(vectors[name].r,vectors[name].g,vectors[name].b) for name in
          ("TextureAxisX","TextureAxisY","TextureAxisZ")]
    for i in range(3):
        for j in range(3):
            if abs(dot(axes[i],axes[j])-(1 if i==j else 0))>1e-6:
                raise RuntimeError("Material local frame is not orthonormal")
    relative=mesh.get_relative_transform().translation/100
    phase=vectors["TextureOrigin"]
    for coordinate,actual in zip((relative.x,relative.y,relative.z),(phase.r,phase.g,phase.b)):
        error=coordinate%512-actual
        error-=round(error/512)*512
        if abs(error)>0.0002:
            raise RuntimeError("Published material phase differs from its mesh origin")
    # UE 5.8 GetSection returns FProcMeshVertex FVector (double). Its scene
    # proxy converts Position to FVector3f. Mirror that conversion explicitly;
    # this is not GPU readback or a test of shader arithmetic.
    def f32(v):
        return struct.unpack("<f",struct.pack("<f",v))[0]
    vertices=[unreal.Vector(f32(v.x),f32(v.y),f32(v.z)) for v in vertices]
    edges={}
    nearby=[]
    focus_world=unreal.MathLibrary.transform_location(terrain.get_actor_transform(),terrain.visible_surface_observer_meters*100)
    focus_local=unreal.MathLibrary.inverse_transform_location(mesh.get_world_transform(),focus_world)
    for i in range(0,len(indices),3):
        ids=[int(indices[i+k]) for k in range(3)]
        a,b,c=[vertices[j] for j in ids]
        if cross(b-a,c-a).length()==0:
            raise RuntimeError("Collapsed float render triangle")
        for k in range(3):
            u,v=ids[k],ids[(k+1)%3]
            key=(min(u,v),max(u,v))
            count,sign=edges.get(key,(0,0))
            edges[key]=(count+1,sign+(1 if u<v else -1))
        if min(a.x,b.x,c.x)<=focus_local.x+2500 and max(a.x,b.x,c.x)>=focus_local.x-2500 and min(a.y,b.y,c.y)<=focus_local.y+2500 and max(a.y,b.y,c.y)>=focus_local.y-2500:
            nearby.append((a,b,c))
    if any(count!=2 or sign!=0 for count,sign in edges.values()):
        raise RuntimeError("Open render edge or unmatched orientation")
    if len(vertices)+len(indices)//3!=len(edges)+2:
        raise RuntimeError("Render mesh Euler characteristic is not a sphere")
    transform=mesh.get_world_transform()
    planet_transform=terrain.get_actor_transform()
    anchor=unit(terrain.visible_surface_observer_meters)
    axis=unreal.Vector(0,0,1) if abs(anchor.z)<0.8 else unreal.Vector(1,0,0)
    east=unit(cross(anchor,axis))
    north=cross(anchor,east)
    max_h_error=max_collision_delta=0
    for i in range(49):
        x=26*((i*0.618033988749895)%1-0.5)
        y=26*((i*0.414213562373095+0.17)%1-0.5)
        d=unit(anchor+east*(x/radius)+north*(y/radius))
        h,_=terrain.evaluate_direction(d)
        point=unreal.MathLibrary.transform_location(planet_transform,d*((radius+h)*100))
        start=unreal.MathLibrary.inverse_transform_location(transform,point+d*1000)
        end=unreal.MathLibrary.inverse_transform_location(transform,point-d*1000)
        ray=end-start
        hits=[]
        for a,b,c in nearby:
            e1=b-a; e2=c-a; p=cross(ray,e2); det=dot(e1,p)
            if abs(det)<1e-12:
                continue
            s=start-a; u=dot(s,p)/det
            if u < -1e-10 or u > 1+1e-10:
                continue
            q=cross(s,e1); v=dot(ray,q)/det
            if v < -1e-10 or u+v > 1+1e-10:
                continue
            t=dot(e2,q)/det
            if 0<=t<=1:
                hits.append(start+ray*t)
        if not hits:
            raise RuntimeError(f"No render triangle at ground sample {i}")
        hit=unreal.MathLibrary.transform_location(transform,hits[0])
        h_error=(hit-point).length()/100
        physical=ground.trace_ground(point+d*1000,point-d*1000)
        if physical is None:
            raise RuntimeError("Physical ground missing under displayed sample")
        difference=(physical[0]-hit).length()/100
        max_h_error=max(max_h_error,h_error)
        max_collision_delta=max(max_collision_delta,difference)
        if h_error>0.02 or difference>0.04:
            raise RuntimeError(f"Surface mismatch: H={h_error}m, collision={difference}m")
    ocean=terrain.get_visible_ocean_mesh()
    if ocean is None or not ocean.is_mesh_section_visible(0): raise RuntimeError("Ocean missing from published pair")
    water_vertices,water_indices,_,_,_=unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(ocean,0)
    if len(water_indices)!=3*terrain.visible_ocean_triangle_count: raise RuntimeError("Ocean counter differs from actual buffer")
    water_edges={}
    for i in range(0,len(water_indices),3):
        tri=water_indices[i:i+3]
        for k in range(3):
            a,b=int(tri[k]),int(tri[(k+1)%3]);key=(min(a,b),max(a,b));count,sign=water_edges.get(key,(0,0))
            water_edges[key]=(count+1,sign+(1 if a<b else -1))
    if any(count!=2 or sign!=0 for count,sign in water_edges.values()): raise RuntimeError("Open ocean edge")
    parents=list(range(len(water_vertices)))
    def root_index(i):
        while parents[i]!=i:
            parents[i]=parents[parents[i]]; i=parents[i]
        return i
    for a,b in water_edges:
        parents[root_index(a)]=root_index(b)
    if len({root_index(i) for i in range(len(parents))})!=1: raise RuntimeError("Disconnected ocean mesh components")
    if len(water_vertices)+len(water_indices)//3!=len(water_edges)+2: raise RuntimeError("Ocean does not form one complete sphere")
    water_transform=ocean.get_relative_transform()
    radial_error=max(abs(unreal.MathLibrary.transform_location(water_transform,v).length()/100-radius) for v in water_vertices)
    if radial_error>0.00002: raise RuntimeError("Ocean vertices do not lie on constant sphere")
    water_material=ocean.get_material(0)
    if not isinstance(water_material,unreal.MaterialInstanceDynamic): raise RuntimeError("Ocean frame material missing")
    water_phase=water_material.get_vector_parameter_value("TextureOrigin");wo=water_transform.translation/100
    for actual,expected in zip((water_phase.r,water_phase.g,water_phase.b),(wo.x,wo.y,wo.z)):
        if abs(math.remainder(actual-expected%512,512))>0.0002: raise RuntimeError("Ocean texture frame mismatch")
    state["snapshots"].append({"revision":terrain.published_revision,"material":material.get_path_name(),"texture_frame_verified":True,"ocean_triangles":len(water_indices)//3,"ocean_radial_error_m":radial_error,"ocean_build_ms":terrain.last_ocean_build_milliseconds,"ocean_float_error_m":terrain.maximum_ocean_float_error_meters,"vertices":len(vertices),"triangles":len(indices)//3,
        "maximum_level":terrain.visible_maximum_level,"leaf_count":terrain.visible_leaf_count,"nearby_triangles":len(nearby),
        "independent_ray_samples":49,"maximum_h_error_m":max_h_error,"maximum_collision_difference_m":max_collision_delta,
        "generation_message":terrain.generation_message,"worker_ms":terrain.last_worker_milliseconds,"upload_ms":terrain.last_upload_milliseconds,
        "float_error_m":terrain.maximum_surface_float_error_meters,"construction_probe_error_m":terrain.maximum_surface_probe_error_meters})

def tick(delta):
    try:
        if time.monotonic()-state["start"]>180:
            raise RuntimeError("Surface render probe timed out")
        terrain.poll_generation(); ground.poll_patch()
        if terrain.published_revision and terrain.generation_busy:
            old=terrain.get_visible_mesh()
            if old is None or not old.is_mesh_section_visible(0):
                raise RuntimeError("Previous render coverage disappeared")
            state["old_mesh_frames"]+=1
        if terrain.generation_busy:
            return
        if state["phase"]=="reject":
            old=terrain.get_visible_mesh()
            if terrain.published_revision!=state["retained_revision"] or old is None or not old.is_mesh_section_visible(0):
                raise RuntimeError("Insufficient near budget replaced valid terrain")
            state["budget_rejected"]=True
            terrain.set_editor_property("leaf_budget",int(args.get("leaf_budget",32768)))
            finish()
            return
        if terrain.published_revision!=state["target"]:
            raise RuntimeError(terrain.generation_message)
        if state["phase"]=="field":
            if args.get("coast",False):
                positive=negative=None
                for i in range(256):
                    z=1-2*(i+0.5)/256; angle=i*2.399963229728653
                    candidate=unreal.Vector(math.sqrt(1-z*z)*math.cos(angle),math.sqrt(1-z*z)*math.sin(angle),z)
                    height,_=terrain.evaluate_direction(candidate)
                    if height>0: positive=candidate
                    elif height<0: negative=candidate
                    if positive is not None and negative is not None: break
                if positive is None or negative is None: raise RuntimeError("No actual coast bracket")
                for _ in range(56):
                    candidate=unit(positive+negative); height,_=terrain.evaluate_direction(candidate)
                    if height>0: positive=candidate
                    else: negative=candidate
                d=positive
            elif "river_reach_index" in args:
                profile=terrain.evaluate_river_profile(int(args["river_reach_index"]),float(args.get("river_fraction",0.5)))
                if profile is None:
                    raise RuntimeError("Invalid river reach for render/collision probe")
                d=unit(profile[0])
            else:
                d=unit(unreal.Vector(1,2,3))
            h,_=terrain.evaluate_direction(d)
            terrain.set_actor_location(unreal.Vector(0,0,500000)-d*((radius+h)*100),False,False)
            terrain.set_editor_property("observer_meters",d*(radius+h+1.6))
            ground.set_editor_property("anchor_direction",d)
            if not ground.request_patch():
                raise RuntimeError(ground.status)
            state["phase"]="near"
            request()
            return
        if ground.busy:
            return
        if ground.published_revision!=ground.requested_revision:
            raise RuntimeError(ground.status)
        inspect()
        if state["phase"]=="near":
            d=unit(terrain.observer_meters)
            east=unit(cross(d,unreal.Vector(0,0,1)))
            if args.get("coast",False):
                _,normal=terrain.evaluate_direction(d)
                gradient=d*dot(normal,d)-normal
                if gradient.length()>1e-8: east=unit(gradient)
            next_d=unit(d+east*(7/radius))
            h,_=terrain.evaluate_direction(next_d)
            terrain.set_editor_property("observer_meters",next_d*(radius+h+1.6))
            ground.set_editor_property("anchor_direction",next_d)
            if not ground.request_patch():
                raise RuntimeError(ground.status)
            state["phase"]="moved"
            request()
        else:
            state["retained_revision"]=terrain.published_revision
            state["phase"]="reject"
            terrain.set_editor_property("leaf_budget",20)
            request()
    except Exception as error:
        finish(str(error))
state["handle"]=unreal.register_slate_post_tick_callback(tick)
RESULT={"status":"started","read_result_with":"Scripts/Editor/read_surface_render_probe.py"}
