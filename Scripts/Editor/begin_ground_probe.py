"""Probe real UE collision independently of the planet's display LOD."""
import builtins
import math
import json
import time
import unreal
runtime=bool(globals().get("ARGS",{}).get("game_world",False))
cells=64 if runtime else 16
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if levels.is_in_play_in_editor()!=runtime:
    raise RuntimeError("Start PIE for runtime probe; stop PIE for editor probe")
old=getattr(builtins,"_planet_ground_probe",None)
if old and old.get("status")=="running":
    raise RuntimeError("Ground probe already running")
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world=editor.get_game_world() if runtime else editor.get_editor_world()
if world is None:
    raise RuntimeError("Requested world is not ready")
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
def get_actor(cls,label,location):
    found=[a for a in unreal.GameplayStatics.get_all_actors_of_class(world,cls) if a.get_actor_label()==label]
    if len(found)>1:
        raise RuntimeError("Duplicate probe actor")
    if found:
        a=found[0]
    elif runtime:
        a=unreal.PlanetGroundPatch.spawn_probe_actor(world,cls,location)
        if a is None:
            raise RuntimeError("Runtime spawn failed")
    else:
        a=actors.spawn_actor_from_class(cls,location,transient=True)
    a.set_actor_label(label)
    return a
terrain=get_actor(unreal.PlanetTerrain,"Planet_GroundProbeSource",unreal.Vector(0,0,1e10))
terrain.set_editor_property("follow_player_camera",False)
terrain.set_editor_property("leaf_budget",512)
terrain.set_editor_property("seed",42)
terrain.set_editor_property("radius_meters",1000000.0)
terrain.set_editor_property("surface_material",unreal.load_asset("/Game/Planet/Materials/M_PlanetGeologyDiagnostic"))
ground=get_actor(unreal.PlanetGroundPatch,"Planet_GroundProbe",unreal.Vector(0,0,0))
ground.set_editor_property("terrain",terrain)
ground.set_editor_property("half_extent_meters",8.0)
ground.set_editor_property("cells_per_side",cells)
state={"status":"running","phase":"terrain","start":time.monotonic(),"handle":None,"terrain":terrain,"ground":ground,
       "traces":0,"ray_regressions":0,"native_ray_misses":0,"ray_diagnostics":[],"capsule_sweeps":0,"max_capsule_error_m":0,"max_error_m":0,"old_ground_frames":0,"request_ms":[],"patches":[]}
builtins._planet_ground_probe=state
if not terrain.request_generation():
    raise RuntimeError(terrain.generation_message)
state["terrain_revision"]=terrain.requested_revision

def unit(v):
    n=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
    return v/n

def request(anchor):
    ground.set_editor_property("anchor_direction",anchor)
    start=time.perf_counter()
    if not ground.request_patch():
        raise RuntimeError(ground.status)
    state["request_ms"].append((time.perf_counter()-start)*1000)
    state["target"]=ground.requested_revision

def expected(anchor,x,y):
    n=unit(anchor)
    axis=unreal.Vector(0,0,1) if abs(n.z)<0.8 else unreal.Vector(1,0,0)
    east=unit(unreal.MathLibrary.cross_vector_vector(n,axis))
    north=unreal.MathLibrary.cross_vector_vector(n,east)
    d=unit(n+east*(x/1000000.0)+north*(y/1000000.0))
    h,_=terrain.evaluate_direction(d)
    return terrain.get_actor_location()+d*((1000000.0+h)*100),d

def trace(point,direction):
    if ground.native_trace_ground(point+direction*1000,point-direction*1000) is None:
        state["native_ray_misses"]+=1
        if len(state["ray_diagnostics"])<6:
            report=json.loads(ground.inspect_ground_ray(point+direction*1000,point-direction*1000))
            if not report["candidate_hit"] or not report["brute_hit"]:
                raise RuntimeError("Candidate query disagrees with cooked-triangle reference")
            state["ray_diagnostics"].append(report)
    result=ground.trace_ground(point+direction*1000,point-direction*1000)
    if result is None:
        raise RuntimeError("Missing physical collision")
    hit,normal=result
    error=(hit-point).length()/100
    if error>0.02 or unreal.MathLibrary.dot_vector_vector(normal,direction)<=0:
        raise RuntimeError(f"Ground collision mismatch: {error} m")
    state["max_error_m"]=max(state["max_error_m"],error)
    state["traces"]+=1

def inspect():
    for x,y,should_hit in ((-7.0,-4.62,True),(0.0,0.0,True),(0.41,0.72,True),(9.0,0.3,False)):
        point,direction=expected(ground.anchor_direction,x,y)
        for length in (100.0,1000.0,10000.0,100000.0):
            start=point+direction*length
            end=point-direction*length
            diagnostic=json.loads(ground.inspect_ground_ray(start,end))
            result=ground.trace_ground(start,end)
            if diagnostic["brute_hit"]!=should_hit or diagnostic["candidate_hit"]!=should_hit or (result is not None)!=should_hit:
                raise RuntimeError(f"Ray regression differs from actual triangles: {x}, {y}, {length}, {diagnostic}")
            state["ray_regressions"]+=1
    for i in range(48):
        # Irregular points do not coincide with the builder's midpoint probes.
        x=14*((i*0.618033988749895)%1-0.5)
        y=14*((i*0.414213562373095+0.17)%1-0.5)
        p,d=expected(ground.anchor_direction,x,y)
        trace(p,d)
        sweep=ground.sweep_ground_capsule(p+d*300,p-d*100,d,34.0,96.0)
        if sweep is None:
            raise RuntimeError("Physical body-sized capsule missed ground")
        impact,normal=sweep
        relative=impact-terrain.get_actor_location()
        hit_direction=unit(relative)
        height,_=terrain.evaluate_direction(hit_direction)
        error=abs(relative.length()/100-1000000.0-height)
        if error>0.02 or unreal.MathLibrary.dot_vector_vector(normal,hit_direction)<=0:
            raise RuntimeError(f"Physical capsule mismatch: {error} m")
        state["capsule_sweeps"]+=1
        state["max_capsule_error_m"]=max(state["max_capsule_error_m"],error)
    state["patches"].append({"revision":ground.published_revision,"worker_ms":ground.last_worker_milliseconds,
                             "construction_probe_error_m":ground.maximum_probe_error_meters})

def finish(error=None):
    state["status"]="failed" if error else "passed"
    if state["handle"] is not None:
        unreal.unregister_slate_post_tick_callback(state["handle"])
        state["handle"]=None
    state["result"]={k:state[k] for k in ("status","traces","ray_regressions","native_ray_misses","ray_diagnostics","capsule_sweeps","max_capsule_error_m","max_error_m","old_ground_frames","request_ms","patches")}
    state["result"].update({"error":error,"seconds":time.monotonic()-state["start"],"physics_world":"PIE" if runtime else "Editor","cells_per_side":cells,
        "runtime_async_cook_validated":runtime and not error,"walking_validated":False,"render_collision_match_validated":False,
        "saved_terrain_assets":False})
    state.pop("terrain",None)
    state.pop("ground",None)

def tick(delta):
    try:
        if time.monotonic()-state["start"]>120:
            raise RuntimeError("Ground probe timed out")
        terrain.poll_generation(); ground.poll_patch()
        if state["phase"]=="terrain":
            if terrain.published_revision!=state["terrain_revision"] or terrain.generation_busy:
                return
            state["phase"]="first"
            request(unreal.Vector(1,2,3))
            return
        if state["phase"]=="second" and ground.published_revision==state["previous"]:
            trace(*state["old_point"])
            state["old_ground_frames"]+=1
        if ground.published_revision!=state["target"] or ground.busy:
            if not ground.busy and ground.published_revision!=state["target"]:
                raise RuntimeError(ground.status)
            return
        inspect()
        if state["phase"]=="first":
            state["old_point"]=expected(ground.anchor_direction,0.31,-0.67)
            state["previous"]=ground.published_revision
            state["phase"]="second"
            request(unreal.Vector(-3,-1,2))
        else:
            revision=ground.published_revision
            ground.set_editor_property("cells_per_side",0)
            if ground.request_patch() or ground.published_revision!=revision:
                raise RuntimeError("Invalid request damaged ground")
            ground.set_editor_property("cells_per_side",cells)
            trace(*expected(ground.anchor_direction,0.12,0.28))
            if runtime and state["old_ground_frames"]==0:
                raise RuntimeError("No pending collision frame observed; runtime retention not tested")
            finish()
    except Exception as error:
        finish(str(error))
state["handle"]=unreal.register_slate_post_tick_callback(tick)
RESULT={"status":"started","read_result_with":"Scripts/Editor/read_ground_probe.py"}
