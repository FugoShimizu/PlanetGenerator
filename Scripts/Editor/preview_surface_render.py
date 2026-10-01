import builtins
import unreal
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE for editor surface preview")
if globals().get("ARGS",{}).get("restore",False):
    previous=getattr(builtins,"_planet_surface_camera",None)
    if previous:
        editor.set_level_viewport_camera_info(*previous)
        del builtins._planet_surface_camera
    RESULT={"restored":previous is not None}
else:
    found=[a for a in unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(),unreal.PlanetTerrain) if a.get_actor_label()=="Planet_SurfaceRenderProbe"]
    if len(found)!=1 or found[0].visible_surface_radius_meters<=0:
        raise RuntimeError("Run successful surface render probe first")
    terrain=found[0]
    if not hasattr(builtins,"_planet_surface_camera"):
        builtins._planet_surface_camera=editor.get_level_viewport_camera_info()
    state=getattr(builtins,"_planet_surface_probe",None)
    if not state or state.get("status")!="passed":
        raise RuntimeError("Complete the static collision probe before moving its visual preview")
    up=terrain.observer_meters/terrain.observer_meters.length()
    forward=unreal.MathLibrary.cross_vector_vector(up,unreal.Vector(0,0,1))
    forward=forward/forward.length()
    if globals().get("ARGS",{}).get("look_downhill",False):
        _,normal=terrain.evaluate_direction(up)
        downhill=normal-up*unreal.MathLibrary.dot_vector_vector(up,normal)
        if downhill.length()>1e-8: forward=downhill/downhill.length()
    if globals().get("ARGS",{}).get("upright",True):
        # Only this owned transient planet is rotated, after collision tests.
        # Align its local surface with the template lighting for material review.
        basis=unreal.MathLibrary.make_rot_from_xz(forward,up)
        inverse=unreal.MathLibrary.invert_transform(unreal.Transform(rotation=basis))
        terrain.set_actor_transform(inverse,False,False)
        rotated=unreal.MathLibrary.transform_location(terrain.get_actor_transform(),terrain.observer_meters*100)
        terrain.set_actor_location(unreal.Vector(300000,0,160)-rotated,False,False)
    sea_distance=float(globals().get("ARGS",{}).get("sea_distance_m",0))
    if not 0<=sea_distance<=1000: raise RuntimeError("Sea preview distance must be finite and in [0,1000] metres")
    if sea_distance:
        # Visual-only camera on the base sea sphere, after the physical probe.
        target=up+forward*(sea_distance/terrain.radius_meters)
        up=target/target.length()
        terrain.set_editor_property("observer_meters",up*(terrain.radius_meters+1.6))
        terrain.set_editor_property("leaf_budget",32768)
        if not terrain.request_generation(): raise RuntimeError(terrain.generation_message)
    world_up=unreal.MathLibrary.transform_direction(terrain.get_actor_transform(),up)
    world_forward=unreal.MathLibrary.transform_direction(terrain.get_actor_transform(),forward)
    camera=unreal.MathLibrary.transform_location(terrain.get_actor_transform(),terrain.observer_meters*100)
    rotation=unreal.MathLibrary.make_rot_from_xz(world_forward-world_up*0.3,world_up)
    editor.set_level_viewport_camera_info(camera,rotation)
    RESULT={"camera_height_m":1.6,"diagnostic_material_only":False,"template_lighting_only":True,"collision_test_precedes_preview_transform":True,"radius_meters":terrain.radius_meters,"sea_distance_m":sea_distance,"generation_pending":terrain.generation_busy}
