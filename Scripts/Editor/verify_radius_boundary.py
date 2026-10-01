"""Check the public generation entry point at the supported minimum radius."""
import math
import unreal
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE before the radius boundary check")
found=[a for a in unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(),unreal.PlanetTerrain)
       if a.get_actor_label()=="Planet_SurfaceRenderProbe"]
if len(found)!=1 or found[0].radius_meters!=1000 or found[0].published_revision<=0 or found[0].generation_busy:
    raise RuntimeError("Run the successful radius 1000 surface probe first")
terrain=found[0]
old_radius=terrain.radius_meters
published=terrain.published_revision
requested=terrain.requested_revision
cases=[]
try:
    for radius in (999.999,math.nextafter(1000.0,0.0),0.0,-1.0,float("nan"),float("inf")):
        terrain.set_editor_property("radius_meters",radius)
        accepted=terrain.request_generation()
        retained=terrain.published_revision==published and terrain.requested_revision==requested and not terrain.generation_busy
        if accepted or not retained:
            raise RuntimeError("Invalid radius accepted or existing publication changed")
        cases.append({"radius_m":radius if math.isfinite(radius) else str(radius),"accepted":False,"publication_retained":retained})
    for radius in (1000.0,math.nextafter(1000.0,math.inf)):
        terrain.set_editor_property("radius_meters",radius)
        if not terrain.request_generation():
            raise RuntimeError("Supported boundary radius rejected")
        cases.append({"radius_m":radius,"accepted":True,"admission_only":True})
finally:
    terrain.set_editor_property("radius_meters",old_radius)
RESULT={"status":"passed","minimum_radius_m":1000,"cases":cases,
        "lower_bound_generated_by_surface_probe":True,"nextafter_upper_generation_complete":False}
