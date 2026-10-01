"""Release only the transient actors and callbacks owned by the streaming probe."""
import builtins
import unreal
state=getattr(builtins,"_planet_stream_probe",None)
RESULT={"cleaned":False}
if state:
    RESULT={"cleaned":True,"previous_status":state.get("status"),"request_ms":state.get("request_ms",[]),
            "result":state.get("result")}
    handle=state.get("handle")
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
    actor=state.get("actor")
    if actor is not None:
        if not unreal.get_editor_subsystem(unreal.EditorActorSubsystem).destroy_actor(actor):
            raise RuntimeError("Could not destroy the transient streaming probe actor")
    camera=state.get("camera_before")
    if camera:
        unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(*camera)
        unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).editor_invalidate_viewports()
        RESULT["camera_restored"]=True
    del builtins._planet_stream_probe
