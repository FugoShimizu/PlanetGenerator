"""Release only the surface probe actors and restore its preview camera."""
import builtins
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE before surface probe cleanup")
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = editor.get_editor_world()
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
state = getattr(builtins, "_planet_surface_probe", None)
if state and state.get("handle") is not None:
    unreal.unregister_slate_post_tick_callback(state["handle"])
    state["handle"] = None
previous = getattr(builtins, "_planet_surface_camera", None)
if previous:
    editor.set_level_viewport_camera_info(*previous)
    del builtins._planet_surface_camera
removed = []
for cls, label in ((unreal.PlanetGroundPatch, "Planet_SurfaceRenderGround"),
                   (unreal.PlanetTerrain, "Planet_SurfaceRenderProbe")):
    for actor in unreal.GameplayStatics.get_all_actors_of_class(world, cls):
        if actor.get_actor_label() == label:
            if not actors.destroy_actor(actor):
                raise RuntimeError("Failed to destroy owned probe actor: " + label)
            removed.append(label)
if hasattr(builtins, "_planet_surface_probe"):
    del builtins._planet_surface_probe
RESULT = {"cleaned": True, "removed_labels": removed, "camera_restored": previous is not None}
