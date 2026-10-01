"""Short editor actions. Executed in UE by Scripts/UE/ue_control.py."""
import math
import os
import sys
import unreal


def vector(value):
    return [value.x, value.y, value.z]


def rotation(value):
    return [value.pitch, value.yaw, value.roll]


def status():
    editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    world = editor.get_editor_world()
    camera = editor.get_level_viewport_camera_info()
    game = editor.get_game_world()
    pawn = unreal.GameplayStatics.get_player_pawn(game, 0) if game else None
    return {
        "pid": os.getpid(), "python": sys.version.split()[0],
        "world": world.get_path_name() if world else None,
        "actor_count": len(actors.get_all_level_actors()),
        "selected_actors": [a.get_actor_label() for a in actors.get_selected_level_actors()][:20],
        "camera": {"location": vector(camera[0]), "rotation": rotation(camera[1])} if camera else None,
        "pie_active": levels.is_in_play_in_editor(),
        "game_world": game.get_path_name() if game else None,
        "player_pawn": pawn.get_path_name() if pawn else None,
        "player_location": vector(pawn.get_actor_location()) if pawn else None,
    }


def smoke():
    editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if levels.is_in_play_in_editor():
        raise RuntimeError("Stop PIE before the editor smoke check.")
    before = status()
    location, facing = editor.get_level_viewport_camera_info()
    selected = list(actors.get_selected_level_actors())
    actor = None
    result = {}
    try:
        actor = actors.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, -100000), transient=True)
        if not actor:
            raise RuntimeError("Transient actor creation failed")
        actor.set_actor_label("AutomationProbe_Transient")
        mesh = unreal.load_asset("/Engine/BasicShapes/Cube.Cube")
        component = actor.get_component_by_class(unreal.StaticMeshComponent)
        if not mesh or not component.set_static_mesh(mesh):
            raise RuntimeError("Mesh assignment failed")
        target = unreal.Vector(12, 34, -100000)
        actor.set_actor_location(target, False, False)
        if (actor.get_actor_location() - target).length() > 0.001:
            raise RuntimeError("Actor transform readback differs")
        actors.set_selected_level_actors([actor])
        if actor not in actors.get_selected_level_actors():
            raise RuntimeError("Actor selection failed")
        shifted = location + unreal.Vector(10, 0, 0)
        editor.set_level_viewport_camera_info(shifted, facing)
        readback = editor.get_level_viewport_camera_info()
        if not readback or (readback[0] - shifted).length() > 0.001:
            raise RuntimeError("Viewport camera readback differs")
        result = {"transient_actor_created": True, "mesh_assigned": True, "transform_verified": True,
                  "selection_verified": True, "viewport_camera_verified": True}
    finally:
        if actor and not actors.destroy_actor(actor):
            raise RuntimeError("Could not remove the transient test actor")
        actors.set_selected_level_actors(selected)
        editor.set_level_viewport_camera_info(location, facing)
        levels.editor_invalidate_viewports()
    after = status()
    if before["actor_count"] != after["actor_count"] or before["camera"] != after["camera"]:
        raise RuntimeError("Smoke check did not restore the editor state")
    result.update(actor_removed=True, selection_restored=before["selected_actors"] == after["selected_actors"],
                  camera_restored=True, assets_saved=False, before=before, after=after)
    return result


def main(args):
    action = args.get("action", "status")
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if action == "status":
        return status()
    if action == "smoke":
        return smoke()
    if action == "camera":
        values = [float(v) for v in args["location"] + args["rotation"]]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise ValueError("camera needs finite location [x,y,z] and rotation [pitch,yaw,roll]")
        if levels.is_in_play_in_editor():
            raise RuntimeError("camera changes the editor viewport; stop PIE first")
        editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        # UE Python Rotator positional arguments are roll/pitch/yaw; use explicit names.
        editor.set_level_viewport_camera_info(unreal.Vector(*values[:3]), unreal.Rotator(pitch=values[3], yaw=values[4], roll=values[5]))
        levels.editor_invalidate_viewports()
        result = status()
        camera = result["camera"]
        if not camera or any(abs(a - b) > 0.001 for a, b in zip(camera["location"], values[:3])):
            raise RuntimeError("Camera position readback differs from the request")
        if any(abs((a - b + 180) % 360 - 180) > 0.0001 for a, b in zip(camera["rotation"], values[3:])):
            raise RuntimeError("Camera rotation readback differs from the request")
        return result
    if action == "play":
        if levels.is_in_play_in_editor():
            raise RuntimeError("PIE is already active")
        levels.editor_request_begin_play()
        return {"requested": "play", "completed": False, "note": "Confirm pie_active and player_pawn using status after editor ticks."}
    if action == "stop":
        levels.editor_request_end_play()
        return {"requested": "stop", "completed": False, "note": "Confirm pie_active is false using status after editor ticks."}
    raise ValueError(f"Unknown action: {action}")


RESULT = main(ARGS)
