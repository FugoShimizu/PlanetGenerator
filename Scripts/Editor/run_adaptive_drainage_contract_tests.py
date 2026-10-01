"""Request adaptive drainage contracts in the active Unreal Editor."""

import unreal

levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)

if levels.is_in_play_in_editor():
	raise RuntimeError("Stop PIE before adaptive drainage contracts.")

world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()

unreal.SystemLibrary.execute_console_command(world, "Automation RunTests PlanetGenerator.AdaptiveDrainage")

RESULT = { "requested": "PlanetGenerator.AdaptiveDrainage", "completed": False, "verify_in_current_editor_log": True }
