"""Request the project's C++ ocean contracts through UE's automation runner."""
import unreal

levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if levels.is_in_play_in_editor():
    raise RuntimeError("Stop PIE before running ocean contract tests")
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
unreal.SystemLibrary.execute_console_command(world, "Automation RunTests PlanetGenerator.Ocean.Contracts")
RESULT = {"requested_test": "PlanetGenerator.Ocean.Contracts", "completion": "pending; inspect the active UE automation log"}
