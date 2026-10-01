"""Request the shared LOD and bounded view-selection contracts inside UE."""
import unreal
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE before the LOD contracts")
unreal.SystemLibrary.execute_console_command(world,"Automation RunTests PlanetGenerator.Lod.Contracts")
RESULT={"requested_test":"PlanetGenerator.Lod.Contracts","completion":"pending; inspect the active automation log"}
