import unreal
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
unreal.SystemLibrary.execute_console_command(world,"Automation RunTests PlanetGenerator.RiverTerrain.Contracts")
RESULT={"requested":"PlanetGenerator.RiverTerrain.Contracts","completed":False,"verify_in_current_editor_log":True}
