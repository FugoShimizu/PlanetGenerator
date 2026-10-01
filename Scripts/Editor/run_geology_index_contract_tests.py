import unreal
world=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
unreal.SystemLibrary.execute_console_command(world,"Automation RunTests PlanetGenerator.Geology.IndexContracts")
RESULT={"requested":"PlanetGenerator.Geology.IndexContracts","completed":False,"verify_in_current_editor_log":True}
