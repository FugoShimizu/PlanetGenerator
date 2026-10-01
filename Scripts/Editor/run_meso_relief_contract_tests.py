"""中距離地形場の C++ 契約検査を UE Automation へ要求する．"""
import unreal

world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
unreal.SystemLibrary.execute_console_command(world, "Automation RunTests PlanetGenerator.Geology.MesoReliefContracts")
RESULT = {"requested": "PlanetGenerator.Geology.MesoReliefContracts", "completed": False,
	"verify_in_current_editor_log": True}
