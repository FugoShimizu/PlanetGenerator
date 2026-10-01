"""UE の C++ 検査を要求する補助処理，完了判定には起動中のログを用いる"""

import unreal

# エディタの実行環境へ渡す処理の為，独立プロセスの起動点は設けない
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("C++ 検査を開始する前に PIE を停止して下さい")

paths = [
	"PlanetGenerator." + name for name in (
		"Aeolian.Contracts",
		"Biome.Contracts",
		"Celestial.Contracts",
		"Climate.Contracts",
		"Drainage.Contracts",
		"Drainage.CoastalOutlets",
		"Geology.IndexContracts",
		"Geology.MesoReliefContracts",
		"Ground.Contracts",
		"Lod.Contracts",
		"Lod.Incremental",
		"Lod.WorkerIsolation",
		"MoistureTransport.Contracts",
		"Ocean.Contracts",
		"RiverTerrain.Contracts",
		"RiverTerrain.EndpointNormals",
		"SurfaceView.Contracts",
		"FoliageLod.Contracts",
		"StreamingFocus.Contracts",
		"SurfaceMaterial.Contracts",
		"OceanSurface.Contracts",
		"SeasonalMap.Contracts",
		"Scatter.Contracts",
		"Runoff.Contracts",
		"TreePalette.Contracts",
		"FluvialErosion.Contracts",
		"ErodedTerrain.Contracts",
		"ErosionIntegration.Contracts",
		"FinalDrainage.Survey",
		"RiverWater.Contracts",
		"RiverWater.DetachedWater",
		"RiverWater.GlobalSeaGrid",
		"RiverWater.SubmergedCenterRoutes",
		"WaterCoverage.Contracts",
		"WaterLod.Contracts"
	)
]

requested = globals().get("ARGS", {}).get("test_path", "PlanetGenerator.")

if requested != "PlanetGenerator." and requested not in paths:
	raise RuntimeError("指定した惑星検査は登録されていません")

world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
unreal.SystemLibrary.execute_console_command(world, "Automation RunTests " + requested)

RESULT = {
	"requested_prefix": requested,
	"completed": False,
	"expected_paths": paths if requested == "PlanetGenerator." else [requested],
	"verify_every_completion_in_current_editor_log": True
}
