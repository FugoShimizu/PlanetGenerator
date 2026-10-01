"""既存の走行検査に河川水面を加え，地表と水面の公開順序を観測する"""
import builtins
from pathlib import Path
import runpy
import unreal

world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if world is None:
	raise RuntimeError("探索モードで PIE を開始して下さい")
previous = getattr(builtins, "_planet_river_traversal_observer", None)
if previous and previous.get("handle") is not None:
	raise RuntimeError("前の河川走行検査が続いています")
mode = unreal.GameplayStatics.get_game_mode(world)
original = mode.planet if isinstance(mode, unreal.PlanetGameMode) else None
if original and (original.generation_busy or original.preparation_busy or original.water_generation_busy):
	raise RuntimeError("元の惑星の生成が終わる迄待って下さい")
original_follow = original.follow_player_camera if original else False
if original:
	original.set_editor_property("follow_player_camera", False)
args = dict(globals().get("ARGS", {}), rendered=True, observe_failed_candidate=True)
state: dict = {
	"handle": None, "samples": 0, "earlyFrames": 0, "publications": [],
	"maximumEarlyCount": 0, "parallelFrames": 0, "landAdvancesWhileWaterBusy": 0,
	"waterPublications": [], "error": None, "restored": False
}
setattr(builtins, "_planet_river_traversal_observer", state)


def restore() -> None:
	"""所有する観測だけを止め，元の惑星の追従設定を復元する"""
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
		state["handle"] = None
	current = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
	if original and current == world and unreal.SystemLibrary.is_valid(original):
		original.set_editor_property("follow_player_camera", original_follow)
	state["restored"] = True
	# 終了
	return


def observe(delta_seconds: float) -> None:
	"""既存の走行を変更せず，完成した地表と残した水面を毎フレーム検査する

	Args:
		delta_seconds: Slate の更新間隔，移動入力や経過時間へは使わない
	"""
	_ = delta_seconds
	probe = getattr(builtins, "_planet_character_probe")
	if probe["status"] != "running":
		restore()
		probe["result"]["water_publication_probe"] = {
			key: value for key, value in state.items() if key != "handle"
		}
		# 終了した検査の結果を保持する終了
		return
	try:
		terrain = probe.get("terrain")
		if terrain and terrain.published_revision > 0:
			land_revision = terrain.published_revision
			water_revision = terrain.published_water_revision
			land = terrain.get_visible_mesh()
			water = terrain.get_visible_ocean_mesh()
			if not land or not land.is_mesh_section_visible(0):
				raise RuntimeError("公開済みの地表が消えました")
			if not water or not water.is_mesh_section_visible(0):
				raise RuntimeError("保持すべき水面が消えました")
			if water_revision > land_revision:
				raise RuntimeError("水面が対応する地表より先に公開されました")
			if state["publications"]:
				previous_pair = state["publications"][-1]["revisions"]
				if land_revision < previous_pair[0] or water_revision < previous_pair[1]:
					raise RuntimeError("水面の完了時に公開版が巻戻されました")
				if land_revision > previous_pair[0] and terrain.water_generation_busy:
					state["landAdvancesWhileWaterBusy"] += 1
			state["parallelFrames"] += int(terrain.generation_busy and terrain.water_generation_busy)
			if not state["waterPublications"] or state["waterPublications"][-1]["revision"] != water_revision:
				state["waterPublications"].append({
					"revision": water_revision, "landRevision": land_revision,
					"waterBuildMs": terrain.last_ocean_build_milliseconds,
					"details": terrain.last_published_water_details
				})
			state["samples"] += 1
			state["earlyFrames"] += int(land_revision > water_revision)
			state["maximumEarlyCount"] = max(state["maximumEarlyCount"], terrain.early_surface_publications)
			pair = [land_revision, water_revision]
			if not state["publications"] or state["publications"][-1]["revisions"] != pair:
				state["publications"].append({
					"revisions": pair, "surfaceWorkerMs": terrain.last_surface_worker_milliseconds,
					"workerMs": terrain.last_worker_milliseconds,
					"earlyCount": terrain.early_surface_publications,
					"bothMeshesVisible": True
				})
	except Exception as error:
		if state["error"] is None:
			state["error"] = str(error)
			if probe["phase"] != "failed_candidate":
				namespace["finish"](str(error))
	# 終了
	return


try:
	namespace = runpy.run_path(
		str(Path(unreal.Paths.project_dir()) / "Scripts/Editor/begin_character_probe.py"),
		init_globals={"ARGS": args}
	)
	probe = getattr(builtins, "_planet_character_probe")
	probe["terrain"].set_editor_property("render_river_water", True)
	probe["river_water_enabled"] = True
	state["handle"] = unreal.register_slate_post_tick_callback(observe)
	RESULT = {
		"status": "started", "riverWater": True,
		"read_with": "Scripts/Editor/read_character_probe.py", "otherPlanetPaused": original is not None
	}
except Exception:
	restore()
	raise
