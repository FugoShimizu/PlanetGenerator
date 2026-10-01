"""PIE の専用水面ワーカーが地表を待たせず，定義変更で旧候補を公開しない事を検査する"""
import builtins
import math
import time
import unreal

editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = editor.get_game_world()
if world is None:
	raise RuntimeError("探索モードの PIE を開始して下さい")
previous = getattr(builtins, "_planet_water_lifecycle_probe", None)
if previous and previous["status"] == "running":
	raise RuntimeError("前の水面更新検査が実行中です")
mode = unreal.GameplayStatics.get_game_mode(world)
if not isinstance(mode, unreal.PlanetGameMode) or mode.explorer is None or mode.planet is None or mode.ground is None:
	raise RuntimeError("C++ 探索モードの準備完了を待って下さい")
terrain = mode.planet
body = mode.explorer
movement = body.get_movement_component()
if (terrain.generation_busy or terrain.water_generation_busy or terrain.preparation_busy or
		body.get_velocity().length() > 0.001 or not body.is_actor_tick_enabled() or
		not movement.is_component_tick_enabled()):
	raise RuntimeError("停止中の身体と生成済みの惑星で開始して下さい")
original = {
	"seed": terrain.seed, "water": terrain.render_river_water, "follow": terrain.follow_player_camera,
	"body_tick": body.is_actor_tick_enabled(), "movement_tick": movement.is_component_tick_enabled()
}
state: dict = {
	"status": "running", "phase": "initial", "start": time.monotonic(), "phase_start": time.monotonic(),
	"handle": None, "samples": 0, "parallel_frames": 0, "publications": [], "original": original
}
builtins._planet_water_lifecycle_probe = state


def samples() -> list[tuple[float, float, float, float]]:
	"""独立した１６方向の標高と解析法線を保持する"""
	values: list[tuple[float, float, float, float]] = []
	for index in range(16):
		z = 1 - 2 * (index + 0.5) / 16
		angle = index * 2.399963229728653
		radius = math.sqrt(1 - z * z)
		height, normal = terrain.evaluate_direction(unreal.Vector(radius * math.cos(angle), radius * math.sin(angle), z))
		values.append((height, normal.x, normal.y, normal.z))
	# 再生成と照合する独立標本の返戻
	return values


def request(phase: str) -> None:
	"""生成要求を受理した版と開始時刻を記録する"""
	if not terrain.request_generation():
		raise RuntimeError(terrain.generation_message)
	state["target"] = terrain.requested_revision
	state["phase"] = phase
	state["phase_start"] = time.monotonic()
	# 終了
	return


def finish(error: str | None) -> None:
	"""一時設定と身体の更新を戻し，合否を保持する"""
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
		state["handle"] = None
	valid = editor.get_game_world() == world and unreal.SystemLibrary.is_valid(terrain)
	if valid:
		terrain.set_editor_property("seed", original["seed"])
		terrain.set_editor_property("render_river_water", original["water"])
		terrain.set_editor_property("follow_player_camera", original["follow"])
		body.set_actor_tick_enabled(original["body_tick"])
		movement.set_component_tick_enabled(original["movement_tick"])
		if error:
			terrain.request_generation()
	state["status"] = "failed" if error else "passed"
	state["error"] = error
	state["seconds"] = time.monotonic() - state["start"]
	state["settings_restored"] = valid and (
		terrain.seed == original["seed"] and terrain.render_river_water == original["water"] and
		terrain.follow_player_camera == original["follow"] and body.is_actor_tick_enabled() == original["body_tick"] and
		movement.is_component_tick_enabled() == original["movement_tick"]
	)
	state["field_restoration_verified"] = error is None
	state["full_gameplay_validated"] = False
	state["saved_assets"] = False
	# 終了
	return


def tick(delta_seconds: float) -> None:
	"""各公開を監視し，同時実行・取消し・再生成の遷移を順番に検査する"""
	_ = delta_seconds
	try:
		if editor.get_game_world() != world:
			raise RuntimeError("検査中に PIE が終了しました")
		if time.monotonic() - state["phase_start"] > 120:
			raise RuntimeError("更新検査の期限超過：" + state["phase"] + "：" + terrain.generation_message)
		land, water = terrain.published_revision, terrain.published_water_revision
		if water > land:
			raise RuntimeError("水面が未公開の地表定義を追越しました")
		pair = [land, water]
		if not state["publications"] or state["publications"][-1]["revisions"] != pair:
			if state["publications"]:
				old_pair = state["publications"][-1]["revisions"]
				if land < old_pair[0] or water < old_pair[1]:
					raise RuntimeError("公開版が巻戻されました")
			state["publications"].append({"phase": state["phase"], "revisions": pair})
		state["samples"] += 1
		state["parallel_frames"] += int(terrain.generation_busy and terrain.water_generation_busy)
		complete = not terrain.generation_busy and not terrain.water_generation_busy and land == state["target"] and water == land
		phase = state["phase"]
		if phase == "initial" and complete:
			state["baseline"] = samples()
			request("water_start")
		elif phase == "water_start" and land == state["target"] and terrain.water_generation_busy:
			state["land_before_overlap"] = land
			request("overlap")
		elif phase == "overlap" and land == state["target"] and terrain.water_generation_busy:
			state["surface_advanced_during_water"] = land > state["land_before_overlap"]
			state["cancellation_requested_while_water_busy"] = True
			state["discarded_before_change"] = terrain.discarded_requests
			terrain.set_editor_property("follow_player_camera", False)
			terrain.set_editor_property("seed", original["seed"] + 1)
			request("different_seed")
		elif phase == "different_seed" and complete:
			state["changed"] = samples()
			if state["changed"] == state["baseline"]:
				raise RuntimeError("別 Seed の地表が旧値のままです")
			state["discarded_after_change"] = terrain.discarded_requests
			terrain.set_editor_property("seed", original["seed"])
			request("restore_seed")
		elif phase == "restore_seed" and complete:
			if samples() != state["baseline"]:
				raise RuntimeError("元の Seed の独立再生成が一致しません")
			terrain.set_editor_property("render_river_water", original["water"])
			request("restore_water")
		elif phase == "restore_water" and complete:
			if samples() != state["baseline"] or not state.get("surface_advanced_during_water") or state["parallel_frames"] == 0:
				raise RuntimeError("地表の同時更新又は最終復元を確認出来ません")
			state["ground_revision_before_recovery"] = mode.ground.published_revision
			state["ground_refresh_before_recovery"] = body.ground_refresh_requests
			body.set_actor_tick_enabled(original["body_tick"])
			movement.set_component_tick_enabled(original["movement_tick"])
			state["phase"] = "ground_recovery"
			state["phase_start"] = time.monotonic()
		elif phase == "ground_recovery":
			refresh = body.ground_refresh_requests > state["ground_refresh_before_recovery"]
			published = mode.ground.published_revision > state["ground_revision_before_recovery"]
			if not refresh and time.monotonic() - state["phase_start"] > 5:
				raise RuntimeError("Ground refresh was not requested after field replacement.")
			if refresh and published and not mode.ground.busy:
				if "stable_misses" not in state:
					state["stable_misses"] = body.coverage_miss_frames
					state["stable_start"] = time.monotonic()
				elif time.monotonic() - state["stable_start"] >= 2:
					if body.coverage_miss_frames != state["stable_misses"]:
						raise RuntimeError("Ground coverage keeps failing after recook.")
					state["ground_recovery_verified"] = True
					finish(None)
	except Exception as error:
		finish(str(error))
	# 終了
	return


try:
	body.set_actor_tick_enabled(False)
	movement.set_component_tick_enabled(False)
	terrain.set_editor_property("render_river_water", True)
	request("initial")
	state["handle"] = unreal.register_slate_post_tick_callback(tick)
	RESULT = {"status": "started", "read_with": "Scripts/Editor/read_water_lifecycle_probe.py"}
except Exception as error:
	finish(str(error))
	raise
