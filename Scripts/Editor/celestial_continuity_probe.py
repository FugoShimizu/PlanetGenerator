"""共通時計の進行・停止と三視点の参照地点の連続性を検査する"""
import builtins
from datetime import datetime
import time
import unreal

key = "_planet_celestial_continuity_probe"
args = globals().get("ARGS", {})
state = getattr(builtins, key, None)
views = [unreal.PlanetViewMode.FIRST_PERSON, unreal.PlanetViewMode.THIRD_PERSON, unreal.PlanetViewMode.OVERVIEW]


def utc_seconds(planet: unreal.PlanetWorld) -> float:
	"""公開日時を秒単位の差分比較用に変換する
	Args:
		planet: 検査する惑星
	Returns:
		任意の固定基準からの秒数，公開文字列の精度は１秒
	"""
	date = datetime.strptime(planet.simulation_utc.export_text(), "%Y.%m.%d-%H.%M.%S")
	# 端数秒を含まない比較値の返戻
	return (date - datetime(2000, 1, 1)).total_seconds()


def finish(current: dict, error: str | None) -> None:
	"""通知を解除して，元の日時・倍率・視点を復元する
	Args:
		current: 所有する検査状態
		error: 失敗内容，成功時は None
	"""
	if current["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(current["handle"])
	current["handle"] = None
	current["completed"] = True
	current["error"] = error
	planet = current["planet"]
	planet.set_editor_property("time_scale", current["originalScale"])
	planet.set_editor_property("advance_time", False)
	if not planet.set_simulation_utc(current["originalUtc"]):
		current["error"] = planet.clock_message
	planet.set_editor_property("advance_time", current["originalAdvance"])
	current["body"].set_view_mode(current["originalView"])
	for name in ("planet", "body", "world", "originalUtc", "originalView"):
		current.pop(name, None)
	# 終了
	return


def tick(delta_seconds: float) -> None:
	"""実際の時計と背景面を，視点と時間倍率の変更後に検査する
	Args:
		delta_seconds: Slate の更新間隔，待機は単調時計を使う
	"""
	_ = delta_seconds
	current = getattr(builtins, key)
	# 次の読取り迄待つ終了
	if time.monotonic() < current["nextRead"]:
		return
	try:
		planet = current["planet"]
		world = current["world"]
		step = current["step"]
		camera = unreal.GameplayStatics.get_player_camera_manager(world, 0)
		dome = planet.get_components_by_class(unreal.StaticMeshComponent)[0]
		center_error = (dome.get_world_location() - camera.get_camera_location()).length() / 100
		if center_error > 0.001:
			raise RuntimeError("時計停止中又は視点変更後に天球が追従していません")
		current["maximumSkyCenterErrorMeters"] = max(current["maximumSkyCenterErrorMeters"], center_error)
		if step < 3:
			observer = planet.earth_observer_degrees_meters
			observer_values = [observer.x, observer.y, observer.z]
			moon = planet.moon_direction_world
			moon_values = [moon.x, moon.y, moon.z]
			if observer_values != current["observer"] or moon_values != current["moon"]:
				raise RuntimeError("視点変更で観測地点又は月の方向が変わりました")
			if utc_seconds(planet) != current["pausedUtc"]:
				raise RuntimeError("停止した日時が視点変更で変わりました")
			current["viewsChecked"].append(str(views[step]))
			if step < 2:
				current["body"].set_view_mode(views[step + 1])
			else:
				planet.set_editor_property("time_scale", 120.0)
				planet.set_editor_property("advance_time", True)
				current["startGameTime"] = unreal.GameplayStatics.get_time_seconds(world)
				current["startUtc"] = utc_seconds(planet)
		elif step == 3:
			elapsed = unreal.GameplayStatics.get_time_seconds(world) - current["startGameTime"]
			advanced = utc_seconds(planet) - current["startUtc"]
			current["scaledTime"] = {"gameSeconds": elapsed, "utcSeconds": advanced, "expectedUtcSeconds": elapsed * 120}
			if abs(advanced - elapsed * 120) > 1.05:
				raise RuntimeError("実ゲーム時間と共通時計の倍率が一致しません")
			planet.set_editor_property("advance_time", False)
			current["pausedUtc"] = utc_seconds(planet)
		elif step == 4:
			if utc_seconds(planet) != current["pausedUtc"]:
				raise RuntimeError("時計の停止が反映されていません")
			planet.set_editor_property("advance_time", True)
			planet.set_editor_property("time_scale", 0.0)
		elif step == 5:
			if utc_seconds(planet) != current["pausedUtc"]:
				raise RuntimeError("倍率０で日時が進みました")
			planet.set_editor_property("time_scale", -1.0)
		elif step == 6:
			if planet.celestial_clock_active or not planet.clock_message or utc_seconds(planet) != current["pausedUtc"]:
				raise RuntimeError("不正な時間倍率で停止せず，日時を変更しました")
			current["invalidScaleStoppedClock"] = True
			current["body"].set_view_mode(unreal.PlanetViewMode.THIRD_PERSON)
		else:
			finish(current, None)
			# 全段階の検査を終えた終了
			return
		current["step"] += 1
		current["nextRead"] = time.monotonic() + (1.0 if current["step"] == 3 else 0.4)
	except Exception as e:
		finish(current, str(e))
	# 終了
	return


if args.get("operation", "read") == "begin":
	if state and not state["completed"]:
		raise RuntimeError("前の時計連続性の検査が続いています")
	world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
	if world is None:
		raise RuntimeError("探索レベルで PIE を開始して下さい")
	mode = unreal.GameplayStatics.get_game_mode(world)
	if not isinstance(mode, unreal.PlanetGameMode) or mode.explorer is None:
		raise RuntimeError("探索の準備が完了していません")
	planet = mode.planet
	state = {
		"world": world, "planet": planet, "body": mode.explorer, "originalUtc": planet.simulation_utc.copy(),
		"originalScale": planet.time_scale, "originalAdvance": planet.advance_time, "originalView": mode.explorer.view_mode,
		"completed": False, "error": None, "handle": None, "step": 0, "viewsChecked": [],
		"maximumSkyCenterErrorMeters": 0, "invalidScaleStoppedClock": False
	}
	setattr(builtins, key, state)
	try:
		planet.set_editor_property("advance_time", False)
		utc = planet.simulation_utc.copy()
		if not utc.import_text("2026.06.21-18.00.00") or not planet.set_simulation_utc(utc):
			raise RuntimeError("検査の基準日時を設定出来ません")
		observer = planet.earth_observer_degrees_meters
		moon = planet.moon_direction_world
		state["observer"] = [observer.x, observer.y, observer.z]
		state["moon"] = [moon.x, moon.y, moon.z]
		state["pausedUtc"] = utc_seconds(planet)
		mode.explorer.set_view_mode(views[0])
		state["nextRead"] = time.monotonic() + 0.4
		state["handle"] = unreal.register_slate_post_tick_callback(tick)
	except Exception as e:
		finish(state, str(e))
	RESULT = {"started": state["handle"] is not None, "error": state["error"]}
else:
	if not state:
		raise RuntimeError("時計連続性の検査を開始して下さい")
	RESULT = {
		name: value for name, value in state.items() if name in (
			"completed", "error", "viewsChecked", "maximumSkyCenterErrorMeters", "scaledTime", "invalidScaleStoppedClock"
		)
	}
