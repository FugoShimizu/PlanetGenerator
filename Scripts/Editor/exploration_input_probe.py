"""実キー入力中の速度・ジャンプ・足場・近景を観測する"""
import builtins
import time
import unreal

key = "_planet_exploration_input_probe"
args = globals().get("ARGS", {})
state = getattr(builtins, key, None)


def sample(delta_seconds: float) -> None:
	"""操作を変更せず，一フレーム分の探索状態を記録する

	Args:
		delta_seconds: Slate の更新間隔，経過時間は単調時計で測る
	"""
	_ = delta_seconds
	current = getattr(builtins, key)
	try:
		body = current["body"]
		planet = current["planet"]
		position = body.get_actor_location()
		movement = body.get_movement_component()
		current["maximumSpeed"] = max(current["maximumSpeed"], body.get_velocity().length() / 100)
		current["travelMeters"] += (position - current["previous"]).length() / 100
		current["previous"] = position
		current["minimumGroundMargin"] = min(current["minimumGroundMargin"], body.ground_margin_meters)
		current["coverageMissFrames"] = body.coverage_miss_frames
		local = unreal.MathLibrary.inverse_transform_location(planet.get_actor_transform(), position) / 100
		distance = (local - planet.visible_surface_observer_meters).length()
		current["maximumSurfaceOffset"] = max(current["maximumSurfaceOffset"], distance)
		if distance + 0.34 > planet.visible_surface_radius_meters:
			raise RuntimeError("身体が公開済みの精密な近景から外れました")
		falling = movement.is_falling()
		current["airFrames"] += int(falling)
		current["landedAfterJump"] |= current["airFrames"] > 0 and not falling
		current["samples"] += 1
	except Exception as e:
		current["error"] = str(e)
	if current["error"] or time.monotonic() - current["start"] >= 30:
		unreal.unregister_slate_post_tick_callback(current["handle"])
		current["handle"] = None
		current["completed"] = True
		for name in ("body", "planet", "previous"):
			current.pop(name, None)
	# 終了
	return


if args.get("operation", "read") == "begin":
	if state and not state["completed"]:
		raise RuntimeError("前の入力観測が続いています")
	world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
	if world is None:
		raise RuntimeError("探索レベルで PIE を開始して下さい")
	mode = unreal.GameplayStatics.get_game_mode(world)
	if not isinstance(mode, unreal.PlanetGameMode) or mode.explorer is None:
		raise RuntimeError("探索用の身体が準備されていません")
	state = {
		"body": mode.explorer, "planet": mode.planet, "previous": mode.explorer.get_actor_location(),
		"start": time.monotonic(), "samples": 0, "maximumSpeed": 0, "travelMeters": 0,
		"minimumGroundMargin": 1.0E9, "maximumSurfaceOffset": 0, "coverageMissFrames": 0,
		"airFrames": 0, "landedAfterJump": False, "completed": False, "error": None, "handle": None
	}
	setattr(builtins, key, state)
	state["handle"] = unreal.register_slate_post_tick_callback(sample)
	RESULT = {"started": True, "durationSeconds": 30, "injectedMovement": False}
else:
	if not state:
		raise RuntimeError("入力観測を開始して下さい")
	RESULT = {name: value for name, value in state.items() if name not in ("body", "planet", "previous", "start", "handle")}
