"""実キー入力で幹へ接近する間の身体と幹の距離を観測する"""
import builtins
import math
import time
import unreal

KEY = "_planet_tree_walk_probe"
ARGS = globals().get("ARGS", {})


def sample(delta_seconds: float) -> None:
	"""移動を変更せず，幹の軸までの距離と経路長を読む
	Args:
		delta_seconds: Slate の更新間隔
	"""
	_ = delta_seconds
	STATE = getattr(builtins, KEY)
	try:
		BODY = STATE["body"]
		TRUNK = STATE["trunk"]
		POSITION = BODY.get_actor_location()
		CENTER = TRUNK.get_world_location()
		AXIS = TRUNK.get_up_vector()
		PROJECTED = CENTER + AXIS * ((POSITION - CENTER).dot(AXIS))
		STATE["minimumAxisDistanceMeters"] = min(STATE["minimumAxisDistanceMeters"], (POSITION - PROJECTED).length() / 100)
		STATE["travelMeters"] += (POSITION - STATE["previous"]).length() / 100
		STATE["previous"] = POSITION
		STATE["coverageMissFrames"] = BODY.coverage_miss_frames
		STATE["samples"] += 1
	except Exception as error:
		STATE["error"] = str(error)
	if STATE["error"] or time.monotonic() - STATE["started"] >= 12:
		unreal.unregister_slate_post_tick_callback(STATE["handle"])
		STATE["completed"] = True
		STATE["handle"] = None
		for NAME in ("body", "trunk", "previous"):
			STATE.pop(NAME, None)
	# 終了
	return


if ARGS.get("operation", "read") == "begin":
	OLD = getattr(builtins, KEY, None)
	if OLD and not OLD["completed"]:
		raise RuntimeError("前の幹への接近検査が続いています")
	WORLD = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
	MODE = unreal.GameplayStatics.get_game_mode(WORLD)
	BODY = MODE.explorer
	VEGETATION = unreal.GameplayStatics.get_all_actors_of_class(WORLD, unreal.PlanetVegetation)[0]
	CAPSULES = VEGETATION.get_components_by_class(unreal.CapsuleComponent)
	TRUNK = min(CAPSULES, key=lambda item: (item.get_world_location() - BODY.get_actor_location()).length())
	POSITION = BODY.get_actor_location()
	CENTER = TRUNK.get_world_location()
	AXIS = TRUNK.get_up_vector()
	TARGET = CENTER + AXIS * ((POSITION - CENTER).dot(AXIS))
	DELTA = TARGET - POSITION
	ANGLE = math.degrees(math.atan2(DELTA.dot(BODY.get_actor_right_vector()), DELTA.dot(BODY.get_actor_forward_vector())))
	BODY.look(ANGLE, 0)
	HIT = unreal.SystemLibrary.line_trace_single(
		WORLD, POSITION, TARGET, unreal.TraceTypeQuery.ECC_VISIBILITY, False,
		[BODY], unreal.DrawDebugTrace.NONE
	)
	if HIT is None or HIT.to_dict()["hit_component"] != TRUNK:
		raise RuntimeError("正面にある幹の実衝突を確認出来ません")
	STATE = {
		"body": BODY, "trunk": TRUNK, "previous": POSITION, "started": time.monotonic(), "completed": False,
		"samples": 0, "minimumAxisDistanceMeters": DELTA.length() / 100, "travelMeters": 0,
		"coverageMissFrames": BODY.coverage_miss_frames, "error": None, "target": TRUNK.get_name(),
		"initialDistanceMeters": DELTA.length() / 100,
		"combinedRadiusMeters": (BODY.capsule_component.get_scaled_capsule_radius() + TRUNK.get_scaled_capsule_radius()) / 100,
		"nativeRayHit": True
	}
	setattr(builtins, KEY, STATE)
	STATE["handle"] = unreal.register_slate_post_tick_callback(sample)
	RESULT = {"started": True, "target": TRUNK.get_name(), "initialDistanceMeters": STATE["initialDistanceMeters"]}
else:
	STATE = getattr(builtins, KEY)
	RESULT = {NAME: VALUE for NAME, VALUE in STATE.items() if NAME not in ("body", "trunk", "previous", "handle", "started")}
