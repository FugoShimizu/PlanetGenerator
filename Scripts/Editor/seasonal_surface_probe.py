"""実際の惑星 Actor で気候の共有・季節材質・形状不変を確認する"""
import builtins
import hashlib
import math
import struct
import time
import unreal

args = globals().get("ARGS", {})
action = args.get("operation", "read")
key = "_planet_seasonal_probe"
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("此の検査は PIE を停止して実行して下さい")


def mesh_hash(actor: unreal.PlanetWorld) -> str:
	"""公開中の実頂点・三角形・法線から形状の識別値を返す

	Args:
		actor: 検査対象の惑星
	Returns:
		公開中の形状の SHA-256
	Raises:
		RuntimeError: 公開済みの表示面が無い場合
	"""
	mesh = actor.get_visible_mesh()
	if mesh is None or not mesh.is_mesh_section_visible(0):
		raise RuntimeError("公開済みの表示面が有りません")
	vertices, indices, normals, _, _ = unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh, 0)
	digest = hashlib.sha256()
	for collection in (vertices, normals):
		for vector in collection:
			digest.update(struct.pack("<ddd", vector.x, vector.y, vector.z))
	for index in indices:
		digest.update(struct.pack("<i", index))
	# 実際の表示形状の識別値の返戻
	return digest.hexdigest()


def finish(error: str | None = None) -> None:
	"""更新通知を解放し，成功又は失敗の証跡を残す

	Args:
		error: 失敗内容，成功時は None
	"""
	state = getattr(builtins, key)
	state["status"] = "failed" if error else "passed"
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
	state["handle"] = None
	state["result"] = {
		"status": state["status"], "error": error, "seconds": time.monotonic() - state["start"],
		"phases": state["phases"], "climatePreparations": state["actor"].climate_preparation_count,
		"climateMilliseconds": state["actor"].last_climate_preparation_milliseconds,
		"sameGeometryThroughSeasons": len(state["phases"]) == 4, "viewUpdateReusedClimate": state.get("viewReuse", False),
		"gpuReadback": False, "walkingVerified": False, "savedGeneratedData": False
	}
	# 終了
	return


def tick(delta_seconds: float) -> None:
	"""描画を止めずに非同期の完了と季節切替を確認する

	Args:
		delta_seconds: Slate の更新間隔，待機判定には単調時計を使う
	"""
	_ = delta_seconds
	state = getattr(builtins, key, None)
	# 完了後の通知を処理しない終了
	if not state or state["status"] != "running":
		return
	actor = state["actor"]
	try:
		if time.monotonic() - state["start"] > 240:
			raise RuntimeError("季節検査の待機時間を超えました：" + actor.generation_message + " / " + actor.climate_message)
		if state["stage"] == "prepare":
			# 地形と気候の公開を待つ終了
			if not actor.climate_ready or actor.generation_busy:
				return
			state["shape"] = mesh_hash(actor)
			state["revision"] = actor.published_revision
			state["count"] = actor.climate_preparation_count
			if state["count"] != 1:
				raise RuntimeError("初期気候の準備回数が一回ではありません")
			state["stage"] = "season"
			state["phaseIndex"] = 0
			actor.set_editor_property("solar_year_phase", 0.0)
			state["nextRead"] = time.monotonic() + 0.2
		elif state["stage"] == "season":
			# 材質の更新を待つ終了
			if time.monotonic() < state["nextRead"]:
				return
			phase = state["phaseIndex"] / 4
			material = actor.get_visible_mesh().get_material(0)
			if not isinstance(material, unreal.MaterialInstanceDynamic):
				raise RuntimeError("材質が動的インスタンスではありません")
			if material.get_scalar_parameter_value("ClimateEnabled") != 1:
				raise RuntimeError("実気候の材質が有効ではありません")
			if abs(material.get_scalar_parameter_value("SolarYearPhase") - phase) > 1.0e-7:
				raise RuntimeError("年周位相が材質へ届いていません")
			texture = material.get_texture_parameter_value("ClimateAtlas")
			if texture is None or not texture.get_path_name().startswith("/Engine/Transient."):
				raise RuntimeError("気候配列が一時テクスチャではありません")
			if actor.published_revision != state["revision"] or mesh_hash(actor) != state["shape"]:
				raise RuntimeError("季節の変更で地形形状又は生成版が変わりました")
			if actor.climate_preparation_count != state["count"]:
				raise RuntimeError("季節変更で全球気候を再準備しました")
			state["phases"].append({"phase": phase, "revision": actor.published_revision, "texture": texture.get_path_name()})
			state["phaseIndex"] += 1
			if state["phaseIndex"] < 4:
				actor.set_editor_property("solar_year_phase", state["phaseIndex"] / 4)
				state["nextRead"] = time.monotonic() + 0.2
			else:
				actor.set_editor_property("observer_meters", unreal.Vector(0.6, 0, 1.6) * actor.radius_meters)
				if not actor.request_generation():
					raise RuntimeError(actor.generation_message)
				state["target"] = actor.requested_revision
				state["stage"] = "view"
		elif state["stage"] == "view":
			if actor.get_visible_mesh() is None:
				raise RuntimeError("視点更新の待機中に旧表示が消えました")
			# 次の表示面の公開を待つ終了
			if actor.generation_busy or actor.published_revision != state["target"]:
				return
			if not actor.climate_ready or actor.climate_preparation_count != state["count"]:
				raise RuntimeError("視点更新で同じ気候を共有出来ませんでした")
			if actor.get_visible_mesh().get_material(0).get_scalar_parameter_value("ClimateEnabled") != 1:
				raise RuntimeError("切替後の表示面に気候が接続されていません")
			state["viewReuse"] = True
			finish()
	except Exception as e:
		finish(str(e))
	# 終了
	return


state = getattr(builtins, key, None)
if action == "begin":
	if state:
		raise RuntimeError("前回の検査を cleanup してから開始して下さい")
	actor = actors.spawn_actor_from_class(unreal.PlanetWorld, unreal.Vector(0, 0, 1.0e10), transient=True)
	actor.set_actor_label("Planet_SeasonalProbe")
	actor.set_editor_property("radius_meters", float(args.get("radiusMeters", 1000)))
	actor.set_editor_property("observer_meters", unreal.Vector(0, 0, float(args.get("observerRatio", 1.6))) *
		actor.radius_meters)
	actor.set_editor_property("follow_player_camera", False)
	actor.set_editor_property("refine_near_surface", bool(args.get("refineNearSurface", False)))
	actor.set_editor_property("leaf_budget", int(args.get("leafBudget", 4096)))
	state = {
		"actor": actor, "camera": editor.get_level_viewport_camera_info(), "start": time.monotonic(),
		"status": "running", "stage": "prepare", "phases": [], "handle": None
	}
	setattr(builtins, key, state)
	if not actor.request_generation():
		finish(actor.generation_message)
	else:
		state["handle"] = unreal.register_slate_post_tick_callback(tick)
	RESULT = {"status": state["status"]}
elif action == "cleanup":
	if not state:
		raise RuntimeError("所有する検査状態が有りません")
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
	editor.set_level_viewport_camera_info(*state["camera"])
	if not actors.destroy_actor(state["actor"]):
		raise RuntimeError("所有する検査 Actor を破棄出来ません")
	delattr(builtins, key)
	RESULT = {"cleaned": True, "cameraRestored": True}
elif action == "preview":
	if not state or state["status"] != "passed":
		raise RuntimeError("実行検査が成功してから表示して下さい")
	actor = state["actor"]
	actor.set_editor_property("solar_year_phase", float(args.get("phase", 0)))
	observer = unreal.Vector(200, -100, 130) * actor.radius_meters
	location = actor.get_actor_location() + observer
	rotation = unreal.MathLibrary.make_rot_from_x(-observer)
	editor.set_level_viewport_camera_info(location, rotation)
	RESULT = {"preview": True, "phase": actor.solar_year_phase, "templateLighting": True}
elif action == "near":
	if not state or state["status"] != "passed":
		raise RuntimeError("実行検査が成功してから近景を準備して下さい")
	actor = state["actor"]
	selection = args.get("selection", "seasonal")
	count = 1024 if selection == "dune" else 512
	best = None
	dry_count = 0
	dry_max_elevation = float("-inf")
	for index in range(count):
		z = -1 + 2 * (index + 0.5) / count
		angle = index * 2.399963229728653
		width = math.sqrt(1 - z * z)
		direction = unreal.Vector(width * math.cos(angle), width * math.sin(angle), z)
		elevation, normal = actor.evaluate_direction(direction)
		if elevation < 2 or unreal.MathLibrary.dot_vector_vector(direction, normal) < 0.95:
			continue
		first, factors = actor.evaluate_seasonal_surface(direction, 0.25)
		dry = factors.b * (1 - factors.g)
		if dry >= 0.8:
			dry_count += 1
			dry_max_elevation = max(dry_max_elevation, elevation)
		if selection == "dune":
			dry = factors.b * (1 - factors.g)
			if dry < 0.8 or elevation < min(50.0, actor.radius_meters * 0.001):
				continue
			east = unreal.MathLibrary.cross_vector_vector(direction, unreal.Vector(0, 0, 1))
			if east.length() < 0.01:
				continue
			east = east / east.length()
			north = unreal.MathLibrary.cross_vector_vector(direction, east)
			score = 0.0
			for tangent in (east, north):
				offset = tangent * (30.0 / actor.radius_meters)
				plus = direction + offset
				minus = direction - offset
				high, _ = actor.evaluate_direction(plus / plus.length())
				low, _ = actor.evaluate_direction(minus / minus.length())
				score = max(score, elevation - 0.5 * (high + low))
			score *= dry
		elif selection == "drySand":
			score = factors.b * (1 - factors.g)
		else:
			second, _ = actor.evaluate_seasonal_surface(direction, 0.75)
			score = abs(first.a - second.a) + 0.1 * (first.r + second.r)
		if best is None or score > best[0]:
			best = (score, direction, elevation)
	if best is None or (selection == "drySand" and best[0] < 0.2) or (selection == "dune" and best[0] < 0.6):
		raise RuntimeError(
			f"検査用の陸上地点が見付かりません: score={best[0] if best else None} "
			f"dryCount={dry_count} dryMaxElevation={dry_max_elevation}")
	_, up, elevation = best
	forward = unreal.MathLibrary.cross_vector_vector(up, unreal.Vector(0, 0, 1))
	forward = forward / forward.length()
	observer = up * (actor.radius_meters + elevation + 1.6)
	actor.set_editor_property("observer_meters", observer)
	actor.set_editor_property("refine_near_surface", True)
	actor.set_editor_property("leaf_budget", 32768)
	if not actor.request_generation():
		raise RuntimeError(actor.generation_message)
	state["near"] = {"up": up, "forward": forward, "observer": observer, "revision": actor.requested_revision}
	RESULT = {"requested": True, "elevationMeters": elevation, "seasonalScore": best[0],
		"selection": selection, "revision": actor.requested_revision,
		"direction": [up.x, up.y, up.z]}
elif action == "nearPreview":
	if not state or "near" not in state:
		raise RuntimeError("先に近景を準備して下さい")
	actor = state["actor"]
	near = state["near"]
	if actor.generation_busy or actor.published_revision != near["revision"]:
		RESULT = {"ready": False, "generationMessage": actor.generation_message}
	else:
		if actor.visible_surface_radius_meters < 24:
			raise RuntimeError("近景の必須範囲が確保されていません")
		if actor.climate_preparation_count != state["count"]:
			raise RuntimeError("近景更新で気候を再準備しました")
		basis = unreal.MathLibrary.make_rot_from_xz(near["forward"], near["up"])
		inverse = unreal.MathLibrary.invert_transform(unreal.Transform(rotation=basis))
		actor.set_actor_transform(inverse, False, False)
		position = unreal.MathLibrary.transform_location(actor.get_actor_transform(), near["observer"] * 100)
		camera = unreal.Vector(300000, 0, 160)
		actor.set_actor_location(camera - position, False, False)
		actor.set_editor_property("solar_year_phase", float(args.get("phase", 0.25)))
		editor.set_level_viewport_camera_info(camera, unreal.Rotator(pitch=-12, yaw=0, roll=0))
		RESULT = {
			"ready": True, "phase": actor.solar_year_phase, "radiusMeters": actor.radius_meters,
			"nearRadiusMeters": actor.visible_surface_radius_meters,
			"maximumSurfaceErrorMeters": actor.maximum_surface_probe_error_meters,
			"climatePreparations": actor.climate_preparation_count,
			"generationMilliseconds": actor.last_worker_milliseconds, "templateLighting": True
		}
elif action == "duneOverview":
	if not state or "near" not in state:
		raise RuntimeError("先に砂丘の近景を公開して下さい")
	actor = state["actor"]
	if actor.generation_busy or actor.published_revision != state["near"]["revision"]:
		raise RuntimeError("砂丘の近景が未公開です")
	editor.set_level_viewport_camera_info(
		unreal.Vector(292000, 0, 1600), unreal.Rotator(pitch=-11, yaw=0, roll=0))
	RESULT = {"overview": True, "distanceMeters": 80, "cameraHeightMeters": 16,
		"gpuReadback": False}
elif action == "duneProfile":
	if not state or "near" not in state:
		raise RuntimeError("先に砂丘の近景を準備して下さい")
	actor = state["actor"]
	up = state["near"]["up"]
	east = state["near"]["forward"]
	north = unreal.MathLibrary.cross_vector_vector(up, east)
	profiles = []
	for label, tangent in (("east", east), ("north", north)):
		heights = []
		for offset in range(-120, 121, 10):
			position = up * actor.radius_meters + tangent * offset
			direction = position / position.length()
			elevation, _ = actor.evaluate_direction(direction)
			heights.append(elevation)
		linear = [heights[0] + (heights[-1] - heights[0]) * index / 24 for index in range(25)]
		profiles.append({"axis": label, "heightsMeters": heights,
			"maximumResidualMeters": max(value - base for value, base in zip(heights, linear))})
	RESULT = {"stepMeters": 10, "minimumOffsetMeters": -120, "profiles": profiles}
elif action == "read":
	if not state:
		raise RuntimeError("季節検査が開始されていません")
	RESULT = state.get("result", {"status": state["status"], "stage": state["stage"],
		"generationBusy": state["actor"].generation_busy,
		"generationMessage": state["actor"].generation_message,
		"requestedRevision": state["actor"].requested_revision,
		"publishedRevision": state["actor"].published_revision,
		"preparationBusy": state["actor"].preparation_busy,
		"preparationSlices": state["actor"].preparation_slices,
		"waterGenerationBusy": state["actor"].water_generation_busy,
		"discardedRequests": state["actor"].discarded_requests,
		"climateBusy": state["actor"].climate_busy,
		"climateMessage": state["actor"].climate_message})
else:
	raise RuntimeError("未対応の検査操作です")
