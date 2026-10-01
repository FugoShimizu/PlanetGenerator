"""UE の更新を止めず，非同期生成の被覆・不変場・排水を検査する"""
import builtins
import math
import time
import unreal

VectorTuple = tuple[float, float, float]
old = getattr(builtins, "_planet_stream_probe", None)
if old and old.get("status") == "running":
	raise RuntimeError("A streaming probe is already running")
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if levels.is_in_play_in_editor():
	raise RuntimeError("Stop PIE before the editor streaming probe")
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = editor.get_editor_world()
camera_before = editor.get_level_viewport_camera_info()
existing = [
	a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetTerrain)
	if a.get_actor_label() == "Planet_AsyncTerrain"
]
if len(existing) > 1:
	raise RuntimeError("Duplicate asynchronous terrain actors")
actor = existing[0] if existing else actors.spawn_actor_from_class(
	unreal.PlanetTerrain, unreal.Vector(0, 0, 1E10), transient=True
)
actor.set_actor_label("Planet_AsyncTerrain")
actor.set_editor_property("follow_player_camera", False)
actor.set_editor_property("seed", 42)
actor.set_editor_property("radius_meters", 1000000.0)
actor.set_editor_property("leaf_budget", 512)
actor.set_editor_property("maximum_level", 10)
actor.set_editor_property("observer_meters", unreal.Vector(0, 0, 1100000))
actor.set_editor_property("surface_material", unreal.load_asset("/Game/Planet/Materials/M_PlanetSurface"))
state: dict[str, object] = {
	"status": "running", "phase": "initial", "start": time.monotonic(), "actor": actor,
	"camera_before": camera_before, "handle": None, "request_ms": [], "retained_frames": 0,
	"snapshots": [], "seen_visible": actor.published_revision > 0
}
builtins._planet_stream_probe = state

def request() -> None:
	"""要求が主処理を長く占有せず，新しい版を受理した事を検査する"""
	start = time.perf_counter()
	if not actor.request_generation():
		raise RuntimeError(actor.generation_message)
	ms = (time.perf_counter() - start) * 1000
	state["request_ms"].append(ms)
	if ms > 100:
		raise RuntimeError(f"Generation request blocked the editor for {ms:.3f} ms")
	state["target"] = actor.requested_revision
	# 終了
	return

def samples() -> list[tuple[float, float, float, float]]:
	"""固定した独立地点の標高と法線を返す"""
	values: list[tuple[float, float, float, float]] = []
	for i in range(32):
		z = 1 - 2 * (i + 0.5) / 32
		p = i * 2.399963229728653
		r = math.sqrt(1 - z * z)
		h, n = actor.evaluate_direction(unreal.Vector(r * math.cos(p), r * math.sin(p), z))
		values.append((h, n.x, n.y, n.z))
	# 再準備と視点移動の前後で照合する値の返戻
	return values

def drainage_samples() -> dict[str, object]:
	"""固定標本と追加出口の数，下降縦断，海面に接続した河口を検査して返す"""
	outlets = actor.get_coastal_outlet_node_count()
	if (
		outlets <= 0 or actor.drainage_node_count != 40962 + outlets or actor.river_reach_count <= 0 or
		actor.river_mouth_count <= 0 or actor.river_mouth_count > outlets
	):
		raise RuntimeError("Missing prepared fixed drainage sites or coastal outlets")
	values: list[tuple[float, ...]] = []
	count = min(64, actor.river_reach_count)
	for i in range(count):
		index = i * actor.river_reach_count // count
		start = actor.evaluate_river_profile(index, 0.0)
		end = actor.evaluate_river_profile(index, 1.0)
		if start is None or end is None or not start[1] > end[1] or not start[2] < start[1] or not end[2] < end[1]:
			raise RuntimeError("Published river profile is not descending above its bed")
		if end[1] == 0.0:
			h, normal = actor.evaluate_direction(end[0])
			radius = math.sqrt(end[0].x ** 2 + end[0].y ** 2 + end[0].z ** 2)
			if h > end[2] + 0.001 or abs(radius - actor.radius_meters) > 1E-6:
				raise RuntimeError("Published river mouth lacks submerged channel bed or spherical sea position")
		for position, water, bed, tangent in (start, end):
			values.append((position.x, position.y, position.z, water, bed, tangent.x, tangent.y, tangent.z))
	# 節点数も含めて不変性を照合する値の返戻
	return {"nodes": actor.drainage_node_count, "outlets": outlets, "values": values}

def dot(a: VectorTuple, b: VectorTuple) -> float:
	"""二つの三次元ベクトルの内積を返す"""
	# 内積の返戻
	return sum(x * y for x, y in zip(a, b))

def cross(a: VectorTuple, b: VectorTuple) -> VectorTuple:
	"""二つの三次元ベクトルの外積を返す"""
	# 外積の返戻
	return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

def inspect_mesh() -> None:
	"""公開した実メッシュの閉じた辺，球面被覆，連結性を検査する"""
	mesh = actor.get_visible_mesh()
	if mesh is None or not mesh.is_mesh_section_visible(0):
		raise RuntimeError("Published terrain has no visible mesh")
	vertices, indices, normals, uvs, tangents = unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh, 0)
	if len(vertices) != actor.visible_vertex_count or len(indices) // 3 != actor.visible_triangle_count:
		raise RuntimeError("Published counters disagree with actual mesh")
	directions: list[VectorTuple] = []
	for v in vertices:
		length = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
		directions.append((v.x / length, v.y / length, v.z / length))
	edges: dict[tuple[int, int], tuple[int, int]] = {}
	adjacency: list[set[int]] = [set() for _ in vertices]
	area = 0.0
	for i in range(0, len(indices), 3):
		ids = [int(indices[i + k]) for k in range(3)]
		a, b, c = [directions[j] for j in ids]
		triple = dot(a, cross(tuple(y - x for x, y in zip(a, b)), tuple(y - x for x, y in zip(a, c))))
		if triple >= 0:
			raise RuntimeError("Degenerate or reversed UE triangle")
		area += 2 * math.atan2(-triple, 1 + dot(a, b) + dot(b, c) + dot(c, a))
		for k, u in enumerate(ids):
			v = ids[(k + 1) % 3]
			key = tuple(sorted((u, v)))
			count, sign = edges.get(key, (0, 0))
			edges[key] = (count + 1, sign + (1 if u < v else -1))
			adjacency[u].add(v)
			adjacency[v].add(u)
	if any(count != 2 or sign != 0 for count, sign in edges.values()):
		raise RuntimeError("Open edge or mismatched LOD seam")
	if len(vertices) + len(indices) // 3 != len(edges) + 2 or abs(area - 4 * math.pi) > 1E-8:
		raise RuntimeError("Published mesh does not form a complete spherical surface")
	visited = {0}
	queue = [0]
	for u in queue:
		for v in adjacency[u]:
			if v not in visited:
				visited.add(v)
				queue.append(v)
	if len(visited) != len(vertices):
		raise RuntimeError("Disconnected surface or unused vertices")
	state["snapshots"].append({
		"revision": actor.published_revision, "vertices": len(vertices), "triangles": len(indices) // 3,
		"leaves": actor.visible_leaf_count, "maximum_level": actor.visible_maximum_level,
		"worker_ms": actor.last_worker_milliseconds, "field_preparation_ms": actor.last_field_preparation_milliseconds,
		"upload_ms": actor.last_upload_milliseconds, "swap_ms": actor.last_swap_milliseconds, "spherical_area": area,
		"drainage_nodes": actor.drainage_node_count, "coastal_outlets": actor.get_coastal_outlet_node_count(),
		"river_reaches": actor.river_reach_count, "river_mouths": actor.river_mouth_count,
		"max_required_bank_raise_m": actor.maximum_required_bank_raise_meters
	})
	# 終了
	return

def finish(error: str | None = None) -> None:
	"""完了又は失敗を保存し，所有する更新コールバックを解除する"""
	state["status"] = "failed" if error else "passed"
	state["error"] = error
	state["elapsed_seconds"] = time.monotonic() - state["start"]
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
		state["handle"] = None
	state["result"] = {
		k: state[k] for k in ("status", "error", "elapsed_seconds", "request_ms", "retained_frames", "snapshots")
	}
	state["result"].update({
		"discarded_requests": actor.discarded_requests, "actor": actor.get_path_name(),
		"saved_terrain_assets": False, "walking_collision_validated": False, "temporal_morph_validated": False,
		"production_performance_validated": False
	})
	# 終了
	return

def tick(delta: float) -> None:
	"""UE の更新毎に完了を受取り，準備中の旧表示と公開順序を検査する"""
	try:
		if time.monotonic() - state["start"] > 120:
			raise RuntimeError("Streaming probe timeout")
		actor.poll_generation()
		if actor.published_revision > 0:
			state["seen_visible"] = True
		if state["seen_visible"]:
			meshes = actor.get_components_by_class(unreal.ProceduralMeshComponent)
			# 地面と海面は別の被覆として公開される為，各対の表示面を独立に照合する
			for names, visible in (
				({"TerrainBufferA", "TerrainBufferB"}, actor.get_visible_mesh()),
				({"OceanBufferA", "OceanBufferB"}, actor.get_visible_ocean_mesh())
			):
				buffers = [m for m in meshes if m.get_name() in names]
				shown = [m for m in buffers if m.is_mesh_section_visible(0)]
				if len(buffers) != 2 or len(shown) != 1 or visible is None or shown[0] != visible:
					raise RuntimeError("Published ground or ocean coverage disappeared or overlapped during replacement")
		phase = state["phase"]
		if phase != "initial" and actor.published_revision == state["previous_revision"]:
			if (
				actor.get_visible_mesh().get_path_name() != state["previous_mesh"] or
				actor.get_visible_ocean_mesh().get_path_name() != state["previous_ocean"]
			):
				raise RuntimeError("Old mesh replaced before a current request was ready")
			state["retained_frames"] += 1
		# 要求した版が公開される迄，又は公開待ちの処理が有る間の終了
		if actor.published_revision != state["target"] or actor.generation_busy:
			return
		inspect_mesh()
		if phase == "initial":
			state["baseline"] = samples()
			state["drainage_baseline"] = drainage_samples()
			state["phase"] = "view"
			actor.set_editor_property("observer_meters", unreal.Vector(1100000, 0, 0))
		elif phase == "view":
			if (
				actor.last_field_preparation_milliseconds != 0 or samples() != state["baseline"] or
				drainage_samples() != state["drainage_baseline"]
			):
				raise RuntimeError("Changing the observer recomputed or changed the immutable field")
			state["phase"] = "replacement"
			actor.set_editor_property("seed", 43)
			request()
			actor.set_editor_property("seed", 44)
		elif phase == "replacement":
			if actor.discarded_requests < 1:
				raise RuntimeError("Obsolete generation was not discarded")
			revision = actor.published_revision
			mesh = actor.get_visible_mesh().get_path_name()
			actor.set_editor_property("radius_meters", 0.0)
			if actor.request_generation() or actor.published_revision != revision or actor.get_visible_mesh().get_path_name() != mesh:
				raise RuntimeError("Invalid radius damaged the published terrain")
			actor.set_editor_property("radius_meters", 1000000.0)
			actor.set_editor_property("seed", 42)
			actor.set_editor_property("observer_meters", unreal.Vector(-2100000, 2100000, 900000))
			state["phase"] = "restore"
		else:
			if samples() != state["baseline"] or drainage_samples() != state["drainage_baseline"]:
				raise RuntimeError("Independent asynchronous regeneration changed the field")
			location = actor.get_actor_location()
			camera = location + unreal.Vector(-210000000, 210000000, 90000000)
			editor.set_level_viewport_camera_info(camera, unreal.MathLibrary.find_look_at_rotation(camera, location))
			actors.set_selected_level_actors([actor])
			levels.editor_invalidate_viewports()
			finish()
			# 全段階を完了した更新の終了
			return
		state["previous_revision"] = actor.published_revision
		state["previous_mesh"] = actor.get_visible_mesh().get_path_name()
		state["previous_ocean"] = actor.get_visible_ocean_mesh().get_path_name()
		request()
	except Exception as error:
		finish(str(error))
	# 次の UE 更新へ処理を戻す終了
	return

try:
	request()
	state["handle"] = unreal.register_slate_post_tick_callback(tick)
	RESULT = {
		"status": "started", "actor": actor.get_path_name(), "first_request_ms": state["request_ms"][0],
		"read_result_with": "Scripts/Editor/read_streaming_probe.py"
	}
except Exception as error:
	finish(str(error))
	RESULT = state["result"]
