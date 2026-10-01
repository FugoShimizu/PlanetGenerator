"""完了済みの地表検査を使い，実際の河川水面と不変な基準水位を照合する"""
import builtins
import hashlib
import math
import struct
import time
import unreal

VectorTuple = tuple[float, float, float]
source_state = getattr(builtins, "_planet_surface_probe", None)
if not source_state or source_state.get("status") != "passed":
	raise RuntimeError("Complete the surface render and collision probe first")
previous = getattr(builtins, "_planet_river_render_probe", None)
if previous and previous.get("status") == "running":
	raise RuntimeError("River water render probe is already running")
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
found = [
	a for a in unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(), unreal.PlanetTerrain)
	if a.get_actor_label() == "Planet_SurfaceRenderProbe"
]
if len(found) != 1:
	raise RuntimeError("Expected one owned surface probe actor")
terrain = found[0]
if terrain.generation_busy:
	raise RuntimeError("Wait for the existing terrain request")
source_result = source_state["result"]
args = globals().get("ARGS", {})
target_fraction = float(args.get("river_fraction", source_result.get("river_fraction", 0.5)))
if not math.isfinite(target_fraction) or not 0 <= target_fraction <= 1:
	raise RuntimeError("River fraction must be finite and in [0, 1]")
reach = source_result.get("river_reach_index")
if reach is None:
	raise RuntimeError("Surface probe must target a river reach")
state: dict[str, object] = {
	"status": "running", "start": time.monotonic(), "handle": None, "actor": terrain,
	"old_mesh": terrain.get_visible_ocean_mesh().get_path_name(), "old_revision": terrain.published_revision,
	"old_flag": terrain.get_editor_property("render_river_water"), "retained_frames": 0
}
builtins._planet_river_render_probe = state


def vector(value: unreal.Vector) -> VectorTuple:
	"""UE の倍精度座標を比較用の成分へ移す"""
	# 同じ座標成分の返戻
	return (value.x, value.y, value.z)


def subtract(a: VectorTuple, b: VectorTuple) -> VectorTuple:
	"""三次元の差分を返す"""
	# 差分の返戻
	return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a: VectorTuple, b: VectorTuple) -> float:
	"""三次元の内積を返す"""
	# 内積の返戻
	return sum(x * y for x, y in zip(a, b))


def cross(a: VectorTuple, b: VectorTuple) -> VectorTuple:
	"""三次元の外積を返す"""
	# 外積の返戻
	return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def inspect() -> dict[str, object]:
	"""実際の描画頂点を単精度に変換し，独立な河道中心の水位へレイを当てる"""
	mesh = terrain.get_visible_ocean_mesh()
	vertices, indices, normals, uvs, tangents = unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh, 0)
	if not vertices or len(indices) % 3:
		raise RuntimeError("Missing river water mesh")
	# SceneProxy の位置変換を模擬するもので，GPU の読戻しではない
	points: list[VectorTuple] = [
		tuple(struct.unpack("<f", struct.pack("<f", c))[0] for c in vector(v)) for v in vertices
	]
	faces = [tuple(int(indices[i + k]) for k in range(3)) for i in range(0, len(indices), 3)]
	if len({tuple(sorted(f)) for f in faces}) != len(faces):
		raise RuntimeError("Duplicate water faces")
	edges: dict[tuple[int, int], tuple[int, int]] = {}
	for face in faces:
		for k, a in enumerate(face):
			b = face[(k + 1) % 3]
			key = (min(a, b), max(a, b))
			count, sign = edges.get(key, (0, 0))
			edges[key] = (count + 1, sign + (1 if a < b else -1))
	if any(count != 1 and (count != 2 or sign != 0) for count, sign in edges.values()):
		raise RuntimeError("Water edge incidence or orientation is inconsistent")
	# 座標と面番号を記録し，再利用の最適化で実形状が変わらない事を照合する
	digest = hashlib.sha256()
	for vertex in vertices:
		digest.update(struct.pack("<ddd", vertex.x, vertex.y, vertex.z))
	for index in indices:
		digest.update(struct.pack("<I", index))
	parents = list(range(len(vertices)))

	def root(index: int) -> int:
		"""共有頂点に依る連結成分の代表を求める"""
		while parents[index] != index:
			parents[index] = parents[parents[index]]
			index = parents[index]
		# 経路圧縮後の代表番号の返戻
		return index

	for face in faces:
		representative = root(face[0])
		for index in face[1:]:
			parents[root(index)] = representative
	component_sizes: dict[int, int] = {}
	for face in faces:
		representative = root(face[0])
		component_sizes[representative] = component_sizes.get(representative, 0) + 1
	transform = mesh.get_world_transform()
	planet_transform = terrain.get_actor_transform()
	start = terrain.evaluate_river_profile(reach, 0.0)[0]
	end = terrain.evaluate_river_profile(reach, 1.0)[0]
	a, b = vector(start / start.length()), vector(end / end.length())
	length = terrain.radius_meters * math.atan2(math.sqrt(dot(cross(a, b), cross(a, b))), dot(a, b))
	fraction = target_fraction
	maximum_error = 0.0
	sample_count = 0
	candidate_count = 0
	if not math.isfinite(length) or length <= 0:
		raise RuntimeError("River reach has no finite positive length")
	# 終点付近でも同じ端点を重複して数えず，利用可能な区間に１７地点を分散する
	first = max(0.0, fraction - 8 / length)
	last = min(1.0, fraction + 8 / length)
	fractions = [first + (last - first) * i / 16 for i in range(17)]
	if len(set(fractions)) != 17:
		raise RuntimeError("River sample interval has insufficient precision")
	for i, f in enumerate(fractions):
		position, water, bed, tangent = terrain.evaluate_river_profile(reach, f)
		d = position / position.length()
		actual_water, ground = terrain.evaluate_water_direction(d)
		if actual_water - ground <= 0.02:
			raise RuntimeError("Expected a submerged channel centre")
		point = unreal.MathLibrary.transform_location(planet_transform, d * ((terrain.radius_meters + actual_water) * 100))
		up = unreal.MathLibrary.transform_direction(planet_transform, d)
		origin = vector(unreal.MathLibrary.inverse_transform_location(transform, point + up * 500))
		end_point = vector(unreal.MathLibrary.inverse_transform_location(transform, point - up * 500))
		ray = subtract(end_point, origin)
		hits: list[float] = []
		for face in faces:
			a, b, c = (points[j] for j in face)
			# １０ m の線分と軸平行境界が離れた面は交差しない
			if any(
				max(origin[k], end_point[k]) < min(a[k], b[k], c[k]) or
				min(origin[k], end_point[k]) > max(a[k], b[k], c[k]) for k in range(3)
			):
				continue
			candidate_count += 1
			e1, e2 = subtract(b, a), subtract(c, a)
			p = cross(ray, e2)
			determinant = dot(e1, p)
			if abs(determinant) < 1E-12:
				continue
			s = subtract(origin, a)
			u = dot(s, p) / determinant
			q = cross(s, e1)
			v = dot(ray, q) / determinant
			t = dot(e2, q) / determinant
			if u >= -1E-10 and v >= -1E-10 and u + v <= 1 + 1E-10 and 0 <= t <= 1:
				hits.append(t)
		if not hits:
			raise RuntimeError(f"Missing rendered water at channel sample {i}")
		if max(hits) - min(hits) > 1E-5:
			raise RuntimeError("Overlapping distinct river water surfaces")
		error = abs(hits[0] - 0.5) * 10
		maximum_error = max(maximum_error, error)
		# 水位補間 1 cm と単精度位置 1 mm，座標変換の丸め余裕 0.01 mm
		if error > 0.01101:
			raise RuntimeError(f"Rendered water differs from the canonical surface by {error:.9g} m")
		sample_count += 1
	# 実際の描画と独立試料の照合結果の返戻
	return {
		"vertices": len(points), "triangles": len(faces), "channel_samples": sample_count,
		"ray_candidates": candidate_count, "maximum_water_error_m": maximum_error,
		"mesh_sha256": digest.hexdigest(), "distinct_channel_samples": len(set(fractions)),
		"water_components": len(component_sizes), "component_faces": sorted(component_sizes.values(), reverse=True),
		"shore_edges": sum(count == 1 for count, sign in edges.values()), "seed": terrain.seed,
		"radius_meters": terrain.radius_meters, "river_reach_index": reach, "river_fraction": fraction,
		"observer_meters": vector(terrain.observer_meters),
		"worker_ms": terrain.last_worker_milliseconds, "details": terrain.generation_message,
		"gpu_readback": False, "gpu_position_conversion_emulated": True, "all_water_coverage_proven": False
	}


def finish(error: str | None = None, details: dict[str, object] | None = None) -> None:
	"""検査の成否を保存し，更新コールバックを解放する"""
	state["status"] = "failed" if error else "passed"
	if state["handle"] is not None:
		unreal.unregister_slate_post_tick_callback(state["handle"])
		state["handle"] = None
	state["result"] = {
		"status": state["status"], "error": error, "seconds": time.monotonic() - state["start"],
		"retained_frames": state["retained_frames"], "details": details, "saved_terrain_assets": False
	}
	state.pop("actor", None)
	# 終了
	return


def tick(delta: float) -> None:
	"""候補の公開と旧表示の維持を確認してから水面を検査する"""
	try:
		if time.monotonic() - state["start"] > 60:
			raise RuntimeError("River water render probe timed out")
		terrain.poll_generation()
		if terrain.generation_busy:
			old = terrain.get_visible_ocean_mesh()
			if old is None or not old.is_mesh_section_visible(0):
				raise RuntimeError("Existing water coverage disappeared")
			state["retained_frames"] += 1
			# 未完成の候補を検査せず次の更新を待つ終了
			return
		if terrain.published_revision != state["target"]:
			raise RuntimeError(terrain.generation_message)
		finish(details=inspect())
	except Exception as error:
		finish(str(error))
	# 終了
	return


try:
	terrain.set_editor_property("render_river_water", True)
	if not terrain.request_generation():
		raise RuntimeError(terrain.generation_message)
	state["target"] = terrain.requested_revision
	state["handle"] = unreal.register_slate_post_tick_callback(tick)
	RESULT = {"status": "started", "read_with": "Scripts/Editor/read_river_render_probe.py"}
except Exception as error:
	finish(str(error))
	RESULT = state["result"]
