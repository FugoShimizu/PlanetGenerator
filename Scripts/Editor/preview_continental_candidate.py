"""解析的な大陸候補を一時表示し，元カメラを復元出来る状態で保持する．"""

from __future__ import annotations
import builtins
import math
from typing import cast
import unreal

# 操作引数と所有資源の識別
args: dict[str, object] = cast(dict[str, object], globals().get("ARGS", {}))
RESULT: dict[str, object]
STATE_KEY = "_planet_continental_candidate_preview_state"
actors: unreal.EditorActorSubsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels: unreal.LevelEditorSubsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
editor: unreal.UnrealEditorSubsystem = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)

if levels.is_in_play_in_editor():
	raise RuntimeError("Stop PIE before the continental candidate comparison.")

world: unreal.World = editor.get_editor_world()
state: dict[str, object]|None = cast(dict[str, object] | None, builtins.__dict__.get(STATE_KEY))

if state is None:
	# 此の操作だけが所有する一時アクターと，開始前の画面状態の保持
	camera: tuple[unreal.Vector, unreal.Rotator] = unreal.EditorLevelLibrary.get_level_viewport_camera_info()
	initial_count: int = len(unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor))
	created: unreal.Actor|None = actors.spawn_actor_from_class(
		unreal.PlanetContinentalPreview,
		unreal.Vector(0, 0, 1000000),
		transient=True
	)
	if created is None:
		raise RuntimeError("The transient comparison actor could not be created.")
	created.set_actor_label("Planet_ContinentalCandidate_Owned")
	state = { "actor": created, "camera": camera, "initialActorCount": initial_count }
	builtins.__dict__[STATE_KEY] = state

actor: unreal.PlanetContinentalPreview = cast(unreal.PlanetContinentalPreview, state["actor"])
seed: int = int(cast(int, args.get("seed", 42)))
radius: float = float(cast(float, args.get("radius_meters", 1000000)))
mode: str = str(args.get("mode", "global"))
initial_preparation: float = 0
refinement_steps: int = 0
coarse_highest: float|None = None

if mode == "global":
	# 全球の表示だけを１０ｍの診断球へ写す，数値場の尺度は不変
	level: int = int(cast(int, args.get("subdivision_level", 6)))
	scale: float = float(cast(float, args.get("height_scale", 1)))
	actor.build_global_preview(seed, radius, level, scale)
	offset: unreal.Vector = unreal.Vector(2600, -3000, 1400)
elif mode == "local":
	# 同じ候補を準備し，指定が無ければ有限標本で高い場所を選ぶ
	actor.build_global_preview(seed, radius, 0, 1)
	if not bool(actor.get_editor_property("HasSucceededLastBuild")):
		raise RuntimeError(str(actor.get_editor_property("LastBuildMessage")))
	initial_preparation = float(actor.get_editor_property("PreparationMilliseconds"))
	anchor: unreal.Vector = unreal.Vector(1, 0, 0)
	highest: float = -math.inf
	sample_count: int = int(cast(int, args.get("candidate_samples", 1024)))
	refinement_limit: int = int(cast(int, args.get("refinement_max_steps", 128)))
	if sample_count < 128 or sample_count > 32768 or refinement_limit < 1 or refinement_limit > 1024:
		raise ValueError("The observation search exceeds its finite limits.")
	for i in range(sample_count):
		z: float = 1 - 2 * (i + 0.5) / sample_count
		angle: float = i * 2.399963229728653
		ring: float = math.sqrt(1 - z * z)
		direction: unreal.Vector = unreal.Vector(ring * math.cos(angle), ring * math.sin(angle), z)
		sample: tuple[float, unreal.Vector]|None = actor.can_evaluate_direction(direction)
		if sample is None:
			raise RuntimeError("A valid comparison direction could not be queried.")
		if float(sample[0]) > highest:
			highest = float(sample[0])
			anchor = direction
	coarse_highest = highest
	if "anchor_direction" in args:
		components: list[float] = cast(list[float], args["anchor_direction"])
		anchor = unreal.Vector(*components)
	elif bool(args.get("should_refine_highland", True)):
		# 観察地点だけを上へ追跡する
		# 本番の生成・表示地形の変更や全球最大の保証ではない
		step: float = 0.005
		for iteration in range(refinement_limit):
			current: tuple[float, unreal.Vector]|None = actor.can_evaluate_direction(anchor)
			if current is None:
				raise RuntimeError("The selected highland could not be queried.")
			normal: unreal.Vector = current[1]
			dot: float = anchor.x * normal.x + anchor.y * normal.y + anchor.z * normal.z
			if not math.isfinite(dot) or dot <= 0:
				raise RuntimeError("The highland normal does not resolve its outward direction.")
			gx: float = anchor.x - normal.x / dot
			gy: float = anchor.y - normal.y / dot
			gz: float = anchor.z - normal.z / dot
			length: float = math.hypot(gx, gy, gz)
			if length <= 1E-14:
				break
			heading: unreal.Vector = unreal.Vector(gx / length, gy / length, gz / length)
			has_improved: bool = False
			trial_step: float = step
			for halving in range(20):
				candidate: unreal.Vector = anchor * math.cos(trial_step) + heading * math.sin(trial_step)
				candidate_length: float = math.hypot(candidate.x, candidate.y, candidate.z)
				candidate = unreal.Vector(candidate.x / candidate_length, candidate.y / candidate_length, candidate.z / candidate_length)
				trial: tuple[float, unreal.Vector]|None = actor.can_evaluate_direction(candidate)
				if trial is None:
					raise RuntimeError("A valid observation direction could not be queried.")
				if float(trial[0]) > float(current[0]):
					anchor = candidate
					highest = float(trial[0])
					step = min(0.01, trial_step * 1.25)
					has_improved = True
					refinement_steps += 1
					break
				trial_step *= 0.5
			if not has_improved:
				break
	extent: float = float(cast(float, args.get("half_extent_meters", min(10000, 0.15 * radius))))
	cells: int = int(cast(int, args.get("cells", 128)))
	actor.build_local_preview(seed, radius, anchor, extent, cells)
	offset = unreal.Vector(extent * 80, -extent * 160, extent * 90)
else:
	raise ValueError("mode must be global or local.")

if not bool(actor.get_editor_property("HasSucceededLastBuild")):
	raise RuntimeError(str(actor.get_editor_property("LastBuildMessage")))

# 確定した表示へカメラを移し，画像を保存せずに観察
anchor_result: unreal.Vector = actor.get_anchor_direction()
position: unreal.Vector = actor.get_actor_location() + offset
yaw: float = math.degrees(math.atan2(-offset.y, -offset.x))
pitch: float = math.degrees(math.atan2(-offset.z, math.hypot(offset.x, offset.y)))
rotation: unreal.Rotator = unreal.Rotator(pitch=pitch, yaw=yaw, roll=0)
unreal.EditorLevelLibrary.set_level_viewport_camera_info(position, rotation)
levels.editor_invalidate_viewports()

RESULT = {
	"passed": True,
	"mode": mode,
	"seed": seed,
	"radiusMeters": radius,
	"preparationMilliseconds": float(actor.get_editor_property("PreparationMilliseconds")) or initial_preparation,
	"meshMilliseconds": float(actor.get_editor_property("MeshMilliseconds")),
	"vertices": int(actor.get_editor_property("GeneratedVertices")),
	"triangles": int(actor.get_editor_property("GeneratedTriangles")),
	"oceanFraction": float(actor.get_editor_property("OceanFraction")),
	"coarseHighlandElevationMeters": coarse_highest,
	"observationRefinementSteps": refinement_steps,
	"candidateSamples": sample_count if mode == "local" else 0,
	"refinementLimit": refinement_limit if mode == "local" else 0,
	"anchorElevationMeters": float(actor.get_editor_property("AnchorElevationMeters")),
	"anchorDirection": [float(anchor_result.x), float(anchor_result.y), float(anchor_result.z)],
	"message": str(actor.get_editor_property("LastBuildMessage")),
	"terrainAcceptance": False
}
