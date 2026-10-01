"""実際の草の根を地形関数・法線と比較し，表示状態と処理時間を読む"""
import unreal

WORLD = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if WORLD is None:
	raise RuntimeError("探索を開始して下さい")
MODE = unreal.GameplayStatics.get_game_mode(WORLD)
if not isinstance(MODE, unreal.PlanetGameMode) or MODE.explorer is None:
	raise RuntimeError("探索の準備完了を待って下さい")
ACTORS = unreal.GameplayStatics.get_all_actors_of_class(WORLD, unreal.PlanetVegetation)
if len(ACTORS) != 1:
	raise RuntimeError("植生の実体が一つではありません")
VEGETATION = ACTORS[0]
MESH = VEGETATION.get_visible_grass()
PLANET = MODE.planet
COUNT = MESH.get_instance_count()
if COUNT != VEGETATION.grass_instance_count:
	raise RuntimeError("公開した草の数と実際の描画部品が一致しません")
ROOTS = []
maximum_height_error = 0
maximum_normal_error = 0
for index in range(0, COUNT, max(1, COUNT // 64)):
	transform = MESH.get_instance_transform(index, world_space=True)
	local = unreal.MathLibrary.inverse_transform_location(PLANET.get_actor_transform(), transform.translation) / 100
	direction = local / local.length()
	height, normal = PLANET.evaluate_direction(direction)
	error = abs(local.length() - PLANET.radius_meters - height)
	maximum_height_error = max(maximum_height_error, error)
	up = unreal.MathLibrary.transform_direction(transform, unreal.Vector(0, 0, 1))
	expected = unreal.MathLibrary.transform_direction(PLANET.get_actor_transform(), normal)
	normal_error = (up / up.length() - expected / expected.length()).length()
	maximum_normal_error = max(maximum_normal_error, normal_error)
	ROOTS.append({"index": index, "heightErrorMeters": error, "normalError": normal_error})
RESULT = {
	"instanceCount": COUNT, "revision": VEGETATION.published_revision, "preparing": VEGETATION.preparing,
	"heightEvaluations": VEGETATION.height_evaluations, "reusedCandidates": VEGETATION.reused_candidates,
	"queryMilliseconds": VEGETATION.last_query_milliseconds,
	"preparationSliceMilliseconds": VEGETATION.last_preparation_milliseconds,
	"uploadMilliseconds": VEGETATION.last_upload_milliseconds,
	"maximumRootHeightErrorMeters": maximum_height_error, "maximumRootNormalError": maximum_normal_error,
	"rootSamples": ROOTS, "status": VEGETATION.status, "radiusMeters": PLANET.radius_meters
}
if maximum_height_error > 0.02 or maximum_normal_error > 0.001:
	raise RuntimeError("草の根が同じ地表から外れました：" + str(RESULT))
