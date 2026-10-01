"""公開された樹木の根・重力方向・幹の衝突部品を実物から検査する"""
import hashlib
import json
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
MESHES = {"broadleaf": VEGETATION.get_trees(), "broadleafVariant": VEGETATION.get_tree_variant(),
	"conifer": VEGETATION.get_conifers()}
PLANET = MODE.planet
COUNT = sum(mesh.get_instance_count() for mesh in MESHES.values())
CAPSULES = VEGETATION.get_components_by_class(unreal.CapsuleComponent)
if COUNT != VEGETATION.tree_instance_count:
	raise RuntimeError("樹木と幹の衝突部品の数が一致しません")
ROOTS = []
TRANSFORMS = []
maximum_height_error = 0
maximum_radial_error = 0
near_collision_count = 0
center_direction = VEGETATION.published_tree_center_meters / VEGETATION.published_tree_center_meters.length()
for species, mesh in MESHES.items():
	for index in range(mesh.get_instance_count()):
		TRANSFORM = mesh.get_instance_transform(index, world_space=True)
		LOCAL = unreal.MathLibrary.inverse_transform_location(PLANET.get_actor_transform(), TRANSFORM.translation) / 100
		DIRECTION = LOCAL / LOCAL.length()
		if (DIRECTION - center_direction).length() * PLANET.radius_meters <= 24:
			near_collision_count += 1
		HEIGHT, NORMAL = PLANET.evaluate_direction(DIRECTION)
		ERROR = abs(LOCAL.length() - PLANET.radius_meters - HEIGHT + 0.1 * TRANSFORM.scale3d.z)
		maximum_height_error = max(maximum_height_error, ERROR)
		UP = unreal.MathLibrary.transform_direction(TRANSFORM, unreal.Vector(0, 0, 1))
		EXPECTED = unreal.MathLibrary.transform_direction(PLANET.get_actor_transform(), DIRECTION)
		RADIAL_ERROR = (UP / UP.length() - EXPECTED / EXPECTED.length()).length()
		maximum_radial_error = max(maximum_radial_error, RADIAL_ERROR)
		TRANSFORMS.append([
			TRANSFORM.translation.x, TRANSFORM.translation.y, TRANSFORM.translation.z,
			TRANSFORM.rotation.x, TRANSFORM.rotation.y, TRANSFORM.rotation.z, TRANSFORM.rotation.w,
			TRANSFORM.scale3d.x, TRANSFORM.scale3d.y, TRANSFORM.scale3d.z
		])
		ROOTS.append({"species": species, "index": index, "heightErrorMeters": ERROR, "radialError": RADIAL_ERROR})
if len(CAPSULES) != near_collision_count:
	raise RuntimeError("近景の幹衝突数が樹木の距離条件と一致しません")
STAGES = VEGETATION.last_upload_stages_milliseconds
RESULT = {
	"treeCount": COUNT, "nearCollisionExpected": near_collision_count,
	"collisionCount": len(CAPSULES), "revision": VEGETATION.tree_published_revision,
	"preparationSliceMilliseconds": VEGETATION.last_tree_slice_milliseconds,
	"uploadMilliseconds": VEGETATION.last_tree_upload_milliseconds,
	"maximumRootHeightErrorMeters": maximum_height_error, "maximumRootRadialError": maximum_radial_error,
	"rootSamples": ROOTS, "radiusMeters": PLANET.radius_meters,
	"transformHash": hashlib.sha256(json.dumps(TRANSFORMS).encode()).hexdigest(),
	"collisionNames": sorted(str(item.get_name()) for item in CAPSULES),
	"grassUploadStagesMilliseconds": {"clear": STAGES.x, "add": STAGES.y, "customData": STAGES.z},
	"grassUploadMilliseconds": VEGETATION.last_upload_milliseconds,
	"speciesCounts": {name: mesh.get_instance_count() for name, mesh in MESHES.items()},
	"instanceCustomData": {name: list(mesh.get_editor_property("per_instance_sm_custom_data"))
		for name, mesh in MESHES.items()}
}
if maximum_height_error > 0.02 or maximum_radial_error > 0.001:
	raise RuntimeError("樹木の根又は重力方向が地形から外れました：" + str(RESULT))
