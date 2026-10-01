"""既定森林の樹形比率と変形後の幹衝突を PIE の実物で検査する"""
import unreal


def unit(vector):
	"""非零の UE ベクトルを正規化する．"""
	# 単位方向の返戻．
	return vector / vector.length()


world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_game_world()
if world is None:
	raise RuntimeError("探索モードで PIE を開始して下さい")
mode = unreal.GameplayStatics.get_game_mode(world)
if not isinstance(mode, unreal.PlanetGameMode) or mode.explorer is None:
	raise RuntimeError("探索開始後に検査して下さい")
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetVegetation)
if len(actors) != 1:
	raise RuntimeError("植生 Actor の数が不正です")
vegetation = actors[0]
meshes = (vegetation.get_trees(), vegetation.get_tree_variant())
conifers = vegetation.get_conifers()
capsules = vegetation.get_components_by_class(unreal.CapsuleComponent)
broadleaf_count = sum(mesh.get_instance_count() for mesh in meshes)
if any(mesh.get_instance_count() == 0 for mesh in meshes) or not capsules:
	raise RuntimeError("広葉樹又は幹衝突が不足しています")

trunk_axis = unit(unreal.Vector(0.05, -0.035, 1))
aspects = []
maximum_center_error_cm = 0.0
maximum_radius_error_cm = 0.0
maximum_half_height_error_cm = 0.0
maximum_axis_error = 0.0
maximum_root_error_m = 0.0
matched = set()
center = unit(vegetation.published_tree_center_meters)
for mesh in meshes:
	for index in range(mesh.get_instance_count()):
		transform = mesh.get_instance_transform(index, world_space=True)
		scale = transform.scale3d
		aspects.append(scale.x / scale.z)
		local = unreal.MathLibrary.inverse_transform_location(mode.planet.get_actor_transform(), transform.translation) / 100
		direction = unit(local)
		if (direction - center).length() * mode.planet.radius_meters > 24:
			continue
		expected_center = unreal.MathLibrary.transform_location(transform, trunk_axis * 255)
		collision = min(capsules, key=lambda item: (item.get_world_location() - expected_center).length())
		if collision.get_name() in matched:
			raise RuntimeError("二本の樹木が同じ幹衝突を参照しています")
		matched.add(collision.get_name())
		maximum_center_error_cm = max(maximum_center_error_cm,
			(collision.get_world_location() - expected_center).length())
		scaled_axis = unreal.Vector(trunk_axis.x * scale.x, trunk_axis.y * scale.y, trunk_axis.z * scale.z)
		maximum_radius_error_cm = max(maximum_radius_error_cm,
			abs(collision.get_scaled_capsule_radius() - 13 * max(scale.x, scale.y)))
		maximum_half_height_error_cm = max(maximum_half_height_error_cm,
			abs(collision.get_scaled_capsule_half_height() - 255 * scaled_axis.length()))
		expected_up = unit(expected_center - transform.translation)
		maximum_axis_error = max(maximum_axis_error,
			(unit(collision.get_up_vector()) - expected_up).length())
		local = unreal.MathLibrary.inverse_transform_location(mode.planet.get_actor_transform(), transform.translation) / 100
		direction = unit(local)
		height, _ = mode.planet.evaluate_direction(direction)
		maximum_root_error_m = max(maximum_root_error_m,
			abs(local.length() - mode.planet.radius_meters - height + 0.1 * scale.z))

RESULT = {
	"seed": mode.planet.seed,
	"radius_meters": mode.planet.radius_meters,
	"broadleaf_count": broadleaf_count,
	"variant_count": meshes[1].get_instance_count(),
	"conifer_count": conifers.get_instance_count(),
	"collision_count": len(capsules),
	"minimum_aspect": min(aspects),
	"maximum_aspect": max(aspects),
	"maximum_center_error_cm": maximum_center_error_cm,
	"maximum_radius_error_cm": maximum_radius_error_cm,
	"maximum_half_height_error_cm": maximum_half_height_error_cm,
	"maximum_axis_error": maximum_axis_error,
	"maximum_root_error_m": maximum_root_error_m,
}
if max(aspects) - min(aspects) < 0.35 or maximum_center_error_cm > 0.01 or (
	maximum_radius_error_cm > 0.01 or maximum_half_height_error_cm > 0.01 or
	maximum_axis_error > 0.001 or maximum_root_error_m > 0.02
):
	raise RuntimeError("広葉樹の樹形又は幹衝突が地表と一致しません: " + str(RESULT))
