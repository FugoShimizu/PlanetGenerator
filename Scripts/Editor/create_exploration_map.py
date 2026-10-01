"""生成済み地形を含まない，C++ 探索用のレベルを作成する"""
import unreal


def create_map() -> dict:
	"""探索用の設定・照明を保存し，作成結果を返す"""
	levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
	actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
	editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
	path = "/Game/Planet/Maps/Lvl_Planet"
	if levels.is_in_play_in_editor():
		raise RuntimeError("探索レベルの作成前に PIE を停止して下さい")
	if (unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages() or
		unreal.EditorLoadingAndSavingUtils.get_dirty_content_packages()):
		raise RuntimeError("未保存の変更が有る為，レベルを切り替えません")
	if unreal.EditorAssetLibrary.does_asset_exist(path):
		if not levels.load_level(path):
			raise RuntimeError("作成済みの探索レベルを開けません")
		# 作成済みの設定を上書きしない返戻
		return {"created": False, "map": path, "opened": True}
	if not levels.new_level(path):
		raise RuntimeError("探索レベルを作成出来ません")
	world = editor.get_editor_world()
	world.get_world_settings().set_editor_property("default_game_mode", unreal.PlanetGameMode)
	planet = actors.spawn_actor_from_class(unreal.PlanetWorld, unreal.Vector())
	planet.set_actor_label("手続型惑星地形生成器")
	planet.set_editor_property("seed", 42)
	planet.set_editor_property("radius_meters", 1000000.0)
	planet.set_editor_property("generate_on_begin_play", False)
	planet.set_editor_property("follow_player_camera", False)
	planet.set_editor_property("leaf_budget", 32768)
	planet.set_editor_property("solar_year_phase", 0.25)
	sun = actors.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0, 0, 1000))
	sun.set_actor_label("Sun")
	sun.set_actor_rotation(unreal.Rotator(pitch=-35, yaw=-30, roll=0), False)
	light = sun.get_component_by_class(unreal.DirectionalLightComponent)
	light.set_mobility(unreal.ComponentMobility.MOVABLE)
	light.set_editor_property("atmosphere_sun_light", True)
	light.set_intensity(80000.0)
	sky = actors.spawn_actor_from_class(unreal.SkyLight, unreal.Vector())
	sky.set_actor_label("SkyLight")
	sky_light = sky.get_component_by_class(unreal.SkyLightComponent)
	sky_light.set_mobility(unreal.ComponentMobility.MOVABLE)
	sky_light.set_real_time_capture(True)
	for kind, label in ((unreal.SkyAtmosphere, "SkyAtmosphere"), (unreal.VolumetricCloud, "Clouds")):
		actor = actors.spawn_actor_from_class(kind, unreal.Vector())
		actor.set_actor_label(label)
	editor.set_level_viewport_camera_info(unreal.Vector(0, -1000, 400), unreal.Rotator(pitch=-15, yaw=90, roll=0))
	actors.set_selected_level_actors([planet])
	if not levels.save_current_level():
		raise RuntimeError("探索レベルの設定を保存出来ません")
	# 地形データではなく，生成設定と照明だけを保存した結果の返戻
	return {
		"created": True, "map": path, "gameMode": "/Script/PlanetGenerator.PlanetGameMode",
		"seed": planet.seed, "radiusMeters": planet.radius_meters,
		"savedGeneratedTerrain": False, "celestialMotionImplemented": False, "blueprintsAdded": False
	}


RESULT = create_map()
