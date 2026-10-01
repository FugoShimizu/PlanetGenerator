"""樹木素材だけを一時表示し，終了時に元の視点と実体数へ戻す"""
import builtins
import unreal

KEY = "_planet_tree_asset_preview"
EDITOR = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
ACTORS = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if EDITOR.get_game_world() is not None:
	raise RuntimeError("樹木素材の確認前に PIE を停止して下さい")
state = getattr(builtins, KEY, None)
if globals().get("ARGS", {}).get("operation", "begin") == "cleanup":
	if state is None:
		raise RuntimeError("終了する樹木素材の確認が有りません")
	for actor in state["actors"]:
		if not ACTORS.destroy_actor(actor):
			raise RuntimeError("所有する一時実体を破棄出来ません")
	EDITOR.set_level_viewport_camera_info(*state["camera"])
	CURRENT = len(ACTORS.get_all_level_actors())
	if CURRENT != state["originalActorCount"]:
		raise RuntimeError("確認前の実体数へ戻っていません")
	delattr(builtins, KEY)
	RESULT = {"restored": True, "actorCount": CURRENT}
else:
	if state is not None:
		raise RuntimeError("既存の樹木素材の確認を終了して下さい")
	state = {
		"actors": [], "camera": EDITOR.get_level_viewport_camera_info(), "originalActorCount": len(ACTORS.get_all_level_actors())
	}
	setattr(builtins, KEY, state)
	TREE = ACTORS.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, 0), transient=True)
	state["actors"].append(TREE)
	TREE.set_actor_label("Planet_TreeAssetPreview")
	ASSET = globals().get("ARGS", {}).get("tree_asset", "/Game/Planet/Vegetation/SM_BroadleafSapling")
	if ASSET not in ("/Game/Planet/Vegetation/SM_BroadleafSapling", "/Game/Planet/Vegetation/SM_ConiferSapling"):
		raise RuntimeError("Unknown tree preview asset")
	TREE.static_mesh_component.set_static_mesh(unreal.load_asset(ASSET))
	TREE.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
	FLOOR = ACTORS.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, -10), transient=True)
	state["actors"].append(FLOOR)
	FLOOR.set_actor_label("Planet_TreeAssetPreviewFloor")
	FLOOR.static_mesh_component.set_static_mesh(unreal.load_asset("/Engine/BasicShapes/Cube"))
	FLOOR.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
	FLOOR.set_actor_scale3d(unreal.Vector(30, 30, 0.2))
	POSITION = unreal.Vector(800, -800, 350)
	ROTATION = unreal.MathLibrary.find_look_at_rotation(POSITION, unreal.Vector(0, 0, 260))
	EDITOR.set_level_viewport_camera_info(POSITION, ROTATION)
	RESULT = {"previewActors": len(state["actors"]), "terrainGeneration": False, "saved": False}
