"""第２広葉樹の静的メッシュへ既存の樹皮・葉材質と距離別形状を設定する．"""
from pathlib import Path

import unreal


if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("樹木素材の構築前に PIE を停止して下さい")
path = "/Game/Planet/Vegetation/SM_BroadleafSaplingVariant"
mesh = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
if mesh is None:
	options = unreal.FbxImportUI()
	options.set_editor_property("import_materials", False)
	options.set_editor_property("import_textures", False)
	options.set_editor_property("import_as_skeletal", False)
	options.set_editor_property("automated_import_should_detect_type", False)
	options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
	options.static_mesh_import_data.set_editor_property("auto_generate_collision", False)
	options.static_mesh_import_data.set_editor_property("convert_scene", False)
	options.static_mesh_import_data.set_editor_property("combine_meshes", True)
	task = unreal.AssetImportTask()
	for key, value in {
		"filename": str(Path(unreal.Paths.project_dir()) / "SourceAssets/Vegetation/BroadleafSaplingVariant.obj"),
		"destination_path": "/Game/Planet/Vegetation", "destination_name": "SM_BroadleafSaplingVariant",
		"automated": True, "save": False, "options": options
	}.items():
		task.set_editor_property(key, value)
	unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
	mesh = unreal.load_asset(path)
if not isinstance(mesh, unreal.StaticMesh):
	raise RuntimeError("別樹冠の広葉樹メッシュを読込めません")
slots = mesh.get_editor_property("static_materials")
names = [str(slot.material_slot_name) for slot in slots]
if set(names) != {"Bark", "Leaves"}:
	raise RuntimeError("樹皮と葉の区画が揃っていません: " + str(names))
for index, name in enumerate(names):
	material = unreal.load_asset("/Game/Planet/Materials/M_PlanetTree" + ("Bark" if name == "Bark" else "Leaves"))
	if not isinstance(material, unreal.Material):
		raise RuntimeError("既存の樹木材質を読込めません: " + name)
	mesh.set_material(index, material)
reductions = []
for ratio, screen_size in ((1, 1), (0.5, 0.3), (0.2, 0.12)):
	setting = unreal.StaticMeshReductionSettings()
	setting.set_editor_property("percent_triangles", ratio)
	setting.set_editor_property("screen_size", screen_size)
	reductions.append(setting)
settings = unreal.StaticMeshReductionOptions()
settings.set_editor_property("auto_compute_lod_screen_size", False)
settings.set_editor_property("reduction_settings", reductions)
subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
if subsystem.set_lods(mesh, settings) != 3 or not unreal.EditorAssetLibrary.save_loaded_asset(mesh):
	raise RuntimeError("別樹冠の距離別形状を保存出来ません")
RESULT = {"mesh": mesh.get_path_name(), "materials": names,
	"lod_count": subsystem.get_lod_count(mesh), "source_triangles": 21856}
