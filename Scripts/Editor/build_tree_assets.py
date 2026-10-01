"""確認済みの原型と葉・樹皮から，軽量な樹木素材を構築する"""
import json
from pathlib import Path
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("樹木の素材を構築する前に PIE を停止して下さい")
ROOT = Path(unreal.Paths.project_dir()).resolve()
ASSETS = unreal.AssetToolsHelpers.get_asset_tools()
LIBRARY = unreal.MaterialEditingLibrary
RECORD = json.loads((ROOT / "SourceAssets/Vegetation/TreeTextures/Sources.json").read_text(encoding="utf-8"))
TEXTURES = {}
for item in RECORD["files"]:
	name = "T_Tree_" + item["map"]
	path = "/Game/Planet/Vegetation/" + name
	texture = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
	if texture is None:
		task = unreal.AssetImportTask()
		for key, value in {
			"filename": str(ROOT / item["path"]), "destination_path": "/Game/Planet/Vegetation", "destination_name": name,
			"automated": True, "save": False
		}.items():
			task.set_editor_property(key, value)
		ASSETS.import_asset_tasks([task])
		texture = unreal.load_asset(path)
	if not isinstance(texture, unreal.Texture2D):
		raise RuntimeError("樹木の画像を読込めません：" + name)
	texture.set_editor_property("srgb", item["map"].endswith("diff"))
	texture.set_editor_property("max_texture_size", 1024)
	if item["map"].endswith("nor_dx"):
		texture.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_NORMALMAP)
	elif item["map"].endswith("alpha"):
		texture.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_MASKS)
	if not unreal.EditorAssetLibrary.save_loaded_asset(texture):
		raise RuntimeError("樹木の画像を保存出来ません")
	TEXTURES[item["map"]] = texture


def create_material(name: str, prefix: str, foliage: bool) -> unreal.Material:
	"""樹皮又は葉の材質を構築する
	Args:
		name: 保存する材質名
		prefix: 対応する画像群
		foliage: 葉の両面照明と透過切抜きを使う場合に真
	Returns:
		確認した材質
	"""
	PATH = "/Game/Planet/Materials/" + name
	material = unreal.load_asset(PATH) if unreal.EditorAssetLibrary.does_asset_exist(PATH) else None
	if material is not None:
		if not isinstance(material, unreal.Material):
			raise RuntimeError("既存の樹木材質の型が異なります")
		# 作成済みの材質の返戻
		return material
	material = ASSETS.create_asset(name, "/Game/Planet/Materials", unreal.Material, unreal.MaterialFactoryNew())
	if not isinstance(material, unreal.Material):
		raise RuntimeError("樹木の材質を作成出来ません")
	material.set_editor_property("two_sided", foliage)
	if foliage:
		material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_TWO_SIDED_FOLIAGE)
		material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
	LIBRARY.set_material_usage(material, unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES)
	for suffix, prop in (("diff", unreal.MaterialProperty.MP_BASE_COLOR), ("nor_dx", unreal.MaterialProperty.MP_NORMAL)):
		node = LIBRARY.create_material_expression(material, unreal.MaterialExpressionTextureSampleParameter2D)
		node.set_editor_property("parameter_name", "SurfaceColor" if suffix == "diff" else "SurfaceNormal")
		node.set_editor_property("texture", TEXTURES[prefix + "_" + suffix])
		if suffix == "nor_dx":
			node.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
		if foliage and suffix == "diff":
			tint = LIBRARY.create_material_expression(material, unreal.MaterialExpressionVectorParameter)
			tint.set_editor_property("parameter_name", "LeafTint")
			tint.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
			mix = LIBRARY.create_material_expression(material, unreal.MaterialExpressionMultiply)
			LIBRARY.connect_material_expressions(node, "RGB", mix, "A")
			LIBRARY.connect_material_expressions(tint, "RGB", mix, "B")
			LIBRARY.connect_material_property(mix, "", prop)
			LIBRARY.connect_material_property(mix, "", unreal.MaterialProperty.MP_SUBSURFACE_COLOR)
		else:
			LIBRARY.connect_material_property(node, "RGB", prop)
	ROUGHNESS = LIBRARY.create_material_expression(material, unreal.MaterialExpressionConstant)
	ROUGHNESS.set_editor_property("r", 0.8)
	LIBRARY.connect_material_property(ROUGHNESS, "", unreal.MaterialProperty.MP_ROUGHNESS)
	if foliage:
		alpha = LIBRARY.create_material_expression(material, unreal.MaterialExpressionTextureSampleParameter2D)
		alpha.set_editor_property("parameter_name", "LeafOpacity")
		alpha.set_editor_property("texture", TEXTURES["leaves_alpha"])
		alpha.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_MASKS)
		LIBRARY.connect_material_property(alpha, "R", unreal.MaterialProperty.MP_OPACITY_MASK)
	LIBRARY.layout_material_expressions(material)
	LIBRARY.recompile_material(material)
	if not unreal.EditorAssetLibrary.save_loaded_asset(material):
		raise RuntimeError("樹木の材質を保存出来ません")
	# 構築した材質の返戻
	return material


BARK = create_material("M_PlanetTreeBark", "branch", False)
LEAVES = create_material("M_PlanetTreeLeaves", "leaves", True)
MESH_PATH = "/Game/Planet/Vegetation/SM_BroadleafSapling"
mesh = unreal.load_asset(MESH_PATH) if unreal.EditorAssetLibrary.does_asset_exist(MESH_PATH) else None
if mesh is None:
	OPTIONS = unreal.FbxImportUI()
	OPTIONS.set_editor_property("import_materials", False)
	OPTIONS.set_editor_property("import_textures", False)
	OPTIONS.set_editor_property("import_as_skeletal", False)
	OPTIONS.set_editor_property("automated_import_should_detect_type", False)
	OPTIONS.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
	OPTIONS.static_mesh_import_data.set_editor_property("auto_generate_collision", False)
	OPTIONS.static_mesh_import_data.set_editor_property("convert_scene", False)
	OPTIONS.static_mesh_import_data.set_editor_property("combine_meshes", True)
	TASK = unreal.AssetImportTask()
	for key, value in {
		"filename": str(ROOT / "SourceAssets/Vegetation/BroadleafSapling.obj"), "destination_path": "/Game/Planet/Vegetation",
		"destination_name": "SM_BroadleafSapling", "automated": True, "save": False, "options": OPTIONS
	}.items():
		TASK.set_editor_property(key, value)
	ASSETS.import_asset_tasks([TASK])
	mesh = unreal.load_asset(MESH_PATH)
if not isinstance(mesh, unreal.StaticMesh):
	raise RuntimeError("樹木の原型を読込めません")
SLOTS = mesh.get_editor_property("static_materials")
NAMES = [str(slot.material_slot_name) for slot in SLOTS]
if set(NAMES) != {"Bark", "Leaves"}:
	raise RuntimeError("樹皮と葉の材質区画を確認出来ません：" + str(NAMES))
for index, name in enumerate(NAMES):
	mesh.set_material(index, BARK if name == "Bark" else LEAVES)
SUBSYSTEM = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
REDUCTIONS = []
for ratio, screen_size in ((1, 1), (0.5, 0.3), (0.2, 0.12)):
	setting = unreal.StaticMeshReductionSettings()
	setting.set_editor_property("percent_triangles", ratio)
	setting.set_editor_property("screen_size", screen_size)
	REDUCTIONS.append(setting)
REDUCTION_OPTIONS = unreal.StaticMeshReductionOptions()
REDUCTION_OPTIONS.set_editor_property("auto_compute_lod_screen_size", False)
REDUCTION_OPTIONS.set_editor_property("reduction_settings", REDUCTIONS)
if SUBSYSTEM.set_lods(mesh, REDUCTION_OPTIONS) != 3:
	raise RuntimeError("樹木の距離別形状を生成出来ません")
if not unreal.EditorAssetLibrary.save_loaded_asset(mesh):
	raise RuntimeError("樹木の原型を保存出来ません")
BOX = mesh.get_bounding_box()
EXTENT = BOX.max - BOX.min
RESULT = {
	"mesh": mesh.get_path_name(), "materials": NAMES, "lodCount": SUBSYSTEM.get_lod_count(mesh),
	"extentCentimeters": [EXTENT.x, EXTENT.y, EXTENT.z], "sourceTriangles": 22848,
	"visualVerified": False, "runtimePlacementImplemented": False
}
