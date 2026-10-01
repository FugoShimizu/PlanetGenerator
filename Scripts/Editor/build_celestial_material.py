"""保存済みの素材だけから，C++ が制御する天球材質を構築する"""
from pathlib import Path
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("天球材質を構築する前に PIE を停止して下さい")

root = Path(unreal.Paths.project_dir()).resolve()
assets = unreal.AssetToolsHelpers.get_asset_tools()
lib = unreal.MaterialEditingLibrary
moon_path = "/Game/Planet/Textures/T_MoonLroc"
moon = unreal.load_asset(moon_path) if unreal.EditorAssetLibrary.does_asset_exist(moon_path) else None
if moon is None:
	task = unreal.AssetImportTask()
	task.set_editor_property("filename", str(root / "SourceAssets/CelestialTextures/lroc_color_2k.jpg"))
	task.set_editor_property("destination_path", "/Game/Planet/Textures")
	task.set_editor_property("destination_name", "T_MoonLroc")
	task.set_editor_property("automated", True)
	task.set_editor_property("save", False)
	assets.import_asset_tasks([task])
	moon = unreal.load_asset(moon_path)
if not isinstance(moon, unreal.Texture2D):
	raise RuntimeError("月面素材を読込めません")
moon.set_editor_property("srgb", True)
moon.set_editor_property("max_texture_size", 2048)
moon.set_editor_property("address_x", unreal.TextureAddress.TA_WRAP)
moon.set_editor_property("address_y", unreal.TextureAddress.TA_CLAMP)
if not unreal.EditorAssetLibrary.save_loaded_asset(moon):
	raise RuntimeError("月面素材を保存出来ません")
stars = unreal.load_asset("/Engine/EngineSky/T_Sky_Stars")
if not isinstance(stars, unreal.Texture2D):
	raise RuntimeError("エンジン標準の星空素材を読込めません")

path = "/Game/Planet/Materials/M_PlanetCelestialSky"
material = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
if material is None:
	material = assets.create_asset("M_PlanetCelestialSky", "/Game/Planet/Materials", unreal.Material, unreal.MaterialFactoryNew())
if not isinstance(material, unreal.Material):
	raise RuntimeError("天球材質の型が不正です")
material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
material.set_editor_property("two_sided", True)
material.set_editor_property("is_sky", True)
custom = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
if custom is None:
	custom = lib.create_material_expression(material, unreal.MaterialExpressionCustom, 0, 0)
if not isinstance(custom, unreal.MaterialExpressionCustom):
	raise RuntimeError("既存の天球材質の構造が異なります")
inputs = {
	str(name): node for name, node in zip(
		lib.get_material_expression_input_names(custom), lib.get_inputs_for_material_expression(material, custom)
	) if node is not None
}


def expression(name: str, kind: type) -> unreal.MaterialExpression:
	"""同名の入力式を再利用し，未作成の場合だけ追加する
	Args:
		name: 材質内の入力名
		kind: 対応する式の型
	Returns:
		接続する入力式
	Raises:
		RuntimeError: 既存の型が異なる場合
	"""
	node = inputs.get(name)
	if node is None:
		node = lib.create_material_expression(material, kind, -700, len(inputs) * 80)
	if not isinstance(node, kind):
		raise RuntimeError("入力式の型が異なります：" + name)
	inputs[name] = node
	# 対応する式の返戻
	return node


for name, value in (
	("MoonDirection", (0, 0, 1)), ("MoonRight", (1, 0, 0)), ("MoonUp", (0, 1, 0)), ("MoonToSun", (0, 0, -1)),
	("StarAxisX", (1, 0, 0)), ("StarAxisY", (0, 1, 0)), ("StarAxisZ", (0, 0, 1)), ("ObserverUp", (0, 0, 1))
):
	node = expression(name, unreal.MaterialExpressionVectorParameter)
	node.set_editor_property("parameter_name", name)
	node.set_editor_property("default_value", unreal.LinearColor(*value, 0))
for name, value in (("MoonSinRadius", 0.00436), ("HorizonCos", 0)):
	node = expression(name, unreal.MaterialExpressionScalarParameter)
	node.set_editor_property("parameter_name", name)
	node.set_editor_property("default_value", value)
for name, texture in (("MoonMap", moon), ("StarMap", stars)):
	node = expression(name, unreal.MaterialExpressionTextureObjectParameter)
	node.set_editor_property("parameter_name", name)
	node.set_editor_property("texture", texture)
	node.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
expression("ViewToCamera", unreal.MaterialExpressionCameraVectorWS)
expression("SkyLuminance", unreal.MaterialExpressionSkyAtmosphereViewLuminance)
expression("SunDisc", unreal.MaterialExpressionSkyAtmosphereLightDiskLuminance)
expression("Aerial", unreal.MaterialExpressionSkyAtmosphereAerialPerspective)
values = []
for name in inputs:
	value = unreal.CustomInput()
	value.set_editor_property("input_name", name)
	values.append(value)
custom.set_editor_property("inputs", values)
custom.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
custom.set_editor_property("code", (root / "Shaders/PlanetCelestialSky.ush").read_text(encoding="utf-8"))
for name, node in inputs.items():
	if not lib.connect_material_expressions(node, "", custom, name):
		raise RuntimeError("天球入力を接続出来ません：" + name)
if not lib.connect_material_property(custom, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR):
	raise RuntimeError("天球の輝度を接続出来ません")
lib.layout_material_expressions(material)
lib.recompile_material(material)
if not unreal.EditorAssetLibrary.save_loaded_asset(material):
	raise RuntimeError("天球材質を保存出来ません")
stats = lib.get_statistics(material)
RESULT = {
	"material": material.get_path_name(), "moonTexture": moon.get_path_name(), "starsTexture": stars.get_path_name(),
	"pixelInstructions": stats.num_pixel_shader_instructions, "vertexInstructions": stats.num_vertex_shader_instructions,
	"visualVerified": False
}
if RESULT["pixelInstructions"] <= 0:
	raise RuntimeError("天球材質のコンパイル結果を確認出来ません：" + str(RESULT))
