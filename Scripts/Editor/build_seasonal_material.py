"""既存の地表材質から，形状を変えない季節材質を構築する"""

import json
from pathlib import Path
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("季節材質を構築する前にPIEを停止して下さい")

root = Path(unreal.Paths.project_dir()).resolve()
records = json.loads((root / "SourceAssets/SeasonalTextures/Sources.json").read_text(encoding="utf-8"))
assets = unreal.AssetToolsHelpers.get_asset_tools()
lib = unreal.MaterialEditingLibrary
textures = {}

for item in records:
	name = Path(item["path"]).stem
	path = "/Game/Planet/Textures/" + name
	texture = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
	if texture is None:
		task = unreal.AssetImportTask()
		task.set_editor_property("filename", str(root / item["path"]))
		task.set_editor_property("destination_path", "/Game/Planet/Textures")
		task.set_editor_property("destination_name", name)
		task.set_editor_property("automated", True)
		task.set_editor_property("save", False)
		assets.import_asset_tasks([task])
		texture = unreal.load_asset(path)
	if not isinstance(texture, unreal.Texture2D):
		raise RuntimeError("素材を読込めません：" + name)
	is_color = item["map"] == "diff"
	is_normal = item["map"] == "nor_dx"
	texture.set_editor_property("srgb", is_color)
	compression = unreal.TextureCompressionSettings.TC_DEFAULT
	if is_normal:
		compression = unreal.TextureCompressionSettings.TC_NORMALMAP
	elif not is_color:
		compression = unreal.TextureCompressionSettings.TC_MASKS
	texture.set_editor_property("compression_settings", compression)
	texture.set_editor_property("max_texture_size", 1024)
	texture.set_editor_property("address_x", unreal.TextureAddress.TA_WRAP)
	texture.set_editor_property("address_y", unreal.TextureAddress.TA_WRAP)
	if not unreal.EditorAssetLibrary.save_loaded_asset(texture):
		raise RuntimeError("素材を保存出来ません：" + name)
	textures[item["asset"], item["map"]] = texture

path = "/Game/Planet/Materials/M_PlanetSeasonalSurface"
material = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None

if material is None:
	base = unreal.load_asset("/Game/Planet/Materials/M_PlanetSurface")
	if not isinstance(base, unreal.Material):
		raise RuntimeError("複製元の地表材質が有りません")
	material = assets.duplicate_asset("M_PlanetSeasonalSurface", "/Game/Planet/Materials", base)

if not isinstance(material, unreal.Material):
	raise RuntimeError("季節材質の型が不正です")

custom = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)

if not isinstance(custom, unreal.MaterialExpressionCustom):
	raise RuntimeError("既存の地表構造を確認出来ません")

names = lib.get_material_expression_input_names(custom)
nodes = lib.get_inputs_for_material_expression(material, custom)
inputs = { str(name): node for name, node in zip(names, nodes) if node is not None }

def expression(name: str, kind: type) -> unreal.MaterialExpression:
	"""既存の入力を再利用する，名前と型から入力式を返す"""
	node = inputs.get(name)
	if node is None:
		node = lib.create_material_expression(material, kind, -900, len(inputs) * 90)
	if not isinstance(node, kind):
		raise RuntimeError("既存の入力式の型が不正です：" + name)
	inputs[name] = node
	# 対応する入力式の返戻
	return node

for index in 0, 1:
	node = expression("UV" + str(index), unreal.MaterialExpressionTextureCoordinate)
	node.set_editor_property("coordinate_index", index)

for name, value in ("ClimateEnabled", 0), ("SolarYearPhase", 0), ("ClimateInverseRadius", 0), ("FlowHydrologyEnabled", 0):
	node = expression(name, unreal.MaterialExpressionScalarParameter)
	node.set_editor_property("parameter_name", name)
	node.set_editor_property("default_value", value)

node = expression("ClimatePlanetOffset", unreal.MaterialExpressionVectorParameter)
node.set_editor_property("parameter_name", "ClimatePlanetOffset")
node.set_editor_property("default_value", unreal.LinearColor(0, 0, 0, 0))

for name, texture, sampler in (
	(
		"SandNormal",
		unreal.load_asset("/Game/Planet/Textures/aerial_beach_01_nor_dx"),
		unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL
	),
	(
		"SandRippleColor",
		unreal.load_asset("/Game/Planet/Textures/aerial_beach_01_diffuse"),
		unreal.MaterialSamplerType.SAMPLERTYPE_COLOR
	),
	("ClimateAtlas", unreal.load_asset("/Game/Planet/Textures/rock_face_arm"), unreal.MaterialSamplerType.SAMPLERTYPE_MASKS),
	("GrassColor", textures["leafy_grass", "diff"], unreal.MaterialSamplerType.SAMPLERTYPE_COLOR),
	("GrassNormal", textures["leafy_grass", "nor_dx"], unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL),
	("GrassArm", textures["leafy_grass", "arm"], unreal.MaterialSamplerType.SAMPLERTYPE_MASKS),
	("LitterColor", textures["forrest_ground_01", "diff"], unreal.MaterialSamplerType.SAMPLERTYPE_COLOR),
	("NeedleLitterColor", textures["forrest_ground_03", "diff"], unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
):
	if texture is None:
		raise RuntimeError("季節材質の入力素材が有りません：" + name)
	node = expression(name, unreal.MaterialExpressionTextureObjectParameter)
	node.set_editor_property("parameter_name", name)
	node.set_editor_property("texture", texture)
	node.set_editor_property("sampler_type", sampler)

values = []

for name in inputs:
	value = unreal.CustomInput()
	value.set_editor_property("input_name", name)
	values.append(value)

custom.set_editor_property("inputs", values)
base_code = (root / "Shaders/PlanetSurface.ush").read_text(encoding="utf-8")
ending = "return saturate(Base * lerp(1, 0.64, Wet));"

if base_code.count(ending) != 1 or base_code.partition(ending)[2].strip():
	raise RuntimeError("共通の地表シェーダーの返戻箇所が変更されています")

seasonal_code = (root / "Shaders/PlanetSeasonalSurface.ush").read_text(encoding="utf-8")
custom.set_editor_property("code", base_code.partition(ending)[0] + seasonal_code)

for name, node in inputs.items():
	output = "A" if name == "Wetness" else ""
	if not lib.connect_material_expressions(node, output, custom, name):
		raise RuntimeError("入力を接続出来ません：" + name)

lib.layout_material_expressions(material)
lib.recompile_material(material)

if not unreal.EditorAssetLibrary.save_loaded_asset(material):
	raise RuntimeError("季節材質を保存出来ません")

RESULT = {
	"material": path,
	"imported_textures": len(textures),
	"source_bytes": sum(item["bytes"] for item in records),
	"shader_compilation_requested": True,
	"terrain_or_climate_saved": False
}
