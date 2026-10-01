"""小容量のPBR素材を読込み，惑星地表の共通材質を構築する"""

import json
from pathlib import Path
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("地表材質の構築前にPIEを停止して下さい")

root = Path(unreal.Paths.project_dir()).resolve()
records = json.loads((root / "SourceAssets/SurfaceTextures/Sources.json").read_text(encoding="utf-8"))
assets = unreal.AssetToolsHelpers.get_asset_tools()
textures = {}

for item in records:
	name = Path(item["path"]).stem
	asset_path = "/Game/Planet/Textures/" + name
	texture = unreal.load_asset(asset_path) if unreal.EditorAssetLibrary.does_asset_exist(asset_path) else None
	if texture is None:
		task = unreal.AssetImportTask()
		task.set_editor_property("filename", str(root / item["path"]))
		task.set_editor_property("destination_path", "/Game/Planet/Textures")
		task.set_editor_property("destination_name", name)
		task.set_editor_property("automated", True)
		task.set_editor_property("save", False)
		task.set_editor_property("factory", unreal.TextureFactory())
		assets.import_asset_tasks([task])
		texture = unreal.load_asset(asset_path)
	if not isinstance(texture, unreal.Texture2D):
		raise RuntimeError("テクスチャを読込めません：" + name)
	is_normal = item["map"] == "nor_dx"
	is_color = item["map"] == "Diffuse"
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
	group = unreal.TextureGroup.TEXTUREGROUP_WORLD_NORMAL_MAP if is_normal else unreal.TextureGroup.TEXTUREGROUP_WORLD
	texture.set_editor_property("lod_group", group)
	if not unreal.EditorAssetLibrary.save_loaded_asset(texture):
		raise RuntimeError("テクスチャを保存出来ません：" + name)
	textures[item["asset"], item["map"]] = texture

path = "/Game/Planet/Materials/M_PlanetSurface"
material = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None

if material is None:
	material = assets.create_asset("M_PlanetSurface", "/Game/Planet/Materials", unreal.Material, unreal.MaterialFactoryNew())

if not isinstance(material, unreal.Material):
	raise RuntimeError("地表材質の型が不正です")

lib = unreal.MaterialEditingLibrary
# この UE 版では既存グラフの削除が MarkAsGarbage で失敗する為，所有中の式を再利用する
custom = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)

if custom is not None and not isinstance(custom, unreal.MaterialExpressionCustom):
	raise RuntimeError("既存の地表材質の構成が異なります")

existing = {}

if custom is not None:
	names = lib.get_material_expression_input_names(custom)
	nodes = lib.get_inputs_for_material_expression(material, custom)
	existing = { str(name): node for name, node in zip(names, nodes) if node is not None }

def expression(name: str, kind: type, x: int, y: int) -> unreal.MaterialExpression:
	"""名前と型が一致する既存式又は新規式を返す"""
	node = existing.get(name)
	if node is not None:
		if not isinstance(node, kind):
			raise RuntimeError("既存の入力式の型が不正です：" + name)
		# 対応する既存式の返戻
		return node
	# 新規入力式の返戻
	return lib.create_material_expression(material, kind, x, y)

material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
material.set_editor_property("tangent_space_normal", True)

if custom is None:
	custom = lib.create_material_expression(material, unreal.MaterialExpressionCustom, 0, 0)

custom.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
custom.set_editor_property("code", (root / "Shaders/PlanetSurface.ush").read_text(encoding="utf-8"))
inputs = {}

for index in 0, 1, 2, 3:
	name = "UV" + str(index)
	node = expression(name, unreal.MaterialExpressionTextureCoordinate, -800, index * 120)
	node.set_editor_property("coordinate_index", index)
	inputs[name] = node

position = expression("LocalPosition", unreal.MaterialExpressionLocalPosition, -800, -250)
position.set_editor_property("included_offsets", unreal.PositionIncludedOffsets.EXCLUDE_OFFSETS)
position.set_editor_property("local_origin", unreal.LocalPositionOrigin.PRIMITIVE)
inputs["LocalPosition"] = position

for name, value in (
	("TextureOrigin", (0, 0, 0)),
	("TextureAxisX", (1, 0, 0)),
	("TextureAxisY", (0, 1, 0)),
	("TextureAxisZ", (0, 0, 1))
):
	node = expression(name, unreal.MaterialExpressionVectorParameter, -1100, len(inputs) * 100)
	node.set_editor_property("parameter_name", name)
	node.set_editor_property("default_value", unreal.LinearColor(value[0], value[1], value[2], 0))
	inputs[name] = node

node = expression("FlowHydrologyEnabled", unreal.MaterialExpressionScalarParameter, -1100, len(inputs) * 100)
node.set_editor_property("parameter_name", "FlowHydrologyEnabled")
node.set_editor_property("default_value", 0)
inputs["FlowHydrologyEnabled"] = node
inputs["Masks"] = expression("Masks", unreal.MaterialExpressionVertexColor, -800, 480)
inputs["Wetness"] = inputs["Masks"]

for prefix, asset in ("Rock", "rock_face"), ("Sand", "sand_01"), ("Soil", "brown_mud"):
	for suffix, key in ("Color", "Diffuse"), ("Normal", "nor_dx"), ("Arm", "arm"):
		name = prefix + suffix
		map_asset = "aerial_beach_01" if name == "SandNormal" else asset
		node = expression(name, unreal.MaterialExpressionTextureObjectParameter, -500, len(inputs) * 85)
		node.set_editor_property("parameter_name", name)
		node.set_editor_property("texture", textures[map_asset, key])
		sampler = unreal.MaterialSamplerType.SAMPLERTYPE_MASKS
		if key == "nor_dx":
			sampler = unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL
		elif key == "Diffuse":
			sampler = unreal.MaterialSamplerType.SAMPLERTYPE_COLOR
		node.set_editor_property("sampler_type", sampler)
		inputs[name] = node

node = expression("SandRippleColor", unreal.MaterialExpressionTextureObjectParameter, -500, len(inputs) * 85)
node.set_editor_property("parameter_name", "SandRippleColor")
node.set_editor_property("texture", textures["aerial_beach_01", "Diffuse"])
node.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)
inputs["SandRippleColor"] = node

def custom_input(name: str) -> unreal.CustomInput:
	"""Custom式の入力名を持つ構造体を返す"""
	value = unreal.CustomInput()
	value.set_editor_property("input_name", name)
	# 設定した入力の返戻
	return value

def custom_output(name: str, kind: unreal.CustomMaterialOutputType) -> unreal.CustomOutput:
	"""Custom式の出力名と型を持つ構造体を返す"""
	value = unreal.CustomOutput()
	value.set_editor_property("output_name", name)
	value.set_editor_property("output_type", kind)
	# 設定した出力の返戻
	return value

custom.set_editor_property("inputs", [custom_input(name) for name in inputs])

custom.set_editor_property(
	"additional_outputs",
	[
		custom_output("SurfaceNormal", unreal.CustomMaterialOutputType.CMOT_FLOAT3),
		custom_output("SurfaceRoughness", unreal.CustomMaterialOutputType.CMOT_FLOAT1),
		custom_output("SurfaceOcclusion", unreal.CustomMaterialOutputType.CMOT_FLOAT1)
	]
)

for name, node in inputs.items():
	output = "A" if name == "Wetness" else ""
	if not lib.connect_material_expressions(node, output, custom, name):
		raise RuntimeError("地表材質の入力を接続出来ません：" + name)

for output, prop in (
	("", unreal.MaterialProperty.MP_BASE_COLOR),
	("SurfaceNormal", unreal.MaterialProperty.MP_NORMAL),
	("SurfaceRoughness", unreal.MaterialProperty.MP_ROUGHNESS),
	("SurfaceOcclusion", unreal.MaterialProperty.MP_AMBIENT_OCCLUSION)
):
	if not lib.connect_material_property(custom, output, prop):
		raise RuntimeError("地表材質の出力を接続出来ません：" + output)

lib.layout_material_expressions(material)
lib.recompile_material(material)

if not unreal.EditorAssetLibrary.save_loaded_asset(material):
	raise RuntimeError("地表材質を保存出来ません")

RESULT = {
	"material": path,
	"textures": len(textures),
	"source_bytes": sum(item["bytes"] for item in records),
	"shader_compilation_requested": True,
	"shader_success_requires_log_verification": True,
	"terrain_assets_saved": False
}
