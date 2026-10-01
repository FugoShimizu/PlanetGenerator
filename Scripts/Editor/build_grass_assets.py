"""保存する草の原型と材質を作る，惑星上の配置や生成地形は保存しない"""
import math
from pathlib import Path
import random
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("草の素材を構築する前に PIE を停止して下さい")
root = Path(unreal.Paths.project_dir()).resolve()
source = root / "SourceAssets/Vegetation/GrassClump.obj"
generator = random.Random(45173)
vertices = []
coordinates = []
faces = []
for blade in range(14):
	angle = generator.uniform(0, 2 * math.pi)
	direction = (math.cos(angle), math.sin(angle))
	side = (-direction[1], direction[0])
	root_angle = generator.uniform(0, 2 * math.pi)
	root_radius = generator.uniform(0, 7)
	origin = (root_radius * math.cos(root_angle), root_radius * math.sin(root_angle))
	height = generator.uniform(25, 55)
	bend = generator.uniform(7, 24)
	width = generator.uniform(0.5, 1.2)
	start = len(vertices) + 1
	for fraction in (0, 0.2, 0.45, 0.7, 0.9):
		for lateral in (-1, 0, 1):
			half_width = width * (1 - fraction) ** 0.65 * 0.5
			rib = 0.13 * width * (1 - fraction) if lateral == 0 else 0
			vertices.append((
				origin[0] + direction[0] * bend * fraction * fraction + side[0] * lateral * half_width,
				origin[1] + direction[1] * bend * fraction * fraction + side[1] * lateral * half_width,
				height * fraction - height * 0.18 * fraction * fraction + rib
			))
			coordinates.append(((lateral + 1) * 0.5, fraction))
	vertices.append((origin[0] + direction[0] * bend, origin[1] + direction[1] * bend, height * 0.82))
	coordinates.append((0.5, 1))
	for row in range(4):
		for column in range(2):
			a = start + row * 3 + column
			faces.extend(((a, a + 1, a + 4), (a, a + 4, a + 3)))
	faces.extend(((start + 12, start + 13, start + 15), (start + 13, start + 14, start + 15)))
lines = ["# Original grass blade geometry, coordinates in centimeters, Z up", "o GrassClump"]
lines.extend("v %.9f %.9f %.9f" % vertex for vertex in vertices)
lines.extend("vt %.9f %.9f" % uv for uv in coordinates)
lines.append("s 1")
lines.extend("f " + " ".join(str(index) + "/" + str(index) for index in face) for face in faces)
content = "\n".join(lines) + "\n"
if source.exists() and source.read_text(encoding="utf-8") != content:
	raise RuntimeError("既存の草の原型が異なる為，上書きしません")
if not source.exists():
	source.parent.mkdir(parents=True, exist_ok=True)
	source.write_text(content, encoding="utf-8", newline="\n")
assets = unreal.AssetToolsHelpers.get_asset_tools()
lib = unreal.MaterialEditingLibrary
mesh_path = "/Game/Planet/Vegetation/SM_GrassClump"
mesh = unreal.load_asset(mesh_path) if unreal.EditorAssetLibrary.does_asset_exist(mesh_path) else None
if mesh is None:
	options = unreal.FbxImportUI()
	options.set_editor_property("import_materials", False)
	options.set_editor_property("import_textures", False)
	options.set_editor_property("import_as_skeletal", False)
	options.set_editor_property("automated_import_should_detect_type", False)
	options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
	options.static_mesh_import_data.set_editor_property("auto_generate_collision", False)
	options.static_mesh_import_data.set_editor_property("convert_scene", False)
	task = unreal.AssetImportTask()
	for name, value in {
		"filename": str(source), "destination_path": "/Game/Planet/Vegetation", "destination_name": "SM_GrassClump",
		"automated": True, "save": False, "options": options
	}.items():
		task.set_editor_property(name, value)
	assets.import_asset_tasks([task])
	mesh = unreal.load_asset(mesh_path)
if not isinstance(mesh, unreal.StaticMesh):
	raise RuntimeError("草の原型を読込めません")
box = mesh.get_bounding_box()
extent = box.max - box.min
if not 20 < extent.z < 60 or abs(box.min.z) > 1:
	raise RuntimeError("草の原型の上下方向又は寸法が想定と異なります：" + str(box))
path = "/Game/Planet/Materials/M_PlanetGrass"
material = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
if material is None:
	material = assets.create_asset("M_PlanetGrass", "/Game/Planet/Materials", unreal.Material, unreal.MaterialFactoryNew())
if not isinstance(material, unreal.Material):
	raise RuntimeError("草の材質の型が不正です")
material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_TWO_SIDED_FOLIAGE)
material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
material.set_editor_property("two_sided", True)
lib.set_material_usage(material, unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES)
custom = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)
if custom is None:
	custom = lib.create_material_expression(material, unreal.MaterialExpressionCustom, 0, 0)
if not isinstance(custom, unreal.MaterialExpressionCustom):
	raise RuntimeError("草の材質の既存構造が異なります")
inputs = {
	str(name): node for name, node in zip(
		lib.get_material_expression_input_names(custom), lib.get_inputs_for_material_expression(material, custom)
	) if node is not None
}


def expression(name: str, kind: type) -> unreal.MaterialExpression:
	"""同名・同型の入力を再利用する
	Args:
		name: 入力名
		kind: 式の型
	Returns:
		作成又は再利用した式
	"""
	node = inputs.get(name)
	if node is None:
		node = lib.create_material_expression(material, kind, -650, len(inputs) * 80)
	if not isinstance(node, kind):
		raise RuntimeError("草の入力式の型が異なります：" + name)
	inputs[name] = node
	# 入力式の返戻
	return node


node = expression("Radial", unreal.MaterialExpressionPerInstanceCustomData3Vector)
node.set_editor_property("data_index", 0)
node.set_editor_property("const_default_value", unreal.LinearColor(0, 0, 1, 0))
expression("UV", unreal.MaterialExpressionTextureCoordinate)
expression("Variation", unreal.MaterialExpressionPerInstanceRandom)
node = expression("SolarYearPhase", unreal.MaterialExpressionScalarParameter)
node.set_editor_property("parameter_name", "SolarYearPhase")
node = expression("ClimateAtlas", unreal.MaterialExpressionTextureObjectParameter)
node.set_editor_property("parameter_name", "ClimateAtlas")
node.set_editor_property("texture", unreal.load_asset("/Engine/EngineResources/WhiteSquareTexture"))
node.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR)
expression("InstanceFade", unreal.MaterialExpressionPerInstanceFadeAmount)
node = expression("NearCenterRelative", unreal.MaterialExpressionVectorParameter)
node.set_editor_property("parameter_name", "NearCenterRelative")
node = expression("NearRadius", unreal.MaterialExpressionScalarParameter)
node.set_editor_property("parameter_name", "NearRadius")
node.set_editor_property("default_value", 0)
relative = inputs.get("ViewRelative")
if relative is None:
	relative = expression("ViewRelative", unreal.MaterialExpressionSubtract)
	position = lib.create_material_expression(material, unreal.MaterialExpressionWorldPosition, -1100, 900)
	camera = lib.create_material_expression(material, unreal.MaterialExpressionCameraPositionWS, -1100, 1000)
	lib.connect_material_expressions(position, "", relative, "A")
	lib.connect_material_expressions(camera, "", relative, "B")
output = unreal.CustomOutput()
output.set_editor_property("output_name", "RenderMask")
output.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT1)
custom.set_editor_property("additional_outputs", [output])
values = []
for name in inputs:
	value = unreal.CustomInput()
	value.set_editor_property("input_name", name)
	values.append(value)
custom.set_editor_property("inputs", values)
custom.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
custom.set_editor_property("code", (root / "Shaders/PlanetGrass.ush").read_text(encoding="utf-8"))
for name, node in inputs.items():
	if not lib.connect_material_expressions(node, "", custom, name):
		raise RuntimeError("草の入力を接続出来ません：" + name)
for prop in (unreal.MaterialProperty.MP_BASE_COLOR, unreal.MaterialProperty.MP_SUBSURFACE_COLOR):
	if not lib.connect_material_property(custom, "", prop):
		raise RuntimeError("草の色を接続出来ません")
roughness = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_ROUGHNESS)
if roughness is None:
	roughness = lib.create_material_expression(material, unreal.MaterialExpressionConstant, 0, 500)
roughness.set_editor_property("r", 0.8)
lib.connect_material_property(roughness, "", unreal.MaterialProperty.MP_ROUGHNESS)
fade = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_OPACITY_MASK)
if fade is None:
	fade = lib.create_material_expression(material, unreal.MaterialExpressionMaterialFunctionCall, 0, 650)
	fade.set_editor_property("material_function", unreal.load_asset(
		"/Engine/Functions/Engine_MaterialFunctions02/Utility/DitherTemporalAA"
	))
if not lib.connect_material_expressions(custom, "RenderMask", fade, "Alpha Threshold"):
	raise RuntimeError("草の距離減衰を接続出来ません")
lib.connect_material_property(fade, "", unreal.MaterialProperty.MP_OPACITY_MASK)
lib.layout_material_expressions(material)
lib.recompile_material(material)
mesh.set_material(0, material)
for asset in (material, mesh):
	if not unreal.EditorAssetLibrary.save_loaded_asset(asset):
		raise RuntimeError("草の素材を保存出来ません")
stats = lib.get_statistics(material)
RESULT = {
	"mesh": mesh.get_path_name(), "material": material.get_path_name(), "triangles": len(faces),
	"sourceBytes": source.stat().st_size, "extentCentimeters": [extent.x, extent.y, extent.z],
	"pixelInstructions": stats.num_pixel_shader_instructions, "vertexInstructions": stats.num_vertex_shader_instructions,
	"placementImplemented": False, "visualVerified": False
}
if stats.num_pixel_shader_instructions <= 0:
	raise RuntimeError("草の材質のコンパイルに失敗しました")
