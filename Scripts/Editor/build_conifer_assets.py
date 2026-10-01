"""針葉樹の形状と枝片画像をUEへ読み込み，葉の材質を作成する"""
from pathlib import Path
import unreal

if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("Stop PIE before building the conifer asset")

ROOT = Path(unreal.Paths.project_dir()).resolve()
ASSETS = unreal.AssetToolsHelpers.get_asset_tools()
LIBRARY = unreal.MaterialEditingLibrary
MATERIAL_PATH = "/Game/Planet/Materials/M_PlanetConiferNeedles"
MESH_PATH = "/Game/Planet/Vegetation/SM_ConiferSapling"
TEXTURE_PATH = "/Game/Planet/Vegetation/T_ConiferTwigCutout"
texture = unreal.load_asset(TEXTURE_PATH) if unreal.EditorAssetLibrary.does_asset_exist(TEXTURE_PATH) else None
if texture is None:
	task = unreal.AssetImportTask()
	for key, value in {
		"filename": str(ROOT / "SourceAssets/Vegetation/ConiferTwig/ConiferTwigCutout.png"),
		"destination_path": "/Game/Planet/Vegetation", "destination_name": "T_ConiferTwigCutout",
		"automated": True, "save": False
	}.items():
		task.set_editor_property(key, value)
	ASSETS.import_asset_tasks([task])
	texture = unreal.load_asset(TEXTURE_PATH)
if not isinstance(texture, unreal.Texture2D):
	raise RuntimeError("Could not import the conifer twig texture")
texture.set_editor_property("srgb", True)
texture.set_editor_property("max_texture_size", 512)
texture.set_editor_property("address_x", unreal.TextureAddress.TA_CLAMP)
texture.set_editor_property("address_y", unreal.TextureAddress.TA_CLAMP)
if not unreal.EditorAssetLibrary.save_loaded_asset(texture):
	raise RuntimeError("Could not save the conifer twig texture")

material = unreal.load_asset(MATERIAL_PATH) if unreal.EditorAssetLibrary.does_asset_exist(MATERIAL_PATH) else None
if material is None:
	material = ASSETS.create_asset(
		"M_PlanetConiferNeedles", "/Game/Planet/Materials", unreal.Material, unreal.MaterialFactoryNew()
	)
	if not isinstance(material, unreal.Material):
		raise RuntimeError("Could not create the conifer foliage material")
	material.set_editor_property("two_sided", True)
	material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_TWO_SIDED_FOLIAGE)
	material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
	LIBRARY.set_material_usage(material, unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES)
	custom = LIBRARY.create_material_expression(material, unreal.MaterialExpressionCustom)
	custom.set_editor_property("description", "PlanetConiferAppearanceV3")
	custom.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
	mask = unreal.CustomOutput()
	mask.set_editor_property("output_name", "RenderMask")
	mask.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT1)
	custom.set_editor_property("additional_outputs", [mask])
	inputs = {}
	color = LIBRARY.create_material_expression(material, unreal.MaterialExpressionVectorParameter)
	color.set_editor_property("parameter_name", "NeedleColor")
	color.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
	inputs["BaseColor"] = (color, "RGB")
	uv = LIBRARY.create_material_expression(material, unreal.MaterialExpressionTextureCoordinate)
	inputs["UV"] = (uv, "")
	twig = LIBRARY.create_material_expression(material, unreal.MaterialExpressionTextureSampleParameter2D)
	twig.set_editor_property("parameter_name", "TwigCutout")
	twig.set_editor_property("texture", texture)
	inputs["TwigColor"] = (twig, "RGB")
	inputs["TwigOpacity"] = (twig, "A")
	for name, default in (("NearRadius", 0), ("ClipEnabled", 0)):
		node = LIBRARY.create_material_expression(material, unreal.MaterialExpressionScalarParameter)
		node.set_editor_property("parameter_name", name)
		node.set_editor_property("default_value", default)
		inputs[name] = (node, "")
	for name in ("NearCenterRelative", "BodyCenterRelative"):
		node = LIBRARY.create_material_expression(material, unreal.MaterialExpressionVectorParameter)
		node.set_editor_property("parameter_name", name)
		inputs[name] = (node, "RGB")
	position = LIBRARY.create_material_expression(material, unreal.MaterialExpressionObjectPositionWS)
	camera = LIBRARY.create_material_expression(material, unreal.MaterialExpressionCameraPositionWS)
	relative = LIBRARY.create_material_expression(material, unreal.MaterialExpressionSubtract)
	LIBRARY.connect_material_expressions(position, "", relative, "A")
	LIBRARY.connect_material_expressions(camera, "", relative, "B")
	inputs["ObjectRelative"] = (relative, "")
	tint = LIBRARY.create_material_expression(material, unreal.MaterialExpressionPerInstanceCustomData3Vector)
	tint.set_editor_property("data_index", 0)
	tint.set_editor_property("const_default_value", unreal.LinearColor(1, 1, 1, 0))
	inputs["Tint"] = (tint, "")
	snow = LIBRARY.create_material_expression(material, unreal.MaterialExpressionPerInstanceCustomData)
	snow.set_editor_property("data_index", 3)
	snow.set_editor_property("const_default_value", 0)
	inputs["Snow"] = (snow, "")
	custom_inputs = []
	for name in inputs:
		entry = unreal.CustomInput()
		entry.set_editor_property("input_name", name)
		custom_inputs.append(entry)
	custom.set_editor_property("inputs", custom_inputs)
	custom.set_editor_property("code", """
float nearFade = 1 - smoothstep(max(0, NearRadius - 200), max(1, NearRadius), length(ObjectRelative - NearCenterRelative));
float bodyFade = 1 - smoothstep(max(0, NearRadius - 400),
    max(1, NearRadius - 200), length(ObjectRelative - BodyCenterRelative));
RenderMask = TwigOpacity * (ClipEnabled > 0.5 ? nearFade * bodyFade : 1);
float shade = 0.78 + 0.22 * saturate(UV.x * 0.7 + UV.y * 0.3);
return lerp(TwigColor * BaseColor * shade * Tint, float3(0.8, 0.84, 0.88), saturate(Snow));
""")
	for name, connection in inputs.items():
		if not LIBRARY.connect_material_expressions(connection[0], connection[1], custom, name):
			raise RuntimeError("Could not connect conifer material input: " + name)
	LIBRARY.connect_material_property(custom, "", unreal.MaterialProperty.MP_BASE_COLOR)
	LIBRARY.connect_material_property(custom, "", unreal.MaterialProperty.MP_SUBSURFACE_COLOR)
	fade = LIBRARY.create_material_expression(material, unreal.MaterialExpressionMaterialFunctionCall)
	fade.set_editor_property("material_function", unreal.load_asset(
		"/Engine/Functions/Engine_MaterialFunctions02/Utility/DitherTemporalAA"
	))
	if not LIBRARY.connect_material_expressions(custom, "RenderMask", fade, "Alpha Threshold"):
		raise RuntimeError("Could not connect the conifer fade")
	LIBRARY.connect_material_property(fade, "", unreal.MaterialProperty.MP_OPACITY_MASK)
	roughness = LIBRARY.create_material_expression(material, unreal.MaterialExpressionConstant)
	roughness.set_editor_property("r", 0.8)
	LIBRARY.connect_material_property(roughness, "", unreal.MaterialProperty.MP_ROUGHNESS)
	LIBRARY.layout_material_expressions(material)
	LIBRARY.recompile_material(material)
	if not unreal.EditorAssetLibrary.save_loaded_asset(material):
		raise RuntimeError("Could not save the conifer foliage material")
if not isinstance(material, unreal.Material):
	raise RuntimeError("Conifer foliage asset has the wrong type")
custom = LIBRARY.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)
if not isinstance(custom, unreal.MaterialExpressionCustom):
	raise RuntimeError("Conifer foliage material has no appearance expression")
if custom.get_editor_property("description") == "PlanetConiferAppearanceV1":
	names = LIBRARY.get_material_expression_input_names(custom)
	nodes = LIBRARY.get_inputs_for_material_expression(material, custom)
	inputs = {str(name): node for name, node in zip(names, nodes) if node is not None}
	base_color = inputs.get("BaseColor")
	if not isinstance(base_color, unreal.MaterialExpressionVectorParameter):
		raise RuntimeError("Conifer foliage tint parameter changed unexpectedly")
	base_color.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
	twig = LIBRARY.create_material_expression(material, unreal.MaterialExpressionTextureSampleParameter2D)
	twig.set_editor_property("parameter_name", "TwigCutout")
	twig.set_editor_property("texture", texture)
	entries = list(custom.get_editor_property("inputs"))
	for name in ("TwigColor", "TwigOpacity"):
		entry = unreal.CustomInput()
		entry.set_editor_property("input_name", name)
		entries.append(entry)
	custom.set_editor_property("inputs", entries)
	custom.set_editor_property("description", "PlanetConiferAppearanceV3")
	custom.set_editor_property("code", """
float nearFade = 1 - smoothstep(max(0, NearRadius - 200), max(1, NearRadius), length(ObjectRelative - NearCenterRelative));
float bodyFade = 1 - smoothstep(max(0, NearRadius - 400),
    max(1, NearRadius - 200), length(ObjectRelative - BodyCenterRelative));
RenderMask = TwigOpacity * (ClipEnabled > 0.5 ? nearFade * bodyFade : 1);
float shade = 0.78 + 0.22 * saturate(UV.x * 0.7 + UV.y * 0.3);
return lerp(TwigColor * BaseColor * shade * Tint, float3(0.8, 0.84, 0.88), saturate(Snow));
""")
	for name, output in (("TwigColor", "RGB"), ("TwigOpacity", "A")):
		if not LIBRARY.connect_material_expressions(twig, output, custom, name):
			raise RuntimeError("Could not connect conifer cutout input: " + name)
	LIBRARY.layout_material_expressions(material)
	LIBRARY.recompile_material(material)
	if not unreal.EditorAssetLibrary.save_loaded_asset(material):
		raise RuntimeError("Could not save the conifer foliage material")
elif custom.get_editor_property("description") == "PlanetConiferAppearanceV2":
	old_fade = "float bodyFade = 1 - smoothstep(1600, 1800, length(ObjectRelative - BodyCenterRelative));"
	new_fade = ("float bodyFade = 1 - smoothstep(max(0, NearRadius - 400),\n"
		"    max(1, NearRadius - 200), length(ObjectRelative - BodyCenterRelative));")
	code = custom.get_editor_property("code")
	if code.count(old_fade) != 1:
		raise RuntimeError("Conifer V2 fade source differs from the expected material")
	custom.set_editor_property("code", code.replace(old_fade, new_fade))
	custom.set_editor_property("description", "PlanetConiferAppearanceV3")
	LIBRARY.recompile_material(material)
	if not unreal.EditorAssetLibrary.save_loaded_asset(material):
		raise RuntimeError("Could not save the extended conifer material")
elif custom.get_editor_property("description") != "PlanetConiferAppearanceV3":
	raise RuntimeError("Conifer foliage material version changed unexpectedly")

mesh = unreal.load_asset(MESH_PATH) if unreal.EditorAssetLibrary.does_asset_exist(MESH_PATH) else None
had_asset = mesh is not None
reimport = globals().get("ARGS", {}).get("reimport", False)
if mesh is None or reimport:
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
		"filename": str(ROOT / "SourceAssets/Vegetation/ConiferSapling.obj"),
		"destination_path": "/Game/Planet/Vegetation",
		"destination_name": "SM_ConiferSapling",
		"automated": True,
		"save": False,
		"replace_existing": mesh is not None,
		"replace_existing_settings": False,
		"options": options
	}.items():
		task.set_editor_property(key, value)
	ASSETS.import_asset_tasks([task])
	mesh = unreal.load_asset(MESH_PATH)
if not isinstance(mesh, unreal.StaticMesh):
	raise RuntimeError("Could not import the conifer mesh")
names = [str(slot.material_slot_name) for slot in mesh.get_editor_property("static_materials")]
if set(names) != {"Bark", "Needles"}:
	raise RuntimeError("Conifer material slots do not match the source: " + str(names))
bark = unreal.load_asset("/Game/Planet/Materials/M_PlanetTreeBark")
if not isinstance(bark, unreal.Material):
	raise RuntimeError("Tree bark material is unavailable")
for index, name in enumerate(names):
	mesh.set_material(index, bark if name == "Bark" else material)
subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
reductions = []
for ratio, screen_size in ((1, 1), (0.5, 0.3), (0.2, 0.12)):
	setting = unreal.StaticMeshReductionSettings()
	setting.set_editor_property("percent_triangles", ratio)
	setting.set_editor_property("screen_size", screen_size)
	reductions.append(setting)
settings = unreal.StaticMeshReductionOptions()
settings.set_editor_property("auto_compute_lod_screen_size", False)
settings.set_editor_property("reduction_settings", reductions)
if subsystem.set_lods(mesh, settings) != 3:
	raise RuntimeError("Could not create three conifer LODs")
if not unreal.EditorAssetLibrary.save_loaded_asset(mesh):
	raise RuntimeError("Could not save the conifer mesh")
bounds = mesh.get_bounding_box()
RESULT = {"mesh": mesh.get_path_name(), "materials": names, "lodCount": subsystem.get_lod_count(mesh),
	"extentCentimeters": [bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y, bounds.max.z - bounds.min.z],
	"reimported": bool(had_asset and reimport)}
