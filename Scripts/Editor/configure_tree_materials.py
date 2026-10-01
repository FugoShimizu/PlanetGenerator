"""樹木材質へ季節色と近景範囲の減衰を接続する"""
import unreal

LIBRARY = unreal.MaterialEditingLibrary
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
	raise RuntimeError("樹木材質の変更前に PIE を停止して下さい")
RESULT = {"materials": []}
for NAME in ("M_PlanetTreeBark", "M_PlanetTreeLeaves"):
	MATERIAL = unreal.load_asset("/Game/Planet/Materials/" + NAME)
	if not isinstance(MATERIAL, unreal.Material):
		raise RuntimeError("樹木材質が見付かりません：" + NAME)
	LEAF = NAME.endswith("Leaves")
	BASE = LIBRARY.get_material_property_input_node(MATERIAL, unreal.MaterialProperty.MP_BASE_COLOR)
	if isinstance(BASE, unreal.MaterialExpressionCustom):
		VERSION = BASE.get_editor_property("description")
		if VERSION == "PlanetTreeAppearanceV2":
			RESULT["materials"].append({"name": NAME, "alreadyConfigured": True})
			continue
		if VERSION == "PlanetTreeAppearanceV1":
			OLD_FADE = "float bodyFade = 1 - smoothstep(1600, 1800, length(ObjectRelative - BodyCenterRelative));"
			NEW_FADE = ("float bodyFade = 1 - smoothstep(max(0, NearRadius - 400),\n"
				"    max(1, NearRadius - 200), length(ObjectRelative - BodyCenterRelative));")
			CODE = BASE.get_editor_property("code")
			if CODE.count(OLD_FADE) != 1:
				raise RuntimeError("V1 tree fade source differs from the expected material")
			BASE.set_editor_property("code", CODE.replace(OLD_FADE, NEW_FADE))
			BASE.set_editor_property("description", "PlanetTreeAppearanceV2")
			LIBRARY.recompile_material(MATERIAL)
			if not unreal.EditorAssetLibrary.save_loaded_asset(MATERIAL):
				raise RuntimeError("Could not save tree material: " + NAME)
			STATS = LIBRARY.get_statistics(MATERIAL)
			if STATS.num_pixel_shader_instructions <= 0:
				raise RuntimeError("Tree material did not compile: " + NAME)
			RESULT["materials"].append({"name": NAME, "upgraded": True,
				"pixelInstructions": STATS.num_pixel_shader_instructions})
			continue
		raise RuntimeError("Unknown tree material version: " + str(VERSION))
	ALPHA = LIBRARY.get_material_property_input_node(MATERIAL, unreal.MaterialProperty.MP_OPACITY_MASK) if LEAF else None
	CUSTOM = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionCustom)
	CUSTOM.set_editor_property("description", "PlanetTreeAppearanceV2")
	CUSTOM.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
	MASK = unreal.CustomOutput()
	MASK.set_editor_property("output_name", "RenderMask")
	MASK.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT1)
	CUSTOM.set_editor_property("additional_outputs", [MASK])
	INPUTS = {"BaseColor": (BASE, "" if LEAF else "RGB")}
	if LEAF:
		INPUTS["Alpha"] = (ALPHA, "R")
	for PARAMETER, DEFAULT in (("NearRadius", 0), ("ClipEnabled", 0)):
		NODE = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionScalarParameter)
		NODE.set_editor_property("parameter_name", PARAMETER)
		NODE.set_editor_property("default_value", DEFAULT)
		INPUTS[PARAMETER] = (NODE, "")
	for PARAMETER in ("NearCenterRelative", "BodyCenterRelative"):
		NODE = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionVectorParameter)
		NODE.set_editor_property("parameter_name", PARAMETER)
		INPUTS[PARAMETER] = (NODE, "RGB")
	POSITION = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionObjectPositionWS)
	CAMERA = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionCameraPositionWS)
	RELATIVE = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionSubtract)
	LIBRARY.connect_material_expressions(POSITION, "", RELATIVE, "A")
	LIBRARY.connect_material_expressions(CAMERA, "", RELATIVE, "B")
	INPUTS["ObjectRelative"] = (RELATIVE, "")
	if LEAF:
		TINT = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionPerInstanceCustomData3Vector)
		TINT.set_editor_property("data_index", 0)
		TINT.set_editor_property("const_default_value", unreal.LinearColor(1, 1, 1, 0))
		INPUTS["Tint"] = (TINT, "")
		SNOW = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionPerInstanceCustomData)
		SNOW.set_editor_property("data_index", 3)
		SNOW.set_editor_property("const_default_value", 0)
		INPUTS["Snow"] = (SNOW, "")
	VALUES = []
	for INPUT in INPUTS:
		VALUE = unreal.CustomInput()
		VALUE.set_editor_property("input_name", INPUT)
		VALUES.append(VALUE)
	CUSTOM.set_editor_property("inputs", VALUES)
	CODE = """
float nearFade = 1 - smoothstep(max(0, NearRadius - 200), max(1, NearRadius), length(ObjectRelative - NearCenterRelative));
float bodyFade = 1 - smoothstep(max(0, NearRadius - 400),
    max(1, NearRadius - 200), length(ObjectRelative - BodyCenterRelative));
RenderMask = ClipEnabled > 0.5 ? nearFade * bodyFade : 1;
"""
	CODE += "return lerp(BaseColor * Tint, float3(0.78, 0.82, 0.86), saturate(Snow));" if LEAF else "return BaseColor;"
	CUSTOM.set_editor_property("code", CODE)
	for INPUT, CONNECTION in INPUTS.items():
		if not LIBRARY.connect_material_expressions(CONNECTION[0], CONNECTION[1], CUSTOM, INPUT):
			raise RuntimeError("樹木材質の入力を接続出来ません：" + INPUT)
	LIBRARY.connect_material_property(CUSTOM, "", unreal.MaterialProperty.MP_BASE_COLOR)
	if LEAF:
		LIBRARY.connect_material_property(CUSTOM, "", unreal.MaterialProperty.MP_SUBSURFACE_COLOR)
	FADE = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionMaterialFunctionCall)
	FADE.set_editor_property("material_function", unreal.load_asset(
		"/Engine/Functions/Engine_MaterialFunctions02/Utility/DitherTemporalAA"
	))
	if not LIBRARY.connect_material_expressions(CUSTOM, "RenderMask", FADE, "Alpha Threshold"):
		raise RuntimeError("樹木の距離減衰を接続出来ません")
	OPACITY = FADE
	if LEAF:
		OPACITY = LIBRARY.create_material_expression(MATERIAL, unreal.MaterialExpressionMultiply)
		LIBRARY.connect_material_expressions(FADE, "", OPACITY, "A")
		LIBRARY.connect_material_expressions(ALPHA, "R", OPACITY, "B")
	LIBRARY.connect_material_property(OPACITY, "", unreal.MaterialProperty.MP_OPACITY_MASK)
	MATERIAL.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
	LIBRARY.layout_material_expressions(MATERIAL)
	LIBRARY.recompile_material(MATERIAL)
	if not unreal.EditorAssetLibrary.save_loaded_asset(MATERIAL):
		raise RuntimeError("樹木材質を保存出来ません")
	STATS = LIBRARY.get_statistics(MATERIAL)
	if STATS.num_pixel_shader_instructions <= 0:
		raise RuntimeError("樹木材質をコンパイル出来ません")
	RESULT["materials"].append({"name": NAME, "pixelInstructions": STATS.num_pixel_shader_instructions})
