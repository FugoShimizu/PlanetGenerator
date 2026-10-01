"""季節材質の接続と，実際にコンパイルされた描画統計を確認する"""
import unreal

material = unreal.load_asset("/Game/Planet/Materials/M_PlanetSeasonalSurface")
if not isinstance(material, unreal.Material): raise RuntimeError("季節材質が有りません")
lib = unreal.MaterialEditingLibrary
custom = lib.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)
if not isinstance(custom, unreal.MaterialExpressionCustom): raise RuntimeError("季節材質の入力が不正です")
names = lib.get_material_expression_input_names(custom)
nodes = lib.get_inputs_for_material_expression(material, custom)
if len(names) != len(nodes) or any(node is None for node in nodes): raise RuntimeError("未接続の材質入力が残っています")
required = {"ClimateAtlas", "ClimateEnabled", "SolarYearPhase", "ClimateInverseRadius", "ClimatePlanetOffset"}
if not required.issubset({str(name) for name in names}): raise RuntimeError("気候入力が欠けています")
statistics = lib.get_statistics(material)
fields = ("num_vertex_shader_instructions", "num_pixel_shader_instructions", "num_samplers", "num_uv_scalars")
values = {name: statistics.get_editor_property(name) for name in fields}
if values["num_pixel_shader_instructions"] <= 0 or values["num_samplers"] < 14:
	raise RuntimeError("コンパイル済みの季節材質を確認出来ません：" + str(values))
if values["num_samplers"] > 16:
	raise RuntimeError("SM5の16 sampler上限を超えています：" + str(values))
RESULT = {
	"material": material.get_path_name(), "connected_inputs": len(names), "compiled_statistics": values,
	"gpu_frame_time_verified": False, "runtime_climate_binding_verified": False
}
