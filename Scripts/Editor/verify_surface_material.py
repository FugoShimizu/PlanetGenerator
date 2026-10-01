import json
from pathlib import Path

import unreal


library = unreal.MaterialEditingLibrary
expected_textures = {
	"SandNormal": "/Game/Planet/Textures/aerial_beach_01_nor_dx",
	"SandRippleColor": "/Game/Planet/Textures/aerial_beach_01_diffuse",
}
materials = {}
for name in ("M_PlanetSurface", "M_PlanetSeasonalSurface"):
	path = "/Game/Planet/Materials/" + name
	material = unreal.load_asset(path)
	if not isinstance(material, unreal.Material):
		raise RuntimeError("Material unavailable: " + path)
	custom = library.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)
	if not isinstance(custom, unreal.MaterialExpressionCustom):
		raise RuntimeError("Surface shader unavailable: " + path)
	code = custom.get_editor_property("code")
	if "RippleUV = P * (1.0 / 32.0)" not in code or "WindSandBase" not in code:
		raise RuntimeError("Ripple shader missing: " + path)
	inputs = {str(key): value for key, value in zip(library.get_material_expression_input_names(custom),
		library.get_inputs_for_material_expression(material, custom))}
	for parameter, texture_path in expected_textures.items():
		node = inputs.get(parameter)
		if not isinstance(node, unreal.MaterialExpressionTextureObjectParameter):
			raise RuntimeError("Texture parameter unavailable: " + parameter)
		texture = node.get_editor_property("texture")
		if texture.get_path_name().split(".")[0] != texture_path:
			raise RuntimeError("Unexpected texture for " + parameter)
	stats = library.get_statistics(material)
	fields = ("num_vertex_shader_instructions", "num_pixel_shader_instructions", "num_samplers",
		"num_pixel_texture_samples", "num_uv_scalars")
	values = {field: stats.get_editor_property(field) for field in fields}
	if values["num_pixel_shader_instructions"] <= 0 or values["num_samplers"] < 11:
		raise RuntimeError("Compiled shader unavailable: " + str(values))
	materials[name] = values

records = json.loads((Path(unreal.Paths.project_dir()) /
	"SourceAssets/SurfaceTextures/Sources.json").read_text(encoding="utf-8"))
textures = []
for item in records:
	path = "/Game/Planet/Textures/" + Path(item["path"]).stem
	texture = unreal.load_asset(path)
	if not isinstance(texture, unreal.Texture2D):
		raise RuntimeError("Imported texture unavailable: " + path)
	is_color = item["map"] == "Diffuse"
	if texture.get_editor_property("srgb") != is_color:
		raise RuntimeError("Incorrect texture color space: " + path)
	textures.append({"asset": path, "srgb": is_color,
		"max_size": texture.get_editor_property("max_texture_size"),
		"compression": str(texture.get_editor_property("compression_settings"))})

RESULT = {"compiled_statistics": materials, "textures": textures,
	"custom_hlsl_texture_sample_count_not_in_estimate": True,
	"visual_quality_verified": False, "gpu_frame_time_verified": False}
