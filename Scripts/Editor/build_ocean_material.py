"""Build/reuse the spherical ocean Single Layer Water material."""
from pathlib import Path
import unreal
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor(): raise RuntimeError("Stop PIE first")
root=Path(unreal.Paths.project_dir())
lib=unreal.MaterialEditingLibrary
path="/Game/Planet/Materials/M_PlanetOcean"
m=unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else unreal.AssetToolsHelpers.get_asset_tools().create_asset("M_PlanetOcean","/Game/Planet/Materials",unreal.Material,unreal.MaterialFactoryNew())
m.set_editor_property("shading_model",unreal.MaterialShadingModel.MSM_SINGLE_LAYER_WATER)
m.set_editor_property("blend_mode",unreal.BlendMode.BLEND_OPAQUE)
m.set_editor_property("tangent_space_normal",True)
custom=lib.get_material_property_input_node(m,unreal.MaterialProperty.MP_NORMAL)
if custom is not None and not isinstance(custom,unreal.MaterialExpressionCustom): raise RuntimeError("Unexpected existing water graph")
old={}
if custom is not None:
    old={str(k):v for k,v in zip(lib.get_material_expression_input_names(custom),lib.get_inputs_for_material_expression(m,custom)) if v is not None}
if custom is None: custom=lib.create_material_expression(m,unreal.MaterialExpressionCustom,0,0)
def node(name,kind):
    existing=old.get(name)
    if existing is not None and not isinstance(existing,kind): raise RuntimeError("Wrong water input: "+name)
    return existing or lib.create_material_expression(m,kind,-600,len(old)*80)
inputs={}
for i in (2,3):
    n=node("UV"+str(i),unreal.MaterialExpressionTextureCoordinate);n.set_editor_property("coordinate_index",i);inputs["UV"+str(i)]=n
n=node("LocalPosition",unreal.MaterialExpressionLocalPosition)
n.set_editor_property("included_offsets",unreal.PositionIncludedOffsets.EXCLUDE_OFFSETS);n.set_editor_property("local_origin",unreal.LocalPositionOrigin.PRIMITIVE);inputs["LocalPosition"]=n
for name,value in (("TextureOrigin",(0,0,0)),("TextureAxisX",(1,0,0)),("TextureAxisY",(0,1,0)),("TextureAxisZ",(0,0,1))):
    n=node(name,unreal.MaterialExpressionVectorParameter);n.set_editor_property("parameter_name",name);n.set_editor_property("default_value",unreal.LinearColor(*value,0));inputs[name]=n
n=node("Clock",unreal.MaterialExpressionTime);n.set_editor_property("override_period",True);n.set_editor_property("period",64.0);inputs["Clock"]=n
custom.set_editor_property("output_type",unreal.CustomMaterialOutputType.CMOT_FLOAT3)
custom.set_editor_property("code",(root/"Shaders/PlanetOcean.ush").read_text(encoding="utf-8"))
values=[]
for name in inputs:
    value=unreal.CustomInput();value.set_editor_property("input_name",name);values.append(value)
custom.set_editor_property("inputs",values)
for name,n in inputs.items():
    if not lib.connect_material_expressions(n,"",custom,name): raise RuntimeError("Water input: "+name)
lib.connect_material_property(custom,"",unreal.MaterialProperty.MP_NORMAL)
for prop,value in ((unreal.MaterialProperty.MP_BASE_COLOR,0),(unreal.MaterialProperty.MP_METALLIC,0),(unreal.MaterialProperty.MP_SPECULAR,0.255),(unreal.MaterialProperty.MP_ROUGHNESS,0.08),(unreal.MaterialProperty.MP_OPACITY,0.05)):
    n=lib.get_material_property_input_node(m,prop) or lib.create_material_expression(m,unreal.MaterialExpressionConstant,250,0)
    n.set_editor_property("r",value);lib.connect_material_property(n,"",prop)
# Persist only expression identifiers for reusable authoring; no terrain data.
output_path=unreal.EditorAssetLibrary.get_metadata_tag(m,"PlanetOcean.VolumeOutput")
water=unreal.load_object(None,output_path) if output_path else lib.create_material_expression(m,unreal.MaterialExpressionSingleLayerWaterMaterialOutput,300,400)
if not isinstance(water,unreal.MaterialExpressionSingleLayerWaterMaterialOutput): raise RuntimeError("Water volume output missing")
unreal.EditorAssetLibrary.set_metadata_tag(m,"PlanetOcean.VolumeOutput",water.get_path_name())
old_volume={str(k):v for k,v in zip(lib.get_material_expression_input_names(water),lib.get_inputs_for_material_expression(m,water)) if v is not None}
for name,value in (("ScatteringCoefficients",(0.00004,0.00009,0.00012)),("AbsorptionCoefficients",(0.004,0.0005,0.0002)),("PhaseG",(0.4,0.4,0.4)),("ColorScaleBehindWater",(1,1,1))):
    n=old_volume.get(name) or lib.create_material_expression(m,unreal.MaterialExpressionConstant3Vector,0,400)
    n.set_editor_property("constant",unreal.LinearColor(*value,1))
    if not lib.connect_material_expressions(n,"",water,name): raise RuntimeError("Water volume input: "+name)
lib.layout_material_expressions(m);lib.recompile_material(m)
if not unreal.EditorAssetLibrary.save_loaded_asset(m): raise RuntimeError("Water material save failed")
stats=lib.get_statistics(m)
RESULT={"material":m.get_path_name(),"pixel_instructions":stats.num_pixel_shader_instructions,"vertex_instructions":stats.num_vertex_shader_instructions,
        "base_sphere_displacement":False,"normal_waves":True,"wave_period_seconds":64,"extra_texture_assets":0,"visual_verified":False}
if RESULT["pixel_instructions"]<=0: raise RuntimeError("Ocean material did not compile: "+str(RESULT))
