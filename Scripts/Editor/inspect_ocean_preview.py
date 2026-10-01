"""Inspect only the owned visual ocean probe; does not certify physical movement."""
from pathlib import Path
import unreal
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE before inspecting the ocean preview")
found=[a for a in unreal.GameplayStatics.get_all_actors_of_class(editor.get_editor_world(),unreal.PlanetTerrain)
       if a.get_actor_label()=="Planet_SurfaceRenderProbe"]
if len(found)!=1: raise RuntimeError("Expected one owned surface preview")
t=found[0]
mesh=t.get_visible_ocean_mesh()
material=unreal.load_asset("/Game/Planet/Materials/M_PlanetOcean")
lib=unreal.MaterialEditingLibrary
stats=lib.get_statistics(material)
camera,_=editor.get_level_viewport_camera_info()
local=unreal.MathLibrary.inverse_transform_location(t.get_actor_transform(),camera)/100
height,_=t.evaluate_direction(local)
RESULT={"busy":t.generation_busy,"published_revision":t.published_revision,"requested_revision":t.requested_revision,
        "message":t.generation_message,"ocean_triangles":t.visible_ocean_triangle_count,
        "ocean_visible":mesh is not None and mesh.is_visible(),"ocean_build_ms":t.last_ocean_build_milliseconds,
        "camera_above_sea_m":local.length()-t.radius_meters,"seabed_height_m":height,
        "pixel_instructions":stats.num_pixel_shader_instructions,"vertex_instructions":stats.num_vertex_shader_instructions,
        "world_position_offset_connected":lib.get_material_property_input_node(material,unreal.MaterialProperty.MP_WORLD_POSITION_OFFSET) is not None,
        "shader_matches_source":lib.get_material_property_input_node(material,unreal.MaterialProperty.MP_NORMAL).get_editor_property("code")==
            (Path(unreal.Paths.project_dir())/"Shaders/PlanetOcean.ush").read_text(encoding="utf-8")}
