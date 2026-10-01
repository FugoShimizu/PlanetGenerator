"""Display and probe the geological base, without saving generated terrain."""
import math
import unreal

actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
if levels.is_in_play_in_editor():
    raise RuntimeError("Stop PIE before the geological base preview")
world = editor.get_editor_world()
label = "Planet_GeologyPreview"
existing = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetSurfacePreview)
            if a.get_actor_label() == label]
if len(existing) > 1:
    raise RuntimeError("Duplicate geological preview actors")
actor = existing[0] if existing else actors.spawn_actor_from_class(unreal.PlanetSurfacePreview, unreal.Vector(0, 0, 1e10), transient=True)
actor.set_actor_label(label)
actor.set_actor_location(unreal.Vector(0, 0, 1e10), False, False)
actor.set_editor_property("connect_ocean_basins", bool(ARGS.get("connect_oceans", False)))
actor.set_editor_property("use_geological_base", True)
actor.set_editor_property("seed", 42)
actor.set_editor_property("radius_meters", 1000000.0)
actor.set_editor_property("subdivision_level", 6)
if not actor.rebuild_preview():
    raise RuntimeError(actor.last_build_message)
first_preparation_ms = actor.prepared_definition_milliseconds
first_build_ms = actor.last_build_milliseconds

def sample(d):
    value = actor.evaluate_direction(unreal.Vector(*d))
    if value is None:
        raise RuntimeError("Valid direction rejected")
    height, normal = value
    result = (float(height), float(normal.x), float(normal.y), float(normal.z))
    if not all(math.isfinite(x) for x in result):
        raise RuntimeError("Non-finite geological sample")
    return result

def direction(index, count):
    z = 1 - 2 * (index + 0.5) / count
    phi = index * 2.399963229728653
    r = math.sqrt(1 - z*z)
    return (r*math.cos(phi), r*math.sin(phi), z)

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def unit(v):
    length = math.sqrt(sum(x*x for x in v))
    return tuple(x/length for x in v)

heights = []
maximum_normal_error = 0.0
for i in range(256):
    d = direction(i, 256)
    h, nx, ny, nz = sample(d)
    heights.append(h)
    normal = (nx, ny, nz)
    if abs(sum(x*x for x in normal) - 1) > 1e-8:
        raise RuntimeError("Non-unit surface normal")
    tangent = unit(cross(d, (0,0,1) if abs(d[2]) < 0.8 else (1,0,0)))
    epsilon = 1e-7
    plus = unit(tuple(d[k]*math.cos(epsilon)+tangent[k]*math.sin(epsilon) for k in range(3)))
    minus = unit(tuple(d[k]*math.cos(epsilon)-tangent[k]*math.sin(epsilon) for k in range(3)))
    hp, hm = sample(plus)[0], sample(minus)[0]
    surface_tangent = tuple(((actor.radius_meters+hp)*plus[k]-(actor.radius_meters+hm)*minus[k])/(2*epsilon) for k in range(3))
    error = abs(sum(normal[k]*surface_tangent[k] for k in range(3))) / actor.radius_meters
    maximum_normal_error = max(maximum_normal_error, error)
    if error > 2e-5:
        raise RuntimeError("Geology normal disagrees with independently displaced surface")

baseline = [sample(direction(i, 32)) for i in range(32)]
try:
    actor.set_editor_property("seed", 43)
    if not actor.rebuild_preview():
        raise RuntimeError("Second seed preparation failed")
    seed_changed = any(sample(direction(i, 32))[0] != baseline[i][0] for i in range(32))
finally:
    actor.set_editor_property("seed", 42)
    if not actor.rebuild_preview():
        raise RuntimeError("Could not restore the original geology")
repreparation_ms = actor.prepared_definition_milliseconds
if not seed_changed or baseline != [sample(direction(i, 32)) for i in range(32)]:
    raise RuntimeError("Seed variation or independent re-preparation repeatability failed")
if actor.generated_vertex_count != 40962 or actor.generated_triangle_count != 81920:
    raise RuntimeError("Unexpected geological preview mesh count")
try:
    actor.set_editor_property("radius_meters", 0.0)
    if actor.rebuild_preview() or sample(direction(0, 32)) != baseline[0]:
        raise RuntimeError("Rejected radius did not retain the previous field")
finally:
    actor.set_editor_property("radius_meters", 1000000.0)
    if not actor.rebuild_preview():
        raise RuntimeError("Could not restore the valid radius")
if actor.last_preparation_milliseconds != 0:
    raise RuntimeError("Unchanged geological definition was unnecessarily recomputed")

# This small material asset is reusable project content, not saved terrain.
# It intentionally shows height colors, not water shading or biome textures.
material_path = "/Game/Planet/Materials/M_PlanetGeologyDiagnostic"
material = unreal.load_asset(material_path) if unreal.EditorAssetLibrary.does_asset_exist(material_path) else None
if material is None:
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "M_PlanetGeologyDiagnostic", "/Game/Planet/Materials", unreal.Material, unreal.MaterialFactoryNew())
    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    color = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionVertexColor, -250, 0)
    if not unreal.MaterialEditingLibrary.connect_material_property(color, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR):
        raise RuntimeError("Could not connect diagnostic vertex colors")
    unreal.MaterialEditingLibrary.recompile_material(material)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material):
        raise RuntimeError("Could not save the reusable diagnostic material")
if not isinstance(material, unreal.Material):
    raise RuntimeError("Diagnostic material path belongs to another asset type")
component = actor.get_component_by_class(unreal.ProceduralMeshComponent)
component.set_material(0, material)
radius_cm = actor.radius_meters * 100
view_offset = ARGS.get("view_offset_radii", (2.1, -2.1, 0.9))
if len(view_offset) != 3 or not all(math.isfinite(float(x)) for x in view_offset) or sum(float(x)**2 for x in view_offset) < 1.21:
    raise ValueError("Preview camera offset must be a finite direction outside the planet")
camera = actor.get_actor_location() + unreal.Vector(*(radius_cm*float(x) for x in view_offset))
editor.set_level_viewport_camera_info(camera, unreal.MathLibrary.find_look_at_rotation(camera, actor.get_actor_location()))
actors.set_selected_level_actors([actor])
levels.editor_invalidate_viewports()
RESULT = {"actor": actor.get_path_name(), "radius_m": actor.radius_meters, "seed": actor.seed,
          "vertices": actor.generated_vertex_count, "triangles": actor.generated_triangle_count,
          "generator_version": actor.active_geology_version, "features": actor.geologic_feature_count, "initial_ocean_fraction_estimate": actor.initial_ocean_fraction_estimate,
          "calibration_probe_count": actor.geology_calibration_probe_count, "independent_area_probe_count": actor.geology_area_probe_count,
          "continental_scale": actor.geology_continental_scale, "datum_shift_m": actor.geology_datum_shift_meters,
          "target_ocean_fraction": actor.geology_target_ocean_fraction, "first_preparation_ms": first_preparation_ms, "repreparation_ms": repreparation_ms, "first_build_ms": first_build_ms,
          "cached_mesh_build_ms": actor.last_build_milliseconds, "height_probe_count": 256,
          "min_probe_height_m": min(heights), "max_probe_height_m": max(heights),
          "maximum_relative_normal_error": maximum_normal_error, "independent_repreparation_repeatable": True,
          "invalid_radius_retains_field": True, "saved_terrain_assets": False,
          "diagnostic_material": material_path, "active_ocean_version": actor.active_ocean_version, "ocean_components_before": actor.ocean_components_before,
          "ocean_components_after": actor.ocean_components_after, "ocean_corridor_count": actor.ocean_corridor_count,
          "connected_ocean_fraction_estimate": actor.connected_ocean_fraction_estimate,
          "ocean_preparation_ms": actor.ocean_preparation_milliseconds,
          "ocean_maximum_sample_lowering_m": actor.ocean_maximum_sample_lowering_meters,
          "final_ocean_connectivity_validated": False,
          "production_terrain_validated": False, "message": actor.last_build_message}
