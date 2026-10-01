"""Show the bounded C++ diagnostic in the editor without saving terrain assets."""
import unreal

actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if levels.is_in_play_in_editor():
    raise RuntimeError("Stop PIE before placing the diagnostic sphere")
label = "Planet_CorePreview"
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
# EditorActorSubsystem excludes transient actors; query the world explicitly.
existing = [actor for actor in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetSurfacePreview)
            if actor.get_actor_label() == label]
if len(existing) > 1:
    raise RuntimeError("Multiple diagnostic preview actors found")
actor = existing[0] if existing else actors.spawn_actor_from_class(unreal.PlanetSurfacePreview, unreal.Vector(0,0,0), transient=True)
if not isinstance(actor, unreal.PlanetSurfacePreview):
    raise RuntimeError("The preview label belongs to another actor class")
actor.set_actor_label(label)
# The template sky dome/fog is intended for ground-level views. Put this
# temporary geometry diagnostic above it, without changing any map assets.
actor.set_actor_location(unreal.Vector(0, 0, 100000000), False, False)
actor.set_editor_property("seed", 90210)
actor.set_editor_property("radius_meters", 10000.0)
actor.set_editor_property("height_amplitude_meters", 1500.0)
actor.set_editor_property("subdivision_level", 5)
if not actor.rebuild_preview():
    raise RuntimeError(actor.get_editor_property("last_build_message"))
radius_cm = actor.get_editor_property("radius_meters") * 100
camera = actor.get_actor_location() + unreal.Vector(radius_cm * 2.1, -radius_cm * 2.1, radius_cm * 0.9)
rotation = unreal.MathLibrary.find_look_at_rotation(camera, actor.get_actor_location())
unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(camera, rotation)
levels.editor_invalidate_viewports()
actors.set_selected_level_actors([actor])
sample = actor.evaluate_direction(unreal.Vector(0,0,1))
if sample is None or actor.evaluate_direction(unreal.Vector(0,0,0)) is not None:
    raise RuntimeError("Directional evaluation validation failed")
if actor.generated_vertex_count != 10242 or actor.generated_triangle_count != 20480:
    raise RuntimeError("Unexpected generated topology counts")
def sample_values(value):
    elevation, normal = value
    return (float(elevation), float(normal.x), float(normal.y), float(normal.z))
if sample_values(sample) != sample_values(actor.evaluate_direction(unreal.Vector(0,0,1))):
    raise RuntimeError("Repeated directional sample differs")
try:
    actor.set_editor_property("radius_meters", 0.0)
    if actor.rebuild_preview() or actor.generated_vertex_count != 10242:
        raise RuntimeError("Invalid radius did not retain the valid preview")
finally:
    actor.set_editor_property("radius_meters", 10000.0)
    if not actor.rebuild_preview():
        raise RuntimeError("Could not restore the valid preview definition")
RESULT = {"actor":actor.get_path_name(), "vertices":actor.generated_vertex_count,
          "triangles":actor.generated_triangle_count, "build_ms":actor.last_build_milliseconds,
          "radius_m":actor.radius_meters, "seed":actor.seed,
          "sample_elevation_and_normal":sample_values(sample), "saved_terrain_assets":False,
          "diagnostic_only":True, "preview_actor_count":len(existing) if existing else 1, "invalid_input_retains_geometry":True, "same_direction_repeatable":True, "message":actor.last_build_message}
