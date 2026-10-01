"""Read the diagnostic actors and their mesh components without changing them."""
import unreal

world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
RESULT = []
for actor in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetSurfacePreview):
    if isinstance(actor, unreal.PlanetSurfacePreview):
        components = actor.get_components_by_class(unreal.ProceduralMeshComponent)
        RESULT.append({
            "path": actor.get_path_name(), "label": actor.get_actor_label(),
            "radius_m": actor.radius_meters, "seed": actor.seed,
            "level": actor.subdivision_level, "vertices": actor.generated_vertex_count,
            "triangles": actor.generated_triangle_count,
            "location": [actor.get_actor_location().x, actor.get_actor_location().y, actor.get_actor_location().z],
            "components": [{"name": c.get_name(), "material": c.get_material(0).get_path_name() if c.get_material(0) else None,
                            "visible": c.is_visible(), "sections": c.get_num_sections()} for c in components],
        })
