"""Exercise geological definitions through the real UE adapter; save no terrain."""
import math
import unreal

actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if levels.is_in_play_in_editor():
    raise RuntimeError("Stop PIE before the geological definition checks")
actor = actors.spawn_actor_from_class(unreal.PlanetSurfacePreview, unreal.Vector(0, 0, 1e10), transient=True)
actor.set_actor_label("Planet_GeologyDefinitionCheck")
results = []
try:
    actor.set_editor_property("subdivision_level", 2)
    actor.set_editor_property("radius_meters", 6371000.0)
    actor.set_editor_property("use_geological_base", True)
    cases = [(s, 6371000.0) for s in (0, 1, 7, 42, 43, 90210, 2147483647, -1)]
    cases += [(42, r) for r in (1000.0, 1000000.0, 1000000000.0)]
    for seed, radius in cases:
        actor.set_editor_property("seed", seed)
        actor.set_editor_property("radius_meters", radius)
        if not actor.rebuild_preview():
            raise RuntimeError(actor.last_build_message)
        ocean = actor.initial_ocean_fraction_estimate
        if not 0.60 <= ocean <= 0.70:
            raise RuntimeError(f"Initial ocean fraction outside interval: {seed}, {radius}, {ocean}")
        if actor.geology_calibration_probe_count == actor.geology_area_probe_count:
            raise RuntimeError("Validation reused the calibration point count")
        heights = []
        normals = []
        for i in range(256):
            z = 1 - 2 * (i + 0.5) / 256
            phi = i * 2.399963229728653
            r = math.sqrt(1 - z*z)
            d = unreal.Vector(r*math.cos(phi), r*math.sin(phi), z)
            value = actor.evaluate_direction(d)
            if value is None:
                raise RuntimeError("Valid direction rejected")
            h, n = value
            if not all(math.isfinite(x) for x in (h, n.x, n.y, n.z)):
                raise RuntimeError("Non-finite sample")
            if radius + h <= 0 or n.x*d.x+n.y*d.y+n.z*d.z <= 0:
                raise RuntimeError("Non-positive surface radius or inward normal")
            if abs(n.x*n.x+n.y*n.y+n.z*n.z-1) > 1e-8:
                raise RuntimeError("Non-unit normal")
            heights.append(h)
            normals.append((n.x, n.y, n.z))
        # Changing display resolution must not change H or its analytic normal.
        actor.set_editor_property("subdivision_level", 3)
        if not actor.rebuild_preview():
            raise RuntimeError("LOD-independent rebuild failed")
        for i in range(255, -1, -1):
            z = 1 - 2 * (i + 0.5) / 256
            phi = i * 2.399963229728653
            r = math.sqrt(1 - z*z)
            h, n = actor.evaluate_direction(unreal.Vector(r*math.cos(phi), r*math.sin(phi), z))
            if h != heights[i] or (n.x, n.y, n.z) != normals[i]:
                raise RuntimeError("Changing LOD or query order changed geological samples")
        actor.set_editor_property("subdivision_level", 2)
        results.append({"seed": seed, "radius_m": radius, "ocean_fraction_estimate": ocean,
                        "target_ocean_fraction": actor.geology_target_ocean_fraction,
                        "min_height_m": min(heights), "max_height_m": max(heights),
                        "prepared_definition_ms": actor.prepared_definition_milliseconds,
                        "datum_shift_m": actor.geology_datum_shift_meters})
    RESULT = {"generator_version": actor.active_geology_version, "cases": results,
              "finite_outward_normal_checks": len(cases)*256,
              "lod_and_query_order_comparisons": len(cases)*256,
              "calibration_points": actor.geology_calibration_probe_count,
              "independent_area_points": actor.geology_area_probe_count,
              "saved_terrain_assets": False, "final_connectivity_validated": False,
              "production_performance_validated": False}
finally:
    actors.destroy_actor(actor)
