"""Exercise the ocean composition through the real UE C++ field."""
import math
import unreal

actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
if unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).is_in_play_in_editor():
    raise RuntimeError("Stop PIE before ocean checks")
actor = actors.spawn_actor_from_class(unreal.PlanetSurfacePreview, unreal.Vector(0,0,1e10), transient=True)
actor.set_actor_label("Planet_OceanDefinitionCheck")
results = []

def unit(v):
    r = math.sqrt(sum(x*x for x in v))
    return tuple(x/r for x in v)

def direction(i, count):
    z = 1-2*(i+0.5)/count
    phi = i*2.399963229728653
    r = math.sqrt(1-z*z)
    return (r*math.cos(phi),r*math.sin(phi),z)

def sample(d):
    v = actor.evaluate_direction(unreal.Vector(*d))
    if v is None:
        raise RuntimeError("Valid direction rejected")
    h, n = v
    if not all(math.isfinite(x) for x in (h,n.x,n.y,n.z)):
        raise RuntimeError("Non-finite ocean sample")
    return (h,n.x,n.y,n.z)

try:
    actor.set_editor_property("subdivision_level", 2)
    actor.set_editor_property("radius_meters",1000000.0)
    actor.set_editor_property("use_geological_base",True)
    for seed in (42,0,1,7,43,90210,2147483647,-1):
        actor.set_editor_property("connect_ocean_basins",False)
        actor.set_editor_property("seed",seed)
        if not actor.rebuild_preview():
            raise RuntimeError(actor.last_build_message)
        count = 4096 if seed == 42 else 1024
        baseline = [sample(direction(i,count))[0] for i in range(count)]
        actor.set_editor_property("connect_ocean_basins",True)
        if not actor.rebuild_preview():
            raise RuntimeError(actor.last_build_message)
        modified = 0
        max_normal_error = 0.0
        saved = []
        for i in range(count):
            d = direction(i,count)
            v = sample(d)
            saved.append(v)
            h,nx,ny,nz = v
            if h > baseline[i]+1e-8:
                raise RuntimeError("A lowering-only ocean corridor raised the terrain")
            if abs(nx*nx+ny*ny+nz*nz-1)>1e-8:
                raise RuntimeError("Non-unit ocean normal")
            if baseline[i]-h<1e-5:
                continue
            modified += 1
            axis = (0,0,1) if abs(d[2])<0.8 else (1,0,0)
            tangent = unit((d[1]*axis[2]-d[2]*axis[1],d[2]*axis[0]-d[0]*axis[2],d[0]*axis[1]-d[1]*axis[0]))
            eps=1e-7
            dp=unit(tuple(d[k]*math.cos(eps)+tangent[k]*math.sin(eps) for k in range(3)))
            dm=unit(tuple(d[k]*math.cos(eps)-tangent[k]*math.sin(eps) for k in range(3)))
            hp,hm=sample(dp)[0],sample(dm)[0]
            st=tuple(((actor.radius_meters+hp)*dp[k]-(actor.radius_meters+hm)*dm[k])/(2*eps) for k in range(3))
            err=abs(nx*st[0]+ny*st[1]+nz*st[2])/actor.radius_meters
            max_normal_error=max(max_normal_error,err)
            if err>2e-5:
                raise RuntimeError("Ocean composition normal disagrees with surface differences")
        actor.set_editor_property("subdivision_level",3)
        if not actor.rebuild_preview():
            raise RuntimeError("Could not rebuild ocean mesh")
        for i in range(count-1,-1,-1):
            if sample(direction(i,count)) != saved[i]:
                raise RuntimeError("LOD or query order changed the ocean field")
        actor.set_editor_property("subdivision_level",2)
        results.append({"seed":seed,"components_before":actor.ocean_components_before,
                        "components_after":actor.ocean_components_after,"corridors":actor.ocean_corridor_count,
                        "ocean_fraction_estimate":actor.connected_ocean_fraction_estimate,
                        "preparation_ms":actor.ocean_preparation_milliseconds,
                        "max_graph_lowering_m":actor.ocean_maximum_sample_lowering_meters,
                        "samples":count,"modified_samples":modified,"max_relative_normal_error":max_normal_error})
    RESULT={"ocean_version":actor.active_ocean_version,"cases":results,
            "lowering_only_and_lod_invariance":"passed","continuous_final_connectivity_validated":False,
            "saved_terrain_assets":False,"production_performance_validated":False}
finally:
    actors.destroy_actor(actor)
