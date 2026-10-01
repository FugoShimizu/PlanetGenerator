"""Coarse ocean diagnostic on the displayed mesh; not a continuous certificate."""
import math
import unreal

editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world = editor.get_editor_world()
found = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.PlanetSurfacePreview)
         if a.get_actor_label() == "Planet_GeologyPreview"]
if len(found) != 1:
    raise RuntimeError("Exactly one geological preview is required")
actor = found[0]
mesh = actor.get_component_by_class(unreal.ProceduralMeshComponent)
vertices, triangles, normals, uvs, tangents = unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh, 0)
positions = [(v.x, v.y, v.z) for v in vertices]

def dot(a, b):
    return sum(x*y for x, y in zip(a, b))

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def unit(v):
    length = math.sqrt(dot(v, v))
    return tuple(x/length for x in v)

def area(a, b, c):
    return 2*math.atan2(abs(dot(a, cross(b, c))), 1+dot(a, b)+dot(b, c)+dot(c, a))

directions = [unit(v) for v in positions]
heights = [math.sqrt(dot(v, v))/100-actor.radius_meters for v in positions]
parent = list(range(len(vertices)))

def root(i):
    while i != parent[i]:
        parent[i] = parent[parent[i]]
        i = parent[i]
    return i

def join(a, b):
    a, b = root(a), root(b)
    if a != b:
        parent[max(a,b)] = min(a,b)

# Piecewise linear height on each spherical triangle. Only negative vertices
# connect; point contacts at exactly zero depth do not connect components.
for offset in range(0, len(triangles), 3):
    ids = [int(triangles[offset+k]) for k in range(3)]
    wet = [i for i in ids if heights[i] < 0]
    for i in wet[1:]:
        join(wet[0], i)

components = {}
sphere_area = 0.0
for offset in range(0, len(triangles), 3):
    ids = [int(triangles[offset+k]) for k in range(3)]
    sphere_area += area(*(directions[i] for i in ids))
    wet = [i for i in ids if heights[i] < 0]
    if not wet:
        continue
    polygon = []
    for k in range(3):
        i, j = ids[k], ids[(k+1)%3]
        if heights[i] < 0:
            polygon.append(directions[i])
        if (heights[i] < 0) != (heights[j] < 0):
            t = heights[i]/(heights[i]-heights[j])
            polygon.append(unit(tuple((1-t)*directions[i][d]+t*directions[j][d] for d in range(3))))
    wet_area = sum(area(polygon[0],polygon[k],polygon[k+1]) for k in range(1,len(polygon)-1))
    component = root(wet[0])
    components[component] = components.get(component,0)+wet_area
if abs(sphere_area-4*math.pi) > 1e-8:
    raise RuntimeError("Diagnostic mesh does not cover spherical area 4pi")
sizes = sorted(components.values(), reverse=True)
RESULT = {"seed": actor.seed, "generator_version": actor.active_geology_version, "ocean_version": actor.active_ocean_version,
          "mesh_vertices": len(vertices), "mesh_triangles": len(triangles)//3,
          "coarse_submerged_components": len(sizes),
          "coarse_ocean_area_fraction": sum(sizes)/(4*math.pi),
          "largest_component_share_of_submerged_area": sizes[0]/sum(sizes) if sizes else 0,
          "other_components_fraction_of_sphere": sum(sizes[1:])/(4*math.pi),
          "largest_component_fractions_of_sphere": [s/(4*math.pi) for s in sizes[:12]],
          "approximation": "vertex heights, linear edge crossings, spherical polygon areas",
          "continuous_connectivity_validated": False, "channel_width_and_depth_validated": False,
          "saved_terrain_assets": False}

