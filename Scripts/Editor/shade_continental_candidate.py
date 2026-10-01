"""所有する大陸候補の一時メッシュだけを局所高低差で着色する．"""

from __future__ import annotations
import builtins
from typing import cast
import unreal

RESULT: dict[str, object]

state: dict[str, object]|None = cast(
	dict[str, object] | None,
	builtins.__dict__.get("_planet_continental_candidate_preview_state")
)

if state is None:
	raise RuntimeError("No owned continental candidate preview exists.")

actor: unreal.PlanetContinentalPreview = cast(unreal.PlanetContinentalPreview, state["actor"])
mesh: unreal.ProceduralMeshComponent = actor.get_editor_property("PreviewMesh")
vertices, indices, normals, uvs, tangents = unreal.ProceduralMeshLibrary.get_section_from_procedural_mesh(mesh, 0)

if not vertices or len(vertices) != int(actor.get_editor_property("GeneratedVertices")):
	raise RuntimeError("The expected transient preview section is missing.")

minimum: float = min(float(vertex.z) for vertex in vertices)
maximum: float = max(float(vertex.z) for vertex in vertices)
span: float = maximum - minimum

if span <= 0:
	raise RuntimeError("The local mesh has no measurable relief.")

colors: list[unreal.LinearColor] = []

for vertex in vertices:
	fraction: float = (float(vertex.z) - minimum) / span
	colors.append(unreal.LinearColor(r=0.06 + 0.74 * fraction, g=0.20 + 0.46 * fraction, b=0.09 + 0.37 * fraction, a=1))

mesh.create_mesh_section_linear_color(0, vertices, indices, normals, uvs, [], [], [], colors, tangents, False)
unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).editor_invalidate_viewports()

RESULT = {
	"passed": True,
	"vertices": len(vertices),
	"triangles": len(indices) // 3,
	"minimumLocalZMeters": minimum / 100,
	"maximumLocalZMeters": maximum / 100,
	"scope": "Transient diagnostic color only; the C++ height field and saved assets are unchanged."
}
