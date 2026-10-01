"""一時表示した大陸候補の実メッシュ頂点範囲を調べる．"""

from __future__ import annotations
import base64
import builtins
import math
import struct
import zlib
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

if not vertices or not indices or len(vertices) != int(actor.get_editor_property("GeneratedVertices")):
	raise RuntimeError("The visible preview mesh was not read back.")

values: list[float] = [float(vertex.z) / 100 for vertex in vertices]
heights: list[float] = sorted(values)
last: int = len(heights) - 1
side: int = math.isqrt(len(values))

if side * side != len(values):
	raise RuntimeError("The owned preview section is not a square local grid.")

grid: str = base64.b64encode(zlib.compress(struct.pack(f"<{len(values)}f", *values))).decode("ascii")

RESULT = {
	"passed": True,
	"vertices": len(vertices),
	"triangles": len(indices) // 3,
	"localZQuantilesMeters": [heights[int(last * fraction)] for fraction in (0, 0.1, 0.5, 0.9, 1)],
	"localZSpanMeters": heights[-1] - heights[0],
	"gridSide": side,
	"gridFloat32ZlibBase64": grid,
	"anchorElevationMeters": float(actor.get_editor_property("AnchorElevationMeters")),
	"scope": "Readback of CPU mesh section in the editor; not a GPU readback or terrain acceptance."
}
