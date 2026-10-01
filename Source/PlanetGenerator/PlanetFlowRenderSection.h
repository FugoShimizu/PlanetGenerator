#pragma once

#include "ProceduralMeshComponent.h"
#include <optional>
#include <stop_token>

namespace PlanetCore {
	struct FlowTerrainMesh;
	struct FlowWaterMesh;
}

// 同じ場の地面・水面を，保存しない描画区画へ構成
std::optional<FProcMeshSection> PreparePlanetFlowSection(
	const PlanetCore::FlowTerrainMesh &Mesh,
	const std::stop_token Stop={}
);
std::optional<FProcMeshSection> PreparePlanetFlowSection(
	const PlanetCore::FlowWaterMesh &Mesh,
	const std::stop_token Stop={}
);
