#pragma once

#include "Core/RealtimeMeshDataStream.h"
#include <optional>
#include <stop_token>

struct FProcMeshSection;

// 完成した描画専用区画を，ワーカーで RMC の転送用配列へ変換
std::optional<RealtimeMesh::FRealtimeMeshStreamSet> PreparePlanetRealtimeSection(
	const FProcMeshSection &Section,
	const std::stop_token Stop=std::stop_token{}
);
