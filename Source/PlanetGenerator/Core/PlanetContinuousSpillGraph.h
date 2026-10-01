#pragma once

#include "PlanetDepressionHierarchy.h"
#include "PlanetSpillConnection.h"
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct ContinuousSpillSettings {
		// 同じ場の物理尺度と極小照合の精度
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		double MinimumMergeDistanceMeters = 0.001;
		double MinimumHeightToleranceMeters = 1E-6;
	};

	/** 同じ元場で確認した極小と鞍点だけを頂点・通路にする有限の排水網 */
	struct ContinuousSpillGraph {
		// 確認済の地点・通路と元の枝への対応
		std::vector<SpillGraphSite> Sites;
		// ０番は検証済の大洋を集約する為，陸上の極小を持たない
		std::vector<std::optional<TerrainCriticalPoint>> Minima;
		std::vector<SpillGraphEdge> Edges;
		std::vector<std::uint32_t> EdgeConnections;
		std::vector<std::array<std::uint32_t, 2>> BranchSites;
		DepressionHierarchy Hierarchy;
	};

	enum class ContinuousSpillFailure : std::uint8_t {
		None, InvalidInput, InvalidConnection, ConflictingMinimum, UnverifiedOcean, UnconnectedNetwork, Cancelled
	};

	struct ContinuousSpillDiagnostics {
		// 不完全な接続集合の拒否理由
		ContinuousSpillFailure Failure = ContinuousSpillFailure::None;
		std::uint32_t Connection = MissingSpillIndex;
	};

	std::optional<ContinuousSpillGraph> BuildContinuousSpillGraph(
		std::span<const SpillConnection> Connections,
		const std::function<bool(UnitDirection)> &IsConnectedOcean,
		const ContinuousSpillSettings &Settings,
		std::stop_token Stop={},
		ContinuousSpillDiagnostics *Diagnostics=nullptr
	);
}
