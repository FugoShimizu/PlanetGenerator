#pragma once

#include "PlanetSpillDiscovery.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	struct SpillSeedSettings {
		// 元場の尺度と，下降・停留点・流出口に共通する評価予算
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		std::uint32_t MaximumEvaluations = 4000000;
	};

	enum class SpillSeedFailure : std::uint8_t {
		None,
		InvalidInput,
		InvalidSource,
		BudgetLimit,
		Descent,
		ReachesSeaLevel,
		Minimum,
		Exit,
		Cancelled
	};

	struct SpillSeedDiagnostics {
		// 地点から実流出口へ至る準備段階と費用
		SpillSeedFailure Failure = SpillSeedFailure::None;
		std::uint64_t Evaluations = 0;
		TerrainDescentEnd DescentEnd = TerrainDescentEnd::BudgetLimit;
		SpillDiscoveryDiagnostics Discovery;
		std::optional<TerrainDescentSample> DescentEndpoint;
		TerrainCriticalDiagnostics Critical;
		TerrainCriticalDiagnostics InitialCritical;
		std::uint32_t Continuations = 0;
	};

	struct SpillSeed {
		// 入力地点から辿った実極小と，其の極小へ接続する実鞍点
		TerrainCriticalPoint Minimum;
		SpillConnection Connection;
	};

	std::optional<SpillSeed> FindSpillSeed(
		UnitDirection Location,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillSeedSettings &Settings,
		std::stop_token Stop= {},
		SpillSeedDiagnostics *Diagnostics=nullptr
	);
}
