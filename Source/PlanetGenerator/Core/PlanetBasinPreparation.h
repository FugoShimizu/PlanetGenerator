#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace PlanetCore {
	/** 盆地準備の相互に重ならない計測段階 */
	enum class BasinPreparationPhase : std::size_t {
		BalancedProfiles,
		CoastalProfiles,
		FloorSamples,
		InitialFill,
		ConstraintAssembly,
		ConstraintApplication,
		FinalProfiles,
		FieldAssembly,
		Count
	};

	/** 実行中の一回の準備だけを計測する，生成定義や地形の保存には含めない */
	struct BasinPreparationStatistics {
		std::array<double, static_cast<std::size_t>(BasinPreparationPhase::Count)> PhaseMilliseconds {};
		std::uint64_t OriginalHeightEvaluations = 0;
		std::uint64_t OriginalHeightHits = 0;
		std::size_t HeightCacheBytes = 0;
		double TotalMilliseconds = 0;
		bool Completed = false;
	};
}
