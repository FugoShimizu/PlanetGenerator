#pragma once

#include "PlanetCriticalPoint.h"
#include <optional>

namespace PlanetCore {
	struct LakeSillSettings {
		// 局所的な静的開削の物理寸法；水位・流量の決定とは分離
		double RadiusMeters = 1000000;
		double HalfLengthMeters = 12;
		double HalfWidthMeters = 12;
		double DepthMeters = 0.25;
	};

	/** 非退化な鞍点の周囲だけを下げる C² の補正；通水の成立は合成した元場で別途検査する */
	class LakeSillPatch {
	private:

		// 元場と時刻に依存しない，主曲率方向の局所射影
		Vec3 Origin;
		Vec3 Along;
		Vec3 Across;
		double RadiusMeters = 0;
		double InverseLengthSquared = 0;
		double InverseWidthSquared = 0;
		double DepthMeters = 0;

		LakeSillPatch() = default;

	public:

		static std::optional<LakeSillPatch> Create(const TerrainCriticalPoint &Pass, const LakeSillSettings &Settings);
		TerrainSample EvaluateCorrection(UnitDirection Direction) const;
	};
}
