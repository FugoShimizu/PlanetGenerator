#pragma once

#include "PlanetFlowChannel.h"
#include "PlanetFlowRoute.h"
#include "PlanetSpillLake.h"
#include <optional>
#include <vector>

namespace PlanetCore {
	/** 水域の発見に使う読取専用の幾何；参照の寿命は所有する場と同じ */
	struct FlowWaterGeometry {
		// 新しい水位又は河床を作らず，準備済の経路・岸だけを参照
		const TerrainFlowRoute &Route;
		const SpillLakeShore &Upstream;
		const SpillLakeShore *Receiving;
		double ChannelHalfWidthMeters;
		double MaximumChannelHalfWidthMeters;
	};

	/** 同じ照会で得た河床・水位と，内側を正とする有限な所属距離 */
	struct FlowWaterSample {
		FlowChannelSection Section;
		double SupportMeters = 0;
	};

	/** 単独河道と水系全体を，同じ描画・衝突経路から照会する不変な場 */
	class FlowSurfaceField {
	public:

		virtual double GetRadiusMeters() const = 0;
		virtual std::vector<FlowWaterGeometry> GetWaterGeometries() const = 0;
		virtual std::optional<double> WaterSupport(const UnitDirection Direction) const = 0;
		virtual std::optional<FlowChannelSection> Evaluate(
			const UnitDirection Direction,
			const bool NeedsWater=true,
			const bool NeedsTerrainGradient=true
		) const = 0;
		virtual std::optional<FlowWaterSample> EvaluateWithSupport(
			const UnitDirection Direction,
			const bool NeedsTerrainGradient=true
		) const = 0;

		virtual ~FlowSurfaceField() = default;
	};
}
