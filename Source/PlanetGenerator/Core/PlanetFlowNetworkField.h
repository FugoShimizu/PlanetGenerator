#pragma once

#include "PlanetFlowNetwork.h"
#include "PlanetFlowSurfaceField.h"
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace PlanetCore {
	/** 同じ元場から準備した有限水系の，単一の河床・水面照会 */
	class FlowNetworkField final : public FlowSurfaceField {
	private:

		struct ChannelBounds {
			// 曲線と湖内部を保守的に含む単位球上の箱
			Vec3 Low;
			Vec3 High;
			std::shared_ptr<const FlowChannelField> Field;
		};

		// 元場と各河道を同じ寿命で保持し，準備後は変更しない
		std::function<TerrainSample(UnitDirection)> Source;
		std::function<double(UnitDirection)> SourceHeight;
		std::vector<ChannelBounds> Channels;
		double RadiusMeters = 0;
		double OutsideSupport = 0;

		std::optional<FlowWaterSample> EvaluateSample(
			const UnitDirection Direction,
			const bool NeedsWater,
			const bool NeedsTerrainGradient
		) const;

		FlowNetworkField() = default;

	public:

		double GetRadiusMeters() const override;
		std::vector<FlowWaterGeometry> GetWaterGeometries() const override;
		std::optional<double> WaterSupport(const UnitDirection Direction) const override;

		std::optional<FlowChannelSection> Evaluate(
			const UnitDirection Direction,
			const bool NeedsWater=true,
			const bool NeedsTerrainGradient=true
		) const override;

		std::optional<FlowWaterSample> EvaluateWithSupport(
			const UnitDirection Direction,
			const bool NeedsTerrainGradient=true
		) const override;

		static std::optional<FlowNetworkField> Create(const PreparedFlowNetwork &Network);
	};
}
