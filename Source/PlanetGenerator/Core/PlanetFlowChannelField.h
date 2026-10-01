#pragma once

#include "PlanetFlowSurfaceField.h"
#include "PlanetFlowWaterProfile.h"
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace PlanetCore {
	class FlowNetworkField;

	/** 同じ元場と確認済の経路・岸を，生成時に一度だけ束縛する入力 */
	struct FlowChannelFieldDefinition {
		// 呼出中も有効な不変場；標高専用の照会は省略時に全微分から取得
		std::function<TerrainSample(UnitDirection)> Source;
		std::function<double(UnitDirection)> SourceHeight;
		// 視点と独立な経路と両側の岸
		std::shared_ptr<const TerrainFlowRoute> Route;
		std::shared_ptr<const SpillLakeShore> Lake;
		std::shared_ptr<const SpillLakeShore> ReceivingLake;
		// 同じ物理尺度の断面・縦断寸法と水位
		FlowChannelShape Shape;
		std::shared_ptr<const FlowChannelProfile> ShapeProfile;
		FlowChannelWaterSettings WaterSettings;
		double PassDistanceMeters = 0;
		double UpstreamWaterMeters = 0;
		double WaterQueryHalfWidthMeters = 0;
	};

	/** 描画・衝突が共有出来る不変な河床・水位・所属の地点評価 */
	class FlowChannelField final : public FlowSurfaceField {
	private:

		friend class FlowNetworkField;

		// 外部の組立途中の変更を保持しない，所有する定義
		FlowChannelFieldDefinition Definition;
		std::optional<FlowWaterProfile> WaterProfile;
		double EvaluateSourceHeight(const UnitDirection Direction) const;

		std::optional<FlowChannelSection> EvaluateOnSource(
			const UnitDirection Direction,
			const TerrainSample Here,
			const bool NeedsWater,
			const bool HasSourceGradient
		) const;

		std::optional<double> WaterSupportOnHeight(const UnitDirection Direction, const double ElevationMeters) const;

		FlowChannelField() = default;

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

		FlowWaterGeometry GetWaterGeometry() const;

		static std::optional<FlowChannelField> Create(
			FlowChannelFieldDefinition Definition,
			const std::stop_token Stop= {},
			FlowWaterProfileDiagnostics *const WaterDiagnostics=nullptr
		);
	};
}
