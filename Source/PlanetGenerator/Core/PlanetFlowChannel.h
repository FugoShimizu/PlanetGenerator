#pragma once

#include "PlanetCore.h"
#include "PlanetFlowCurve.h"
#include <optional>
#include <span>
#include <vector>

namespace PlanetCore {
	struct FlowChannelShape {
		double RadiusMeters = 1000000;
		double HalfWidthMeters = 6;
		double DepthMeters = 1.5;
		// 同じ経路の公称距離に対する寸法の微分
		double HalfWidthDerivativePerMeter = 0;
		double DepthDerivativePerMeter = 0;
		// 河道端を元地形へ戻す強度と，同じ公称距離に対する微分
		double CutWeight = 1;
		double CutWeightDerivativePerMeter = 0;
		bool IsValid() const;
	};

	/** 生成時に固定する淵の有限な広がり，値は流量則の代用にしない */
	struct FlowChannelPool {
		double CenterDistanceMeters = 0;
		double HalfLengthMeters = 0;
		double AddedHalfWidthMeters = 0;
		double AddedDepthMeters = 0;
	};

	/** 元の断面と，重ならない淵を共有する軽量な縦断寸法 */
	class FlowChannelProfile {
	private:

		FlowChannelShape Base;
		std::vector<FlowChannelPool> Pools;
		double LengthMeters = 0;
		double MaximumHalfWidthMeters = 0;

		FlowChannelProfile() = default;

	public:

		static std::optional<FlowChannelProfile> Create(
			const FlowChannelShape &Base,
			const double LengthMeters,
			const std::span<const FlowChannelPool> Pools
		);

		std::optional<FlowChannelShape> Sample(const double DistanceMeters) const;
		double GetMaximumHalfWidthMeters() const;
	};

	std::optional<FlowChannelShape> FadeFlowChannelEnds(
		const FlowChannelShape &Shape,
		const double DistanceMeters,
		const double LengthMeters,
		const double FadeLengthMeters
	);

	struct FlowChannelWaterLevel {
		double ElevationMeters = 0;
		double DerivativePerMeter = 0;
	};

	struct FlowChannelWaterSettings {
		double FreeboardMeters = 0.3;
		double RampLengthMeters = 25;
		double DownstreamWaterMeters = 0;
		double ShoreBlendMeters = 0.05;
	};

	std::optional<FlowChannelWaterLevel> SampleFlowWaterLevel(
		const FlowChannelWaterLevel Source,
		const double DistanceMeters,
		const FlowChannelWaterSettings &Settings
	);

	// 鞍点を原点とし，上流の一定水面から下降する河川へ接続する
	std::optional<FlowChannelWaterLevel> SampleFlowOutletWaterLevel(
		const FlowChannelWaterLevel Source,
		const double SignedDistanceMeters,
		const double UpstreamWaterMeters,
		const FlowChannelWaterSettings &Settings
	);

	/** 元場の狭い開削と，中心線から延ばした水位の候補 */
	struct FlowChannelSection {
		TerrainSample Terrain;
		TerrainSample Water;
		double BedCoverage = 0;
		bool HasWater = false;
	};

	// 地面・衝突だけの照会は，水位を準備せず同じ浅い開削を評価する
	std::optional<FlowChannelSection> SampleFlowChannelTerrain(
		const TerrainFlowFootpoint &Footpoint,
		const TerrainSample &Here,
		const FlowChannelShape &Shape
	);

	// 岸の成立・流量・他の河道との合成は呼出側で検査する
	std::optional<FlowChannelSection> SampleFlowChannel(
		const TerrainFlowFootpoint &Footpoint,
		const TerrainSample &Here,
		const FlowChannelWaterLevel WaterLevel,
		const FlowChannelShape &Shape
	);
}
