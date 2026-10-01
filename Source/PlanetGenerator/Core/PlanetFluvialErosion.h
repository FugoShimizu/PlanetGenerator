#pragma once

#include "PlanetRunoff.h"

namespace PlanetCore {
	inline constexpr std::uint32_t FluvialErosionVersion = 1;

	/** 固定流路での侵食の参照時間と縮尺，観測日時とは独立 */
	struct FluvialErosionSettings {
		double Years = 200000;
		double ReferenceRadiusMeters = 6371000;
		double ReferenceDischargeCubicMetersPerSecond = 10;
		double ReferenceCelerityMetersPerYear = 1;
		double DischargeExponent = 0.4;
		double MaximumFractionalLowering = 0.8;
		bool IsValid() const;
	};

	struct FluvialErosionNode {
		double ElevationMeters = 0, RelativeReliefRatio = 1;
		double ErodedCubicMeters = 0, OutgoingSedimentCubicMeters = 0;
	};

	/** 離散的な放射状の土柱に対する体積収支，連続補間後の質量保証とは区別する */
	struct FluvialErosionStatistics {
		std::uint32_t ErodedNodes = 0, LoweringLimitedNodes = 0;
		double MaximumLoweringMeters = 0, TotalErodedCubicMeters = 0, OceanDeliveredCubicMeters = 0;
		double SedimentRelativeResidual = 0;
	};

	/** 流量を用いた n＝１の陰解法の比較基準，地形関数への再構成は別段階 */
	class PlanetFluvialErosion {
	private:
		std::vector<FluvialErosionNode> Values;
		FluvialErosionStatistics Stats;
		std::uint64_t SourceHash = 0;
		PlanetFluvialErosion() = default;

	public:
		static std::optional<PlanetFluvialErosion> Create(const DrainageNetwork &Network, const PlanetRunoff &Runoff,
			const FluvialErosionSettings &Settings = {}, const std::stop_token Stop = {});

		std::span<const FluvialErosionNode> GetNodes() const;
		const FluvialErosionStatistics &GetStatistics() const;
		std::uint64_t GetSourceDefinitionHash() const;
	};
}
