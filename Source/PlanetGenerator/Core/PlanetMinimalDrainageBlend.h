#pragma once

#include "PlanetMinimalDrainageFill.h"
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t MinimalDrainageBlendVersion = 1;

	enum class MinimalDrainageBlendError : std::uint8_t {
		None,
		InvalidInput,
		FillRejected,
		ShoreSlope,
		NewOcean,
		NumericFailure,
		DrainageRejected,
		Cancelled
	};

	struct MinimalDrainageBlendStatistics {
		std::uint32_t Vertices = 0;
		std::uint32_t OceanVertices = 0;
		std::uint32_t ChangedLandVertices = 0;
		double MaximumRaiseMeters = 0;
		double MaximumCutMeters = 0;
		double SumAbsoluteChangeMeters = 0;
		double MinimumLandDropMeters = 0;
		double AppliedFillWeight = 0;
	};

	/**
	 * 同じ大洋根排水木上の最小盛土と切下げ包絡を線形結合する比較場
	 * 海陸の符号を保てない入力は拒否し，有限網の大洋排水だけを保証する
	 * 地質抵抗・水位・河道幅・連続補間・景観は別途拘束する
	 */
	class MinimalDrainageBlend {
	private:

		std::vector<double> HeightsMeters;
		std::vector<std::uint32_t> DownstreamVertices;
		MinimalDrainageBlendStatistics Stats;

		MinimalDrainageBlend() = default;

		static std::optional<MinimalDrainageBlend> CreateImpl(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			const MinimalDrainageFillSettings &Settings,
			double FillWeight,
			bool AdjustCoast,
			MinimalDrainageBlendError &Error,
			std::stop_token Stop
		);

	public:

		static std::optional<MinimalDrainageBlend> Create(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			const MinimalDrainageFillSettings &Settings,
			double FillWeight,
			MinimalDrainageBlendError &Error,
			std::stop_token Stop={}
		);

		/** 必要な海岸の比率だけ盛土側へ寄せ，元の海陸を保てる最小比率を選ぶ */
		static std::optional<MinimalDrainageBlend> CreateWithCoastalBalance(
			std::span<const UnitDirection> Directions,
			std::span<const std::array<std::uint32_t, 3>> Triangles,
			std::span<const double> OriginalHeightsMeters,
			const MinimalDrainageFillSettings &Settings,
			double MinimumFillWeight,
			MinimalDrainageBlendError &Error,
			std::stop_token Stop={}
		);

		std::span<const double> GetHeightsMeters() const;
		std::span<const std::uint32_t> GetDownstreamVertices() const;
		const MinimalDrainageBlendStatistics &GetStatistics() const;
	};
}
