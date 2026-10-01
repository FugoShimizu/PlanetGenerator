#pragma once

#include "PlanetContinentalPotential.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t ContinentalDescentVersion = 1;

	enum class ContinentalDescentError : std::uint8_t {
		None,
		InvalidSettings,
		InvalidStart,
		Cancelled,
		DerivativeBoundsRejected,
		UnresolvedProgress,
		SegmentBudget
	};

	enum class ContinentalDescentControl : std::uint8_t { Gradient, NegativeCurvature };

	struct ContinentalDescentSettings {
		double MaximumSegmentLength = 0.04;
		std::size_t MaximumSegments = 4096;
	};

	struct ContinentalDescentPoint {
		double X = 0;
		double Y = 0;
		double PotentialHeightMeters = 0;
	};

	/** 円板内の直線区間に対する下降証明値，水面の補間又は球面曲線とは別 */
	struct ContinentalDescentSegment {
		double LengthPerUnit = 0;
		double InitialDirectionalDerivative = 0;
		double DirectionalDerivativeUpperBound = 0;
		double DirectionalCurvatureUpperBound = 0;
		ContinentalDescentControl Control = ContinentalDescentControl::Gradient;
	};

	struct ContinentalDescentStatistics {
		std::size_t BoundsEvaluations = 0;
		std::size_t CurvatureEscapes = 0;
		double EndBoundaryResidual = 0;
		double EndPotentialHeightMeters = 0;
	};

	/**
	 * 正の円板ポテンシャル上で，近傍微分の上界を用いて下降区間を構成する
	 * 勾配が解像出来ない場合は負の曲率を使い，数値的な海岸へ到達した経路だけを返す
	 * 球面・最終の河床・侵食・河川描画の完成証明ではない
	 */
	class ContinentalDescentPath {
	private:

		std::vector<ContinentalDescentPoint> Points;
		std::vector<ContinentalDescentSegment> Segments;
		ContinentalDescentStatistics Stats;
		std::uint64_t Hash = 0;

		ContinentalDescentPath() = default;

	public:

		static std::optional<ContinentalDescentPath> Create(
			const ContinentalPotential &Potential,
			const std::array<double, 2> Start,
			const ContinentalDescentSettings &Settings,
			ContinentalDescentError &Error,
			std::stop_token Stop={}
		);

		std::span<const ContinentalDescentPoint> GetPoints() const;
		std::span<const ContinentalDescentSegment> GetSegments() const;
		const ContinentalDescentStatistics &GetStatistics() const;
		std::uint64_t GetDefinitionHash() const;
	};
}
