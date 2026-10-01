#pragma once

#include "PlanetContinentalDescent.h"
#include "PlanetCore.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	class SphericalContinentalField;

	inline constexpr std::uint32_t SphericalContinentalDescentVersion = 1;

	enum class SphericalContinentalDescentReason : std::uint8_t {
		None,
		Cancelled,
		InvalidField,
		StartInOcean,
		UnresolvedAtlas,
		DiskPathRejected,
		InvalidProfile
	};

	struct SphericalContinentalDescentError {
		SphericalContinentalDescentReason Reason = SphericalContinentalDescentReason::None;
		ContinentalDescentError DiskReason = ContinentalDescentError::None;
	};

	/** 同じ球面地形上の下降曲線，水位は侵食前の比較用標高 */
	struct SphericalContinentalDescentSample {
		UnitDirection Direction;
		Vec3 DirectionDerivativePerFraction;
		double ElevationMeters = 0;
		double ElevationDerivativePerFraction = 0;
	};

	/**
	 * 証明値を持つ円板経路を，同じ球面の不変場へ戻す比較用の下降曲線
	 * 区間内の水位は単調な三乗変換，河口の基準水位は球状海面の零へ接続する
	 * 河床の侵食，河川の合流，水面描画と本番の景観は此の型の外で扱う
	 */
	class SphericalContinentalDescent {
	private:

		std::shared_ptr<const SphericalContinentalField> Field;
		std::optional<ContinentalDescentPath> DiskPath;
		std::size_t ContinentIndex = 0;
		std::uint64_t Hash = 0;

		SphericalContinentalDescent() = default;

		std::optional<SphericalContinentalDescentSample> EvaluateCoordinates(
			std::array<double, 2> Coordinates,
			std::array<double, 2> CoordinateDerivative,
			bool IsOutlet
		) const;

	public:

		static std::optional<SphericalContinentalDescent> Create(
			std::shared_ptr<const SphericalContinentalField> PreparedField,
			UnitDirection Start,
			const ContinentalDescentSettings &Settings,
			SphericalContinentalDescentError &Error,
			std::stop_token Stop={}
		);

		std::optional<SphericalContinentalDescentSample> Evaluate(std::size_t Segment, double Fraction) const;
		std::optional<SphericalContinentalDescentSample> EvaluateOutlet() const;
		const ContinentalDescentStatistics &GetDiskStatistics() const;
		std::size_t GetSegmentCount() const;
		std::uint64_t GetDefinitionHash() const;
	};
}
