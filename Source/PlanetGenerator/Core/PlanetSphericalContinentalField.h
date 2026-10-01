#pragma once

#include "PlanetConnectedOceanDomain.h"
#include "PlanetContinentalDiskWarp.h"
#include "PlanetContinentalPotential.h"
#include "PlanetCore.h"
#include "PlanetSphericalWarp.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	class SphericalContinentalDescent;

	inline constexpr std::uint32_t SphericalContinentalFieldVersion = 6;

	struct SphericalContinentalFieldSettings {
		// 連結海域と大陸毎の正の隆起
		ConnectedOceanDomainSettings Domain;
		std::vector<ContinentalPotentialSettings> Uplifts;
		std::vector<std::vector<ContinentalPotentialSettings>> UpliftLayers;
		std::vector<ContinentalDiskWarpSettings> DiskShapes;
		double BlendTemperatureMeters = 0;
		/** 正の高さを三乗する単調変換の物理尺度 */
		double PotentialReferenceMeters = 1000;
		double OceanDepthMeters = 4000;
	};

	enum class SphericalContinentalFieldError : std::uint8_t {
		None,
		Cancelled,
		InvalidSettings,
		OceanDomainRejected,
		UpliftRejected,
		ShapeRejected,
		UnresolvedAtlas,
		ReliefOutsideSphere
	};

	struct SphericalContinentalFieldStatistics {
		// 準備規模と解析的な面積・高さの上界
		std::size_t ContinentalCount = 0;
		std::size_t SourceCount = 0;
		std::size_t ShapeTwistCount = 0;
		double OceanFraction = 0;
		double AbsoluteReliefBoundMeters = 0;
		/** 単位円板の二乗半径に許可する丸めの上界，地形誤差の許容値ではない */
		double MaximumCoordinateRoundoffBound = 0;
	};

	/**
	 * 接続された海域と，内部極小を持たない陸の候補を，同じ解析的な球面の場へ合成する
	 * 単調な三乗変換は下降方向を保持し，海岸で高さ・勾配・２階微分を零へつなぐ
	 * 数学的な構成の比較用であり，完成した侵食・海底・河川・景観ではない
	 * 山頂・鞍点の離脱と全河川の有限時間の大洋到達は別に検証する
	 */
	class SphericalContinentalField {
	private:

		friend class SphericalContinentalDescent;

		struct ContinentalCoordinates {
			std::size_t Index = 0;
			double X = 0;
			double Y = 0;
			double CenterDot = 0;
			double Denominator = 0;
		};

		struct PreparedContinent {
			// 大陸の立体射影と有限源
			Vec3 Center;
			Vec3 East;
			Vec3 North;
			double CoastCosine = 0;
			double HalfAngleTangent = 0;
			double CoordinateRoundoffBound = 0;
			ContinentalPotential Potential;
			ContinentalDiskWarp Shape;
		};

		// 照会順に依らない領域と共有隆起
		std::optional<ConnectedOceanDomain> Domain;
		std::optional<PlanetSphericalWarp> Warp;
		std::vector<PreparedContinent> Continents;
		// 不変な物理尺度と再利用識別子
		SphericalContinentalFieldStatistics Stats;
		double RadiusMeters = 0;
		double ReferenceMeters = 0;
		double OceanDepthMeters = 0;
		std::uint64_t Hash = 0;

		SphericalContinentalField() = default;

		std::optional<ContinentalCoordinates> FindContinentalCoordinates(Vec3 Source) const;

		template<bool HasDerivatives> std::optional<TerrainSample> EvaluateImpl(UnitDirection Direction) const;

	public:

		static std::optional<SphericalContinentalField> Create(
			const SphericalContinentalFieldSettings &Settings,
			SphericalContinentalFieldError &Error,
			std::stop_token Stop={}
		);

		static std::optional<SphericalContinentalField> CreateSeeded(
			std::uint64_t Seed,
			double RadiusMeters,
			SphericalContinentalFieldError &Error,
			std::stop_token Stop={}
		);

		std::optional<TerrainSample> Evaluate(UnitDirection Direction) const;
		std::optional<double> EvaluateElevation(UnitDirection Direction) const;
		UnitDirection GetOceanAnchor() const;
		const SphericalContinentalFieldStatistics &GetStatistics() const;
		double GetRadiusMeters() const;
		std::uint64_t GetDefinitionHash() const;
	};
}
