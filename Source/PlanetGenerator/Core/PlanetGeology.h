#pragma once

#include "PlanetCore.h"
#include <span>
#include <stop_token>

namespace PlanetCore {
	inline constexpr std::uint32_t GeologyGeneratorVersion = 3;
	struct GeologySettings {
		std::uint64_t Seed = 42;
		std::uint32_t Version = GeologyGeneratorVersion;
		double RadiusMeters = 6371000;
		double SeaLevelMeters = 0;
		std::uint32_t PlateCount = 12;
		std::uint32_t HistoryIntervals = 8;
		double HistoryMillionYears = 120;
		bool IsValid() const;
		std::uint64_t IdentityHash() const;
	};

	enum class GeologicFeatureKind : std::uint8_t {
		ContinentalCrust,
		Collision,
		Spreading,
		Trench,
		OuterRise,
		Forearc,
		VolcanicArc,
		Transform
	};

	// An oriented, compact C2 lobe in a spherical cap. Parameters are generated
	// once from the finite history, independently of render tiles and visit order.
	struct GeologicFeature {
		GeologicFeatureKind Kind = GeologicFeatureKind::ContinentalCrust;
		Vec3 Center, Along, Across;
		double AlongSine = 0, AcrossSine = 0, SupportCosine = 1;
		double AmplitudeMeters = 0, AgeMillionYears = 0;
		std::uint32_t PlateA = 0, PlateB = 0;
	};

	struct PlateHistoryFrame {
		double AgeMillionYears = 0;
		std::vector<Vec3> Sites;
	};

	struct GeologySample {
		TerrainSample Terrain;
		double ContinentalCoordinate = 0;
		double OceanCrustAgeMillionYears = 0;
		double TectonicDisplacementMeters = 0;
		double RockResistance = 0;
	};

	struct GeologyStatistics {
		std::size_t FeatureCount = 0;
		std::uint32_t CalibrationProbeCount = 0;
		std::uint32_t AreaProbeCount = 0;
		double ContinentalThreshold = 0;
		double ContinentalScale = 1;
		double DatumShiftMeters = 0;
		double TargetOceanFraction = 0;
		// Equal-area point estimate only, not a final ocean-area/connectivity test.
		double InitialOceanFractionEstimate = 0;
	};

	// Geological base for the production pipeline. Global ocean reconciliation,
	// drainage, erosion, climate and local detail must still be applied before the
	// complete V1 terrain can be published. No persistent terrain cache is used.
	class GeologyField {
	  public:
		static std::optional<GeologyField> Create(const GeologySettings &Settings, std::stop_token Stop = {});
		GeologySample Evaluate(UnitDirection Direction) const;
		double EvaluateElevation(UnitDirection Direction) const;
		GeologySample EvaluateReference(UnitDirection Direction) const;
		std::size_t FeatureCandidateCount(UnitDirection Direction) const;
		Vec3 SurfacePosition(UnitDirection Direction) const;
		UnitDirection SurfaceNormal(UnitDirection Direction) const;
		double AbsoluteReliefBoundMeters() const;
		const GeologySettings &Settings() const {
			return Parameters;
		}
		const GeologyStatistics &Statistics() const {
			return Stats;
		}
		std::span<const GeologicFeature> Features() const {
			return Lobes;
		}
		std::span<const PlateHistoryFrame> History() const {
			return Frames;
		}

	  private:
		struct FeatureIndexNode {
			Vec3 Min, Max;
			std::uint32_t Begin = 0, Count = 0, Left = 0, Right = 0;
		};
		std::vector<std::uint32_t> FeatureOrder;
		std::vector<FeatureIndexNode> FeatureIndex;
		bool BuildFeatureIndex(std::stop_token Stop);
		std::vector<std::uint32_t> QueryFeatures(Vec3 Direction) const;
		template <bool Derivatives> GeologySample EvaluateImpl(UnitDirection Direction, bool Indexed) const;
		struct Wave {
			Vec3 Axis;
			double Frequency = 0, Phase = 0;
		};
		GeologySettings Parameters;
		GeologyStatistics Stats;
		std::vector<GeologicFeature> Crust;
		std::vector<GeologicFeature> Lobes;
		std::vector<PlateHistoryFrame> Frames;
		std::array<Wave, 12> Waves;
		TerrainSample ContinentalScore(Vec3 Direction) const;
		template <bool Derivatives> TerrainSample ContinentalScoreImpl(Vec3 Direction) const;
	};
} // namespace PlanetCore
