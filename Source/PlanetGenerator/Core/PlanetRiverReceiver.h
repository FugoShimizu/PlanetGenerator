#pragma once

#include "PlanetCore.h"
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	class RiverTerrainField;

	enum class RiverReceiverError : std::uint8_t { None, InvalidInput, InvalidDrainage, NoSupportedSite, Cancelled };

	struct RiverReceiverSite {
		UnitDirection Direction;
		std::uint32_t Reach = 0;
		double Fraction = 0;
		double WaterMeters = 0;
		double GroundMeters = 0;
		double BedMeters = 0;
		double WaterSupport = 0;
	};

	struct RiverReceiverStatistics {
		std::uint32_t Reaches = 0;
		std::uint32_t Sampled = 0;
		std::uint32_t Supported = 0;
		std::uint32_t Rejected = 0;
	};

	struct RiverReceiverNearest {
		RiverReceiverSite Site;
		double DistanceMeters = 0;
	};

	/** 同じ最終地形と大洋へ下る既存河川から，補助流路の受入境界候補を保持する */
	class RiverReceiverIndex {
	private:

		std::vector<RiverReceiverSite> Sites;
		RiverReceiverStatistics Stats;
		double RadiusMeters = 0;

		RiverReceiverIndex() = default;

	public:

		static std::optional<RiverReceiverIndex> Create(
			const RiverTerrainField &Field,
			std::uint32_t SegmentsPerReach,
			RiverReceiverError &Error,
			std::stop_token Stop={}
		);

		std::optional<RiverReceiverNearest> FindNearest(
			UnitDirection Origin,
			double MaximumWaterMeters,
			double MaximumDistanceMeters=std::numeric_limits<double>::infinity(),
			std::stop_token Stop={}
		) const;

		std::optional<std::vector<RiverReceiverNearest>> FindDistanceWaterPareto(
			UnitDirection Origin,
			double MaximumWaterMeters,
			double MaximumDistanceMeters=std::numeric_limits<double>::infinity(),
			std::stop_token Stop={}
		) const;

		std::span<const RiverReceiverSite> GetSites() const;
		const RiverReceiverStatistics &GetStatistics() const;
	};
}
