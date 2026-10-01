#pragma once

#include "PlanetClimate.h"
#include "PlanetDrainage.h"
#include <functional>

namespace PlanetCore {
	inline constexpr std::uint32_t RunoffVersion = 1;

	/** 海への河川流出と，未モデル化の氷河輸送を区別した年水収支 */
	struct RunoffStatistics {
		std::uint32_t LandNodes = 0, ReceivingOceanNodes = 0;
		double LandAreaSquareMeters = 0;
		double AnnualPrecipitationCubicMeters = 0, AnnualEvaporationCubicMeters = 0;
		double AnnualLocalRunoffCubicMeters = 0, AnnualOceanRunoffCubicMeters = 0, AnnualFrozenExportCubicMeters = 0;
		double RoutingRelativeResidual = 0, WaterBudgetRelativeResidual = 0, AreaRelativeResidual = 0;
		double MaximumMeanDischargeCubicMetersPerSecond = 0;
	};

	/** 固定の排水グラフで運ぶ１２期の流量，時刻で地形や流路を変更しない */
	class PlanetRunoff {
	private:
		std::vector<std::array<double, ClimatePeriodCount>> Discharge;
		std::vector<double> LocalAreas;
		RunoffStatistics Stats;
		std::uint64_t SourceHash = 0;
		PlanetRunoff() = default;

	public:
		// グレゴリオ暦の平均年を流量換算の参照時間とし，１２期を等時間で扱う
		static constexpr double ReferenceYearSeconds = 365.2425 * 86400;
		using ProfileProvider = std::function<std::optional<BiomeProfile>(UnitDirection)>;

		static std::optional<PlanetRunoff> Create(
			const DrainageNetwork &Network, const PlanetClimate &Climate, const std::stop_token Stop = {});

		static std::optional<PlanetRunoff> FromProfiles(
			const DrainageNetwork &Network, const ProfileProvider &Provider, const std::stop_token Stop = {});

		std::optional<double> Sample(const std::uint32_t Node, const double SolarYearPhase) const;
		std::span<const std::array<double, ClimatePeriodCount>> GetPeriods() const;
		std::span<const double> GetNodeAreas() const;
		std::uint64_t GetSourceDefinitionHash() const;
		const RunoffStatistics &GetStatistics() const;
	};
}
