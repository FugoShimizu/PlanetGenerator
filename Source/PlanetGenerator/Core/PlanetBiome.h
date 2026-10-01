#pragma once
#include <array>
#include <cstdint>
#include <optional>

namespace PlanetCore
{
inline constexpr std::uint32_t BiomeVersion=1;
inline constexpr unsigned ClimatePeriodCount=12;
// Equal-duration bins of the common solar-year phase, NOT Gregorian months.
// Values are bin means (temperature) and bin totals (water, in mm).
struct ClimatePeriod { double TemperatureC=0,PrecipitationMm=0,PotentialEvapotranspirationMm=0; };
struct ClimateCycle
{
    std::array<ClimatePeriod,ClimatePeriodCount> Periods{};
    bool IsValid() const;
};
enum class LandBiome : std::uint8_t
{
    TropicalRainforest,TropicalSeasonalForest,Savanna,TemperateDeciduousForest,
    TemperateRainforest,Mediterranean,TemperateGrassland,Steppe,Taiga,Tundra,HotDesert,ColdDesert,Count
};
inline constexpr unsigned LandBiomeCount=static_cast<unsigned>(LandBiome::Count);
enum class SurfaceDomain : std::uint8_t { Land,RiverBed,LakeBed,ShallowSeaBed,DeepSeaBed };
struct BiomeSite
{
    SurfaceDomain Domain=SurfaceDomain::Land;
    double RockFraction=0,SandFraction=0,Slope=0;
    bool IsValid() const;
};
struct BiomeWaterPeriod
{
    double SoilWaterMm=0,SnowWaterMm=0,ActualEvapotranspirationMm=0,RunoffMm=0,FrozenExportMm=0;
};
struct BiomeStatistics
{
    double MeanTemperatureC=0,MinimumTemperatureC=0,MaximumTemperatureC=0;
    double AnnualPrecipitationMm=0,AnnualPotentialEvapotranspirationMm=0,MoistureRatio=0;
    double MinimumPeriodMoistureRatio=0,SummerDryness=0;
    double WaterBudgetResidualMm=0,PeriodicStorageResidualMm=0;
};
struct BiomeHabitat
{
    std::array<double,LandBiomeCount> Weights{};
    // Suitability, NOT an object density or a placement algorithm. Time invariant.
    double TreeSuitability=0,GrassSuitability=0;
    std::optional<LandBiome> Dominant;
};
struct SeasonalAppearance
{
    // Ground fractions sum to one, including underwater substrates.
    double Rock=0,Sand=0,Soil=0,GreenGrass=0,DryGrass=0,LeafLitter=0,Snow=0,Ice=0;
    double LeafGreenness=0,AutumnTint=0,Wetness=0;
};
// A compact, immutable climate response. No LOD, world axes, radius, current
// date, render state or height evaluator can alter its classification.
// Climate transport and discharge coupling are separate generation stages.
class BiomeProfile
{
public:
    static constexpr double SoilCapacityMm=150,SnowAppearanceCapacityMm=200;
    // Per climate bin. Make the reservoirs contractive and record every loss.
    static constexpr double SoilDrainageFraction=.02,SnowTurnoverFraction=.01;
    static std::optional<BiomeProfile> Create(const ClimateCycle& Climate);
    const BiomeStatistics& Statistics() const { return Stats; }
    const std::array<BiomeWaterPeriod,ClimatePeriodCount>& WaterCycle() const { return Water; }
    std::optional<BiomeHabitat> Habitat(const BiomeSite& Site) const;
    std::optional<SeasonalAppearance> Appearance(double SolarYearPhase,const BiomeSite& Site) const;
private:
    ClimateCycle Climate;
    BiomeStatistics Stats;
    std::array<BiomeWaterPeriod,ClimatePeriodCount> Water{};
    std::array<double,LandBiomeCount> Weights{};
};
}
