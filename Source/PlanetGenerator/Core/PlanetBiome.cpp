#include "PlanetBiome.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace PlanetCore
{
namespace
{
double Smooth(double A,double B,double X)
{
    const double T=std::clamp((X-A)/(B-A),0.0,1.0); return std::clamp(T*T*T*(10+T*(-15+6*T)),0.0,1.0);
}
double Ratio(double P,double E) { return P/std::max(1.0,E); }
struct Storage { double Soil=0,Snow=0; };
BiomeWaterPeriod Step(const ClimatePeriod& C,Storage& S)
{
    BiomeWaterPeriod W;
    const double Snowfall=C.PrecipitationMm*(1-Smooth(-1,2,C.TemperatureC));
    const double AvailableSnow=S.Snow+Snowfall;
    const double Melt=std::min(AvailableSnow,std::max(0.0,C.TemperatureC)*3*(365.2422/12));
    S.Snow=std::min(BiomeProfile::SnowAppearanceCapacityMm,AvailableSnow-Melt);
    // Only the bounded superficial snow reservoir is represented. Record
    // excess explicitly, rather than deleting water or simulating glacier H.
    W.FrozenExportMm=AvailableSnow-Melt-S.Snow;
    const double SnowTurnover=S.Snow*BiomeProfile::SnowTurnoverFraction;
    S.Snow-=SnowTurnover; W.FrozenExportMm+=SnowTurnover;
    const double AvailableWater=S.Soil+C.PrecipitationMm-Snowfall+Melt;
    W.ActualEvapotranspirationMm=std::min(AvailableWater,C.PotentialEvapotranspirationMm);
    S.Soil=std::min(BiomeProfile::SoilCapacityMm,AvailableWater-W.ActualEvapotranspirationMm);
    W.RunoffMm=AvailableWater-W.ActualEvapotranspirationMm-S.Soil;
    const double SoilDrainage=S.Soil*BiomeProfile::SoilDrainageFraction;
    S.Soil-=SoilDrainage; W.RunoffMm+=SoilDrainage;
    W.SoilWaterMm=S.Soil; W.SnowWaterMm=S.Snow; return W;
}
Storage Year(const ClimateCycle& C,Storage S)
{
    for (const auto& P:C.Periods) Step(P,S); return S;
}
// Unique fixed point of a monotone, contractive clipped-reservoir map.
// Fixed work; no spin-up count dependent on a tiny annual water imbalance.
template<class F> double FixedStorage(double Capacity,F&& Map)
{
    if (Map(0)==0) return 0;
    double Low=0,High=Capacity;
    for (unsigned I=0;I<56;++I)
    {
        const double Mid=(Low+High)*0.5;
        if (Map(Mid)>Mid) Low=Mid; else High=Mid;
    }
    return (Low+High)*0.5;
}
constexpr unsigned Index(LandBiome B) { return static_cast<unsigned>(B); }
// Explicit production palette response, not published ecological thresholds.
constexpr std::array<double,LandBiomeCount> Trees{1,.85,.12,.9,1,.35,0,0,.85,0,0,0};
constexpr std::array<double,LandBiomeCount> Grass{.35,.5,.95,.45,.4,.5,1,.6,.3,.45,.05,.03};
constexpr std::array<double,LandBiomeCount> Litter{.85,.75,.08,.9,.9,.25,.02,0,.8,.05,0,0};
double Dot(const std::array<double,LandBiomeCount>& A,const std::array<double,LandBiomeCount>& B)
{
    double V=0; for (unsigned I=0;I<LandBiomeCount;++I) V+=A[I]*B[I]; return V;
}
}
bool ClimateCycle::IsValid() const
{
    for (const auto& P:Periods)
        if (!std::isfinite(P.TemperatureC) || P.TemperatureC < -150 || P.TemperatureC > 100 ||
            !std::isfinite(P.PrecipitationMm) || P.PrecipitationMm<0 || P.PrecipitationMm>100000 ||
            !std::isfinite(P.PotentialEvapotranspirationMm) || P.PotentialEvapotranspirationMm<0 ||
            P.PotentialEvapotranspirationMm>100000) return false;
    return true;
}
bool BiomeSite::IsValid() const
{
    return static_cast<unsigned>(Domain)<=static_cast<unsigned>(SurfaceDomain::DeepSeaBed) &&
        std::isfinite(RockFraction) && std::isfinite(SandFraction) && RockFraction>=0 && SandFraction>=0 &&
        RockFraction+SandFraction<=1 && std::isfinite(Slope) && Slope>=0;
}
std::optional<BiomeProfile> BiomeProfile::Create(const ClimateCycle& C)
{
    if (!C.IsValid()) return {};
    BiomeProfile B; B.Climate=C; auto& S=B.Stats;
    S.MinimumTemperatureC=std::numeric_limits<double>::infinity(); S.MaximumTemperatureC=-S.MinimumTemperatureC;
    S.MinimumPeriodMoistureRatio=std::numeric_limits<double>::infinity();
    for (const auto& P:C.Periods)
    {
        S.MeanTemperatureC+=P.TemperatureC/12;
        S.MinimumTemperatureC=std::min(S.MinimumTemperatureC,P.TemperatureC);
        S.MaximumTemperatureC=std::max(S.MaximumTemperatureC,P.TemperatureC);
        S.AnnualPrecipitationMm+=P.PrecipitationMm;
        S.AnnualPotentialEvapotranspirationMm+=P.PotentialEvapotranspirationMm;
        S.MinimumPeriodMoistureRatio=std::min(S.MinimumPeriodMoistureRatio,Ratio(P.PrecipitationMm,P.PotentialEvapotranspirationMm));
    }
    S.MoistureRatio=S.AnnualPrecipitationMm/std::max(12.0,S.AnnualPotentialEvapotranspirationMm);
    // Warm/cool selection follows local temperature, not hemisphere labels.
    // Smooth selectors remain well defined for flat equatorial cycles.
    double WarmP=0,WarmE=0,CoolP=0,CoolE=0;
    for (const auto& P:C.Periods)
    {
        const double W=Smooth(-2,2,P.TemperatureC-S.MeanTemperatureC);
        WarmP+=W*P.PrecipitationMm; WarmE+=W*P.PotentialEvapotranspirationMm;
        CoolP+=(1-W)*P.PrecipitationMm; CoolE+=(1-W)*P.PotentialEvapotranspirationMm;
    }
    S.SummerDryness=(1-Smooth(.3,.65,Ratio(WarmP,WarmE)))*Smooth(.8,1.2,Ratio(CoolP,CoolE))*
        Smooth(2,6,S.MaximumTemperatureC-S.MinimumTemperatureC);
    auto& W=B.Weights; double Remaining=1;
    const auto Allocate=[&](LandBiome Type,double Fraction) {
        const double Taken=Remaining*Fraction; W[Index(Type)]+=Taken; Remaining-=Taken;
    };
    const double Desert=1-Smooth(.16,.24,S.MoistureRatio),Hot=Smooth(16,20,S.MeanTemperatureC);
    W[Index(LandBiome::HotDesert)]=Desert*Hot; W[Index(LandBiome::ColdDesert)]=Desert*(1-Hot); Remaining-=Desert;
    Allocate(LandBiome::Steppe,1-Smooth(.42,.60,S.MoistureRatio));
    Allocate(LandBiome::Tundra,1-Smooth(8,12,S.MaximumTemperatureC));
    Allocate(LandBiome::Taiga,(1-Smooth(3,9,S.MeanTemperatureC))*(1-Smooth(-1,3,S.MinimumTemperatureC)));
    const double Tropical=Remaining*Smooth(16,20,S.MinimumTemperatureC); Remaining-=Tropical;
    const double Rain=Smooth(.7,1.05,S.MinimumPeriodMoistureRatio)*Smooth(1,1.4,S.MoistureRatio);
    const double Forest=Smooth(.75,1.1,S.MoistureRatio);
    W[Index(LandBiome::TropicalRainforest)]=Tropical*Rain;
    W[Index(LandBiome::TropicalSeasonalForest)]=Tropical*(1-Rain)*Forest;
    W[Index(LandBiome::Savanna)]=Tropical*(1-Rain)*(1-Forest);
    Allocate(LandBiome::Mediterranean,S.SummerDryness*Smooth(1,7,S.MinimumTemperatureC)*Smooth(14,20,S.MaximumTemperatureC));
    Allocate(LandBiome::TemperateGrassland,1-Smooth(.65,1,S.MoistureRatio));
    Allocate(LandBiome::TemperateRainforest,Smooth(1.4,1.9,S.MoistureRatio)*Smooth(.25,.6,S.MinimumPeriodMoistureRatio));
    W[Index(LandBiome::TemperateDeciduousForest)]=Remaining;
    Storage Initial;
    Initial.Snow=FixedStorage(SnowAppearanceCapacityMm,[&](double X) { return Year(C,{0,X}).Snow; });
    Initial.Soil=FixedStorage(SoilCapacityMm,[&](double X) { return Year(C,{X,Initial.Snow}).Soil; });
    auto State=Initial; double Export=0;
    for (unsigned I=0;I<ClimatePeriodCount;++I)
    {
        B.Water[I]=Step(C.Periods[I],State);
        Export+=B.Water[I].ActualEvapotranspirationMm+B.Water[I].RunoffMm+B.Water[I].FrozenExportMm;
    }
    S.PeriodicStorageResidualMm=std::max(std::abs(State.Soil-Initial.Soil),std::abs(State.Snow-Initial.Snow));
    S.WaterBudgetResidualMm=S.AnnualPrecipitationMm-Export;
    if (S.PeriodicStorageResidualMm>1e-8 || std::abs(S.WaterBudgetResidualMm)>1e-7) return {};
    return B;
}
std::optional<BiomeHabitat> BiomeProfile::Habitat(const BiomeSite& Site) const
{
    if (!Site.IsValid()) return {};
    BiomeHabitat H;
    if (Site.Domain!=SurfaceDomain::Land) return H;
    H.Weights=Weights;
    H.Dominant=static_cast<LandBiome>(std::max_element(Weights.begin(),Weights.end())-Weights.begin());
    const double Soil=1-Site.RockFraction-Site.SandFraction;
    H.TreeSuitability=Dot(Weights,Trees)*Soil*(1-Smooth(.5,1.1,Site.Slope))*Smooth(8,12,Stats.MaximumTemperatureC);
    H.GrassSuitability=Dot(Weights,Grass)*(Soil+.2*Site.SandFraction)*(1-Smooth(.65,1.5,Site.Slope))*
        Smooth(-2,8,Stats.MaximumTemperatureC)*Smooth(.05,.35,Stats.MoistureRatio);
    return H;
}
std::optional<SeasonalAppearance> BiomeProfile::Appearance(double Phase,const BiomeSite& Site) const
{
    if (!std::isfinite(Phase) || !Site.IsValid()) return {};
    SeasonalAppearance A; A.Rock=Site.RockFraction; A.Sand=Site.SandFraction; A.Soil=1-A.Rock-A.Sand;
    if (Site.Domain!=SurfaceDomain::Land) { A.Wetness=1; return A; }
    const auto H=*Habitat(Site);
    const double X=(Phase-std::floor(Phase))*12;
    const unsigned I=static_cast<unsigned>(X)%12,J=(I+1)%12;
    const double T=X-std::floor(X),Blend=Smooth(0,1,T);
    const auto Mix=[&](double U,double V) { return U+(V-U)*Blend; };
    const double Temperature=Mix(Climate.Periods[I].TemperatureC,Climate.Periods[J].TemperatureC);
    const double SoilWater=Mix(Water[I].SoilWaterMm,Water[J].SoilWaterMm);
    const double SnowWater=Mix(Water[I].SnowWaterMm,Water[J].SnowWaterMm);
    // The water values are end-of-bin climatology samples. Their phase is
    // defined here consistently; calendar/ephemeris conversion is external.
    A.Wetness=Smooth(0,SoilCapacityMm,SoilWater);
    const double Supply=Mix(Ratio(Water[I].ActualEvapotranspirationMm,Climate.Periods[I].PotentialEvapotranspirationMm),
        Ratio(Water[J].ActualEvapotranspirationMm,Climate.Periods[J].PotentialEvapotranspirationMm));
    const double Moisture=std::max(Smooth(.03,.45,SoilWater/SoilCapacityMm),Smooth(.15,.9,Supply));
    const double Growth=Smooth(0,12,Temperature)*Moisture;
    A.LeafGreenness=Growth;
    const auto CoolingAt=[&](unsigned K) { return Smooth(0,8,
        Climate.Periods[(K+11)%12].TemperatureC-Climate.Periods[(K+1)%12].TemperatureC); };
    const double Cooling=Mix(CoolingAt(I),CoolingAt(J));
    A.AutumnTint=Weights[Index(LandBiome::TemperateDeciduousForest)]*Cooling*Smooth(0,6,Temperature)*(1-Smooth(10,18,Temperature));
    const double SnowCover=Smooth(1,35,SnowWater)*(1-Smooth(.5,1.2,Site.Slope));
    A.Ice=SnowCover*(1-Smooth(-3,1,Stats.MaximumTemperatureC)); A.Snow=SnowCover-A.Ice;
    const double Exposed=1-SnowCover;
    const double Ground=Exposed*A.Soil;
    A.Rock*=Exposed; A.Sand*=Exposed;
    A.LeafLitter=Ground*Dot(Weights,Litter)*.65;
    const double GrassCover=std::min(Ground-A.LeafLitter,Exposed*H.GrassSuitability);
    A.GreenGrass=GrassCover*Growth; A.DryGrass=GrassCover-A.GreenGrass;
    A.Soil=Ground-A.LeafLitter-GrassCover;
    return A;
}
}
