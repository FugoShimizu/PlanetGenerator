#include "Misc/AutomationTest.h"
#include "Core/PlanetBiome.h"
#include "HAL/PlatformTime.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#if WITH_DEV_AUTOMATION_TESTS
namespace
{
using namespace PlanetCore;
ClimateCycle Cycle(double Mean,double Amplitude,double P,double E)
{
    ClimateCycle C;
    for (unsigned I=0;I<12;++I) C.Periods[I]={Mean+Amplitude*std::cos((I+.5)*6.283185307179586/12),P,E};
    return C;
}
std::array<double,11> Values(const SeasonalAppearance& A)
{
    return {A.Rock,A.Sand,A.Soil,A.GreenGrass,A.DryGrass,A.LeafLitter,A.Snow,A.Ice,A.LeafGreenness,A.AutumnTint,A.Wetness};
}
double Cover(const SeasonalAppearance& A) { const auto V=Values(A); return std::accumulate(V.begin(),V.begin()+8,0.0); }
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBiomeContracts,"PlanetGenerator.Biome.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPlanetBiomeContracts::RunTest(const FString& Parameters)
{
    (void)Parameters; using namespace PlanetCore; const double Begin=FPlatformTime::Seconds();
    std::array<ClimateCycle,12> Cases{
        Cycle(26,0,200,100),Cycle(26,0,200,80),Cycle(26,0,110,100),
        Cycle(12,14,90,60),Cycle(12,8,160,60),Cycle(16,9,150,80),
        Cycle(12,14,45,60),Cycle(10,15,30,90),Cycle(0,20,50,60),
        Cycle(-8,15,50,60),Cycle(28,8,5,100),Cycle(2,18,5,100)};
    for (unsigned I=0;I<6;++I) { Cases[1].Periods[I].PrecipitationMm=20; Cases[2].Periods[I].PrecipitationMm=10; }
    for (auto& P:Cases[5].Periods) if (P.TemperatureC>16) P.PrecipitationMm=5;
    std::uint64_t Samples=0; double MaxBudget=0,MaxBoundaryJump=0,MaxShiftError=0;
    for (unsigned K=0;K<Cases.size();++K)
    {
        const auto B=BiomeProfile::Create(Cases[K]);
        if (!TestTrue(TEXT("Representative climate prepares"),B.has_value())) return false;
        const auto Habitat=*B->Habitat({});
        if (Habitat.Dominant!=static_cast<LandBiome>(K))
        {
            AddError(FString::Printf(TEXT("Biome fixture %u classified as %u"),K,static_cast<unsigned>(*Habitat.Dominant))); return false;
        }
        TestTrue(TEXT("Land mixture normalized"),std::abs(std::accumulate(Habitat.Weights.begin(),Habitat.Weights.end(),0.0)-1)<1e-12);
        const auto& Water=B->WaterCycle();
        for (unsigned I=0;I<12;++I)
        {
            const auto& Prev=Water[(I+11)%12]; const auto& W=Water[I];
            const double Residual=Prev.SoilWaterMm+Prev.SnowWaterMm+Cases[K].Periods[I].PrecipitationMm-
                W.SoilWaterMm-W.SnowWaterMm-W.ActualEvapotranspirationMm-W.RunoffMm-W.FrozenExportMm;
            MaxBudget=std::max(MaxBudget,std::abs(Residual));
            TestTrue(TEXT("Independent per-bin water balance"),std::abs(Residual)<1e-8);
        }
        auto Shifted=Cases[K]; for (unsigned I=0;I<12;++I) Shifted.Periods[I]=Cases[K].Periods[(I+6)%12];
        const auto Other=BiomeProfile::Create(Shifted);
        if (!TestTrue(TEXT("Shifted year prepares"),Other.has_value())) return false;
        const auto H2=*Other->Habitat({});
        for (unsigned I=0;I<12;++I) TestTrue(TEXT("Hemisphere season shift preserves annual biome"),std::abs(Habitat.Weights[I]-H2.Weights[I])<1e-12);
        for (unsigned I=0;I<481;++I)
        {
            const double Phase=I/480.0;
            const BiomeSite Site{SurfaceDomain::Land,.2,.1,.15};
            const auto A=*B->Appearance(Phase,Site),A2=*Other->Appearance(Phase+.5,Site);
            const auto V=Values(A),V2=Values(A2);
            for (unsigned J=0;J<V.size();++J)
            {
                if (!TestTrue(TEXT("Every appearance channel finite and bounded"),std::isfinite(V[J]) && V[J]>=-1e-12 && V[J]<=1+1e-12)) return false;
                MaxShiftError=std::max(MaxShiftError,std::abs(V[J]-V2[J]));
            }
            TestTrue(TEXT("All ground fractions sum to one"),std::abs(Cover(A)-1)<1e-12);
            ++Samples;
        }
        for (unsigned I=0;I<=12;++I)
        {
            const double Phase=I/12.0,Step=1e-6;
            const auto A=Values(*B->Appearance(Phase-Step,{})),M=Values(*B->Appearance(Phase,{})),Z=Values(*B->Appearance(Phase+Step,{}));
            for (unsigned J=0;J<A.size();++J)
            {
                MaxBoundaryJump=std::max(MaxBoundaryJump,std::abs(A[J]-Z[J]));
                TestTrue(TEXT("Period and year boundaries are continuous with matching derivatives"),
                    std::abs(A[J]-Z[J])<1e-7 && std::abs((Z[J]-M[J])-(M[J]-A[J]))/Step<1e-4);
            }
        }
        for (unsigned Domain=1;Domain<=4;++Domain)
        {
            const BiomeSite Site{static_cast<SurfaceDomain>(Domain),.2,.3,.1};
            const auto H=*B->Habitat(Site);
            TestTrue(TEXT("Water is outside land mixture and terrestrial plant placement"),!H.Dominant && H.TreeSuitability==0 && H.GrassSuitability==0 &&
                std::accumulate(H.Weights.begin(),H.Weights.end(),0.0)==0);
            for (const double Phase:{-.01,0.0,.25,.5,.75,1.0})
            {
                const auto A=*B->Appearance(Phase,Site);
                TestTrue(TEXT("Waterbed has no terrestrial seasonal cover"),A.Snow==0 && A.Ice==0 && A.GreenGrass==0 && A.DryGrass==0 &&
                    A.LeafLitter==0 && A.LeafGreenness==0 && A.AutumnTint==0 && A.Wetness==1 && std::abs(Cover(A)-1)<1e-12);
            }
        }
    }
    TestTrue(TEXT("Continuous seasonal shift equivalence"),MaxShiftError<1e-9);
    const auto Desert=*BiomeProfile::Create(Cycle(-30,0,0,0));
    TestTrue(TEXT("Cold temperature alone does not invent snowfall"),Desert.Appearance(.5,{})->Snow==0 && Desert.Appearance(.5,{})->Ice==0);
    const auto Polar=*BiomeProfile::Create(Cycle(-30,0,50,0));
    TestTrue(TEXT("Perennial snowfall has a bounded cover and explicit frozen export"),Polar.Appearance(.5,{})->Ice==1 &&
        Polar.WaterCycle()[0].FrozenExportMm>0 && Polar.Habitat({})->TreeSuitability==0);
    auto Wet=Cycle(15,10,150,50),Dry=Wet; for (auto& P:Dry.Periods) P.PrecipitationMm=10;
    const auto WetB=*BiomeProfile::Create(Wet),DryB=*BiomeProfile::Create(Dry);
    double WetRunoff=0,DryRunoff=0; for (unsigned I=0;I<12;++I) { WetRunoff+=WetB.WaterCycle()[I].RunoffMm; DryRunoff+=DryB.WaterCycle()[I].RunoffMm; }
    TestTrue(TEXT("Additional precipitation increases available water and runoff"),WetRunoff>DryRunoff && WetB.Appearance(.5,{})->Wetness>DryB.Appearance(.5,{})->Wetness);
    auto WinterDry=Cases[5]; for (unsigned I=0;I<12;++I) WinterDry.Periods[I].PrecipitationMm=Cases[5].Periods[(I+6)%12].PrecipitationMm;
    const auto SummerDry=*BiomeProfile::Create(Cases[5]),WinterDryB=*BiomeProfile::Create(WinterDry);
    TestTrue(TEXT("Equal annual rainfall with a different season is not Mediterranean by accident"),
        SummerDry.Statistics().SummerDryness>.8 && WinterDryB.Statistics().SummerDryness<.01);
    // Scan transition neighborhoods and many physically admissible cycles;
    // test outputs and conservation, rather than reproducing classification rules.
    for (unsigned I=0;I<1024;++I)
    {
        auto C=Cycle(-35.0+static_cast<double>(I%97),static_cast<double>(I%23),static_cast<double>(I%201),static_cast<double>(I%139));
        for (unsigned J=0;J<12;++J) C.Periods[J].PrecipitationMm*=.1+(I*(J+1)%17)/8.0;
        const auto B=BiomeProfile::Create(C);
        if (!TestTrue(TEXT("Bounded water solve prepares for climate sweep"),B.has_value()))
        {
            AddError(FString::Printf(TEXT("Rejected sweep %u, first bin T %.17g P %.17g PET %.17g valid=%d"),
                I,C.Periods[0].TemperatureC,C.Periods[0].PrecipitationMm,C.Periods[0].PotentialEvapotranspirationMm,C.IsValid())); return false;
        }
        const auto S=B->Statistics(); MaxBudget=std::max(MaxBudget,std::abs(S.WaterBudgetResidualMm));
        auto Epsilon=C; for (auto& P:Epsilon.Periods) { P.TemperatureC+=1e-7; P.PrecipitationMm+=1e-7; }
        const auto Neighbor=BiomeProfile::Create(Epsilon);
        if (!TestTrue(TEXT("Perturbed cycle prepares"),Neighbor.has_value())) return false;
        const auto A=*B->Habitat({}),Z=*Neighbor->Habitat({});
        double Delta=0; for (unsigned J=0;J<12;++J) { Delta+=std::abs(A.Weights[J]-Z.Weights[J]); TestTrue(TEXT("Mixture has nonnegative weights"),A.Weights[J]>=-1e-14); }
        TestTrue(TEXT("Classification blends continuously across input changes"),Delta<1e-4);
    }
    // A bucket without drainage has nonunique equilibria at P=PET and
    // snaps between empty and full for an arbitrarily small input change.
    // The conservative leaks must remove that climate-space discontinuity.
    double MaxNeutralInputChange=0;
    for (const auto Neutral : {Cycle(20,0,100,100),Cycle(-30,0,0,0),Cycle(0,0,0,0)})
    {
        auto Wetter=Neutral; for (auto& P:Wetter.Periods) P.PrecipitationMm+=1e-6;
        const auto A=BiomeProfile::Create(Neutral),B=BiomeProfile::Create(Wetter);
        if (!TestTrue(TEXT("Near-neutral water cycles prepare"),A.has_value() && B.has_value())) return false;
        for (unsigned I=0;I<25;++I)
        {
            const auto V=Values(*A->Appearance(I/24.0,{})),W=Values(*B->Appearance(I/24.0,{}));
            for (unsigned J=0;J<V.size();++J) MaxNeutralInputChange=std::max(MaxNeutralInputChange,std::abs(V[J]-W[J]));
        }
    }
    TestTrue(TEXT("Tiny rainfall change cannot switch neutral soil or snow to a fully saturated appearance"),MaxNeutralInputChange<1e-4);
    AddInfo(FString::Printf(TEXT("Biome neutral-input continuity: maximum appearance change %.9g for +1e-6 mm rain per bin"),MaxNeutralInputChange));
    for (const auto Extreme : {Cycle(-150,0,100000,0),Cycle(100,0,100000,100000),Cycle(0,0,0,0)})
    {
        const auto B=BiomeProfile::Create(Extreme);
        if (!TestTrue(TEXT("Accepted climate limits keep a bounded periodic water budget"),B.has_value())) return false;
        TestTrue(TEXT("Extreme climate budget closes"),std::abs(B->Statistics().WaterBudgetResidualMm)<1e-7);
    }
    auto Bad=Cases[0]; Bad.Periods[3].PrecipitationMm=-1;
    TestFalse(TEXT("Reject negative rain"),BiomeProfile::Create(Bad).has_value());
    Bad=Cases[0]; Bad.Periods[5].TemperatureC=std::numeric_limits<double>::quiet_NaN();
    TestFalse(TEXT("Reject nonfinite climate"),BiomeProfile::Create(Bad).has_value());
    TestFalse(TEXT("Reject infinite time"),WetB.Appearance(std::numeric_limits<double>::infinity(),{}).has_value());
    TestFalse(TEXT("Reject invalid substrate mixture"),WetB.Habitat({SurfaceDomain::Land,.8,.3,0}).has_value());
    TestFalse(TEXT("Reject invalid domain"),WetB.Habitat({static_cast<SurfaceDomain>(255),0,0,0}).has_value());
    const auto Before=*WetB.Habitat({}); volatile double Checksum=0; const double EvaluationBegin=FPlatformTime::Seconds();
    for (unsigned I=0;I<100000;++I) Checksum=Checksum+WetB.Appearance(I*.00037,{})->GreenGrass;
    const double EvaluationMs=(FPlatformTime::Seconds()-EvaluationBegin)*1000;
    const auto After=*WetB.Habitat({});
    TestTrue(TEXT("Date visits do not change habitat or plant suitability"),Before.Weights==After.Weights && Before.TreeSuitability==After.TreeSuitability && Before.GrassSuitability==After.GrassSuitability);
    AddInfo(FString::Printf(TEXT("Biome contracts: 12 palettes, %llu seasonal samples, 1024 climate perturbations, water balance %.9g mm, boundary jump %.9g, hemisphere shift %.9g, profile %llu bytes; 100000 appearance queries %.3f ms checksum %.9g; %.3f seconds"),
        static_cast<unsigned long long>(Samples),MaxBudget,MaxBoundaryJump,MaxShiftError,static_cast<unsigned long long>(sizeof(BiomeProfile)),EvaluationMs,static_cast<double>(Checksum),FPlatformTime::Seconds()-Begin));
    return true;
}
#endif
