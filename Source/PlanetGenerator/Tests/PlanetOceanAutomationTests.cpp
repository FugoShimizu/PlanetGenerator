#include "Core/PlanetOcean.h"
#include "Misc/AutomationTest.h"
#include "HAL/PlatformTime.h"
#include <cmath>
#include <bit>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetOceanContracts, "PlanetGenerator.Ocean.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPlanetOceanContracts::RunTest(const FString& Parameters)
{
    using namespace PlanetCore;
    (void)Parameters;
    const double StartTime=FPlatformTime::Seconds();
    TestFalse(TEXT("Null geological source rejected"),OceanConnectivityField::Create({}).has_value());
    std::uint32_t ClearanceChecks=0, DerivativeChecks=0, RepeatedChecks=0;
    double MaximumClearanceExcess=-1e30, MaximumGradientError=0;
    const std::array<std::pair<std::uint64_t,double>,5> Cases{{{42,1000000},{0,6371000},{90210,6371000},{42,1000},{42,1e9}}};
    for (const auto& Case : Cases)
    {
        GeologySettings Settings; Settings.Seed=Case.first; Settings.RadiusMeters=Case.second;
        auto Prepared=GeologyField::Create(Settings);
        if (!TestTrue(TEXT("Geological preparation succeeded"),Prepared.has_value())) return false;
        const auto Base=std::make_shared<const GeologyField>(std::move(*Prepared));
        std::stop_source Stop; Stop.request_stop();
        TestFalse(TEXT("Cancelled ocean preparation rejected"),OceanConnectivityField::Create(Base,Stop.get_token()).has_value());
        auto Ocean=OceanConnectivityField::Create(Base);
        if (!TestTrue(TEXT("Ocean preparation succeeded"),Ocean.has_value())) return false;
        TestEqual(TEXT("Coarse graph is connected after correction"),Ocean->Statistics().ComponentsAfter,1U);
        TestTrue(TEXT("Independent initial ocean fraction in requested interval"),Ocean->Statistics().IndependentOceanFractionEstimate>=0.60 && Ocean->Statistics().IndependentOceanFractionEstimate<=0.70);
        TestTrue(TEXT("Spatial index stays bounded"),Ocean->Statistics().IndexNodeCount<=2*Ocean->Statistics().CorridorCount);
        const double Limit=Settings.SeaLevelMeters-Ocean->Statistics().MinimumDepthMeters;
        for (const auto& Channel : Ocean->Corridors())
        {
            const double Inner=std::acos(1-0.5*Channel.InnerChordSquared);
            const double Outer=std::acos(1-0.5*Channel.OuterChordSquared);
            const auto Side=UnitDirection::FromVector(Channel.Start.Cross(Channel.Tangent))->Vector();
            for (double Along : {0.0,0.25,0.5,0.75,1.0})
            {
                const double Angle=Along*Channel.ArcAngle;
                const Vec3 Center=Channel.Start*std::cos(Angle)+Channel.Tangent*std::sin(Angle);
                for (double Across : {-0.999,-0.5,0.0,0.5,0.999})
                {
                    const double Offset=Across*Inner;
                    const auto D=*UnitDirection::FromVector(Center*std::cos(Offset)+Side*std::sin(Offset));
                    const auto Sample=Ocean->Evaluate(D);
                    if (!TestTrue(TEXT("Height-only ocean is bit identical"),std::bit_cast<std::uint64_t>(Ocean->EvaluateElevation(D))==
                        std::bit_cast<std::uint64_t>(Sample.ElevationMeters))) return false;
                    const double Excess=Sample.ElevationMeters-Limit;
                    MaximumClearanceExcess=std::max(MaximumClearanceExcess,Excess);
                    ++ClearanceChecks;
                    if (!TestTrue(TEXT("Entire inner channel sample below fixed sea-depth limit"),Excess<=1e-7)) return false;
                    if (!TestTrue(TEXT("Ocean correction never raises terrain"),Sample.ElevationMeters<=Base->Evaluate(D).Terrain.ElevationMeters+1e-8)) return false;
                    if (!TestTrue(TEXT("Channel stays outside planet center"),Settings.RadiusMeters+Sample.ElevationMeters>0)) return false;
                }
                // Probe both banks in the transition band, not just flat centers.
                for (double Sign : {-1.0,1.0})
                {
                    const double Offset=Sign*(Inner+0.37*(Outer-Inner));
                    const auto D=*UnitDirection::FromVector(Center*std::cos(Offset)+Side*std::sin(Offset));
                    const Vec3 V=D.Vector();
                    const auto Tangent=UnitDirection::FromVector(Side-V*Side.Dot(V))->Vector();
                    const auto DifferenceAt=[&](double Step) {
                        const auto Plus=*UnitDirection::FromVector(V*std::cos(Step)+Tangent*std::sin(Step));
                        const auto Minus=*UnitDirection::FromVector(V*std::cos(Step)-Tangent*std::sin(Step));
                        return (Ocean->Evaluate(Plus).ElevationMeters-Ocean->Evaluate(Minus).ElevationMeters)/(2*Step);
                    };
                    const auto Sample=Ocean->Evaluate(D);
                    if (!TestTrue(TEXT("Height-only ocean is bit identical"),std::bit_cast<std::uint64_t>(Ocean->EvaluateElevation(D))==
                        std::bit_cast<std::uint64_t>(Sample.ElevationMeters))) return false;
                    const double Analytic=Sample.TangentialGradientPerRadian.Dot(Tangent);
                    // A single central difference has O(h^2) truncation error.
                    // The narrow positive-part transition exposed it at seed
                    // 90210. Use two independent Richardson estimates and also
                    // require their convergence, with the original allowance.
                    const double Full=DifferenceAt(1e-7), Half=DifferenceAt(5e-8), Quarter=DifferenceAt(2.5e-8);
                    const double Estimate=(4*Half-Full)/3, Refined=(4*Quarter-Half)/3;
                    const double Error=std::abs(Refined-Analytic), Convergence=std::abs(Refined-Estimate);
                    const double Allowance=0.03+2e-6*std::abs(Analytic);
                    MaximumGradientError=std::max(MaximumGradientError,Error);
                    ++DerivativeChecks;
                    if (Error>Allowance || Convergence>Allowance)
                    {
                        AddError(FString::Printf(TEXT("Channel derivative discrepancy: seed=%llu radius=%.17g along=%.17g side=%.17g direction=(%.17g,%.17g,%.17g) analytic=%.17g FD_1e7=%.17g Richardson=%.17g refined=%.17g error=%.17g convergence=%.17g allowance=%.17g"),
                            static_cast<unsigned long long>(Settings.Seed),Settings.RadiusMeters,Along,Sign,V.X,V.Y,V.Z,Analytic,Full,Estimate,Refined,Error,Convergence,Allowance));
                        return false;
                    }
                    if (!TestTrue(TEXT("Composed gradient remains tangential"),std::abs(Sample.TangentialGradientPerRadian.Dot(V))<=1e-6)) return false;
                }
            }
        }
        auto Again=OceanConnectivityField::Create(Base);
        if (!TestTrue(TEXT("Independent ocean re-preparation succeeded"),Again.has_value())) return false;
        for (unsigned I=0; I<256; ++I)
        {
            const double Z=1-2*(I+0.5)/256, Phi=I*2.399963229728653, R=std::sqrt(1-Z*Z);
            const auto D=*UnitDirection::FromVector({R*std::cos(Phi),R*std::sin(Phi),Z});
            const auto A=Ocean->Evaluate(D), B=Again->Evaluate(D);
            ++RepeatedChecks;
            if (!TestTrue(TEXT("Re-prepared ocean height and gradient repeat exactly"),A.ElevationMeters==B.ElevationMeters && A.TangentialGradientPerRadian.X==B.TangentialGradientPerRadian.X && A.TangentialGradientPerRadian.Y==B.TangentialGradientPerRadian.Y && A.TangentialGradientPerRadian.Z==B.TangentialGradientPerRadian.Z)) return false;
        }
    }
    AddInfo(FString::Printf(TEXT("Ocean contracts: cases=%u, clearance=%u, bank derivatives=%u, repeats=%u, max clearance excess=%.12g m, max gradient error=%.12g m/rad, elapsed=%.3f s. Continuous final terrain certificate remains incomplete."),
        static_cast<unsigned>(Cases.size()),ClearanceChecks,DerivativeChecks,RepeatedChecks,MaximumClearanceExcess,MaximumGradientError,FPlatformTime::Seconds()-StartTime));
    return true;
}
#endif
