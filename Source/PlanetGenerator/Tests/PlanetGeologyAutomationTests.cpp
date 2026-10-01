#include "Misc/AutomationTest.h"
#include "Core/PlanetGeology.h"
#include "HAL/PlatformTime.h"
#include <bit>
#include <cmath>
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetGeologyIndexContracts,"PlanetGenerator.Geology.IndexContracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPlanetGeologyIndexContracts::RunTest(const FString& Parameters)
{
    using namespace PlanetCore;
    (void)Parameters; const double Start=FPlatformTime::Seconds();
    std::uint64_t Checked=0,Candidates=0,FullFeatures=0;
    for (const auto Case : std::array<std::pair<std::uint64_t,double>,4>{{{42,1000000},{42,1000},{42,1e9},{7,6371000}}})
    {
        GeologySettings Settings; Settings.Seed=Case.first; Settings.RadiusMeters=Case.second;
        const auto F=GeologyField::Create(Settings);
        if (!TestTrue(TEXT("Indexed geological field prepared"),F.has_value())) return false;
        std::vector<UnitDirection> Points;
        for (unsigned I=0;I<2048;++I)
        {
            const double Z=1-2*(I+0.5)/2048,Angle=I*2.399963229728653,R=std::sqrt(1-Z*Z);
            Points.push_back(*UnitDirection::FromVector({R*std::cos(Angle),R*std::sin(Angle),Z}));
        }
        for (const auto& Feature : F->Features())
        {
            Points.push_back(*UnitDirection::FromVector(Feature.Center));
            // Probe both sides of each cap boundary and both ellipse axes.
            for (const auto Axis : {Feature.Along,Feature.Across})
            for (const double Offset : {-1e-10,1e-10})
            {
                const double A=std::acos(Feature.SupportCosine)+Offset;
                Points.push_back(*UnitDirection::FromVector(Feature.Center*std::cos(A)+Axis*std::sin(A)));
            }
        }
        const auto Components=[](GeologySample S) { const auto G=S.Terrain.TangentialGradientPerRadian;
            return std::array<double,8>{S.Terrain.ElevationMeters,G.X,G.Y,G.Z,S.ContinentalCoordinate,
                S.OceanCrustAgeMillionYears,S.TectonicDisplacementMeters,S.RockResistance}; };
        for (const auto D : Points)
        {
            const auto A=Components(F->Evaluate(D)),B=Components(F->EvaluateReference(D));
            for (unsigned K=0;K<A.size();++K)
                if (!std::isfinite(A[K]) || std::bit_cast<std::uint64_t>(A[K])!=std::bit_cast<std::uint64_t>(B[K]))
                {
                    AddError(FString::Printf(TEXT("Index changed geology: seed %llu radius %.9g sample %llu component %u, %.17g != %.17g"),
                        static_cast<unsigned long long>(Case.first),Case.second,static_cast<unsigned long long>(Checked),K,A[K],B[K])); return false;
                }
            if (!TestTrue(TEXT("Height-only geology is bit identical"),
                std::bit_cast<std::uint64_t>(F->EvaluateElevation(D))==std::bit_cast<std::uint64_t>(A[0]))) return false;
            ++Checked; Candidates+=F->FeatureCandidateCount(D); FullFeatures+=F->Features().size();
        }
        // Report measured CPU cost without making a flaky timing pass condition.
        volatile double Checksum=0;
        const auto Near=*UnitDirection::FromVector({0.83494097103907794,0.29297845173323173,-0.46587251657542544});
        std::array<double,3> Milliseconds{};
        for (unsigned Pass=0;Pass<3;++Pass)
        {
            const double Begin=FPlatformTime::Seconds();
            for (unsigned I=0;I<2048;++I)
            {
                const auto D=*UnitDirection::FromVector(Near.Vector()+Vec3{I*1e-9,0,0});
                Checksum=Checksum+(Pass==2?F->EvaluateElevation(D):(Pass==0?F->EvaluateReference(D):F->Evaluate(D)).Terrain.ElevationMeters);
            }
            Milliseconds[Pass]=(FPlatformTime::Seconds()-Begin)*1000;
        }
        AddInfo(FString::Printf(TEXT("Geology index: seed %llu radius %.9g features %llu samples %llu reference %.3f ms indexed %.3f ms scalar %.3f ms checksum %.9g"),
            static_cast<unsigned long long>(Case.first),Case.second,static_cast<unsigned long long>(F->Features().size()),
            static_cast<unsigned long long>(Points.size()),Milliseconds[0],Milliseconds[1],Milliseconds[2],static_cast<double>(Checksum)));
    }
    TestTrue(TEXT("Spatial index reduces candidate visits"),Candidates<FullFeatures);
    std::stop_source Stop; Stop.request_stop();
    TestFalse(TEXT("Cancelled preparation rejected"),GeologyField::Create({},Stop.get_token()).has_value());
    AddInfo(FString::Printf(TEXT("Geology index contracts: %llu bitwise sample comparisons, candidates %llu / %llu, %.3f seconds"),
        static_cast<unsigned long long>(Checked),static_cast<unsigned long long>(Candidates),
        static_cast<unsigned long long>(FullFeatures),FPlatformTime::Seconds()-Start));
    return true;
}
#endif
