#include "Misc/AutomationTest.h"
#include "HAL/PlatformTime.h"
#include "Core/PlanetView.h"
#include "../../../Tests/Core/PlanetLodContractChecks.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetLodContracts, "PlanetGenerator.Lod.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPlanetLodContracts::RunTest(const FString& Parameters)
{
    (void)Parameters;
    const double Start=FPlatformTime::Seconds();
    PlanetTests::LodCheckReport Report;
    if (!PlanetTests::RunLodContractChecks(Report))
    {
        AddError(FString::Printf(TEXT("LOD contract failure after %llu checks: %s"),
            static_cast<unsigned long long>(Report.Checks),UTF8_TO_TCHAR(Report.Failure.c_str())));
        return false;
    }
    auto SmallCache=std::make_shared<PlanetCore::LodNeighborCache>(37);
    PlanetTests::LodCheckReport CachedReport;
    if (!PlanetTests::RunLodContractChecks(CachedReport,SmallCache))
    { AddError(UTF8_TO_TCHAR(CachedReport.Failure.c_str())); return false; }
    if (!TestTrue(TEXT("Adjacency cache with forced eviction preserves all topology contracts"),
        CachedReport.Checks==Report.Checks && SmallCache->TileCount()<=37 && SmallCache->Hits()>0)) return false;
    AddInfo(FString::Printf(TEXT("Cached LOD contracts checks=%llu tiles=%llu hits=%llu; full masks/depth/mixed edits independently verified."),
        static_cast<unsigned long long>(CachedReport.Checks),static_cast<unsigned long long>(SmallCache->TileCount()),static_cast<unsigned long long>(SmallCache->Hits())));
    PlanetCore::LodView View;
    View.LeafBudget=512; View.MaximumLevel=10;
    std::stop_source Cancellation; Cancellation.request_stop();
    TestFalse(TEXT("Cancelled view selection rejected"),PlanetCore::SelectLodForView(View,Cancellation.get_token()).has_value());
    auto Invalid=View; Invalid.LeafBudget=19;
    TestFalse(TEXT("View budget below global coverage rejected"),PlanetCore::SelectLodForView(Invalid).has_value());
    for (const auto Observer : {PlanetCore::Vec3{0,0,1100000},PlanetCore::Vec3{1100000,0,0},PlanetCore::Vec3{-2100000,2100000,900000}})
    {
        View.ObserverMeters=Observer;
        const auto A=PlanetCore::SelectLodForView(View), B=PlanetCore::SelectLodForView(View);
        if (!TestTrue(TEXT("Bounded view selection succeeds"),A.has_value() && B.has_value())) return false;
        TestTrue(TEXT("View selection respects leaf budget"),A->Tree.Leaves().size()<=View.LeafBudget);
        TestTrue(TEXT("View selection respects depth bound"),A->DeepestLevel<=View.MaximumLevel);
        TestTrue(TEXT("View selection repeats deterministically"),A->Tree.Leaves()==B->Tree.Leaves());
        if (!PlanetTests::CheckPartition(A->Tree,Report) || !PlanetTests::CheckMesh(A->Tree,Report))
        { AddError(UTF8_TO_TCHAR(Report.Failure.c_str())); return false; }
    }
    {
        using namespace PlanetCore;
        LodView V; V.RadiusMeters=1000; V.ObserverMeters={1001.6,0,0}; V.ReliefBoundMeters=160;
        V.SurfaceRadiusMeters=24; V.LeafBudget=32768; V.MaximumLevel=32;
        const auto Height=[](UnitDirection) { return 0.0; };
        double Begin=FPlatformTime::Seconds();
        const auto Reference=SelectLodForView(V,{},Height);
        const double ReferenceMs=(FPlatformTime::Seconds()-Begin)*1000;
        LodSurfaceCache Cache(V.RadiusMeters,Height);
        const auto Cold=SelectLodForView(V,{}, {},&Cache);
        Begin=FPlatformTime::Seconds();
        const auto Warm=SelectLodForView(V,{}, {},&Cache);
        const double WarmMs=(FPlatformTime::Seconds()-Begin)*1000;
        if (!TestTrue(TEXT("Warm cached selection equals uncached selection exactly"),Reference && Cold && Warm &&
            Reference->Tree.Leaves()==Cold->Tree.Leaves() && Reference->Tree.Leaves()==Warm->Tree.Leaves() && Warm->NeighborCacheHits>0)) return false;
        const auto A=BuildAdaptiveSphere(Reference->Tree),B=BuildAdaptiveSphere(Warm->Tree);
        if (!TestTrue(TEXT("Cached adjacency preserves canonical mesh order"),A && B && A->Mesh.Keys==B->Mesh.Keys && A->Mesh.Triangles==B->Mesh.Triangles)) return false;
        AddInfo(FString::Printf(TEXT("Neighbor selection benchmark leaves=%llu uncached=%.3f ms warm=%.3f ms neighbor_hits=%llu tiles=%llu"),
            static_cast<unsigned long long>(Warm->Tree.Leaves().size()),ReferenceMs,WarmMs,
            static_cast<unsigned long long>(Warm->NeighborCacheHits),static_cast<unsigned long long>(Warm->CachedNeighborTiles)));
    }
    AddInfo(FString::Printf(TEXT("LOD contracts: checks=%llu, masks=8, roots=20, max level=32, mixed edits=500, elapsed=%.3f s. Runtime performance is a separate test."),
        static_cast<unsigned long long>(Report.Checks),FPlatformTime::Seconds()-Start));
    return true;
}
#endif
