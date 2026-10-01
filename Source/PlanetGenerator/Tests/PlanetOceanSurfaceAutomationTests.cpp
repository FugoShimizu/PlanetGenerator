#include "Misc/AutomationTest.h"
#include "Core/PlanetOceanSurface.h"
#include "../../../Tests/Core/PlanetLodContractChecks.h"
#include <limits>
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetOceanSurfaceContracts,"PlanetGenerator.OceanSurface.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPlanetOceanSurfaceContracts::RunTest(const FString& Parameters)
{
    (void)Parameters; using namespace PlanetCore;
    for (double Scale : {1.0,1e-3,1e-9})
    {
        const std::array<UnitDirection,3> Triangle{*UnitDirection::FromVector({-Scale,0,1}),
            *UnitDirection::FromVector({Scale,0,1}),*UnitDirection::FromVector({0,Scale,1})};
        const auto Middle=*UnitDirection::FromVector({0,0,1});
        const auto Inside=*UnitDirection::FromVector({0,0.25*Scale,1});
        const auto Outside=*UnitDirection::FromVector({0,-0.25*Scale,1});
        const auto Beyond=*UnitDirection::FromVector({1.5*Scale,0,1});
        if (!TestTrue(TEXT("Spherical face and edge have zero distance"),SphericalTriangleChordDistance(Inside,Triangle)==0 && SphericalTriangleChordDistance(Middle,Triangle)<1e-14)) return false;
        if (!TestTrue(TEXT("Outside triangle resolves nearest minor arc"),std::abs(SphericalTriangleChordDistance(Outside,Triangle)-(Outside.Vector()-Middle.Vector()).Length())<1e-14)) return false;
        if (!TestTrue(TEXT("Outside arc resolves nearest vertex"),std::abs(SphericalTriangleChordDistance(Beyond,Triangle)-(Beyond.Vector()-Triangle[1].Vector()).Length())<1e-14)) return false;
    }
    const auto Boundary=*GetDirection(*GetVertexKey({0,0,0},0));
    double MaxRadialError=0,MaxProbeError=0,MaxFloatError=0; unsigned Samples=0;
    const auto CheckRays=[&](const OceanSurfaceMesh* M,double R) {
        // Ray-plane interpolation uses the actual float vertex storage. Probe
        // coverage includes an icosahedron vertex shared by five root faces.
        for (unsigned I=0;I<49;++I)
        {
            const double X=20*(std::fmod(I*0.618033988749895,1.0)-0.5),Y=20*(std::fmod(I*0.414213562373095+0.17,1.0)-0.5);
            bool Found=false;
            for (const auto& T : M->Sphere.Triangles)
            {
                std::array<Vec3,3> P;
                for (unsigned J=0;J<3;++J)
                { const auto Q=M->Positions[T[J]]; P[J]={static_cast<float>(Q.X*100)/100.0,static_cast<float>(Q.Y*100)/100.0,static_cast<float>(Q.Z*100)/100.0}; }
                if (std::max({P[0].Z,P[1].Z,P[2].Z}) < -R*0.25) continue;
                const auto A=P[1]-P[0],B=P[2]-P[0],Q=Vec3{X,Y,0}-P[0];
                const double Det=A.Cross(B).Z; if (Det<=0) continue;
                const double U=Q.Cross(B).Z/Det,W=A.Cross(Q).Z/Det;
                if (U<-1e-8 || W<-1e-8 || U+W>1+1e-8) continue;
                const auto Hit=P[0]+A*U+B*W;
                const double Increment=2*R*Hit.Z+Hit.Dot(Hit);
                const double Height=Increment/(std::sqrt(R*R+Increment)+R);
                MaxProbeError=std::max(MaxProbeError,std::abs(Height));
                Found=true; ++Samples; break;
            }
            if (!TestTrue(TEXT("Ocean covers every independent near-surface ray"),Found)) return false;
        }
        return true;
    };
    for (double R : {1000.0,1e6,1e9})
    for (auto Anchor : {*UnitDirection::FromVector({1,2,3}),Boundary})
    {
        LodView V; V.RadiusMeters=R; V.ReliefBoundMeters=0;
        V.ObserverMeters=Anchor.Vector()*(R+1.6); V.LeafBudget=8192; V.MaximumLevel=32;
        const auto M=BuildOceanSurface(V,0);
        if (!TestTrue(TEXT("Complete ocean mesh generated near sea level"),M && M->NearRadiusMeters==24 && M->Positions.size()==M->Sphere.Directions.size())) return false;
        MaxFloatError=std::max(MaxFloatError,M->MaximumFloatErrorMeters);
        for (const auto P : M->Positions)
            MaxRadialError=std::max(MaxRadialError,std::abs(M->Frame.LocalToPlanet(P).Length()-R));
        if (!CheckRays(&*M,R)) return false;
        OceanReuseDiagnostics ReuseDiagnostics;
        const auto Reused=ReuseOceanSurface(V,0,M->Sphere,{}, {},&ReuseDiagnostics);
        if (!Reused) AddInfo(FString::Printf(TEXT("Reuse rejected R=%.0f near=%d edge=%.9g height=%.9g float=%.9g pixel_rejected=%d pixels=%.9g"),
            R,ReuseDiagnostics.RejectedNear?1:0,ReuseDiagnostics.NearEdgeMeters,ReuseDiagnostics.NearErrorMeters,ReuseDiagnostics.FloatErrorMeters,
            ReuseDiagnostics.RejectedProjection?1:0,ReuseDiagnostics.ProjectionErrorPixels));
        if (!TestTrue(TEXT("Certified sphere topology can be projected without rebuilding LOD"),Reused && Reused->ReusedTopology && Reused->MaximumProjectionErrorPixels<=1 && Reused->Sphere.Triangles==M->Sphere.Triangles)) return false;
        for (std::size_t I=0;I<M->Positions.size();++I)
            if (!TestTrue(TEXT("Reused sea projection equals independent sea vertices"),(Reused->Positions[I]-M->Positions[I]).Length()==0)) return false;
        const auto Coarse=BuildAdaptiveSphere(LodTree{});
        if (!TestFalse(TEXT("Deep-bottom or coarse terrain cannot stand in for near sea LOD"),ReuseOceanSurface(V,0,Coarse->Mesh).has_value())) return false;
        auto Tight=V; Tight.SurfaceErrorMeters=1e-10;
        if (!TestFalse(TEXT("Reused sea obeys the requested geometric accuracy"),ReuseOceanSurface(Tight,0,M->Sphere).has_value())) return false;
        std::stop_source ReuseStop; ReuseStop.request_stop();
        if (!TestFalse(TEXT("Topology reuse obeys cancellation"),ReuseOceanSurface(V,0,M->Sphere,{},ReuseStop.get_token()).has_value())) return false;
        AddInfo(FString::Printf(TEXT("Ocean topology reuse R=%.0f geometric pixel metric=%.9g"),R,Reused->MaximumProjectionErrorPixels));
        auto CoarseView=V; CoarseView.SurfaceRadiusMeters=24; CoarseView.SurfaceEdgeMeters=R==1000?8:4;
        CoarseView.PixelLeafBudget=20;
        const auto CoarseSelection=SelectLodForView(CoarseView,{},[](UnitDirection) { return 0.0; });
        if (!TestTrue(TEXT("Coarse starting partition exists"),CoarseSelection.has_value())) return false;
        const auto Starting=BuildAdaptiveSphere(CoarseSelection->Tree);
        const auto OriginalLeaves=CoarseSelection->Tree.Leaves();
        if (!TestFalse(TEXT("Starting partition needs sea-specific refinement"),ReuseOceanSurface(V,0,Starting->Mesh).has_value())) return false;
        OceanReuseDiagnostics Initial;
        const auto Refined=BuildOceanSurfaceFromTerrain(V,0,CoarseSelection->Tree,*Starting,{}, {},&Initial);
        if (!TestTrue(TEXT("Only failed sea owners are refined within the original budget"),Refined && Refined->RefinedTerrainParents>0 &&
            Refined->RefinementPasses<=4 && Refined->LeafCount<=V.LeafBudget && Refined->MaximumProjectionErrorPixels<=1)) return false;
        if (!TestTrue(TEXT("Sea refinement preserves the immutable terrain partition"),CoarseSelection->Tree.Leaves()==OriginalLeaves)) return false;
        if (!CheckRays(&*Refined,R)) return false;
        std::map<std::pair<std::uint32_t,std::uint32_t>,std::pair<unsigned,int>> Edges;
        for (const auto T : Refined->Sphere.Triangles)
            for (unsigned J=0;J<3;++J)
            {
                const auto A=T[J],B=T[(J+1)%3];
                auto& E=Edges[{std::min(A,B),std::max(A,B)}]; ++E.first; E.second+=A<B?1:-1;
            }
        for (const auto& E : Edges)
            if (!TestTrue(TEXT("Refined water has two oppositely oriented faces at every edge"),E.second.first==2 && E.second.second==0)) return false;
        if (!TestTrue(TEXT("Refined water retains spherical Euler characteristic"),
            static_cast<std::int64_t>(Refined->Positions.size())-static_cast<std::int64_t>(Edges.size())+static_cast<std::int64_t>(Refined->Sphere.Triangles.size())==2)) return false;
        auto NoBudget=V; NoBudget.LeafBudget=OriginalLeaves.size();
        if (!TestFalse(TEXT("No refinement budget yields no partial sea"),BuildOceanSurfaceFromTerrain(NoBudget,0,CoarseSelection->Tree,*Starting).has_value())) return false;
        if (!TestFalse(TEXT("Selective refinement obeys cancellation"),BuildOceanSurfaceFromTerrain(V,0,CoarseSelection->Tree,*Starting,{},ReuseStop.get_token()).has_value())) return false;
        const auto Again=BuildOceanSurfaceFromTerrain(V,0,CoarseSelection->Tree,*Starting);
        if (!TestTrue(TEXT("Selective refinement is independent of earlier attempts"),Again && Again->Sphere.Triangles==Refined->Sphere.Triangles && Again->Sphere.Keys==Refined->Sphere.Keys)) return false;
        AddInfo(FString::Printf(TEXT("Selective ocean R=%.0f leaves %llu -> %llu passes=%u parents=%u initial near=%llu pixels=%llu metric=%.9g"),
            R,static_cast<unsigned long long>(OriginalLeaves.size()),static_cast<unsigned long long>(Refined->LeafCount),Refined->RefinementPasses,
            Refined->RefinedTerrainParents,static_cast<unsigned long long>(Initial.RejectedNearTriangles),
            static_cast<unsigned long long>(Initial.RejectedProjectionTriangles),Refined->MaximumProjectionErrorPixels));
        OceanSurfaceCache RetainedCache(R,0);
        const auto FirstRetained=BuildOceanSurfaceFromTerrain(V,0,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache);
        const auto SameRetained=BuildOceanSurfaceFromTerrain(V,0,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache);
        if (!TestTrue(TEXT("A validated previous sea needs no repeated partition construction"),FirstRetained && SameRetained &&
            !FirstRetained->ReusedPreviousTopology && SameRetained->ReusedPreviousTopology && SameRetained->RefinementPasses==0 &&
            SameRetained->Sphere.Triangles==FirstRetained->Sphere.Triangles && SameRetained->Sphere.Keys==FirstRetained->Sphere.Keys)) return false;
        auto Shifted=V; Shifted.ObserverMeters=FirstRetained->Frame.DirectionAt(7,0).Vector()*(R+1.6);
        const auto ShiftedSea=BuildOceanSurfaceFromTerrain(Shifted,0,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache);
        if (!TestTrue(TEXT("Retained partition is revalidated and refined for actual moved view"),ShiftedSea &&
            ShiftedSea->ReusedPreviousTopology && ShiftedSea->LeafCount<=V.LeafBudget && ShiftedSea->MaximumProjectionErrorPixels<=1)) return false;
        if (!CheckRays(&*ShiftedSea,R)) return false;
        if (!TestFalse(TEXT("Retained topology cannot bypass the new leaf budget"),
            BuildOceanSurfaceFromTerrain(NoBudget,0,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache).has_value())) return false;
        if (!TestFalse(TEXT("Retained topology rejects a different sea level"),
            BuildOceanSurfaceFromTerrain(V,1,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache).has_value())) return false;
        auto DifferentRadius=V; DifferentRadius.RadiusMeters*=2;
        if (!TestFalse(TEXT("Retained topology rejects a different planet radius"),
            BuildOceanSurfaceFromTerrain(DifferentRadius,0,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache).has_value())) return false;
        if (!TestFalse(TEXT("Retained topology obeys cancellation"),
            BuildOceanSurfaceFromTerrain(Shifted,0,CoarseSelection->Tree,*Starting,{},ReuseStop.get_token(),nullptr,&RetainedCache).has_value())) return false;
        const auto AfterRejection=BuildOceanSurfaceFromTerrain(Shifted,0,CoarseSelection->Tree,*Starting,{}, {},nullptr,&RetainedCache);
        if (!TestTrue(TEXT("Rejected requests preserve the complete cached sea"),AfterRejection && AfterRejection->ReusedPreviousTopology &&
            AfterRejection->RefinementPasses==0 && AfterRejection->Sphere.Keys==ShiftedSea->Sphere.Keys && AfterRejection->Sphere.Triangles==ShiftedSea->Sphere.Triangles)) return false;
        AddInfo(FString::Printf(TEXT("Retained ocean R=%.0f leaves=%llu moved leaves=%llu passes=%u parents=%u metric=%.9g"),
            R,static_cast<unsigned long long>(FirstRetained->LeafCount),static_cast<unsigned long long>(ShiftedSea->LeafCount),
            ShiftedSea->RefinementPasses,ShiftedSea->RefinedTerrainParents,ShiftedSea->MaximumProjectionErrorPixels));
        auto FineView=V; FineView.SurfaceRadiusMeters=24; FineView.SurfaceEdgeMeters=2;
        const auto FineSelection=SelectLodForView(FineView,{},[](UnitDirection) { return 0.0; });
        if (!TestTrue(TEXT("Fine sea partition available"),FineSelection.has_value())) return false;
        const auto FineMesh=BuildAdaptiveSphere(FineSelection->Tree);
        const auto DirectCurrent=BuildOceanSurfaceFromTerrain(V,0,FineSelection->Tree,*FineMesh,{}, {},nullptr,&RetainedCache);
        if (!TestTrue(TEXT("Already sufficient current terrain avoids growing previous sea"),DirectCurrent && !DirectCurrent->ReusedPreviousTopology &&
            DirectCurrent->RefinementPasses==0 && DirectCurrent->Sphere.Triangles==FineMesh->Mesh.Triangles)) return false;
        OceanSurfaceCache Cache(R,0);
        const auto Cached=BuildOceanSurface(V,0,{}, {},&Cache);
        const auto Warm=BuildOceanSurface(V,0,{}, {},&Cache);
        if (!TestTrue(TEXT("Cached sea retains exact cold topology and reuses probes"),Cached && Warm && Warm->CachedProbeHits>0 && Warm->Sphere.Triangles==M->Sphere.Triangles && Warm->Positions.size()==M->Positions.size())) return false;
        for (std::size_t I=0;I<M->Positions.size();++I)
            if (!TestTrue(TEXT("Cached and cold sea vertices agree exactly"),(Warm->Positions[I]-M->Positions[I]).Length()==0)) return false;
        if (!TestFalse(TEXT("A cache from a different water level cannot be reused"),BuildOceanSurface(V,1,{}, {},&Cache).has_value())) return false;
        AddInfo(FString::Printf(TEXT("Ocean cache R=%.0f tiles=%llu probe_hits=%llu"),R,static_cast<unsigned long long>(Warm->CachedTiles),static_cast<unsigned long long>(Warm->CachedProbeHits)));
        auto Moved=V; Moved.ObserverMeters=M->Frame.DirectionAt(64,0).Vector()*(R+1.6);
        const auto N=BuildOceanSurface(Moved,0,M->Frame);
        if (!TestTrue(TEXT("Ocean retains its origin inside 128 metres"),N && (N->Frame.Up-M->Frame.Up).Length()==0)) return false;
        Moved.ObserverMeters=M->Frame.DirectionAt(160,0).Vector()*(R+1.6);
        const auto Far=BuildOceanSurface(Moved,0,M->Frame);
        if (!TestTrue(TEXT("Ocean rebases beyond its numerical-origin range"),Far && (Far->Frame.Up-M->Frame.Up).Length()>0)) return false;
        auto Bad=V; Bad.LeafBudget=20;
        if (!TestFalse(TEXT("Insufficient mandatory ocean budget is rejected"),BuildOceanSurface(Bad,0).has_value())) return false;
        if (!TestFalse(TEXT("Nonfinite sea level is rejected"),BuildOceanSurface(V,std::numeric_limits<double>::quiet_NaN()).has_value())) return false;
        std::stop_source Stop; Stop.request_stop();
        if (!TestFalse(TEXT("Ocean generation obeys cancellation"),BuildOceanSurface(V,0,{},Stop.get_token()).has_value())) return false;
    }
    TestTrue(TEXT("Base ocean vertices lie on one sphere"),MaxRadialError<2e-6);
    TestTrue(TEXT("Near ocean float vertices keep millimetre precision"),MaxFloatError<0.001);
    TestTrue(TEXT("Independent near ocean height error stays below two centimetres"),MaxProbeError<0.02);
    AddInfo(FString::Printf(TEXT("Ocean surface: %u independent rays; max radial error %.9g m, float error %.9g m, triangle height error %.9g m. Three radii and a five-face root junction; no GPU readback."),Samples,MaxRadialError,MaxFloatError,MaxProbeError));
    return true;
}
#endif
