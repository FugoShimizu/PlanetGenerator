#include "Misc/AutomationTest.h"
#include "Core/PlanetView.h"
#include "Core/PlanetGround.h"
#include "../../../Tests/Core/PlanetLodContractChecks.h"
#include "HAL/PlatformTime.h"
#include <cmath>
#include <algorithm>
#include <limits>
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSurfaceViewContracts,"PlanetGenerator.SurfaceView.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPlanetSurfaceViewContracts::RunTest(const FString& Parameters)
{
    (void)Parameters; const double Start=FPlatformTime::Seconds();
    PlanetTests::LodCheckReport Report; unsigned Samples=0;
    const auto Boundary=*PlanetCore::GetDirection(*PlanetCore::GetVertexKey({0,0,0},0));
    for (const double Radius : {1000.0,1000000.0,1e9})
    for (const auto Anchor : {*PlanetCore::UnitDirection::FromVector({1,2,3}),Boundary})
    {
        PlanetCore::LodView View; View.RadiusMeters=Radius; View.ReliefBoundMeters=0;
        View.ObserverMeters=Anchor.Vector()*(Radius+1.6); View.LeafBudget=4096;
        View.MaximumLevel=32; View.TargetEdgePixels=1024; View.SurfaceRadiusMeters=8; View.SurfaceEdgeMeters=2;
        const auto Selection=PlanetCore::SelectLodForView(View);
        if (!TestTrue(TEXT("Mandatory surface resolution reached"),Selection && Selection->SurfaceResolutionSatisfied)) return false;
        if (!PlanetTests::CheckPartition(Selection->Tree,Report) || !PlanetTests::CheckMesh(Selection->Tree,Report))
        { AddError(UTF8_TO_TCHAR(Report.Failure.c_str())); return false; }
        const auto Mesh=PlanetCore::BuildAdaptiveSphere(Selection->Tree);
        if (!TestTrue(TEXT("Conforming surface mesh available"),Mesh.has_value())) return false;
        const auto Frame=*PlanetCore::GroundFrame::Create(Anchor,Radius,0);
        // Independent gnomonic point-in-triangle checks on the emitted mesh,
        // including a root vertex shared by five root faces.
        for (unsigned I=0;I<49;++I)
        {
            const double X=10*(std::fmod(I*0.618033988749895,1.0)-0.5);
            const double Y=10*(std::fmod(I*0.414213562373095+0.17,1.0)-0.5);
            bool Found=false;
            for (const auto T : Mesh->Mesh.Triangles)
            {
                std::array<PlanetCore::Vec3,3> Q; bool Front=true;
                for (unsigned K=0;K<3;++K)
                {
                    const auto V=Mesh->Mesh.Directions[T[K]].Vector();
                    const double W=V.Dot(Frame.Up); if (W<0.25) { Front=false; break; }
                    Q[K]={Radius*V.Dot(Frame.East)/W,Radius*V.Dot(Frame.North)/W,0};
                }
                if (!Front) continue;
                const auto A=Q[1]-Q[0],B=Q[2]-Q[0],P=PlanetCore::Vec3{X,Y,0}-Q[0];
                const double Den=A.Cross(B).Z; if (Den<=0) continue;
                const double U=P.Cross(B).Z/Den,V=A.Cross(P).Z/Den;
                if (U<-1e-8 || V<-1e-8 || U+V>1+1e-8) continue;
                for (unsigned K=0;K<3;++K)
                {
                    const double Edge=(Mesh->Mesh.Directions[T[K]].Vector()-Mesh->Mesh.Directions[T[(K+1)%3]].Vector()).Length()*Radius;
                    if (!TestTrue(TEXT("Independent near sample has meter-scale edges"),Edge<=2.000001)) return false;
                }
                Found=true; break;
            }
            if (!TestTrue(TEXT("Independent near sample covered"),Found)) return false;
            ++Samples;
        }
        View.LeafBudget=20;
        const auto Limited=PlanetCore::SelectLodForView(View);
        if (!TestTrue(TEXT("Insufficient budget reports incomplete accuracy without holes"),Limited && !Limited->SurfaceResolutionSatisfied && Limited->Tree.Leaves().size()==20)) return false;
    }

    // A narrow analytic ridge reproduces the fixed-edge weakness without
    // depending on the LOD predictor's sample positions. Inspect float mesh
    // triangles at different barycentric points, including a root junction.
    for (const double Radius : {1000.0,1000000.0,1e9})
    {
        const auto Anchor=Radius==1000?Boundary:*PlanetCore::UnitDirection::FromVector({0,0,1});
        const auto Basis=*PlanetCore::GroundFrame::Create(Anchor,Radius,0.8);
        const double Width=Radius==1e9?8.0:0.5;
        const PlanetCore::SurfaceHeightEvaluator Height=[&](PlanetCore::UnitDirection D) {
            const auto V=D.Vector(); const double X=Radius*std::atan2(V.Dot(Basis.East),V.Dot(Basis.Up));
            return 0.8*std::exp(-X*X/(Width*Width));
        };
        PlanetCore::LodView View; View.RadiusMeters=Radius; View.ReliefBoundMeters=1;
        View.ObserverMeters=Anchor.Vector()*(Radius+2.4); View.LeafBudget=8192;
        View.SurfaceRadiusMeters=8; View.SurfaceEdgeMeters=2; View.TargetEdgePixels=1024;
        const auto Fixed=PlanetCore::SelectLodForView(View);
        const auto Improved=PlanetCore::SelectLodForView(View,{},Height);
        if (!TestTrue(FString::Printf(TEXT("Height-aware surface resolution R=%.0f"),Radius),
            Fixed && Improved && Improved->SurfaceResolutionSatisfied)) return false;
        const auto Inspect=[&](const PlanetCore::LodSelection& Selection)->double {
            const auto Mesh=PlanetCore::BuildAdaptiveSphere(Selection.Tree);
            if (!Mesh) return std::numeric_limits<double>::infinity();
            std::vector<PlanetCore::Vec3> Positions; Positions.reserve(Mesh->Mesh.Directions.size());
            for (const auto D : Mesh->Mesh.Directions)
            {
                const auto P=Basis.SurfaceToLocal(D,Height(D));
                Positions.push_back({double(float(P.X*100))/100,double(float(P.Y*100))/100,double(float(P.Z*100))/100});
            }
            double Maximum=0;
            for (const auto T : Mesh->Mesh.Triangles)
            {
                const auto A=Positions[T[0]],B=Positions[T[1]],C=Positions[T[2]],Center=(A+B+C)*(1.0/3);
                const double Cap=std::max({(A-Center).Length(),(B-Center).Length(),(C-Center).Length()});
                if (Center.Length()>10+Cap) continue;
                for (const auto P : {A*0.2+B*0.3+C*0.5,A*0.6+B*0.1+C*0.3,A*0.15+B*0.7+C*0.15})
                {
                    const auto D=PlanetCore::UnitDirection::FromVector(Basis.LocalToPlanet(P));
                    if (!D || (D->Vector()-Anchor.Vector()).Length()*Radius>8) continue;
                    const double R0=Radius+0.8,Increment=2*R0*P.Z+P.Dot(P);
                    const double H=0.8+Increment/(std::sqrt(R0*R0+Increment)+R0);
                    Maximum=std::max(Maximum,std::abs(H-Height(*D)));
                }
            }
            return Maximum;
        };
        const double Before=Inspect(*Fixed),After=Inspect(*Improved);
        AddInfo(FString::Printf(TEXT("Height-aware LOD R=%.0f fixed_error=%.9g m refined_error=%.9g m leaves=%llu height_evaluations=%llu predicted_error=%.9g m"),
            Radius,Before,After,static_cast<unsigned long long>(Improved->Tree.Leaves().size()),
            static_cast<unsigned long long>(Improved->SurfaceHeightEvaluations),Improved->MaximumSurfaceErrorMeters));
        if (Radius<1e9 && !TestTrue(TEXT("Fixed two-metre edges fail on the independent narrow ridge"),Before>0.02)) return false;
        if (!TestTrue(TEXT("Independent float mesh height probes meet unchanged 2cm limit"),After<=0.02)) return false;
        if (!PlanetTests::CheckPartition(Improved->Tree,Report) || !PlanetTests::CheckMesh(Improved->Tree,Report))
        { AddError(UTF8_TO_TCHAR(Report.Failure.c_str())); return false; }
        if (Radius==1000)
        {
            auto Dense=View; Dense.SurfaceRadiusMeters=12;
            const auto LimitedDense=PlanetCore::SelectLodForView(Dense,{},Height);
            Dense.LeafBudget=PlanetCore::LodView::MaximumViewLeaves;
            const auto DenseResult=PlanetCore::SelectLodForView(Dense,{},Height);
            if (!TestTrue(TEXT("Dense near terrain may reserve more leaves while pixel budget remains bounded"),
                LimitedDense && !LimitedDense->SurfaceResolutionSatisfied && DenseResult && DenseResult->SurfaceResolutionSatisfied &&
                DenseResult->Tree.Leaves().size()>8192 && DenseResult->Tree.Leaves().size()<=Dense.LeafBudget)) return false;
            if (!PlanetTests::CheckPartition(DenseResult->Tree,Report) || !PlanetTests::CheckMesh(DenseResult->Tree,Report))
            { AddError(UTF8_TO_TCHAR(Report.Failure.c_str())); return false; }
            AddInfo(FString::Printf(TEXT("Dense surface: %llu leaves; 8192-leaf error %.9g m; reserved error %.9g m"),
                static_cast<unsigned long long>(DenseResult->Tree.Leaves().size()),LimitedDense->MaximumSurfaceErrorMeters,DenseResult->MaximumSurfaceErrorMeters));
            Dense.SurfaceRadiusMeters=0;
            const auto Distant=PlanetCore::SelectLodForView(Dense);
            if (!TestTrue(TEXT("Extra near budget does not increase distant pixel work"),Distant && Distant->Tree.Leaves().size()<=Dense.PixelLeafBudget)) return false;
        }
        auto EagerView=View; EagerView.DeferCoarseSurfaceProbes=false;
        const auto Eager=PlanetCore::SelectLodForView(EagerView,{},Height);
        if (!TestTrue(TEXT("Deferred forced splits retain final ridge topology and probe errors"),Eager && Eager->SurfaceResolutionSatisfied &&
            Eager->Tree.Leaves()==Improved->Tree.Leaves() && Eager->MaximumSurfaceErrorMeters==Improved->MaximumSurfaceErrorMeters &&
            Eager->MaximumStitchErrorMeters==Improved->MaximumStitchErrorMeters &&
            Improved->SurfaceHeightEvaluations<Eager->SurfaceHeightEvaluations)) return false;
        AddInfo(FString::Printf(TEXT("Deferred coarse ridge R=%.0f evaluations=%llu eager=%llu"),Radius,
            static_cast<unsigned long long>(Improved->SurfaceHeightEvaluations),static_cast<unsigned long long>(Eager->SurfaceHeightEvaluations)));
        PlanetCore::LodSurfaceCache Cache(Radius,Height);
        const auto Cold=PlanetCore::SelectLodForView(View,{},{},&Cache);
        const auto Warm=PlanetCore::SelectLodForView(View,{},{},&Cache);
        if (!TestTrue(TEXT("Cold and warm cache preserve the uncached partition and error"),
            Cold && Warm && Cold->Tree.Leaves()==Improved->Tree.Leaves() && Warm->Tree.Leaves()==Improved->Tree.Leaves() &&
            Warm->MaximumSurfaceErrorMeters==Improved->MaximumSurfaceErrorMeters &&
            Warm->MaximumStitchErrorMeters==Improved->MaximumStitchErrorMeters && Warm->SurfaceHeightEvaluations==0 &&
            Warm->SurfaceProbeCacheHits>0 && Warm->CachedTiles<=PlanetCore::LodSurfaceCache::MaximumTiles)) return false;
        auto Moved=View;
        Moved.SurfaceObserverMeters=Basis.DirectionAt(2,0).Vector()*(Radius+2.4);
        Moved.ObserverMeters=Basis.DirectionAt(100,0).Vector()*(Radius+102.4);
        const auto MovedReference=PlanetCore::SelectLodForView(Moved,{},Height);
        const auto MovedCached=PlanetCore::SelectLodForView(Moved,{},{},&Cache);
        if (!TestTrue(TEXT("Moved body remains independent of distant camera and cache visitation"),
            MovedReference && MovedCached && MovedCached->SurfaceResolutionSatisfied &&
            MovedReference->Tree.Leaves()==MovedCached->Tree.Leaves() &&
            MovedReference->MaximumSurfaceErrorMeters==MovedCached->MaximumSurfaceErrorMeters &&
            MovedReference->MaximumStitchErrorMeters==MovedCached->MaximumStitchErrorMeters &&
            MovedCached->SurfaceHeightEvaluations<MovedReference->SurfaceHeightEvaluations)) return false;
        auto PreparationCache=std::make_shared<PlanetCore::LodSurfaceCache>(Radius,Height);
        auto Preparation=PlanetCore::LodSurfacePreparation::Create(Moved,PreparationCache);
        if (!TestTrue(TEXT("Preparation binds a valid immutable view/cache"),Preparation!=nullptr)) return false;
        auto Progress=Preparation->Advance(17);
        if (!TestTrue(TEXT("Preparation honors a short slice"),!Progress.Complete && Progress.VisitedTiles==17)) return false;
        std::stop_source Pause; Pause.request_stop();
        const auto Paused=Preparation->Advance(256,Pause.get_token());
        if (!TestTrue(TEXT("Stopped preparation does not consume pending tiles"),Paused.VisitedTiles==Progress.VisitedTiles &&
            Paused.HeightEvaluations==Progress.HeightEvaluations)) return false;
        // A foreground selection may use and evict entries between slices.
        const auto Interleaved=PlanetCore::SelectLodForView(View,{},{},PreparationCache.get());
        if (!TestTrue(TEXT("Foreground selection stays valid between preparation slices"),Interleaved && Interleaved->SurfaceResolutionSatisfied)) return false;
        while (!Progress.Complete) Progress=Preparation->Advance(113);
        if (!TestTrue(TEXT("Preparation completes within bounded visits"),Progress.Valid && !Progress.BudgetExhausted &&
            Progress.VisitedTiles<=4*Moved.LeafBudget+20 && PreparationCache->TileCount()<=PlanetCore::LodSurfaceCache::MaximumTiles)) return false;
        const auto AfterPreparation=PlanetCore::SelectLodForView(Moved,{},{},PreparationCache.get());
        if (!TestTrue(TEXT("Resumed preparation preserves final topology and errors"),AfterPreparation &&
            AfterPreparation->Tree.Leaves()==MovedReference->Tree.Leaves() &&
            AfterPreparation->MaximumSurfaceErrorMeters==MovedReference->MaximumSurfaceErrorMeters &&
            AfterPreparation->MaximumStitchErrorMeters==MovedReference->MaximumStitchErrorMeters &&
            AfterPreparation->SurfaceHeightEvaluations<MovedReference->SurfaceHeightEvaluations)) return false;
        AddInfo(FString::Printf(TEXT("Resumable preparation R=%.0f visited=%llu preparation_H=%llu foreground_H=%llu cold_H=%llu"),Radius,
            static_cast<unsigned long long>(Progress.VisitedTiles),static_cast<unsigned long long>(Progress.HeightEvaluations),
            static_cast<unsigned long long>(AfterPreparation->SurfaceHeightEvaluations),static_cast<unsigned long long>(MovedReference->SurfaceHeightEvaluations)));
        if (Radius==1000)
        {
            std::stop_source DuringProbe; std::uint64_t Calls=0;
            auto DuringCache=std::make_shared<PlanetCore::LodSurfaceCache>(Radius,[&](PlanetCore::UnitDirection D) {
                if (++Calls==3) DuringProbe.request_stop();
                return Height(D);
            });
            auto During=PlanetCore::LodSurfacePreparation::Create(Moved,DuringCache);
            if (!During) return false;
            const auto Interrupted=During->Advance(100000,DuringProbe.get_token());
            if (!TestTrue(TEXT("Cancellation during a tile stops before the next tile"),
                DuringProbe.stop_requested() && !Interrupted.Complete && Interrupted.HeightEvaluations>=3)) return false;
            auto Resumed=Interrupted;
            do { Resumed=During->Advance(127); } while (!Resumed.Complete);
            const auto Checked=PlanetCore::SelectLodForView(Moved,{},{},DuringCache.get());
            if (!TestTrue(TEXT("Cancellation leaves no partial probe accepted as complete"),Resumed.Valid && Checked &&
                Checked->Tree.Leaves()==MovedReference->Tree.Leaves() &&
                Checked->MaximumSurfaceErrorMeters==MovedReference->MaximumSurfaceErrorMeters &&
                Checked->MaximumStitchErrorMeters==MovedReference->MaximumStitchErrorMeters)) return false;
        }
        auto WrongPreparation=Moved; WrongPreparation.RadiusMeters=Radius==1000?1e6:1000;
        TestFalse(TEXT("Preparation rejects a different radius"),bool(PlanetCore::LodSurfacePreparation::Create(WrongPreparation,PreparationCache)));
        PlanetCore::LodSurfaceCache SmallCache(Radius,Height,64);
        const auto Evicted=PlanetCore::SelectLodForView(Moved,{},{},&SmallCache);
        if (!TestTrue(TEXT("Repeated eviction preserves output and bounds storage"),
            Evicted && Evicted->Tree.Leaves()==MovedReference->Tree.Leaves() && SmallCache.TileCount()<=64)) return false;
        auto WrongRadius=View; WrongRadius.RadiusMeters=Radius==1000?1000000:1000;
        TestFalse(TEXT("Cache cannot cross field radius"),PlanetCore::SelectLodForView(WrongRadius,{},{},&Cache).has_value());
        TestFalse(TEXT("Supplying a different evaluator beside a bound cache is rejected"),PlanetCore::SelectLodForView(View,{},Height,&Cache).has_value());
        AddInfo(FString::Printf(TEXT("LOD cache R=%.0f cold_evaluations=%llu warm_evaluations=%llu warm_hits=%llu moved_evaluations=%llu / %llu cached_tiles=%llu"),
            Radius,static_cast<unsigned long long>(Cold->SurfaceHeightEvaluations),static_cast<unsigned long long>(Warm->SurfaceHeightEvaluations),
            static_cast<unsigned long long>(Warm->SurfaceProbeCacheHits),static_cast<unsigned long long>(MovedCached->SurfaceHeightEvaluations),
            static_cast<unsigned long long>(MovedReference->SurfaceHeightEvaluations),static_cast<unsigned long long>(Cache.TileCount())));
        std::stop_source Stop; Stop.request_stop();
        TestFalse(TEXT("Cancelled height-aware selection rejected"),PlanetCore::SelectLodForView(View,Stop.get_token(),Height).has_value());
        TestFalse(TEXT("Nonfinite height rejected before publication"),PlanetCore::SelectLodForView(View,{},[](PlanetCore::UnitDirection) {
            return std::numeric_limits<double>::quiet_NaN(); }).has_value());
        View.LeafBudget=20;
        const auto Limited=PlanetCore::SelectLodForView(View,{},Height);
        if (!TestTrue(TEXT("Insufficient terrain-error budget retains a complete rejected partition"),
            Limited && !Limited->SurfaceResolutionSatisfied && Limited->Tree.Leaves().size()==20)) return false;
    }
    AddInfo(FString::Printf(TEXT("Surface view: 6 radius/root-boundary conditions, %u independent samples; topology checks %llu; %.3f s. Finite tests, not a continuous height-error proof."),Samples,static_cast<unsigned long long>(Report.Checks),FPlatformTime::Seconds()-Start));
    return true;
}
#endif
