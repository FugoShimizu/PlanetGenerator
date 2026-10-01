#include "Core/PlanetGround.h"
#include "Core/PlanetView.h"
#include "../../../Tests/Core/PlanetLodContractChecks.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFoliageLodContracts, "PlanetGenerator.FoliageLod.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/** 公開用の遠景植生域に対し，実際の浮動小数点メッシュの独立点を検査する． */
bool FPlanetFoliageLodContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	constexpr double Radius = 1000000;
	const PlanetCore::UnitDirection Anchor = *PlanetCore::UnitDirection::FromVector({0, 0, 1});
	const PlanetCore::GroundFrame Frame = *PlanetCore::GroundFrame::Create(Anchor, Radius, 0);
	const PlanetCore::SurfaceHeightEvaluator Height = [&](PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 V = Direction.Vector();
		const double X = Radius * std::atan2(V.Dot(Frame.East), V.Dot(Frame.Up));
		const double Offset = (X - 40) / 5;
		// 解析関数はLODの誤差試料と独立に定義する．
		return 1.2 * std::exp(-Offset * Offset);
	};
	PlanetCore::LodView View;
	View.RadiusMeters = Radius;
	View.ReliefBoundMeters = 2;
	View.ObserverMeters = Anchor.Vector() * (Radius + 2);
	View.LeafBudget = 8192;
	View.PixelLeafBudget = 2048;
	View.TargetEdgePixels = 1024;
	View.SurfaceRadiusMeters = 24;
	View.SurfaceEdgeMeters = 2;
	View.SurfaceErrorMeters = 0.01;
	View.FoliageRadiusMeters = 64;
	View.FoliageEdgeMeters = 8;
	View.FoliageErrorMeters = 0.02;
	if(!TestTrue(TEXT("Foliage view is valid"), View.IsValid())) return false;
	const std::optional<PlanetCore::LodSelection> Selection = PlanetCore::SelectLodForView(View, {}, Height);
	if(!TestTrue(TEXT("Both mandatory regions fit the leaf budget"), Selection &&
		Selection->SurfaceResolutionSatisfied && Selection->FoliageResolutionSatisfied)) return false;
	PlanetTests::LodCheckReport Report;
	if(!PlanetTests::CheckPartition(Selection->Tree, Report) || !PlanetTests::CheckMesh(Selection->Tree, Report)) {
		AddError(UTF8_TO_TCHAR(Report.Failure.c_str()));
		return false;
	}
	const std::optional<PlanetCore::AdaptiveSphereMesh> Mesh = PlanetCore::BuildAdaptiveSphere(Selection->Tree);
	if(!TestTrue(TEXT("Shared mesh exists"), Mesh.has_value())) return false;
	std::vector<PlanetCore::Vec3> Positions;
	Positions.reserve(Mesh->Mesh.Directions.size());
	for(const PlanetCore::UnitDirection Direction : Mesh->Mesh.Directions) {
		const PlanetCore::Vec3 P = Frame.SurfaceToLocal(Direction, Height(Direction));
		Positions.push_back({double(float(P.X * 100)) / 100, double(float(P.Y * 100)) / 100,
			double(float(P.Z * 100)) / 100});
	}
	double MaximumError = 0;
	unsigned Samples = 0;
	for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Mesh.Triangles) {
		const PlanetCore::Vec3 A = Positions[Triangle[0]], B = Positions[Triangle[1]], C = Positions[Triangle[2]];
		const PlanetCore::Vec3 Center = (A + B + C) * (1.0 / 3);
		const double Cap = std::max({(A - Center).Length(), (B - Center).Length(), (C - Center).Length()});
		if(Center.X < 30 - Cap || Center.X > 50 + Cap || std::abs(Center.Y) > 12 + Cap) continue;
		for(const PlanetCore::Vec3 P : {A * 0.2 + B * 0.3 + C * 0.5, A * 0.6 + B * 0.1 + C * 0.3,
			A * 0.15 + B * 0.7 + C * 0.15}) {
			if(P.X < 30 || P.X > 50 || std::abs(P.Y) > 12) continue;
			const std::optional<PlanetCore::UnitDirection> D = PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(P));
			if(!D) return false;
			const double Increment = 2 * Radius * P.Z + P.Dot(P);
			const double Reconstructed = Increment / (std::sqrt(Radius * Radius + Increment) + Radius);
			MaximumError = std::max(MaximumError, std::abs(Reconstructed - Height(*D)));
			++Samples;
		}
	}
	AddInfo(FString::Printf(TEXT("Foliage LOD: leaves=%llu independent_samples=%u max_error=%.9g m"),
		static_cast<unsigned long long>(Selection->Tree.Leaves().size()), Samples, MaximumError));
	if(!TestTrue(TEXT("Independent distant foliage samples exist"), Samples >= 20)) return false;
	if(!TestTrue(TEXT("Float mesh stays within 2 cm at distant vegetation"), MaximumError <= 0.02)) return false;
	const std::shared_ptr<PlanetCore::LodSurfaceCache> Cache =
		std::make_shared<PlanetCore::LodSurfaceCache>(Radius, Height);
	const std::shared_ptr<PlanetCore::LodSurfacePreparation> Preparation =
		PlanetCore::LodSurfacePreparation::Create(View, Cache);
	if(!TestTrue(TEXT("Foliage prefetch accepts the current view"), Preparation != nullptr)) return false;
	PlanetCore::SurfacePreparationProgress Progress;
	for(unsigned Step = 0; Step < 512 && !Progress.Complete; ++Step) Progress = Preparation->Advance(512);
	if(!TestTrue(TEXT("Foliage probes finish inside their bounded prefetch budget"),
		Progress.Complete && Progress.Valid && !Progress.BudgetExhausted)) return false;
	const std::optional<PlanetCore::LodSelection> Warm =
		PlanetCore::SelectLodForView(View, {}, {}, Cache.get());
	if(!TestTrue(TEXT("Foliage prefetch preserves the published topology"), Warm &&
		Warm->SurfaceResolutionSatisfied && Warm->FoliageResolutionSatisfied &&
		Warm->Tree.Leaves() == Selection->Tree.Leaves() &&
		Warm->MaximumFoliageErrorMeters == Selection->MaximumFoliageErrorMeters &&
		Warm->SurfaceHeightEvaluations < Selection->SurfaceHeightEvaluations)) return false;
	AddInfo(FString::Printf(TEXT("Foliage prefetch: visits=%llu cold_evaluations=%llu warm_evaluations=%llu"),
		static_cast<unsigned long long>(Progress.VisitedTiles),
		static_cast<unsigned long long>(Selection->SurfaceHeightEvaluations),
		static_cast<unsigned long long>(Warm->SurfaceHeightEvaluations)));
	View.LeafBudget = 20;
	const std::optional<PlanetCore::LodSelection> Limited = PlanetCore::SelectLodForView(View, {}, Height);
	if(!TestTrue(TEXT("Insufficient foliage budget cannot be published"), Limited &&
		!Limited->FoliageResolutionSatisfied && Limited->Tree.Leaves().size() == 20)) return false;
	return true;
}
#endif