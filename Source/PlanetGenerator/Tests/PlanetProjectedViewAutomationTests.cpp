#include "../../../Tests/Core/PlanetLodContractChecks.h"
#include "Core/PlanetDrainage.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetGround.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetStreamingFocus.h"
#include "Core/PlanetView.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 独立した放射線と三角形の交点から，実際の描画標高を取得する
	 * @param Mesh 閉じた共有分割
	 * @param Positions 標高適用後の惑星座標
	 * @param Direction 標高を調べる単位方向
	 * @param TriangleIndex 最近の交点の面番号を必要時に返す出力先
	 * @return 最初の正の交点距離，欠落時は無限大
	 */
	double RadialMeshDistance(
		const PlanetCore::SphereMesh &Mesh,
		const std::vector<PlanetCore::Vec3> &Positions,
		const PlanetCore::UnitDirection Direction,
		std::size_t *const TriangleIndex=nullptr
	) {
		const PlanetCore::Vec3 Ray = Direction.Vector();
		double Nearest = std::numeric_limits<double>::infinity();
		for(std::size_t Index = 0; Index < Mesh.Triangles.size(); ++Index) {
			const std::array<std::uint32_t, 3> &Triangle = Mesh.Triangles[Index];
			const PlanetCore::Vec3 A = Positions[Triangle[0]], E1 = Positions[Triangle[1]] - A, E2 = Positions[Triangle[2]] - A;
			const PlanetCore::Vec3 P = Ray.Cross(E2);
			const double Determinant = E1.Dot(P);
			if(std::abs(Determinant) < 1E-12) continue;
			const PlanetCore::Vec3 Offset = A * -1, Q = Offset.Cross(E1);
			const double U = Offset.Dot(P) / Determinant, V = Ray.Dot(Q) / Determinant;
			if(U < -1E-9 || V < -1E-9 || U + V > 1 + 1E-9) continue;
			const double Distance = E2.Dot(Q) / Determinant;
			if(Distance > 0 && Distance < Nearest) {
				Nearest = Distance;
				if(TriangleIndex) *TriangleIndex = Index;
			}
		}
		// 候補を生成した標本とは独立な交点の返戻
		return Nearest;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetProjectedViewContracts, "PlanetGenerator.SurfaceView.ProjectedDistance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 全球一律の高度余裕と実三角形の範囲で，遠景の独立な標高誤差を比較する
 * @param Parameters 未使用の検査引数
 * @return 全条件が成功した場合に真
 */
bool FPlanetProjectedViewContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const double Started = FPlatformTime::Seconds();
	PlanetTests::LodCheckReport Topology;
	PlanetCore::LodView Extreme;
	const PlanetCore::SurfaceHeightEvaluator Overflow = [](const PlanetCore::UnitDirection Direction) -> double {
		static_cast<void>(Direction);
		// 有限な標高でも投影計算が溢れる入力の返戻
		return std::numeric_limits<double>::max();
	};
	const bool Rejected = !PlanetCore::SelectLodForView(Extreme, {}, Overflow).has_value();
	// 非有限な優先度を持つ分割が公開された場合の返戻
	if(!TestTrue(TEXT("Overflowing projected geometry is rejected"), Rejected)) return false;
	for(const double Radius : { 1000000.0, 6371000.0 }) {
		const PlanetCore::UnitDirection Anchor = *PlanetCore::UnitDirection::FromVector({ 1, 2, 3 });
		const PlanetCore::GroundFrame Frame = *PlanetCore::GroundFrame::Create(Anchor, Radius, 1000);
		const PlanetCore::SurfaceHeightEvaluator Height = [&](const PlanetCore::UnitDirection Direction) -> double {
			const PlanetCore::Vec3 Point = Direction.Vector();
			const double X = Radius * std::atan2(Point.Dot(Frame.East), Point.Dot(Frame.Up));
			const double Y = Radius * std::atan2(Point.Dot(Frame.North), Point.Dot(Frame.Up));
			const double Q = std::pow((X - 5000) / 1200, 2) + std::pow(Y / 2000, 2);
			// 滑らかな山体の解析標高の返戻
			return 1000 + 500 * std::exp(-Q);
		};
		PlanetCore::LodView View;
		View.RadiusMeters = Radius;
		View.ReliefBoundMeters = 40000;
		View.ObserverMeters = Anchor.Vector() * (Radius + Height(Anchor) + 1.65);
		View.LeafBudget = 8192;
		View.PixelLeafBudget = 8192;
		const std::optional<PlanetCore::LodSelection> Global = PlanetCore::SelectLodForView(View);
		const std::optional<PlanetCore::LodSelection> Local = PlanetCore::SelectLodForView(View, {}, Height);
		// どちらかの完全分割が失われた場合の返戻
		if(!TestTrue(TEXT("Both ranking paths retain complete candidates"), Global.has_value() && Local.has_value())) return false;
		std::array<double, 2> Errors {};
		for(std::size_t Mode = 0; Mode < Errors.size(); ++Mode) {
			const PlanetCore::LodSelection &Selection = Mode ? *Local : *Global;
			// 共通の閉じた境界を壊した場合の返戻
			if(!PlanetTests::CheckPartition(Selection.Tree, Topology) || !PlanetTests::CheckMesh(Selection.Tree, Topology)) {
				AddError(UTF8_TO_TCHAR(Topology.Failure.c_str()));
				// 位相不整合を地形の誤差比較へ進めない返戻
				return false;
			}
			const std::optional<PlanetCore::AdaptiveSphereMesh> Adaptive = PlanetCore::BuildAdaptiveSphere(Selection.Tree);
			// 描画形状が得られない場合の返戻
			if(!TestTrue(TEXT("Shared render topology exists"), Adaptive.has_value())) return false;
			const PlanetCore::SphereMesh &Mesh = Adaptive->Mesh;
			std::vector<PlanetCore::Vec3> Positions;
			Positions.reserve(Mesh.Directions.size());
			for(const PlanetCore::UnitDirection Direction : Mesh.Directions) {
				Positions.push_back(Direction.Vector() * (Radius + Height(Direction)));
			}
			for(unsigned X = 0; X < 9; ++X) for(unsigned Y = 0; Y < 9; ++Y) {
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(
					Anchor.Vector() * Radius + Frame.East * (3500 + X * 375) + Frame.North * (-1000.0 + Y * 250)
				);
				const double ExpectedRadius = Radius + Height(Direction), ActualRadius = RadialMeshDistance(Mesh, Positions, Direction);
				const double Distance = (Direction.Vector() * ExpectedRadius - View.ObserverMeters).Length();
				const double Pixels = std::abs(ActualRadius - ExpectedRadius) * View.FocalLengthPixels / Distance;
				// 欠落又は非有限な描画標高を除外しない返戻
				if(!TestTrue(TEXT("Independent mountain ray is covered"), std::isfinite(Pixels))) return false;
				Errors[Mode] = std::max(Errors[Mode], Pixels);
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("ProjectedDistance radius=%.0f globalError=%.9g localError=%.9g pixels localLeaves=%llu localHeights=%llu"),
				Radius,
				Errors[0],
				Errors[1],
				static_cast<unsigned long long>(Local->Tree.Leaves().size()),
				static_cast<unsigned long long>(Local->SurfaceHeightEvaluations)
			)
		);
		// 全球の高さ上限に依る平坦化の回帰を検出した場合の返戻
		if(!TestTrue(TEXT("Local terrain bounds improve independent skyline probes"), Errors[1] < Errors[0] * 0.25)) return false;
		// 同じ葉数上限で山体の投影誤差が残る場合の返戻
		if(!TestTrue(TEXT("Independent mountain probes stay within two pixels"), Errors[1] < 2.0)) return false;
	}
	AddInfo(
		FString::Printf(
			TEXT("ProjectedDistance independentRays=324 topologyChecks=%llu seconds=%.3f"),
			static_cast<unsigned long long>(Topology.Checks),
			FPlatformTime::Seconds() - Started
		)
	);
	// 独立な標高と完全被覆を確認した結果の返戻
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetExplorationDistanceContracts, "PlanetGenerator.SurfaceView.ExplorationDistance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 通常の侵食済地形の２視点で，山体の独立な放射線と実接合面を照合する
 * @param Parameters 未使用の検査引数
 * @return 全視点の遠景精度と近景の必須条件を満たす場合に真
 */
bool FPlanetExplorationDistanceContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const double Started = FPlatformTime::Seconds();
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	// 本番と同じ地質が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Exploration geology prepared"), Geology.has_value())) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 本番と同じ海域が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Exploration ocean prepared"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 本番と同じ排水が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Exploration drainage prepared"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage));
	std::optional<PlanetCore::RiverTerrainField> Prepared = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 通常探索と同じ侵食済地形が準備出来ない場合の返戻
	if(!TestTrue(TEXT("Exploration eroded terrain prepared"), Prepared.has_value())) return false;
	const std::shared_ptr<const PlanetCore::RiverTerrainField> Field =
	std::make_shared<const PlanetCore::RiverTerrainField>(std::move(*Prepared));
	const PlanetCore::SurfaceHeightEvaluator Height = [Field](const PlanetCore::UnitDirection Direction) -> double {
		// 視点によって変更しない同じ元標高の返戻
		return Field->EvaluateElevation(Direction);
	};
	const std::array<PlanetCore::Vec3, 2> Eyes {
		PlanetCore::Vec3{ 965462.9397112095, 14392.978549244528, -263408.5046324193 },
		PlanetCore::Vec3{ 965473.8256302007, 14393.140834912972, -263379.5751107575 }
	};
	const std::array<PlanetCore::Vec3, 2> Forwards {
		PlanetCore::Vec3{ 0.26315435313004654, 0.003923066131531503, 0.9647457675371448 },
		PlanetCore::Vec3{ 0.26312469173781594, 0.0039226239446275395, 0.9647538596031988 }
	};
	PlanetCore::LodSurfaceCache Cache(Settings.RadiusMeters, Height);
	bool Accurate = true;
	unsigned Samples = 0;
	for(std::size_t Site = 0; Site < Eyes.size(); ++Site) {
		const PlanetCore::UnitDirection Up = *PlanetCore::UnitDirection::FromVector(Eyes[Site]);
		const PlanetCore::Vec3 Forward = Forwards[Site], Right = Up.Vector().Cross(Forward);
		const PlanetCore::Vec3 Body = Eyes[Site] - Up.Vector() * 0.64;
		const std::optional<PlanetCore::StreamingFocus> Focus =
		PlanetCore::PredictStreamingFocus(Body, {}, Forward, Settings.RadiusMeters, 1.5, 9);
		// 身体中心の近景通路が準備出来ない場合の返戻
		if(!TestTrue(TEXT("Exploration focus prepared"), Focus.has_value())) return false;
		PlanetCore::LodView View;
		View.ObserverMeters = Eyes[Site];
		View.RadiusMeters = Settings.RadiusMeters;
		View.ReliefBoundMeters = Field->AbsoluteReliefBoundMeters();
		View.LeafBudget = 32768;
		View.PixelLeafBudget = 8192;
		View.SurfaceObserverMeters = Focus->StartMeters;
		View.SurfacePathEndMeters = Focus->EndMeters;
		View.SurfaceRadiusMeters = Focus->RadiusMeters;
		View.FoliageRadiusMeters = 64;
		const std::optional<PlanetCore::LodSelection> Selection = PlanetCore::SelectLodForView(View, {}, {}, &Cache);
		const bool Ready = Selection && Selection->SurfaceResolutionSatisfied && Selection->FoliageResolutionSatisfied;
		// 遠景の為に足元と植生の必須精度を損ねた場合の返戻
		if(!TestTrue(TEXT("Exploration near terrain remains valid"), Ready)) return false;
		const std::optional<PlanetCore::AdaptiveSphereMesh> Adaptive = PlanetCore::BuildAdaptiveSphere(Selection->Tree);
		// 描画用の接合面が準備出来ない場合の返戻
		if(!TestTrue(TEXT("Exploration adaptive mesh exists"), Adaptive.has_value())) return false;
		const PlanetCore::SphereMesh &Mesh = Adaptive->Mesh;
		std::vector<PlanetCore::Vec3> Positions;
		Positions.reserve(Mesh.Directions.size());
		for(const PlanetCore::UnitDirection Direction : Mesh.Directions) {
			Positions.push_back(Direction.Vector() * (View.RadiusMeters + Height(Direction)));
		}
		double MaximumPixels = 0, MaximumMeters = 0, WorstDistance = 0, WorstLateral = 0;
		PlanetCore::UnitDirection WorstDirection = Up;
		std::size_t WorstTriangle = 0;
		for(const double Distance : { 500.0, 1000.0, 2000.0, 3000.0, 5000.0, 7500.0, 10000.0 }) {
			for(const double Lateral : { -0.1, 0.0, 0.1 }) {
				const PlanetCore::UnitDirection Direction =
				*PlanetCore::UnitDirection::FromVector(Up.Vector() * View.RadiusMeters + Forward * Distance + Right * (Distance * Lateral));
				std::size_t Triangle = 0;
				const double Radius = RadialMeshDistance(Mesh, Positions, Direction, &Triangle);
				const double Expected = View.RadiusMeters + Height(Direction), Meters = std::abs(Radius - Expected);
				const double Pixels = Meters * View.FocalLengthPixels / (Direction.Vector() * Expected - Eyes[Site]).Length();
				// 元場とは独立な放射線が面を見失った場合の返戻
				if(!TestTrue(TEXT("Exploration independent ray is covered"), std::isfinite(Pixels))) return false;
				if(Pixels > MaximumPixels) {
					MaximumPixels = Pixels;
					MaximumMeters = Meters;
					WorstDistance = Distance;
					WorstLateral = Lateral;
					WorstDirection = Direction;
					WorstTriangle = Triangle;
				}
				++Samples;
			}
		}
		const PlanetCore::TileId Owner = Adaptive->TriangleOwners[WorstTriangle];
		unsigned Mask = 0;
		for(std::uint8_t Edge = 0; Edge < 3; ++Edge) if(Selection->Tree.HasFinerNeighbor(Owner, Edge)) Mask |= 1U << Edge;
		PlanetCore::SphereMesh Parent;
		Parent.Triangles.push_back({ 0, 1, 2 });
		std::vector<PlanetCore::Vec3> ParentPositions;
		for(std::uint8_t Corner = 0; Corner < 3; ++Corner) {
			const PlanetCore::UnitDirection Direction = *PlanetCore::GetDirection(*PlanetCore::GetVertexKey(Owner, Corner));
			ParentPositions.push_back(Direction.Vector() * (View.RadiusMeters + Height(Direction)));
		}
		const double Expected = View.RadiusMeters + Height(WorstDirection);
		const double Scale = View.FocalLengthPixels / (WorstDirection.Vector() * Expected - Eyes[Site]).Length();
		const double ParentError = std::abs(RadialMeshDistance(Parent, ParentPositions, WorstDirection) - Expected) * Scale;
		double ParentProbePixels = 0;
		for(
			const PlanetCore::Vec3 Position :
			{
				(ParentPositions[0] + ParentPositions[1]) * 0.5,
				(ParentPositions[1] + ParentPositions[2]) * 0.5,
				(ParentPositions[2] + ParentPositions[0]) * 0.5,
				(ParentPositions[0] + ParentPositions[1] + ParentPositions[2]) * (1.0 / 3)
			}
		) {
			const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Position);
			const double Error = std::abs(View.RadiusMeters + Height(Direction) - Position.Length());
			ParentProbePixels = std::max(ParentProbePixels, Error * Scale);
		}
		AddInfo(
			FString::Printf(
				TEXT("ExplorationDistance site=%llu errorPx=%.9g errorM=%.9g distance=%.0f lateral=%.1f level=%u mask=%u ")
				TEXT("parentAtRayPx=%.9g parentProbePx=%.9g leaves=%llu heights=%llu"),
				static_cast<unsigned long long>(Site),
				MaximumPixels,
				MaximumMeters,
				WorstDistance,
				WorstLateral,
				static_cast<unsigned>(Owner.Level),
				Mask,
				ParentError,
				ParentProbePixels,
				static_cast<unsigned long long>(Selection->Tree.Leaves().size()),
				static_cast<unsigned long long>(Selection->SurfaceHeightEvaluations)
			)
		);
		Accurate &= TestTrue(TEXT("Independent exploration terrain stays within two pixels"), MaximumPixels < 2.0);
	}
	AddInfo(
		FString::Printf(TEXT("ExplorationDistance independentRays=%u seconds=%.3f"), Samples, FPlatformTime::Seconds() - Started)
	);
	// 全視点の遠景と足元を照合した結果の返戻
	return Accurate;
}

#endif
