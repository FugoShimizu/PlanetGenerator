#include "Core/PlanetFlowView.h"
#include "PlanetSphericalSurfacePreview.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 全球の辺対・向き・根面の被覆を，公開候補の三角形から独立に確認する
	 * @param Test 結果受取先
	 * @param Candidate 同じ場から生成した候補
	 * @return 閉じた完全被覆を満たした場合に真
	 */
	bool CheckClosedFlowMesh(FAutomationTestBase &Test, const PlanetCore::FlowTerrainMesh &Candidate) {
		const PlanetCore::SphereMesh &Mesh = Candidate.Sphere.Mesh;
		using Edge = std::pair<std::uint32_t, std::uint32_t>;
		std::map<Edge, std::pair<unsigned, int>> Edges;
		std::set<PlanetCore::TileId, PlanetCore::TileOrder> Owners;
		for(const std::array<std::uint32_t, 3> Triangle : Mesh.Triangles) for(unsigned K = 0; K < 3; ++K) {
			const std::uint32_t A = Triangle[K], B = Triangle[(K + 1) % 3];
			// 不正な頂点番号の返戻
			if(!Test.TestTrue(TEXT("出力三角形の頂点が有効"), A < Mesh.Directions.size() && B < Mesh.Directions.size() && A != B)) return false;
			std::pair<unsigned, int> &Count = Edges[{ std::min(A, B), std::max(A, B) }];
			++Count.first;
			Count.second += A < B ? 1 : -1;
		}
		for(const std::pair<const Edge, std::pair<unsigned, int>> &Entry : Edges) {
			// 穴又は同じ向きの重複面を拒否する返戻
			if(!Test.TestTrue(TEXT("全ての辺が逆向きの二面に所属"), Entry.second.first == 2 && !Entry.second.second)) return false;
		}
		Test.TestEqual(
			TEXT("閉じた球面の Euler 特性"),
			static_cast<int64>(Mesh.Directions.size()) - static_cast<int64>(Edges.size()) + static_cast<int64>(Mesh.Triangles.size()),
			static_cast<int64>(2)
		);
		for(const PlanetCore::TileId Owner : Candidate.Sphere.TriangleOwners) Owners.insert(Owner);
		std::array<double, 20> Areas {};
		for(const PlanetCore::TileId Owner : Owners) Areas[Owner.RootFace] += std::ldexp(1.0, -2 * Owner.Level);
		for(const double Area : Areas) Test.TestTrue(TEXT("二十根面を一重に被覆"), std::abs(Area - 1) < 1E-12);
		// 完全な位相の独立検査結果の返戻
		return !Test.HasAnyErrors();
	}

	/**
	 * 生成時と異なる内部点で，単精度にした実頂点と元場を照合する
	 * @param Test 結果受取先
	 * @param Candidate 全球の実候補面
	 * @param Field 描画・衝突が所有する同じ不変場
	 * @param Radius 精細範囲の半径
	 * @return 独立標本で二センチメートル精度を満たした場合に真
	 */
	bool CheckFlowInterpolation(
		FAutomationTestBase &Test,
		const PlanetCore::FlowTerrainMesh &Candidate,
		const PlanetCore::FlowChannelField &Field,
		const double Radius
	) {
		const PlanetCore::GroundFrame &Frame = Candidate.Frame;
		const double R0 = Frame.RadiusMeters + Frame.AnchorElevationMeters;
		const auto FloatPosition = [](const PlanetCore::Vec3 P) -> PlanetCore::Vec3 {
			// UE が受け取るセンチメートル単精度への丸めの返戻
			return {
				static_cast<double>(static_cast<float>(P.X * 100)) / 100,
				static_cast<double>(static_cast<float>(P.Y * 100)) / 100,
				static_cast<double>(static_cast<float>(P.Z * 100)) / 100
			};
		};
		unsigned Samples = 0;
		double Maximum = 0;
		for(const std::array<std::uint32_t, 3> Triangle : Candidate.Sphere.Mesh.Triangles) {
			const PlanetCore::Vec3 A = FloatPosition(Candidate.Positions[Triangle[0]]), B = FloatPosition(Candidate.Positions[Triangle[1]]);
			const PlanetCore::Vec3 C = FloatPosition(Candidate.Positions[Triangle[2]]), Center = (A + B + C) * (1.0 / 3);
			const double Cap = std::max({ (A - Center).Length(), (B - Center).Length(), (C - Center).Length() });
			if(Center.Length() > Radius + Cap + 8) continue;
			for(const PlanetCore::Vec3 P : { A * 0.2 + B * 0.3 + C * 0.5, A * 0.6 + B * 0.1 + C * 0.3, A * 0.15 + B * 0.7 + C * 0.15 }) {
				const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(P));
				if((Direction.Vector() - Frame.Up).Length() * Frame.RadiusMeters > Radius) continue;
				const double Increment = 2 * R0 * P.Z + P.Dot(P);
				const double Interpolated = Frame.AnchorElevationMeters + Increment / (std::sqrt(R0 * R0 + Increment) + R0);
				const std::optional<PlanetCore::FlowChannelSection> Expected = Field.Evaluate(Direction, false, false);
				// 独立な照会の失敗を標本除外にしない返戻
				if(!Test.TestTrue(TEXT("近景の独立標本が定義"), Expected.has_value())) return false;
				Maximum = std::max(Maximum, std::abs(Interpolated - Expected->Terrain.ElevationMeters));
				++Samples;
			}
		}
		Test.TestTrue(TEXT("独立に千地点以上の近景を検査"), Samples > 1000);
		Test.TestTrue(TEXT("独立な実頂点の内部点も二センチメートル以内"), Maximum < 0.02);
		Test.AddInfo(
			FString::Printf(
				TEXT("Global flow mesh: vertices=%llu triangles=%llu independent=%u maximumError=%.12g"),
				static_cast<unsigned long long>(Candidate.Positions.size()),
				static_cast<unsigned long long>(Candidate.Sphere.Mesh.Triangles.size()),
				Samples,
				Maximum
			)
		);
		// 独立な描画精度検査の結果の返戻
		return !Test.HasAnyErrors();
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowViewContracts,
	"PlanetGenerator.FlowChannel.GlobalView",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実地形の河床を全球の共通分割へ載せ，視点移動と拒否後の再利用を確認する
 * @param Parameters 未使用の実行引数
 * @return 完全な位相と独立地点の精度を確認した場合に真
 */
bool FPlanetFlowViewContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	UWorld *World = nullptr;
	if(GEngine) {
		for(const FWorldContext &Context : GEngine->GetWorldContexts()) {
			if(Context.WorldType == EWorldType::Editor && Context.World()) World = Context.World();
		}
	}
	// エディタの検査世界が無い場合の返戻
	if(!TestNotNull(TEXT("検査用の世界"), World)) return false;
	FActorSpawnParameters Spawn;
	Spawn.ObjectFlags |= RF_Transient;
	const auto Destroy = [](APlanetSphericalSurfacePreview *const Actor) -> void {
		if(IsValid(Actor)) Actor->Destroy();
		// 所有する一時 Actor だけを解放する終了
		return;
	};
	const std::unique_ptr<APlanetSphericalSurfacePreview, decltype(Destroy)> Actor(
		World->SpawnActor<APlanetSphericalSurfacePreview>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn),
		Destroy
	);
	// 一時アクターを作れない場合の返戻
	if(!TestNotNull(TEXT("元場の準備入口"), Actor.get())) return false;
	const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
	const bool Prepared = Actor->BuildPreview(42, 1E6, Pass, 200, 16, false, false, false, true) &&
	Actor->BuildConnectedFlowChannelPreview(Pass, -100, 32, 32);
	// 部分的な元場を描画しない返戻
	if(!TestTrue(TEXT("大洋へ繋がる水系の河床を準備"), Prepared)) return false;
	const std::shared_ptr<const PlanetCore::FlowChannelField> Field = Actor->GetFlowField();
	const std::unique_ptr<PlanetCore::FlowTerrainView> Builder = PlanetCore::FlowTerrainView::Create(Field);
	// 正規の共有場への束縛を確認する返戻
	if(!TestTrue(TEXT("同じ不変場で全球描画を準備"), Builder && Builder->Uses(Field))) return false;
	TestFalse(TEXT("元場無しの描画準備を拒否"), static_cast<bool>(PlanetCore::FlowTerrainView::Create({})));
	const FVector Initial = Actor->GetDiagnosticAnchorDirection();
	const PlanetCore::UnitDirection Anchor = *PlanetCore::UnitDirection::FromVector({ Initial.X, Initial.Y, Initial.Z });
	const PlanetCore::GroundFrame Frame = *PlanetCore::GroundFrame::Create(Anchor, 1E6, Actor->AnchorElevationMeters);
	PlanetCore::LodView View;
	View.RadiusMeters = 1E6;
	View.ObserverMeters = Anchor.Vector() * (1E6 + Actor->AnchorElevationMeters + 1.6);
	View.SurfaceRadiusMeters = 24;
	View.LeafBudget = 32768;
	View.PixelLeafBudget = 2048;
	View.TargetEdgePixels = 64;
	PlanetCore::FlowViewDiagnostics Report;
	const double Started = FPlatformTime::Seconds();
	const std::optional<PlanetCore::FlowTerrainMesh> First = Builder->Build(View, {}, &Report);
	AddInfo(
		FString::Printf(
			TEXT("Flow view first: status=%u leaves=%llu heightCalls=%llu cacheHits=%llu probes=%llu ")
			TEXT("selectionError=%.12g floatError=%.12g renderError=%.12g seconds=%.9g"),
			static_cast<unsigned>(Report.Failure),
			static_cast<unsigned long long>(Report.Leaves),
			static_cast<unsigned long long>(Report.HeightEvaluations),
			static_cast<unsigned long long>(Report.ProbeCacheHits),
			static_cast<unsigned long long>(Report.ValidationSamples),
			Report.MaximumSelectionErrorMeters,
			Report.MaximumFloatErrorMeters,
			Report.MaximumSurfaceErrorMeters,
			FPlatformTime::Seconds() - Started
		)
	);
	// 初回の近景を公開出来ない場合の返戻
	if(!TestTrue(TEXT("全球と近景が同じ候補で成立"), First.has_value())) return false;
	// 不正な描画候補への返戻
	if(!CheckClosedFlowMesh(*this, *First) || !CheckFlowInterpolation(*this, *First, *Field, 24)) return false;
	const std::uint64_t InitialEvaluations = Report.HeightEvaluations;
	PlanetCore::LodView Wrong = View;
	Wrong.RadiusMeters = 1E9;
	TestFalse(TEXT("別の半径の標高再利用を拒否"), Builder->Build(Wrong, {}, &Report).has_value());
	TestTrue(TEXT("半径不一致の拒否理由"), Report.Failure == PlanetCore::FlowViewFailure::InvalidView);
	Wrong = View;
	Wrong.LeafBudget = 20;
	TestFalse(TEXT("精度未達の予算では候補を返さない"), Builder->Build(Wrong, {}, &Report).has_value());
	TestTrue(TEXT("完全被覆でも精度不足を区別"), Report.Failure == PlanetCore::FlowViewFailure::Resolution);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断済の描画を拒否"), Builder->Build(View, Cancel.get_token(), &Report).has_value());
	TestTrue(TEXT("中断理由を保持"), Report.Failure == PlanetCore::FlowViewFailure::Cancelled);
	const std::optional<PlanetCore::FlowTerrainMesh> Repeat = Builder->Build(View, {}, &Report);
	// 拒否後も正規の生成経路が使える事を確認する返戻
	if(!TestTrue(TEXT("拒否後も有効な同じ視点を生成"), Repeat.has_value())) return false;
	TestTrue(TEXT("標高の再利用で同じ位相"), Repeat->Sphere.Mesh.Triangles == First->Sphere.Mesh.Triangles);
	TestTrue(TEXT("標高の再利用で評価回数を削減"), Report.HeightEvaluations < InitialEvaluations);
	TestEqual(TEXT("再利用後も同じ頂点数"), Repeat->Positions.size(), First->Positions.size());
	for(std::size_t I = 0; I < std::min(Repeat->Positions.size(), First->Positions.size()); ++I) {
		TestEqual(TEXT("再利用後の局所頂点が完全一致"), (Repeat->Positions[I] - First->Positions[I]).Length(), 0.0);
	}
	const PlanetCore::UnitDirection Moved = Frame.DirectionAt(10, 3);
	const double MovedHeight = Field->Evaluate(Moved, false, false)->Terrain.ElevationMeters;
	View.SurfaceObserverMeters = Moved.Vector() * (1E6 + MovedHeight + 1);
	View.ObserverMeters = Anchor.Vector() * (1E6 + Actor->AnchorElevationMeters + 500);
	const std::optional<PlanetCore::FlowTerrainMesh> Next = Builder->Build(View, {}, &Report);
	// カメラと身体が別の位置に有る場合の返戻
	if(!TestTrue(TEXT("俯瞰中も移動した身体の近景を予約"), Next.has_value())) return false;
	CheckClosedFlowMesh(*this, *Next);
	CheckFlowInterpolation(*this, *Next, *Field, 24);
	// 検査する点を減らさず，同じ端点対の元場照会だけを共有する事を確認
	TestTrue(TEXT("共有辺の中点標高を同一候補内で再利用"), Report.ValidationReuses > 0);
	TestEqual(TEXT("全ての精度検査点を実評価又は同一点の再利用で説明"), Report.ValidationSamples, Report.ValidationEvaluations + Report.ValidationReuses);
	AddInfo(
		FString::Printf(
			TEXT("Flow validation: probes=%llu evaluations=%llu sharedEdges=%llu"),
			static_cast<unsigned long long>(Report.ValidationSamples),
			static_cast<unsigned long long>(Report.ValidationEvaluations),
			static_cast<unsigned long long>(Report.ValidationReuses)
		)
	);
	TestEqual(TEXT("視点変更で河道を再準備しない"), Actor->DiagnosticFlowPreparations, static_cast<int64>(1));
	AddInfo(
		FString::Printf(
			TEXT("Flow view moved: leaves=%llu heightCalls=%llu cacheHits=%llu probes=%llu renderError=%.12g"),
			static_cast<unsigned long long>(Report.Leaves),
			static_cast<unsigned long long>(Report.HeightEvaluations),
			static_cast<unsigned long long>(Report.ProbeCacheHits),
			static_cast<unsigned long long>(Report.ValidationSamples),
			Report.MaximumSurfaceErrorMeters
		)
	);
	// 独立な位相・精度・移動と拒否後の回復を併せた検査結果の返戻
	return !HasAnyErrors();
}

#endif
