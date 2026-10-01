#include "Misc/AutomationTest.h"
#include "Core/PlanetGround.h"
#include "PlanetGroundPatch.h"
#include "PlanetSphericalSurfacePreview.h"
#include "PlanetTerrain.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "ProceduralMeshComponent.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/** 実衝突の調理をフレーム間で待ち，所有する一時アクターだけを片付ける検査 */
	class FFlowCollisionCommand final : public IAutomationLatentCommand {
	private:

		// 検査の所有物と有界な待機状態
		FAutomationTestBase &Test;
		UWorld *World;
		bool OwnsWorld = false;
		APlanetSphericalSurfacePreview *Preview = nullptr;
		APlanetGroundPatch *Ground = nullptr;
		APlanetTerrain *Terrain = nullptr;
		unsigned Stage = 0;
		double Started = 0;
		int64 PriorRevision = 0;
		FVector PriorStart = FVector::ZeroVector;
		FVector PriorEnd = FVector::ZeroVector;
		FVector PriorImpact = FVector::ZeroVector;
		unsigned RetainedFrames = 0;
		void Cleanup();
		bool CheckFlow();
		bool CheckOrdinary();

	public:

		virtual bool Update() override;

		FFlowCollisionCommand(FAutomationTestBase &Owner, UWorld *const TestWorld, const bool OwnsTestWorld=false);
		virtual ~FFlowCollisionCommand() override;
	};

	/**
	 * 所有する一時アクターを破棄し，検査世界の既存アクターを保持する
	 */
	void FFlowCollisionCommand::Cleanup() {
		// 所有する非同期作業と一時物の解放
		if(IsValid(Ground)) Ground->Destroy();
		if(IsValid(Preview)) Preview->Destroy();
		if(IsValid(Terrain)) Terrain->Destroy();
		Ground = nullptr;
		Preview = nullptr;
		Terrain = nullptr;
		if(OwnsWorld && World) {
			// アクターの破棄が参照する登録を，世界の解放迄保持
			World->DestroyWorld(false);
			if(GEngine) {
				GEngine->DestroyWorldContext(World);
				Test.TestNull(TEXT("所有する世界の登録を解放"), GEngine->GetWorldContextFromWorld(World));
			}
			Test.TestFalse(TEXT("所有する世界を根参照から解放"), World->IsRooted());
			World = nullptr;
			OwnsWorld = false;
		}
		// 終了
		return;
	}

	/**
	 * 公開済描画と実衝突を，同じ球面方向の独立な線分で比較する
	 * 最悪 O(SN)，S は照会数，N は描画の三角形数
	 * @return 全地点で有限な被覆・実線分・身体掃引と２センチメートル精度を満たす場合に真
	 */
	bool FFlowCollisionCommand::CheckFlow() {
		// 描画側の原点と実際に公開された三角形の取得
		const FVector Anchor = Preview->GetDiagnosticAnchorDirection();
		const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector({ Anchor.X, Anchor.Y, Anchor.Z });
		const PlanetCore::GroundFrame Frame = *PlanetCore::GroundFrame::Create(Direction, 1E6, Preview->AnchorElevationMeters);
		TArray<UProceduralMeshComponent *> Components;
		Preview->GetComponents(Components);
		FProcMeshSection *Section = nullptr;
		for(UProceduralMeshComponent *const Component : Components) {
			if(Component->GetFName() == TEXT("SphericalSurfaceDiagnostic")) Section = Component->GetProcMeshSection(0);
		}
		// 公開面の欠落を衝突だけの成功にしない返戻
		if(!Test.TestNotNull(TEXT("河床の公開済描画面"), Section)) return false;
		const auto Vector = [](const PlanetCore::Vec3 Point) -> FVector {
			return { Point.X, Point.Y, Point.Z };
		};
		FTransform PublishedTransform;
		double PublishedRadius = 0;
		Test.TestTrue(TEXT("衝突と同じ惑星定義を取得"), Ground->GetPublishedPlanet(PublishedTransform, PublishedRadius));
		Test.TestEqual(TEXT("同じ公開半径"), PublishedRadius, 1E6);
		double MaximumGroundError = 0, MaximumRenderGap = 0;
		unsigned Rays = 0, Capsules = 0;
		for(unsigned i = 0; i < 9; ++i) for(unsigned j = 0; j < 9; ++j) {
			const PlanetCore::UnitDirection Query = Frame.DirectionAt(-13.7 + 3.4 * i, -13.6 + 3.3 * j);
			const PlanetCore::Vec3 Q = Query.Vector();
			double Height = 0, Water = 0, Coverage = 0, Original = 0;
			if(!Test.TestTrue(TEXT("独立方向の共有場評価"), Preview->EvaluateFlowDirection(Vector(Q), Height, Water, Coverage, Original))) {
				// 標本を選別して失敗を隠さない返戻
				return false;
			}
			const FVector Local = 100 * Vector(Frame.SurfaceToLocal(Query, Height));
			const FVector Up = Preview->GetActorTransform().TransformVectorNoScale(Vector(Frame.VectorToLocal(Q)));
			const FVector Expected = Preview->GetActorTransform().TransformPosition(Local);
			double GroundHeight = 0;
			Test.TestTrue(TEXT("衝突と同じ公開標高を取得"), Ground->EvaluatePublishedElevation(Vector(Q), GroundHeight));
			Test.TestEqual(TEXT("別の公開入口も同じ河床標高"), GroundHeight, Height);
			const FVector FromPublished = PublishedTransform.TransformPosition(100 * (PublishedRadius + GroundHeight) * Vector(Q));
			Test.TestTrue(TEXT("身体が使う座標と描画の座標が一致"), FVector::Distance(FromPublished, Expected) < 1E-4);
			const FVector Start = Expected + 1000 * Up, End = Expected - 1000 * Up;
			FVector Impact, Normal;
			double Margin = 0;
			Test.TestTrue(TEXT("同じ河床の足場被覆"), Ground->QueryCoverage(Expected, 0.35, Margin) && Margin > 0);
			// 元の線分をずらさず，同じ実衝突を比較する返戻
			if(!Test.TestTrue(TEXT("共有河床の点線分命中"), Ground->TraceGround(Start, End, Impact, Normal))) return false;
			MaximumGroundError = std::max(MaximumGroundError, FVector::Distance(Impact, Expected) * 0.01);
			++Rays;
			if(!i && !j) {
				PriorStart = Start;
				PriorEnd = End;
				PriorImpact = Impact;
			}
			// 描画へ渡す単精度頂点を，物理面と異なる全三角形探索で確認
			const FVector LocalStart = Preview->GetActorTransform().InverseTransformPosition(Start);
			const FVector LocalEnd = Preview->GetActorTransform().InverseTransformPosition(End);
			bool HasRenderHit = false;
			FVector RenderPoint;
			for(int32 Triangle = 0; Triangle < Section->ProcIndexBuffer.Num(); Triangle += 3) {
				std::array<FVector, 3> Points;
				for(unsigned Corner = 0; Corner < 3; ++Corner) {
					const FVector P = Section->ProcVertexBuffer[Section->ProcIndexBuffer[Triangle + Corner]].Position;
					Points[Corner] = FVector(static_cast<float>(P.X), static_cast<float>(P.Y), static_cast<float>(P.Z));
				}
				FVector RenderNormal;
				if(FMath::SegmentTriangleIntersection(LocalStart, LocalEnd, Points[0], Points[1], Points[2], RenderPoint, RenderNormal)) {
					HasRenderHit = true;
					break;
				}
			}
			// 描画穴を河床への衝突成功で代替しない返戻
			if(!Test.TestTrue(TEXT("単精度の描画面にも同じ線分が命中"), HasRenderHit)) return false;
			MaximumRenderGap =
			std::max(MaximumRenderGap, FVector::Distance(Impact, Preview->GetActorTransform().TransformPosition(RenderPoint)) * 0.01);
			if(!(i % 2) && !(j % 2)) {
				Test.TestTrue(TEXT("同じ足場へ身体寸法の掃引が命中"), Ground->SweepGroundCapsule(Start, End, Up, 35, 88, Impact, Normal));
				++Capsules;
			}
		}
		Test.TestTrue(TEXT("実衝突の河床誤差２センチメートル以内"), MaximumGroundError < 0.02);
		Test.TestTrue(TEXT("描画と実衝突の差２センチメートル以内"), MaximumRenderGap < 0.02);
		Test.AddInfo(
			FString::Printf(
				TEXT("Flow collision: gameWorld=%d stage=%u rays=%u capsules=%u groundError=%.12g renderGap=%.12g workerMs=%.9g"),
				static_cast<int>(World->IsGameWorld()),
				Stage,
				Rays,
				Capsules,
				MaximumGroundError,
				MaximumRenderGap,
				Ground->LastWorkerMilliseconds
			)
		);
		// 同じ元場から別々に生成した描画・衝突の照合結果の返戻
		return !Test.HasAnyErrors();
	}

	/**
	 * 共有河道から従来の惑星場へ切替えた後の実衝突を確認する
	 * @return 既存の公開場と足場が同じ高さを使う場合に真
	 */
	bool FFlowCollisionCommand::CheckOrdinary() {
		// 従来の正規の地点評価と半径方向線分の照合
		double Height = 0;
		FVector Normal, Impact;
		// 既存経路の評価欠落への返戻
		if(!Test.TestTrue(TEXT("従来の公開地形を評価"), Terrain->EvaluateDirection(FVector::UpVector, Height, Normal))) return false;
		const FVector Expected(0, 0, 100 * (Terrain->RadiusMeters + Height));
		double Margin = 0;
		Test.TestTrue(TEXT("従来の被覆経路が継続"), Ground->QueryCoverage(Expected, 0.35, Margin) && Margin > 0);
		Test.TestTrue(
			TEXT("従来の衝突生成も継続"),
			Ground->TraceGround(Expected + FVector(0, 0, 1000), Expected - FVector(0, 0, 1000), Impact, Normal)
		);
		const double Error = FVector::Distance(Expected, Impact) * 0.01;
		Test.TestTrue(TEXT("従来の実衝突の精度を保持"), Error < 0.02);
		Test.AddInfo(
			FString::Printf(TEXT("Ordinary collision after shared flow: error=%.12g revision=%lld"), Error, Ground->PublishedRevision)
		);
		// 正規経路への切替結果の返戻
		return !Test.HasAnyErrors();
	}

	/**
	 * 調理と生成をエンジンの更新間に待ち，成功・拒否・旧足場保持を順に確認する
	 * @return 検査終了時に真，待機中は偽；１２０秒の上限又は不成立は失敗を記録
	 */
	bool FFlowCollisionCommand::Update() {
		// 待機の上限と失敗時の確実な解放
		if(FPlatformTime::Seconds() - Started > 120 || Test.HasAnyErrors()) {
			if(!Test.HasAnyErrors()) Test.AddError(TEXT("衝突検査の待機上限；未完了を成功にしない"));
			Cleanup();
			// 失敗を記録して検査を終了する返戻
			return true;
		}
		if(!Stage) {
			// 同じ実地形の受入湖へ接続する河口の準備
			FActorSpawnParameters Spawn;
			Spawn.ObjectFlags |= RF_Transient;
			Preview = World->SpawnActor<APlanetSphericalSurfacePreview>(FVector(34567, -45678, 56789), FRotator(17, 31, -9), Spawn);
			Ground = World->SpawnActor<APlanetGroundPatch>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
			// 生成失敗の解放へ進む返戻
			if(!Test.TestNotNull(TEXT("一時描画の生成"), Preview) || !Test.TestNotNull(TEXT("一時足場の生成"), Ground)) return false;
			const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
			Test.TestTrue(TEXT("実際の元場を準備"), Preview->BuildPreview(42, 1E6, Pass, 200, 16, false, false, false, true));
			Test.TestTrue(TEXT("大洋へ接続する水系を準備"), Preview->BuildConnectedFlowChannelPreview(Pass, -100, 32, 32));
			Test.TestTrue(
				TEXT("実際の受入湖側の描画を準備"),
				Preview->BuildConnectedFlowChannelPreview(Pass, Preview->DiagnosticRouteEndMeters - 20, 32, 32)
			);
			Test.TestEqual(TEXT("河口へ視点を変えても同じ場を再利用"), Preview->DiagnosticFlowPreparations, INT64_C(1));
			Test.TestTrue(TEXT("共有河床の衝突要求を受理"), Preview->RequestFlowCollision(Ground));
			Stage = 1;
			// 非同期の候補を待つ返戻
			return false;
		}
		if(Stage == 3) {
			Terrain->PollGeneration();
			// 従来の準備完了を待つ返戻
			if(Terrain->bGenerationBusy) return false;
			Test.TestTrue(TEXT("従来の場も公開"), Terrain->PublishedRevision > 0);
			Ground->Terrain = Terrain;
			Ground->AnchorDirection = FVector::UpVector;
			Test.TestTrue(TEXT("共有河道から従来の足場へ切替"), Ground->RequestPatch());
			Stage = 4;
			// 同じ調理経路の完了を待つ返戻
			return false;
		}
		if(Stage == 2 && Ground->PublishedRevision == PriorRevision) {
			// 新候補の完了前に旧足場が実際に衝突する事の確認
			FVector Impact, Normal;
			Test.TestTrue(TEXT("更新待ちも旧線分が命中"), Ground->TraceGround(PriorStart, PriorEnd, Impact, Normal));
			Test.TestTrue(TEXT("更新待ちの旧足場は動かない"), FVector::Distance(Impact, PriorImpact) < 1E-5);
			++RetainedFrames;
		}
		Ground->PollPatch();
		// 調理済の候補だけを比較する返戻
		if(Ground->bBusy) return false;
		Test.TestEqual(TEXT("最新要求だけが公開"), Ground->PublishedRevision, Ground->RequestedRevision);
		if(Stage == 4) {
			CheckOrdinary();
			Cleanup();
			// 正規経路と一時物の検査完了の返戻
			return true;
		}
		CheckFlow();
		if(Stage == 1) {
			// 不正な入力が通番と公開済の足場を壊さない事の確認
			PriorRevision = Ground->PublishedRevision;
			Ground->HalfExtentMeters = -1;
			Test.TestFalse(TEXT("不正な寸法を拒否"), Preview->RequestFlowCollision(Ground));
			Ground->HalfExtentMeters = 16;
			Preview->SetActorScale3D(FVector(2));
			Test.TestFalse(TEXT("拡縮された描画を足場として拒否"), Preview->RequestFlowCollision(Ground));
			Preview->SetActorScale3D(FVector::OneVector);
			Test.TestEqual(TEXT("拒否時の要求通番を保持"), Ground->RequestedRevision, PriorRevision);
			Test.TestEqual(TEXT("拒否時の公開通番を保持"), Ground->PublishedRevision, PriorRevision);
			Ground->HalfExtentMeters = 12;
			Test.TestTrue(TEXT("次の候補を受理"), Preview->RequestFlowCollision(Ground));
			Ground->HalfExtentMeters = 16;
			Test.TestTrue(TEXT("身体位置から最新要求を受理"), Ground->RequestAtWorldPosition(Preview->GetActorLocation()));
			Stage = 2;
			// 旧足場を保持した最新要求の完了を待つ返戻
			return false;
		}
		// 既存の通常の惑星生成を同じ足場へ接続する準備
		Test.TestTrue(TEXT("更新待ちの旧足場を一フレーム以上確認"), RetainedFrames > 0);
		FActorSpawnParameters Spawn;
		Spawn.ObjectFlags |= RF_Transient;
		Terrain = World->SpawnActor<APlanetTerrain>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn);
		// 不成立は次の更新で確実に解放する返戻
		if(!Test.TestNotNull(TEXT("通常の地形を一時生成"), Terrain)) return false;
		Terrain->bFollowPlayerCamera = false;
		Terrain->bGenerateOnBeginPlay = false;
		Terrain->bRefineNearSurface = false;
		Terrain->LeafBudget = 128;
		Terrain->ObserverMeters = FVector(0, 0, 2E6);
		Test.TestTrue(TEXT("従来の正規の地形要求"), Terrain->RequestGeneration());
		Stage = 3;
		Test.AddInfo(FString::Printf(TEXT("Retained collision frames=%u"), RetainedFrames));
		// 既存の生成経路の準備を待つ返戻
		return false;
	}

	/**
	 * 検査世界と結果の受取先を束縛する
	 * @param Owner 実行中の自動検査
	 * @param TestWorld 一時アクターを所有する検査世界
	 * @param OwnsTestWorld 自身が生成したゲーム世界を終了時に解放する指定
	 */
	FFlowCollisionCommand::FFlowCollisionCommand(
		FAutomationTestBase &Owner,
		UWorld *const TestWorld,
		const bool OwnsTestWorld
	) : Test(Owner), World(TestWorld), OwnsWorld(OwnsTestWorld), Started(FPlatformTime::Seconds()) {}

	/**
	 * 検査中断時も所有する一時アクターを解放する
	 */
	FFlowCollisionCommand::~FFlowCollisionCommand() {
		Cleanup();
		// 終了
		return;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowCollisionContracts,
	"PlanetGenerator.FlowChannel.Collision",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ共有河床に対する実衝突と従来経路の非同期検査を開始する
 * @param Parameters 未使用の実行引数
 * @return エディタ世界に検査を登録出来た場合に真；合格は遅延検査の完了で決定
 */
bool FPlanetFlowCollisionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	if(GEngine) {
		for(const FWorldContext &Context : GEngine->GetWorldContexts()) if(Context.WorldType == EWorldType::Editor && Context.World()) {
			ADD_LATENT_AUTOMATION_COMMAND(FFlowCollisionCommand(*this, Context.World()));
			// 実衝突を待つ検査の登録結果の返戻
			return true;
		}
	}
	AddError(TEXT("検査対象のエディタ世界が存在しない"));
	// 正規の実行世界が無い場合の返戻
	return false;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowGameCollisionContracts,
	"PlanetGenerator.FlowChannel.GameCollision",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実行世界の非同期物理調理を，同じ描画・実衝突の照会で検査する
 * @param Parameters 未使用の実行引数
 * @return 所有する検査世界へ遅延検査を登録出来た場合に真
 */
bool FPlanetFlowGameCollisionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 世界の所有登録を使えない場合の返戻
	if(!TestNotNull(TEXT("検査世界を登録するエンジン"), GEngine)) return false;
	// 他の世界を変更しない，一時的なゲーム世界の準備
	UWorld *const World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, GetTransientPackage());
	// 物理世界の生成失敗への返戻
	if(!TestNotNull(TEXT("非同期の調理を使うゲーム世界を生成"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	TestNotNull(TEXT("生成と破棄に使う世界の登録"), GEngine->GetWorldContextFromWorld(World));
	TestTrue(TEXT("ゲーム世界の物理経路を使用"), World->IsGameWorld());
	ADD_LATENT_AUTOMATION_COMMAND(FFlowCollisionCommand(*this, World, true));
	// 完了と解放を遅延検査へ移譲した結果の返戻
	return true;
}

#endif
