#include "PlanetSphericalSurfacePreview.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"
#include "ProceduralMeshComponent.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <memory>
#include <optional>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 公開した二面の位置・法線・接線と接続を順序付きで照合する
	 * @param Actor 生成した一時地形
	 * @return 描画内容の固定ハッシュ
	 */
	std::optional<std::uint64_t> MeshHash(APlanetSphericalSurfacePreview &Actor) {
		// 順序付きハッシュと公開面の個数
		std::uint64_t Hash = UINT64_C(0XCBF29CE484222325);
		unsigned Found = 0;
		const auto Append = [&](const std::uint64_t Value) -> void {
			Hash = (Hash ^ Value) * UINT64_C(0X100000001B3);
		};
		const auto Vector = [&](const FVector V) -> void {
			Append(std::bit_cast<std::uint64_t>(V.X));
			Append(std::bit_cast<std::uint64_t>(V.Y));
			Append(std::bit_cast<std::uint64_t>(V.Z));
		};
		TArray<UProceduralMeshComponent *> Components;
		Actor.GetComponents(Components);
		// 地面・水面の順に依る描画内容の照合
		for(const FName Name : { FName(TEXT("SphericalSurfaceDiagnostic")), FName(TEXT("ContourWaterDiagnostic")) }) {
			for(UProceduralMeshComponent *const Component : Components) {
				if(Component->GetFName() != Name) continue;
				const FProcMeshSection *const Section = Component->GetProcMeshSection(0);
				// 空の面を同一描画の証拠にしない返戻
				if(!Section || Section->ProcVertexBuffer.IsEmpty() || Section->ProcIndexBuffer.IsEmpty()) return {};
				++Found;
				Append(Section->ProcVertexBuffer.Num());
				Append(Section->ProcIndexBuffer.Num());
				for(const FProcMeshVertex &Vertex : Section->ProcVertexBuffer) {
					Vector(Vertex.Position);
					Vector(Vertex.Normal);
					Vector(Vertex.Tangent.TangentX);
				}
				for(const std::uint32_t Index : Section->ProcIndexBuffer) Append(Index);
			}
		}
		// 欠落又は重複した面を成功にしない返戻
		if(Found != 2) return {};
		// 描画内容の要約の返戻
		return Hash;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowRenderTimingContracts,
	"PlanetGenerator.FlowChannel.RenderTiming",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同一プロセス・同一元場で全微分と標高専用の生成時間及び描画内容を交互に比較する
 * @param Parameters 未使用の実行引数
 * @return 全生成が成立し，公開内容が完全一致する場合に真；時間の採否は測定値から別に判断する
 */
bool FPlanetFlowRenderTimingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	UWorld *World = nullptr;
	if(GEngine) for(const FWorldContext &Context : GEngine->GetWorldContexts()) if(Context.WorldType == EWorldType::Editor) {
		World = Context.World();
		break;
	}
	// 正規の検証世界が無い場合の返戻
	if(!TestNotNull(TEXT("エディタ世界が存在"), World)) return false;
	FActorSpawnParameters Spawn;
	Spawn.ObjectFlags |= RF_Transient;
	const auto Destroy = [](APlanetSphericalSurfacePreview *const Value) -> void {
		if(Value) Value->Destroy();
	};
	const std::unique_ptr<APlanetSphericalSurfacePreview, decltype(Destroy)> Actor(
		World->SpawnActor<APlanetSphericalSurfacePreview>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn),
		Destroy
	);
	// 所有する一時アクターの生成失敗への返戻
	if(!TestNotNull(TEXT("一時描画アクターを生成"), Actor.get())) return false;
	const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
	// 同じ実地形を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較元の地形を準備"), Actor->BuildPreview(42, 1E6, Pass, 200, 16, false, false, false, true))) return false;
	std::array<double, 3> Full, Heights;
	std::uint64_t ExpectedHash = 0;
	// 起動直後の１回を分離した上で，同じ描画要求を交互に計測する
	for(unsigned Run = 0; Run < 7; ++Run) {
		const bool HeightOnly = !(Run & 1);
		// 同じ要求の生成に失敗した場合の返戻
		if(!TestTrue(TEXT("同じ要求の生成に成功"), Actor->BuildFlowChannelPreview(Pass, -100, 681.98759, 256, 96, HeightOnly))) return false;
		const std::optional<std::uint64_t> Hash = MeshHash(*Actor);
		// 公開内容が欠落した場合の返戻
		if(!TestTrue(TEXT("地面と水面の実体が存在"), Hash.has_value())) return false;
		if(!Run) ExpectedHash = *Hash;
		else {
			TestEqual(TEXT("全微分と標高専用で位置・法線・接線・接続が一致"), *Hash, ExpectedHash);
			(HeightOnly ? Heights : Full)[(Run - 1) / 2] = Actor->MeshMilliseconds;
		}
		AddInfo(
			FString::Printf(
				TEXT("Render timing run=%u heightOnly=%u meshMs=%.9g hash=%llu"),
				Run,
				static_cast<unsigned>(HeightOnly),
				Actor->MeshMilliseconds,
				static_cast<unsigned long long>(*Hash)
			)
		);
		AddInfo(Actor->LastBuildMessage);
	}
	std::sort(Full.begin(), Full.end());
	std::sort(Heights.begin(), Heights.end());
	AddInfo(
		FString::Printf(TEXT("Render timing median full=%.9g heightOnly=%.9g ratio=%.9g"), Full[1], Heights[1], Heights[1] / Full[1])
	);
	// 速度の合否と混同しない描画一致の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowReuseContracts,
	"PlanetGenerator.FlowChannel.Reuse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 視点のみの変更で場を再準備せず，生成定義の変更では独立生成と一致する事を確認する
 * @param Parameters 未使用の実行引数
 * @return 描画の同一性と再利用・拒否・無効化の全条件を満たす場合に真
 */
bool FPlanetFlowReuseContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 所有する二つの一時表示と，同じ実地形の準備
	UWorld *World = nullptr;
	if(GEngine) for(const FWorldContext &Context : GEngine->GetWorldContexts()) if(Context.WorldType == EWorldType::Editor) {
		World = Context.World();
		break;
	}
	// 正規の実行世界が無い場合の返戻
	if(!TestNotNull(TEXT("再利用を検査するエディタ世界"), World)) return false;
	FActorSpawnParameters Spawn;
	Spawn.ObjectFlags |= RF_Transient;
	const auto Destroy = [](APlanetSphericalSurfacePreview *const Actor) -> void {
		if(Actor) Actor->Destroy();
	};
	using OwnedPreview = std::unique_ptr<APlanetSphericalSurfacePreview, decltype(Destroy)>;
	const OwnedPreview Actor(
		World->SpawnActor<APlanetSphericalSurfacePreview>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn),
		Destroy
	);
	const OwnedPreview Fresh(
		World->SpawnActor<APlanetSphericalSurfacePreview>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn),
		Destroy
	);
	// 所有する表示の生成失敗への返戻
	if(!TestNotNull(TEXT("再利用側を生成"), Actor.get()) || !TestNotNull(TEXT("独立生成側を生成"), Fresh.get())) return false;
	const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
	const auto Prepare = [&](APlanetSphericalSurfacePreview &Target, const int64 Seed) -> bool {
		// 同じ入口から，元場の実体を準備した結果の返戻
		return Target.BuildPreview(Seed, 1E6, Pass, 200, 16, false, false, false, true);
	};
	const auto Show = [&](APlanetSphericalSurfacePreview &Target, const double Water, const double Distance) -> bool {
		// 同じ表示解像度と有限な範囲で比較する返戻
		return Target.BuildFlowChannelPreview(Pass, Distance, Water, 32, 32);
	};
	// 正規の河道を準備出来ない場合の返戻
	if(!TestTrue(TEXT("元場の準備"), Prepare(*Actor, 42)) || !TestTrue(TEXT("最初の河道準備"), Show(*Actor, 681.98759, -100))) return false;
	const std::optional<std::uint64_t> First = MeshHash(*Actor);
	// 公開面の欠落を一致として扱わない返戻
	if(!TestTrue(TEXT("初回の実描画が存在"), First.has_value())) return false;
	const double InitialMilliseconds = Actor->PreparationMilliseconds;
	TestEqual(TEXT("初回だけ河道と岸を準備"), Actor->DiagnosticFlowPreparations, INT64_C(1));
	// 距離・範囲・格子の変更後も同じ元場を再利用し，元の表示へ戻す
	TestTrue(TEXT("視点だけを変更"), Actor->BuildFlowChannelPreview(Pass, -90, 681.98759, 24, 24));
	TestEqual(TEXT("視点と格子では再準備しない"), Actor->DiagnosticFlowPreparations, INT64_C(1));
	TestTrue(TEXT("元の表示へ戻す"), Show(*Actor, 681.98759, -100));
	const double ReusedMilliseconds = Actor->PreparationMilliseconds;
	TestTrue(TEXT("再利用でも公開メッシュが一致"), MeshHash(*Actor) == First);
	TestEqual(TEXT("往復後も同じ一件を使用"), Actor->DiagnosticFlowPreparations, INT64_C(1));
	TestFalse(TEXT("範囲外の距離は再利用中も拒否"), Show(*Actor, 681.98759, 1E8));
	TestTrue(TEXT("拒否しても公開面は保持"), MeshHash(*Actor) == First);
	TestEqual(TEXT("拒否で準備通番を進めない"), Actor->DiagnosticFlowPreparations, INT64_C(1));
	// 下流水位を変更し，同じ入力を独立に構築した実描画と比較
	TestTrue(TEXT("下流水位の変更を受理"), Show(*Actor, 680, -100));
	TestEqual(TEXT("水位変更では再準備"), Actor->DiagnosticFlowPreparations, INT64_C(2));
	TestTrue(TEXT("独立な元場を準備"), Prepare(*Fresh, 42));
	TestTrue(TEXT("独立な水位変更後の場を準備"), Show(*Fresh, 680, -100));
	const double NearMouth = Actor->DiagnosticRouteEndMeters - 20;
	TestTrue(TEXT("再利用側の河口へ移動"), Show(*Actor, 680, NearMouth));
	TestTrue(TEXT("独立生成側の同じ河口へ移動"), Show(*Fresh, 680, NearMouth));
	const std::optional<std::uint64_t> Changed = MeshHash(*Actor);
	TestTrue(TEXT("水位変更後の河口は独立生成と一致"), Changed.has_value() && Changed == MeshHash(*Fresh));
	TestTrue(TEXT("水位変更後の原点も独立生成と一致"), Actor->GetDiagnosticAnchorDirection() == Fresh->GetDiagnosticAnchorDirection());
	TestEqual(TEXT("河口視点でも再準備しない"), Actor->DiagnosticFlowPreparations, INT64_C(2));
	// 逆向きには此の低い水域へ至らないため，古い経路を誤って再利用しない
	TestFalse(TEXT("到達出来ない逆向きの枝を拒否"), Actor->BuildFlowChannelPreview(Pass, -100, 680, 32, 32, true, false));
	TestTrue(TEXT("別の枝の失敗でも旧描画を保持"), MeshHash(*Actor) == Changed);
	TestEqual(TEXT("別の枝の失敗で通番を進めない"), Actor->DiagnosticFlowPreparations, INT64_C(2));
	// 元場が変更された後は，旧場の水面と河床を照会出来ない
	TestTrue(TEXT("異なる Seed の元場へ変更"), Prepare(*Actor, 7));
	double Height = 0, Water = 0, Coverage = 0, Original = 0;
	TestFalse(TEXT("元場を替えた後の旧河道照会を拒否"), Actor->EvaluateFlowDirection(Pass, Height, Water, Coverage, Original));
	TestTrue(TEXT("元の Seed から新しい元場を再構築"), Prepare(*Actor, 42));
	TestTrue(TEXT("元場変更後の河道を再構築"), Show(*Actor, 680, NearMouth));
	TestEqual(TEXT("元場を替えた後は再準備"), Actor->DiagnosticFlowPreparations, INT64_C(3));
	TestTrue(TEXT("新しい元場でも独立生成と同じ河口"), MeshHash(*Actor) == Changed);
	AddInfo(
		FString::Printf(
			TEXT("Flow preparation reuse: initialMs=%.9g reusedMs=%.9g rebuilds=%lld initialHash=%llu mouthHash=%llu"),
			InitialMilliseconds,
			ReusedMilliseconds,
			Actor->DiagnosticFlowPreparations,
			static_cast<unsigned long long>(*First),
			static_cast<unsigned long long>(Changed.value_or(0))
		)
	);
	// 視点に依らない同一性と生成定義の無効化を併せた結果の返戻
	return !HasAnyErrors();
}

#endif
