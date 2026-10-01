#include "PlanetCelestialComponent.h"
#include "PlanetExplorerCharacter.h"
#include "PlanetFlowTerrain.h"
#include "PlanetWorld.h"
#include "Camera/CameraComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "UObject/UnrealType.h"
#include "UObject/UObjectGlobals.h"
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetCelestialSceneContracts, "PlanetGenerator.Celestial.Scene",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 従来の惑星の操作口・部品名・時計停止と，世界終了時の露出復元を確認する
 * @param Parameters 未使用の実行引数
 * @return 検査を最後迄実行出来た場合に真
 */
bool FPlanetCelestialSceneContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 世界の所有先が無い場合の返戻
	if(!TestNotNull(TEXT("天球検査のエンジン"), GEngine)) return false;
	const UWorld::InitializationValues Values =
	UWorld::InitializationValues().CreateNavigation(false).CreateAISystem(false).AllowAudioPlayback(false).ShouldSimulatePhysics(
		false
	).SetTransactional(false);
	const FName Name = MakeUniqueObjectName(GetTransientPackage(), UWorld::StaticClass(), TEXT("PlanetCelestialWorld"));
	UWorld *const World =
	UWorld::CreateWorld(EWorldType::Game, false, Name, GetTransientPackage(), true, ERHIFeatureLevel::Num, &Values);
	// 世界生成失敗への返戻
	if(!TestNotNull(TEXT("天球用の所有世界"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	APlanetWorld *const Planet = World->SpawnActor<APlanetWorld>();
	if(TestNotNull(TEXT("従来の公開惑星"), Planet)) {
		Planet->bGenerateOnBeginPlay = false;
		Planet->bFollowPlayerCamera = false;
		Planet->RadiusMeters = 1000000;
		Planet->SolarYearPhase = 0.375;
		Planet->bAdvanceTime = false;
		Planet->SetActorTransform(FTransform(FRotator(23, 57, 11), FVector(12300, -45000, 67800)));
		World->InitializeActorsForPlay(FURL());
		World->BeginPlay();
		World->GetWorldSettings()->NotifyBeginPlay();
		UPlanetCelestialComponent *const Clock = Planet->FindComponentByClass<UPlanetCelestialComponent>();
		if(TestNotNull(TEXT("共有した時計部品"), Clock)) {
			IConsoleVariable *const Exposure =
			IConsoleManager::Get().FindConsoleVariable(TEXT("r.EyeAdaptation.CachedLightingPreExposure"));
			const float OriginalExposure = Exposure ? Exposure->GetFloat() : 0;
			TestFalse(TEXT("観測地点を渡す前は開始しない"), Clock->StartCelestialClock());
			TestEqual(TEXT("開始前の手動季節を保持"), Planet->SolarYearPhase, 0.375);
			Planet->InitialUtc = FDateTime(2026, 6, 21, 12);
			TestTrue(TEXT("従来の開始操作を保持"), Planet->StartCelestialClock());
			TestTrue(TEXT("共通部品の開始を公開"), Planet->bCelestialClockActive && Clock->bCelestialClockActive);
			TestTrue(TEXT("従来の既定観測地点は北極の海面"), Planet->EarthObserverDegreesMeters.Equals(FVector(90, 0, 0), 1E-6));
			TestEqual(TEXT("日時の公開値を保持"), Planet->SimulationUtc.GetTicks(), Clock->SimulationUtc.GetTicks());
			TestTrue(TEXT("太陽の公開方向を保持"), Planet->SunDirectionWorld.Equals(Clock->SunDirectionWorld, 0));
			TArray<UDirectionalLightComponent *> Lights;
			Planet->GetComponents(Lights);
			TestEqual(TEXT("太陽と月の照明だけを生成"), Lights.Num(), 2);
			for(const UDirectionalLightComponent *const Light : Lights) {
				const bool IsSun = Light->GetFName() == TEXT("PlanetSun");
				TestTrue(TEXT("既存の部品名を保持"), IsSun || Light->GetFName() == TEXT("PlanetMoon"));
				const FVector Direction = IsSun ? Planet->SunDirectionWorld : Planet->MoonDirectionWorld;
				TestTrue(TEXT("実照明の回転は天体方向に一致"), (-Light->GetForwardVector()).Equals(Direction, 1E-6));
			}
			// 通常の地形所有者から，生成時に身体の露出へ時計が結び付く事を確認
			APlanetExplorerCharacter *const Body =
			World->SpawnActorDeferred<APlanetExplorerCharacter>(APlanetExplorerCharacter::StaticClass(), FTransform::Identity);
			if(TestNotNull(TEXT("従来惑星の露出を使う身体"), Body)) {
				Body->Terrain = Planet;
				Body->FinishSpawning(FTransform::Identity);
				TestTrue(TEXT("通常の初期化で身体の開始処理を実行"), Body->HasActorBegunPlay());
				TArray<UCameraComponent *> Cameras;
				Body->GetComponents(Cameras);
				TestEqual(TEXT("従来経路の三視点"), Cameras.Num(), 3);
				TestTrue(TEXT("日中の太陽高度で初期化を検査"), Clock->SunAltitudeDegrees >= 20.0);
				for(const UCameraComponent *const Camera : Cameras) {
					TestEqual(TEXT("従来経路の EV100 下限"), Camera->PostProcessSettings.AutoExposureMinBrightness, 0.0F);
					TestEqual(TEXT("従来経路の EV100 上限"), Camera->PostProcessSettings.AutoExposureMaxBrightness, 16.0F);
					TestEqual(TEXT("日中の自動露出に余分な減光を加えない"), Camera->PostProcessSettings.AutoExposureBias, 0.0F);
				}
				// Python の編集通知と同じ経路で再構築しても，公開済の身体を初期値へ戻さない
				APlanetFlowTerrain *const Flow = World->SpawnActor<APlanetFlowTerrain>();
				if(TestNotNull(TEXT("時計を編集する水系アクター"), Flow)) {
					Flow->Explorer = Body;
					UPlanetCelestialComponent *const FlowClock = Flow->CelestialClock;
					FProperty *const Property = FindFProperty<FProperty>(
						UPlanetCelestialComponent::StaticClass(),
						GET_MEMBER_NAME_CHECKED(UPlanetCelestialComponent, bAdvanceTime)
					);
					if(TestNotNull(TEXT("時計の編集プロパティ"), Property)) {
						FlowClock->PreEditChange(Property);
						FlowClock->bAdvanceTime = !FlowClock->bAdvanceTime;
						FPropertyChangedEvent Event(Property, EPropertyChangeType::ValueSet);
						FlowClock->PostEditChangeProperty(Event);
						TestTrue(TEXT("編集による再構築で身体参照を保持"), Flow->Explorer == Body);
						TestTrue(TEXT("同じ時計部品と編集値を保持"), Flow->CelestialClock == FlowClock && !FlowClock->bAdvanceTime);
					}
				}
			}
			const FDateTime Valid = Planet->SimulationUtc;
			Planet->TimeScale = std::numeric_limits<double>::quiet_NaN();
			TestFalse(TEXT("旧操作口も不正な倍率を拒否"), Planet->StartCelestialClock());
			TestEqual(TEXT("開始拒否で時刻を破棄しない"), Planet->SimulationUtc.GetTicks(), Valid.GetTicks());
			Planet->TimeScale = 60;
			TestFalse(TEXT("旧操作口も範囲外の日時を拒否"), Planet->SetSimulationUtc(FDateTime(2101, 1, 1)));
			TestEqual(TEXT("日時拒否で時刻を破棄しない"), Planet->SimulationUtc.GetTicks(), Valid.GetTicks());
			TestFalse(
				TEXT("異なる尺度の観測地点を拒否"),
				Clock->SetObserver(FTransform(FQuat::Identity, FVector::ZeroVector, FVector(2)), 1000, FVector(0, 0, 100000))
			);
			TestFalse(TEXT("惑星中心を観測地点にしない"), Clock->SetObserver(FTransform::Identity, 1000, FVector::ZeroVector));
			World->EndPlay(EEndPlayReason::Quit);
			if(Exposure) TestEqual(TEXT("所有世界終了で元の露出を復元"), Exposure->GetFloat(), OriginalExposure);
			AddInfo(
				TEXT("CelestialScene legacyEntryPoints=2 exposureCameras=3 directionalLights=2 ")
				TEXT("invalidClockInputs=2 invalidObservers=2 exposureRestored=true editorReferencePreserved=true")
			);
		} else World->EndPlay(EEndPlayReason::Quit);
	}
	World->DestroyWorld(false);
	GEngine->DestroyWorldContext(World);
	TestFalse(TEXT("検査世界の根参照を解放"), World->IsRooted());
	TestNull(TEXT("検査世界の登録を解放"), GEngine->GetWorldContextFromWorld(World));
	// 検査全体を実行した返戻
	return true;
}

#endif
