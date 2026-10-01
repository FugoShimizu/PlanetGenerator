#include "PlanetAtmosphere.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/AutomationTest.h"
#include "UObject/UObjectGlobals.h"
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetAtmosphereContracts, "PlanetGenerator.Celestial.Atmosphere",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 大気・雲の実部品で，半径補正・冪等・元材質の維持・復元を確認する
 * @param Parameters 未使用の実行引数
 * @return 所有世界の検査を最後迄実行出来た場合に真
 */
bool FPlanetAtmosphereContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 世界の登録先が無い場合の返戻
	if(!TestNotNull(TEXT("大気検査のエンジン"), GEngine)) return false;
	// 他の世界の大気や雲を変更しない検査世界の登録
	const UWorld::InitializationValues Values =
	UWorld::InitializationValues().CreateNavigation(false).CreateAISystem(false).AllowAudioPlayback(false).ShouldSimulatePhysics(
		false
	).SetTransactional(false);
	const FName Name = MakeUniqueObjectName(GetTransientPackage(), UWorld::StaticClass(), TEXT("PlanetAtmosphereWorld"));
	UWorld *const World =
	UWorld::CreateWorld(EWorldType::Game, false, Name, GetTransientPackage(), true, ERHIFeatureLevel::Num, &Values);
	// 世界生成失敗への返戻
	if(!TestNotNull(TEXT("大気の所有世界"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ASkyAtmosphere *const Sky = World->SpawnActor<ASkyAtmosphere>();
	AVolumetricCloud *const Cloud = World->SpawnActor<AVolumetricCloud>();
	FPlanetAtmosphere Atmosphere;
	if(TestNotNull(TEXT("実大気"), Sky) && TestNotNull(TEXT("実雲"), Cloud)) {
		// 元の部品値と，同じ元材質から作った動的な雲の準備
		USkyAtmosphereComponent *const Air = Sky->GetComponent();
		UVolumetricCloudComponent *const Volume = Cloud->FindComponentByClass<UVolumetricCloudComponent>();
		UMaterialInterface *const Base = LoadObject<UMaterialInterface>(
			nullptr,
			TEXT("/Engine/EngineSky/VolumetricClouds/m_SimpleVolumetricCloud_Inst.m_SimpleVolumetricCloud_Inst")
		);
		if(TestNotNull(TEXT("既存の雲材質"), Base) && TestNotNull(TEXT("雲部品"), Volume)) {
			UMaterialInstanceDynamic *const Original = UMaterialInstanceDynamic::Create(Base, Cloud);
			Original->SetScalarParameterValue(TEXT("Layout_CloudGlobalScale"), 123);
			Original->SetScalarParameterValue(TEXT("Cloud_GlobalDensity"), 0.55F);
			Volume->SetMaterial(Original);
			const float AirHeight = Air->AtmosphereHeight, AirRayleigh = Air->RayleighExponentialDistribution;
			const float AirMie = Air->MieExponentialDistribution, AirDensity = Air->RayleighScatteringScale;
			const float CloudBase = Volume->LayerBottomAltitude, CloudHeight = Volume->LayerHeight, OriginalRadius = Air->BottomRadius;
			const bool OriginalTransmittance = Volume->bUsePerSampleAtmosphericLightTransmittance;
			const ESkyAtmosphereTransformMode OriginalMode = Air->TransformMode;
			const FTransform OriginalTransform = Air->GetComponentTransform();
			IConsoleVariable *const LightDistance =
			IConsoleManager::Get().FindConsoleVariable(TEXT("r.SkyAtmosphere.DistantSkyLightLUT.Altitude"));
			IConsoleVariable *const CloudDistance =
			IConsoleManager::Get().FindConsoleVariable(TEXT("r.VolumetricCloud.DistanceToSampleMaxCount"));
			const float OriginalLight = LightDistance ? LightDistance->GetFloat() : 0;
			const float OriginalCloud = CloudDistance ? CloudDistance->GetFloat() : 0;
			const FVector Center(120000, -340000, -100000);
			// 拒否される入力で初期値を変更しない事の確認
			TestFalse(TEXT("小さ過ぎる半径を拒否"), Atmosphere.Apply(*Sky, Center, 999));
			TestFalse(TEXT("非有限半径を拒否"), Atmosphere.Apply(*Sky, Center, std::numeric_limits<double>::infinity()));
			TestEqual(TEXT("拒否で大気半径を変更しない"), Air->BottomRadius, OriginalRadius);
			// 最小半径と中間・基準半径を往復し，同じ初期値から計算
			for(const double Radius : { 1000.0, 1000.0, 1000000.0, 250000.0, 1000.0 }) {
				const float Scale = static_cast<float>(Radius / 1000000);
				TestTrue(TEXT("有効な半径補正を受理"), Atmosphere.Apply(*Sky, Center, Radius));
				TestEqual(TEXT("大気の同じ球面半径"), Air->BottomRadius, static_cast<float>(Radius / 1000));
				TestTrue(TEXT("大気の同じ中心"), Air->GetComponentLocation().Equals(Center, 0));
				TestEqual(TEXT("大気の球心モード"), Air->TransformMode, ESkyAtmosphereTransformMode::PlanetCenterAtComponentTransform);
				TestEqual(TEXT("大気上端の既存の最低値"), Air->AtmosphereHeight, FMath::Max(0.1F, AirHeight * Scale));
				TestEqual(TEXT("Rayleigh 高さ"), Air->RayleighExponentialDistribution, AirRayleigh * Scale);
				TestEqual(TEXT("Mie 高さ"), Air->MieExponentialDistribution, AirMie * Scale);
				TestEqual(TEXT("光学的な厚さを保つ密度"), Air->RayleighScatteringScale, AirDensity / Scale);
				TestEqual(TEXT("雲底の既存の最低値"), Volume->LayerBottomAltitude, FMath::Max(0.02F, CloudBase * Scale));
				TestEqual(TEXT("雲層厚の既存の最低値"), Volume->LayerHeight, FMath::Max(0.01F, CloudHeight * Scale));
				UMaterialInstanceDynamic *const Material = Cast<UMaterialInstanceDynamic>(Volume->GetMaterial());
				if(TestNotNull(TEXT("共有の雲材質"), Material)) {
					TestTrue(TEXT("動的材質を親にしない"), Material->Parent.Get() == Base);
					TestEqual(TEXT("雲の模様尺度"), Material->K2_GetScalarParameterValue(TEXT("Layout_CloudGlobalScale")), 123 * Scale);
					TestEqual(TEXT("雲の光学的な密度"), Material->K2_GetScalarParameterValue(TEXT("Cloud_GlobalDensity")), 0.55F / Scale);
				}
			}
			// 所有した設定の解除は，他の利用者の設定を保持
			const FName OtherTag(TEXT("PlanetAtmosphereContractOther"));
			if(LightDistance) LightDistance->Set(3.25F, ECVF_SetByTemp, OtherTag);
			Atmosphere.Restore();
			if(LightDistance) {
				TestEqual(TEXT("別の一時設定を保持"), LightDistance->GetFloat(), 3.25F);
				LightDistance->Unset(ECVF_SetByTemp, OtherTag);
				TestEqual(TEXT("元の採光距離へ復元"), LightDistance->GetFloat(), OriginalLight);
			}
			if(CloudDistance) TestEqual(TEXT("元の雲採光距離へ復元"), CloudDistance->GetFloat(), OriginalCloud);
			TestEqual(TEXT("大気の元半径を復元"), Air->BottomRadius, OriginalRadius);
			TestEqual(TEXT("大気の元の高さを復元"), Air->AtmosphereHeight, AirHeight);
			TestEqual(TEXT("大気の元モードを復元"), Air->TransformMode, OriginalMode);
			TestTrue(TEXT("大気の元の姿勢を復元"), Air->GetComponentTransform().Equals(OriginalTransform, 0));
			TestTrue(TEXT("同じ元の動的材質へ復元"), Volume->GetMaterial() == Original);
			TestEqual(TEXT("雲の元の高さを復元"), Volume->LayerHeight, CloudHeight);
			TestEqual(TEXT("雲の元の透過指定を復元"), bool(Volume->bUsePerSampleAtmosphericLightTransmittance), OriginalTransmittance);
			TestEqual(TEXT("元の雲材質を変更しない"), Original->K2_GetScalarParameterValue(TEXT("Layout_CloudGlobalScale")), 123.0F);
			AddInfo(TEXT("PlanetAtmosphere radiusChanges=5 minimumRadius=1000 originalDynamicParentPreserved=true settingsRestored=true"));
		}
	}
	Atmosphere.Restore();
	World->DestroyWorld(false);
	GEngine->DestroyWorldContext(World);
	TestNull(TEXT("大気検査の世界を解放"), GEngine->GetWorldContextFromWorld(World));
	// 独立した部品の検査結果の返戻
	return true;
}

#endif
