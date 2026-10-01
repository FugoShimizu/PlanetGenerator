#include "PlanetCelestialComponent.h"
#include "Core/PlanetCelestial.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Kismet/GameplayStatics.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include <cmath>
#include <numbers>
#include <optional>

/**
 * 身体の目線を地球参照へ対応させ，同じ日時で天体照明と季節を更新する
 * @param DeltaSeconds 前回の更新からの秒数
 */
void UPlanetCelestialComponent::UpdateCelestialClock(const float DeltaSeconds) {
	// Editor の季節単独検査や，まだ時計を開始していない惑星では更新しない終了
	if(!bCelestialClockActive || !GetWorld() || !GetWorld()->IsGameWorld()) return;
	if(!std::isfinite(TimeScale) || TimeScale < 0 || TimeScale > 1000000 || !std::isfinite(DeltaSeconds) || DeltaSeconds < 0) {
		ClockMessage = TEXT("日時の更新値が不正な為，共通時計を停止しました");
		bCelestialClockActive = false;
		// 無効な時間を加算しない終了
		return;
	}
	if(bAdvanceTime) {
		const double Step = DeltaSeconds * TimeScale, Remaining = (FDateTime(2101, 1, 1) - SimulationUtc).GetTotalSeconds();
		if(Step >= Remaining) {
			ClockMessage = TEXT("対応年代の終端へ達した為，共通時計を停止しました");
			bCelestialClockActive = false;
			// 対応年代外へ進めない終了
			return;
		}
		SimulationUtc += FTimespan::FromSeconds(Step);
	}
	// 身体の目線を地球参照の緯度・経度・高度へ変換
	const FVector Local = PlanetTransform.InverseTransformPosition(ObserverWorld) / 100;
	const FVector Up = Local.GetSafeNormal();
	// 極では経度が未定義の為，座標変換の丸めだけで経度を回転させない
	const double Height = Local.Length() - RadiusMeters, Longitude = Up.X * Up.X + Up.Y * Up.Y < 1E-24 ? 0 : std::atan2(Up.Y, Up.X);
	const double Latitude = std::asin(FMath::Clamp(Up.Z, -1.0, 1.0));
	const PlanetCore::CelestialObserver Observer { Latitude * 180 / std::numbers::pi, Longitude * 180 / std::numbers::pi, Height };
	const double Days = (SimulationUtc - FDateTime(2000, 1, 1, 12)).GetTotalDays();
	const std::optional<PlanetCore::CelestialState> Sky = PlanetCore::PlanetCelestial::Evaluate(Days, Observer);
	if(!Sky) {
		ClockMessage = TEXT("対応する地球参照の日時・緯度・経度・高度を確認して下さい");
		bCelestialClockActive = false;
		// 不正な観測地点で最後の有効な照明を上書きしない終了
		return;
	}
	// 同じ惑星姿勢の東・北・上の基底
	const FVector East(-std::sin(Longitude), std::cos(Longitude), 0);
	const FVector North = FVector::CrossProduct(Up, East).GetSafeNormal();
	const FTransform Transform = PlanetTransform;
	/**
	 * 地平座標の東・北・上を，惑星姿勢に従う描画方向へ変換する
	 * @param Direction 地球参照の東・北・上の方向
	 * @return UE ワールドの単位方向
	 */
	const auto WorldDirection = [&East, &North, &Up, &Transform](const PlanetCore::Vec3 Direction) -> FVector {
		// 表示方向の返戻
		return Transform.TransformVectorNoScale(East * Direction.X + North * Direction.Y + Up * Direction.Z).GetSafeNormal();
	};
	SunDirectionWorld = WorldDirection(Sky->Sun.Direction);
	MoonDirectionWorld = WorldDirection(Sky->Moon.Direction);
	MoonToSunDirectionWorld = WorldDirection(Sky->MoonToSunDirection);
	MoonIlluminatedFraction = Sky->MoonIlluminatedFraction;
	MoonAngularDiameterDegrees = Sky->Moon.AngularDiameterDegrees;
	SunAltitudeDegrees = Sky->Sun.AltitudeDegrees;
	MoonAltitudeDegrees = Sky->Moon.AltitudeDegrees;
	GreenwichSiderealHours = Sky->GreenwichSiderealHours;
	EarthObserverDegreesMeters = FVector(Observer.LatitudeDegrees, Observer.LongitudeDegrees, Observer.HeightMeters);
	SolarYearPhase = Sky->SolarYearPhase;
	SunLight->SetWorldRotation((-SunDirectionWorld).Rotation());
	MoonLight->SetWorldRotation((-MoonDirectionWorld).Rotation());
	SunLight->SetLightSourceAngle(static_cast<float>(Sky->Sun.AngularDiameterDegrees));
	MoonLight->SetLightSourceAngle(static_cast<float>(Sky->Moon.AngularDiameterDegrees));
	MoonLight->SetIntensity(static_cast<float>(0.25 * MoonIlluminatedFraction));
	const bool MoonDominates = SunAltitudeDegrees < -4 && MoonAltitudeDegrees > 0;
	SunLight->SetForwardShadingPriority(MoonDominates ? 0 : 1);
	MoonLight->SetForwardShadingPriority(MoonDominates ? 1 : 0);
	SunLight->SetCastShadows(SunAltitudeDegrees >= -4);
	MoonLight->SetCastShadows(MoonDominates);
	SunLight->SetVisibility(true);
	MoonLight->SetVisibility(true);
	ClockMessage.Empty();
	if(
		IConsoleVariable *const Cached = IConsoleManager::Get().FindConsoleVariable(TEXT("r.EyeAdaptation.CachedLightingPreExposure"))
	) {
		const float Exposure = static_cast<float>(FMath::Pow(8.0, FMath::Clamp((SunAltitudeDegrees + 10.0) / 20.0, 0.0, 1.0)));
		if(FMath::Abs(Cached->GetFloat() - Exposure) > 0.05F) {
			if(!ExposureApplied) ExposureTag = FName(*GetPathName());
			Cached->Set(Exposure, ECVF_SetByTemp, ExposureTag);
			ExposureApplied = true;
		}
	}
	// 終了
	return;
}

/**
 * 現在の視点を中心とする背景面へ，天体暦と恒星時を渡す
 */
void UPlanetCelestialComponent::UpdateCelestialSky() {
	// 描画中の実カメラと天球の既存材質
	APlayerCameraManager *const Camera = UGameplayStatics::GetPlayerCameraManager(GetWorld(), 0);
	// 実際の視点と天球資産が揃う迄は背景を公開しない終了
	if(!Camera || !SkyDome->GetStaticMesh() || !SkyDome->GetMaterial(0)) return;
	if(!SkyMaterial) SkyMaterial = SkyDome->CreateDynamicMaterialInstance(0);
	// 材質確保失敗の場合の終了
	if(!SkyMaterial) return;
	// 視点を中心とし，全惑星より外側に在る背景面
	const FVector Eye = Camera->GetCameraLocation(), CenterToEye = Eye - PlanetTransform.GetLocation();
	const double Distance = CenterToEye.Length(), Radius = RadiusMeters * 100;
	const double DomeRadius = FMath::Max(100000000.0, 3 * Radius + Distance);
	SkyDome->SetWorldLocation(Eye);
	SkyDome->SetWorldScale3D(FVector(DomeRadius / 50));
	// 月面の極方向と，恒星時で回転する一般的な星空
	const FTransform Transform = PlanetTransform;
	const FVector Pole = Transform.TransformVectorNoScale(FVector::UpVector).GetSafeNormal();
	FVector MoonUp = (Pole - MoonDirectionWorld * FVector::DotProduct(Pole, MoonDirectionWorld)).GetSafeNormal();
	if(MoonUp.IsNearlyZero()) MoonUp = Transform.TransformVectorNoScale(FVector::ForwardVector).GetSafeNormal();
	const FVector MoonRight = FVector::CrossProduct(MoonDirectionWorld, MoonUp).GetSafeNormal();
	const double Sidereal = GreenwichSiderealHours * std::numbers::pi / 12;
	const FVector StarX = Transform.TransformVectorNoScale(FVector(std::cos(Sidereal), -std::sin(Sidereal), 0));
	const FVector StarY = Transform.TransformVectorNoScale(FVector(std::sin(Sidereal), std::cos(Sidereal), 0));
	const FVector ObserverUp = CenterToEye.GetSafeNormal();
	const double Ratio = Distance > Radius ? Radius / Distance : 1;
	SkyMaterial->SetVectorParameterValue(TEXT("MoonDirection"), FLinearColor(MoonDirectionWorld));
	SkyMaterial->SetVectorParameterValue(TEXT("MoonRight"), FLinearColor(MoonRight));
	SkyMaterial->SetVectorParameterValue(TEXT("MoonUp"), FLinearColor(MoonUp));
	SkyMaterial->SetVectorParameterValue(TEXT("MoonToSun"), FLinearColor(MoonToSunDirectionWorld));
	SkyMaterial->SetVectorParameterValue(TEXT("StarAxisX"), FLinearColor(StarX));
	SkyMaterial->SetVectorParameterValue(TEXT("StarAxisY"), FLinearColor(StarY));
	SkyMaterial->SetVectorParameterValue(TEXT("StarAxisZ"), FLinearColor(Pole));
	SkyMaterial->SetVectorParameterValue(TEXT("ObserverUp"), FLinearColor(ObserverUp));
	SkyMaterial->SetScalarParameterValue(TEXT("HorizonCos"), -std::sqrt(FMath::Max(0.0, 1 - Ratio * Ratio)));
	SkyMaterial->SetScalarParameterValue(TEXT("MoonSinRadius"), std::sin(MoonAngularDiameterDegrees * std::numbers::pi / 360));
	SkyDome->SetVisibility(true);
	// 終了
	return;
}

/**
 * 同じ公開済の惑星座標と身体の目線を時計へ渡す
 * @param Transform 公開済の惑星中心・姿勢；尺度は１
 * @param Radius 公開済の海面半径，単位 m
 * @param EyeWorld 身体の目線，単位 cm
 * @return 有限な座標と半径を受理した場合に真
 */
bool UPlanetCelestialComponent::SetObserver(const FTransform &Transform, const double Radius, const FVector EyeWorld) {
	// 無効な尺度又は座標で最後の有効な観測地点を上書きしない返戻
	if(!std::isfinite(Radius) || Radius < 1000 || !Transform.IsValid() || EyeWorld.ContainsNaN()) return false;
	// 身体の高さを異なる尺度で解釈しない返戻
	if(!Transform.GetScale3D().Equals(FVector::OneVector, 0)) return false;
	// 球心と一致しない有効な観測方向の確認
	const FVector Local = Transform.InverseTransformPosition(EyeWorld);
	// 惑星中心は観測方向を持たない返戻
	if(Local.IsNearlyZero()) return false;
	PlanetTransform = Transform;
	RadiusMeters = Radius;
	ObserverWorld = EyeWorld;
	HasObserver = true;
	// 有効な観測地点の返戻
	return true;
}

/**
 * 公開済の惑星中心と半径から，大気・雲の寸法を一度だけ準備する
 * @return 同じゲーム世界へ適用出来た場合に真
 */
bool UPlanetCelestialComponent::PrepareAtmosphere() {
	// 同じ世界の大気を所有する Actor
	AActor *const Owner = GetOwner();
	// まだ公開されていない惑星の尺度を使わない返戻
	if(!Owner || !HasObserver) return false;
	// 元の設定を基準とする適用結果の返戻
	return Atmosphere.Apply(*Owner, PlanetTransform.GetLocation(), RadiusMeters);
}

/**
 * 同じ日時の照明を進め，停止時も最後の天球を実カメラへ追従させる
 * @param DeltaSeconds 実時間の経過秒数
 */
void UPlanetCelestialComponent::Advance(const float DeltaSeconds) {
	UpdateCelestialClock(DeltaSeconds);
	if(bCelestialClockActive || SkyMaterial) {
		PrepareAtmosphere();
		UpdateCelestialSky();
	}
	// 終了
	return;
}

/**
 * 生成地形を変えず，昼夜・月・季節に共通する日時を変更する
 * @param Utc １９００～２１００年の UTC 日時
 * @return 実行中の惑星へ有効な日時を設定出来た場合に真
 */
bool UPlanetCelestialComponent::SetSimulationUtc(const FDateTime Utc) {
	if(!GetWorld() || !GetWorld()->IsGameWorld() || !HasObserver || Utc.GetYear() < 1900 || Utc.GetYear() > 2100) {
		ClockMessage = TEXT("実行中の惑星へ，１９００～２１００年の UTC 日時を指定して下さい");
		// 現在の日時を不正な値で上書きしない返戻
		return false;
	}
	SimulationUtc = Utc;
	bCelestialClockActive = true;
	UpdateCelestialClock(0);
	if(bCelestialClockActive) PrepareAtmosphere();
	// 天体を評価出来た場合の返戻
	return bCelestialClockActive;
}

/**
 * 設定した初期日時から，共通時計と太陽・月の照明を開始する
 * @return 有効な日時と倍率で開始出来た場合に真
 */
bool UPlanetCelestialComponent::StartCelestialClock() {
	if(!std::isfinite(TimeScale) || TimeScale < 0 || TimeScale > 1000000) {
		ClockMessage = TEXT("時間倍率は０以上１００万以下の有限値に設定して下さい");
		// 不正な時間倍率の場合の返戻
		return false;
	}
	// 明示した初期日時の設定結果の返戻
	return SetSimulationUtc(InitialUtc);
}

/**
 * 所有 Actor の構築中に照明と天球を作り，開始時迄非表示にする
 * @param Owner 部品を所有する構築中の Actor
 */
void UPlanetCelestialComponent::CreateScene(AActor &Owner) {
	SunLight = Owner.CreateDefaultSubobject<UDirectionalLightComponent>(TEXT("PlanetSun"));
	MoonLight = Owner.CreateDefaultSubobject<UDirectionalLightComponent>(TEXT("PlanetMoon"));
	for(UDirectionalLightComponent *const Light : { SunLight.Get(), MoonLight.Get() }) {
		Light->SetupAttachment(Owner.GetRootComponent());
		Light->SetMobility(EComponentMobility::Movable);
		Light->SetUsingAbsoluteRotation(true);
		Light->SetAtmosphereSunLight(true);
		Light->SetVisibility(false);
	}
	SunLight->SetAtmosphereSunLightIndex(0);
	SunLight->SetIntensity(80000);
	MoonLight->SetAtmosphereSunLightIndex(1);
	MoonLight->SetIntensity(0.25F);
	// 満欠けを天球材質で描く為，標準の常時満月の円盤を表示しない
	MoonLight->SetAtmosphereSunDiskColorScale(FLinearColor::Black);
	SunLight->SetForwardShadingPriority(1);
	SkyDome = Owner.CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PlanetCelestialSky"));
	SkyDome->SetupAttachment(Owner.GetRootComponent());
	SkyDome->SetAbsolute(true, true, true);
	SkyDome->SetMobility(EComponentMobility::Movable);
	SkyDome->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SkyDome->SetCastShadow(false);
	SkyDome->SetAffectDistanceFieldLighting(false);
	SkyDome->SetAffectDynamicIndirectLighting(false);
	SkyDome->SetVisibleInRayTracing(false);
	SkyDome->SetReceivesDecals(false);
	SkyDome->SetVisibility(false);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Sky(
		TEXT("/Game/Planet/Materials/M_PlanetCelestialSky.M_PlanetCelestialSky")
	);
	if(Sphere.Succeeded()) SkyDome->SetStaticMesh(Sphere.Object);
	if(Sky.Succeeded()) SkyDome->SetMaterial(0, Sky.Object);
	// 終了
	return;
}

/**
 * 自分が変更した露出を，ゲーム世界の終了時に復元する
 * @param Reason 終了理由
 */
void UPlanetCelestialComponent::EndPlay(const EEndPlayReason::Type Reason) {
	Atmosphere.Restore();
	if(ExposureApplied) {
		if(
			IConsoleVariable *const Cached = IConsoleManager::Get().FindConsoleVariable(TEXT("r.EyeAdaptation.CachedLightingPreExposure"))
		) Cached->Unset(ECVF_SetByTemp, ExposureTag);
		ExposureApplied = false;
	}
	Super::EndPlay(Reason);
	// 終了
	return;
}
