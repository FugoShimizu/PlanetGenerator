#pragma once

#include "PlanetAtmosphere.h"
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include "PlanetCelestialComponent.generated.h"

class UDirectionalLightComponent;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;

/** 公開済の惑星座標と身体の目線に，共通時計・地球参照の太陽・月・天球を対応させる */
UCLASS()
class PLANETGENERATOR_API UPlanetCelestialComponent : public UActorComponent {
	GENERATED_BODY()

private:

	// 所有 Actor の既定部品を参照し，公開後の天球材質を GC から保持
	UPROPERTY(Transient)
	TObjectPtr<UDirectionalLightComponent> SunLight;
	UPROPERTY(Transient)
	TObjectPtr<UDirectionalLightComponent> MoonLight;
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> SkyDome;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> SkyMaterial;
	// 観測地点は描画カメラでなく身体の目線，天球の中心だけは実カメラ
	FTransform PlanetTransform;
	FVector ObserverWorld = FVector::ZeroVector;
	double RadiusMeters = 0;
	bool HasObserver = false;
	FPlanetAtmosphere Atmosphere;
	FName ExposureTag;
	bool ExposureApplied = false;
	void UpdateCelestialClock(const float DeltaSeconds);
	void UpdateCelestialSky();

public:

	// 日時設定と，同じ天体暦から得た照明・季節の公開値
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Clock")
	FDateTime InitialUtc = FDateTime(2026, 6, 21, 12, 0, 0);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Clock", meta = (ClampMin = "0", ClampMax = "1000000"))
	double TimeScale = 60;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Clock")
	bool bAdvanceTime = true;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Clock")
	bool bCelestialClockActive = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Clock")
	FDateTime SimulationUtc;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Clock")
	FString ClockMessage;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	FVector SunDirectionWorld = FVector::UpVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	FVector MoonDirectionWorld = FVector::UpVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	FVector MoonToSunDirectionWorld = FVector::UpVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	double MoonIlluminatedFraction = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	double MoonAngularDiameterDegrees = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	double SunAltitudeDegrees = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	double MoonAltitudeDegrees = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	double GreenwichSiderealHours = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Sky")
	FVector EarthObserverDegreesMeters = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Climate")
	double SolarYearPhase = 0;

	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	bool SetObserver(const FTransform &Transform, const double Radius, const FVector EyeWorld);
	bool PrepareAtmosphere();
	void Advance(const float DeltaSeconds);
	UFUNCTION(BlueprintCallable, Category="Planet|Clock")
	bool SetSimulationUtc(const FDateTime Utc);
	UFUNCTION(BlueprintCallable, Category="Planet|Clock")
	bool StartCelestialClock();
	// Actor の構築中に一度だけ呼ぶ；既存の部品名を保持
	void CreateScene(AActor &Owner);
};
