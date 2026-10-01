#pragma once

#include "Core/PlanetBiome.h"
#include "PlanetTerrain.h"
#include <memory>
#include <optional>
#include "PlanetWorld.generated.h"

class UPlanetCelestialComponent;
class UTexture2D;
struct FPlanetWorldState;
namespace PlanetCore {
class PlanetRunoff;
}

/** 不変な地形・季節材質と，一つの日時に従う天体照明を結合する惑星 */
UCLASS()
class PLANETGENERATOR_API APlanetWorld : public APlanetTerrain {
	GENERATED_BODY()

private:

	// 同じ公開地形の気候資源と，共通の天体時計
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> SeasonalTexture;
	UPROPERTY(VisibleAnywhere, Category = "Components")
	TObjectPtr<UPlanetCelestialComponent> CelestialClock;
	TSharedPtr<FPlanetWorldState> ClimateState;

	void ReadCelestialClock();
	void ConfigureCelestialObserver();
	void UpdateClimate();
	void ApplySeasonalMaterial();
	void UpdateCelestialClock(const float DeltaSeconds);

public:

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
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Climate")
	bool bClimateReady = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Climate")
	bool bClimateBusy = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Climate")
	int32 ClimatePreparationCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Climate")
	double LastClimatePreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Climate")
	FString ClimateMessage;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Hydrology")
	bool bRunoffReady = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Hydrology")
	int32 RunoffPreparationCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Hydrology")
	double LastRunoffPreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Hydrology")
	double AnnualOceanRunoffCubicMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Hydrology")
	double RunoffWaterBudgetRelativeResidual = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Hydrology")
	FString RunoffMessage;

	virtual void Tick(const float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	UFUNCTION(BlueprintPure, Category = "Planet|Climate")
	bool EvaluateSeasonalSurface(
		const FVector Direction, const double Phase, FLinearColor &Coverage, FLinearColor &Factors
	) const;
	std::shared_ptr<const PlanetCore::PlanetRunoff> GetRunoffField() const;
	UFUNCTION(BlueprintPure, Category = "Planet|Hydrology")
	bool EvaluateRiverDischarge(
		const int32 ReachIndex, const double Phase, double &CubicMetersPerSecond, double &AnnualMeanCubicMetersPerSecond
	) const;
	std::optional<PlanetCore::BiomeProfile> GetBiomeProfile(const FVector Direction) const;
	UTexture2D *GetSeasonalTexture() const;
	bool PrepareCelestialAtmosphere();
	UFUNCTION(BlueprintCallable, Category = "Planet|Clock")
	bool StartCelestialClock();
	UFUNCTION(BlueprintCallable, Category = "Planet|Clock")
	bool SetSimulationUtc(const FDateTime Utc);
	APlanetWorld();
};
