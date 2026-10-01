#pragma once

#include "PlanetFlowEnvironment.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <memory>
#include <optional>
#include "PlanetFlowTerrain.generated.h"

class APlanetExplorerCharacter;
class APlanetGroundPatch;
class UProceduralMeshComponent;
class UPlanetCelestialComponent;
class USceneComponent;
class UMaterialInterface;
class UTexture2D;
namespace PlanetCore {
	class FlowSurfaceField;
	class OceanConnectivityField;
}
struct FPlanetFlowTerrainState;

/** 足場と同じ不変な河床を，全球被覆を保持して非同期に表示する */
UCLASS()
class PLANETGENERATOR_API APlanetFlowTerrain : public AActor {
	GENERATED_BODY()

private:

	// 一件の生成と地面・水面それぞれ二つの公開用部品
	UPROPERTY(Transient)
	TObjectPtr<USceneComponent> TerrainRoot;
	UPROPERTY(Transient)
	TObjectPtr<UProceduralMeshComponent> BufferA;
	UPROPERTY(Transient)
	TObjectPtr<UProceduralMeshComponent> BufferB;
	UPROPERTY(Transient)
	TObjectPtr<UProceduralMeshComponent> WaterBufferA;
	UPROPERTY(Transient)
	TObjectPtr<UProceduralMeshComponent> WaterBufferB;
	UPROPERTY(Transient)
	TObjectPtr<APlanetGroundPatch> GroundSource;
	// 公開面と転送候補の気候配列を，それぞれ GC の参照に保持
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> SeasonalTexture;
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> UploadSeasonalTexture;
	TSharedPtr<FPlanetFlowTerrainState> State;
	void StartPending();
	void Cancel();
	void ApplySeasonalMaterial();
	bool ConfigureCelestialObserver();

public:

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Planet|Clock")
	TObjectPtr<UPlanetCelestialComponent> CelestialClock;
	// 実行中の身体と描画の状態；生成した地形は保存しない
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	TObjectPtr<APlanetExplorerCharacter> Explorer;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet")
	TObjectPtr<UMaterialInterface> SurfaceMaterial;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet")
	TObjectPtr<UMaterialInterface> WaterMaterial;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	bool IsBusy = false;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	int64 RequestedRevision = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	int64 PublishedRevision = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	int64 DiscardedCandidates = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	double LastWorkerMilliseconds = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	double LastUploadMilliseconds = 0;
	// ワーカー内の頂点配列構築と，ゲームスレッドの部品更新の費用
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	double LastBufferPreparationMilliseconds = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	double LastSectionUploadMilliseconds = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	double LastWaterMilliseconds = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	int64 WaterPreparations = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet")
	FString Status;
	// 年周位相は材質だけを更新し，河床・水位・衝突の定義へ混ぜない
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Climate")
	double SolarYearPhase = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet|Climate")
	bool bClimateReady = false;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet|Climate")
	int32 ClimatePreparationCount = 0;
	UPROPERTY(Transient, BlueprintReadOnly, Category="Planet|Climate")
	double LastClimatePreparationMilliseconds = 0;
	virtual bool ShouldTickIfViewportsOnly() const override;
	virtual void Tick(const float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	UFUNCTION(BlueprintPure, Category="Planet")
	UProceduralMeshComponent *GetVisibleMesh() const;
	UFUNCTION(BlueprintPure, Category="Planet")
	UProceduralMeshComponent *GetVisibleWaterMesh() const;
	std::optional<FPlanetFlowEnvironment> GetPublishedEnvironment() const;
	bool ConfigureClimate(
		std::shared_ptr<const PlanetCore::FlowSurfaceField> Field,
		std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean
	);
	UFUNCTION(BlueprintCallable, Category="Planet|Clock")
	bool StartCelestialClock();
	UFUNCTION(BlueprintCallable, Category="Planet")
	bool RequestView(APlanetGroundPatch *const Ground, const FVector ObserverWorld, const FVector BodyWorld);
	UFUNCTION(BlueprintCallable, Category="Planet")
	void PollView();
	APlanetFlowTerrain();
};
