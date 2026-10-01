#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <memory>
#include "PlanetTerrain.generated.h"
namespace PlanetCore {
	class RiverTerrainField;
	class DrainageNetwork;
}
class USceneComponent;
class UProceduralMeshComponent;
class UMaterialInterface;
struct FPlanetTerrainState;

/** 不変な地形場と非同期の生成を共有し，準備中も公開済みの被覆を保持する */
UCLASS()
class PLANETGENERATOR_API APlanetTerrain : public AActor {
	GENERATED_BODY()
private:
	UPROPERTY(VisibleAnywhere, Transient, Category = "Components")
	TObjectPtr<USceneComponent> PlanetRoot;
	UPROPERTY(VisibleAnywhere, Transient, Category = "Components")
	TObjectPtr<UProceduralMeshComponent> MeshA;
	UPROPERTY(VisibleAnywhere, Transient, Category = "Components")
	TObjectPtr<UProceduralMeshComponent> MeshB;
	UPROPERTY(VisibleAnywhere, Transient, Category = "Components")
	TObjectPtr<UProceduralMeshComponent> OceanA;
	UPROPERTY(VisibleAnywhere, Transient, Category = "Components")
	TObjectPtr<UProceduralMeshComponent> OceanB;
	TSharedPtr<FPlanetTerrainState> State;
	void StartPendingGeneration();
	void StartPendingWaterGeneration();
	void StartSurfacePreparation(FVector BodyMeters, FVector VelocityMeters);
	void CancelGeneration();

public:
	APlanetTerrain();
	std::shared_ptr<const PlanetCore::RiverTerrainField> GetPublishedField() const;
	std::shared_ptr<const PlanetCore::DrainageNetwork> GetDrainageNetwork() const;
	UFUNCTION(BlueprintPure, Category = "Planet|Diagnostics|Drainage")
	int32 GetCoastalOutletNodeCount() const;
	virtual void Tick(float DeltaSeconds) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	virtual bool ShouldTickIfViewportsOnly() const override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Definition")
	int64 Seed = 42;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Material")
	bool bDiagnosticHeightColors = false;
	// 水位と水際の描画精度を実機検証してから通常表示へ有効化する
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Diagnostics|Water")
	bool bRenderRiverWater = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Definition",
		meta = (Units = "m", ClampMin = "1000", ClampMax = "1000000000"))
	double RadiusMeters = 1000000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View")
	FVector ObserverMeters = FVector(0, 0, 1100000);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View", meta = (ClampMin = "20", ClampMax = "32768"))
	int32 LeafBudget = 32768;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View", meta = (ClampMin = "1", ClampMax = "32"))
	int32 MaximumLevel = 32;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View", meta = (ClampMin = "1", ClampMax = "1024"))
	double TargetEdgePixels = 32;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View")
	double FocalLengthPixels = 540;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View")
	bool bFollowPlayerCamera = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Definition")
	bool bGenerateOnBeginPlay = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View")
	bool bRefineNearSurface = true;
	// The wider surface accuracy region is enabled only for the exploration game.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|View")
	double FoliagePrecisionRadiusMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double VisibleSurfaceRadiusMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double VisibleFoliageRadiusMeters = 0;
	// 保持する数値原点とは別に，近景の細分化範囲の中心を記録する
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	FVector VisibleSurfaceObserverMeters = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	FVector VisibleSurfacePathEndMeters = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double MaximumSurfaceProbeErrorMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double MaximumSurfaceFloatErrorMeters = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Appearance")
	TObjectPtr<UMaterialInterface> SurfaceMaterial;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Appearance")
	TObjectPtr<UMaterialInterface> OceanMaterial;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Ocean")
	int32 VisibleOceanTriangleCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Ocean")
	double LastOceanBuildMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Ocean")
	double MaximumOceanFloatErrorMeters = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Generation")
	bool bEnableSurfacePreparation = true;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	bool bPreparationBusy = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 PreparationSlices = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 InterruptedPreparations = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 PreparedTileVisits = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 PreparedHeightEvaluations = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double LastPreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	bool bGenerationBusy = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	bool bWaterGenerationBusy = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 RequestedRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 PublishedRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 PublishedWaterRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int64 EarlySurfacePublications = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double LastSurfaceWorkerMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int32 DiscardedRequests = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Drainage")
	int32 DrainageNodeCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Drainage")
	int32 RiverReachCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Drainage")
	int32 RiverMouthCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Drainage")
	double MaximumRequiredBankRaiseMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int32 VisibleVertexCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int32 VisibleTriangleCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int32 VisibleLeafCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	int32 VisibleMaximumLevel = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double LastWorkerMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double LastFieldPreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double LastUploadMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	double LastSwapMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	FString GenerationMessage;
	// Retained while the next candidate is running, for stage-cost diagnosis.
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet")
	FString LastPublishedGenerationDetails;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet|Generation")
	FString LastPublishedWaterDetails;

	UFUNCTION(BlueprintCallable, Category = "Planet|Diagnostics|Drainage")
	bool EvaluateRiverProfile(int32 ReachIndex, double DownstreamFraction, FVector &LocalWaterPositionMeters,
		double &WaterElevationMeters, double &BedElevationMeters, FVector &DownstreamTangent) const;
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Planet|Generation")
	bool RequestGeneration();
	UFUNCTION(BlueprintCallable, Category = "Planet|Generation")
	void PollGeneration();
	UFUNCTION(BlueprintPure, Category = "Planet|Generation")
	UProceduralMeshComponent *GetVisibleMesh() const;
	UFUNCTION(BlueprintPure, Category = "Planet|Ocean")
	UProceduralMeshComponent *GetVisibleOceanMesh() const;
	UFUNCTION(BlueprintPure, Category = "Planet|Generation")
	bool EvaluateDirection(FVector Direction, double &ElevationMeters, FVector &SurfaceNormal) const;
	UFUNCTION(BlueprintPure, Category = "Planet|Diagnostics|Water")
	bool EvaluateWaterDirection(FVector Direction, double &WaterMeters, double &GroundMeters) const;
};
