#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <memory>
#include "PlanetGroundPatch.generated.h"

class APlanetTerrain;
class UProceduralMeshComponent;
class USceneComponent;
struct FPlanetGroundState;
struct FGroundRequest;

namespace PlanetCore {
	class FlowSurfaceField;
}

/** 視点の解像度変更から独立し，旧衝突を保持する局所的な足場 */
UCLASS()
class PLANETGENERATOR_API APlanetGroundPatch : public AActor {
	GENERATED_BODY()

private:

	// 所有する二面と非同期の生成状態
	UPROPERTY(Transient)
	TObjectPtr<USceneComponent> PatchRoot;
	UPROPERTY(Transient)
	TObjectPtr<UProceduralMeshComponent> BufferA;
	UPROPERTY(Transient)
	TObjectPtr<UProceduralMeshComponent> BufferB;
	TSharedPtr<FPlanetGroundState> State;
	bool IsCurrentSource(const FGroundRequest &Request) const;
	void StartPending();
	void Cancel();

public:

	// 既存のエディタ・検証スクリプトが参照する定義と公開状態
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet")
	TObjectPtr<APlanetTerrain> Terrain;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet")
	FVector AnchorDirection = FVector::UpVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet")
	double HalfExtentMeters = 16;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet")
	int32 CellsPerSide = 32;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	bool bBusy = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	int64 RequestedRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	int64 PublishedRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	double MaximumProbeErrorMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	double LastWorkerMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	FString Status;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostics", meta=(WorldContext="WorldContextObject"))
	static AActor *SpawnProbeActor(
		UObject *const WorldContextObject, const TSubclassOf<AActor> ActorClass, const FVector Location
	);
	virtual bool ShouldTickIfViewportsOnly() const override;
	virtual void Tick(const float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	UFUNCTION(BlueprintPure, Category="Planet")
	UProceduralMeshComponent *GetGroundMesh() const;
	std::shared_ptr<const PlanetCore::FlowSurfaceField> GetPublishedFlowField() const;
	UFUNCTION(BlueprintPure, Category="Planet")
	bool GetPublishedPlanet(FTransform &PlanetTransform, double &RadiusMeters) const;
	UFUNCTION(BlueprintPure, Category="Planet")
	bool EvaluatePublishedElevation(const FVector Direction, double &ElevationMeters) const;
	UFUNCTION(BlueprintPure, Category="Planet")
	bool QueryCoverage(const FVector WorldPosition, const double FootprintRadiusMeters, double &MarginMeters) const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostics")
	bool NativeTraceGround(const FVector Start, const FVector End, FVector &Impact, FVector &Normal) const;
	UFUNCTION(BlueprintCallable, Category="Planet")
	bool TraceGround(const FVector Start, const FVector End, FVector &Impact, FVector &Normal) const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostics")
	FString InspectGroundRay(const FVector Start, const FVector End) const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostics")
	bool SweepGroundCapsule(
		const FVector Start,
		const FVector End,
		const FVector Up,
		const double RadiusCm,
		const double HalfHeightCm,
		FVector &Impact,
		FVector &Normal
	) const;
	bool RequestFlowPatch(
		const std::shared_ptr<const PlanetCore::FlowSurfaceField> &Field,
		const FVector Direction,
		const FTransform &PlanetTransform
	);
	UFUNCTION(BlueprintCallable, Category="Planet")
	bool RequestPatch();
	UFUNCTION(BlueprintCallable, Category="Planet")
	bool RequestAtWorldPosition(const FVector WorldPosition);
	UFUNCTION(BlueprintCallable, Category="Planet")
	void PollPatch();
	APlanetGroundPatch();
};
