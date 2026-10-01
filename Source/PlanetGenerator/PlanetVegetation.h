#pragma once

#include "Core/PlanetBiome.h"
#include "Core/PlanetRiverTerrain.h"
#include "GameFramework/Actor.h"
#include <optional>
#include "PlanetVegetation.generated.h"

class APlanetExplorerCharacter;
class APlanetFlowTerrain;
class APlanetWorld;
class UCapsuleComponent;
class UInstancedStaticMeshComponent;
class UMaterialInstanceDynamic;
class UTexture2D;
struct FPlanetVegetationState;

/** 同じ地表と季節から，身体の近くの草を保存せず生成する */
UCLASS()
class PLANETGENERATOR_API APlanetVegetation : public AActor {
	GENERATED_BODY()

private:

	// 同じ身体へ追従する地表と，所有する植生の描画部品
	UPROPERTY(Transient)
	TObjectPtr<APlanetWorld> Planet;
	UPROPERTY(Transient)
	TObjectPtr<APlanetFlowTerrain> FlowPlanet;
	UPROPERTY(Transient)
	TObjectPtr<APlanetExplorerCharacter> Explorer;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UInstancedStaticMeshComponent> GrassA;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UInstancedStaticMeshComponent> GrassB;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> GrassMaterial;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UInstancedStaticMeshComponent> Trees;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UInstancedStaticMeshComponent> TreesVariant;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UInstancedStaticMeshComponent> Conifers;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> TreeBarkMaterial;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> TreeLeafMaterial;
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInstanceDynamic> ConiferNeedleMaterial;
	TSharedPtr<FPlanetVegetationState> State;

	bool RefreshEnvironment(bool &SourceChanged);
	std::optional<PlanetCore::RiverSurfaceSample> SampleSurface(const PlanetCore::UnitDirection Direction) const;
	bool SampleSeason(const FVector Direction, const double Phase, FLinearColor &Coverage, FLinearColor &Factors) const;
	std::optional<PlanetCore::BiomeProfile> SampleBiome(const FVector Direction) const;
	UTexture2D *GetSeasonalTexture() const;
	void PrepareCandidate();
	void PublishGrass();
	void UpdateAppearance();
	void ResetTrees();
	void UpdateTrees();
	void PublishTrees();
	void UpdateTreeAppearance();

public:

	// 配置の尺度と，実際に公開した個体数・更新費用
	static constexpr double TreeSpacingMeters = 6;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	int32 TreeInstanceCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	double PublishedTreeRadiusMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	FVector PublishedTreeCenterMeters = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	int64 TreePublishedRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	double LastTreeSliceMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	double LastTreeUploadMilliseconds = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	int32 GrassInstanceCount = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	int64 PublishedRevision = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	int64 HeightEvaluations = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	int64 ReusedCandidates = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	double LastQueryMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	double LastPreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	double LastUploadMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	FVector LastUploadStagesMilliseconds = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	bool bPreparing = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Vegetation")
	FString Status;

	void Initialize(APlanetWorld *World, APlanetExplorerCharacter *Body);
	UFUNCTION(BlueprintCallable, Category="Planet|Vegetation")
	void InitializeFlow(APlanetFlowTerrain *World, APlanetExplorerCharacter *Body);
	UFUNCTION(BlueprintPure, Category="Planet|Vegetation")
	UInstancedStaticMeshComponent *GetVisibleGrass() const;
	UFUNCTION(BlueprintPure, Category="Planet|Vegetation")
	UInstancedStaticMeshComponent *GetTrees() const;
	UFUNCTION(BlueprintPure, Category="Planet|Vegetation")
	UInstancedStaticMeshComponent *GetTreeVariant() const;
	UFUNCTION(BlueprintPure, Category="Planet|Vegetation")
	UInstancedStaticMeshComponent *GetConifers() const;
	virtual void Tick(float DeltaSeconds) override;
	APlanetVegetation();
};
