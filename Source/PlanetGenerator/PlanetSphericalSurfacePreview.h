#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <cstdint>
#include <memory>
#include "PlanetSphericalSurfacePreview.generated.h"

class UProceduralMeshComponent;
class APlanetGroundPatch;
class APlanetFlowTerrain;
struct FPlanetFlowPreparation;

namespace PlanetCore {
	class FlowChannelField;
	class FlowSurfaceField;
	class PlanetSphericalDrainage;
	class PlanetSphericalWarp;
	class UnitDirection;
	struct TerrainSample;
	struct GeologySettings;
	struct SpillNetwork;
	class SpillLakeShore;
}

/** 比較中の球面排水基盤を局所座標で観察する一時的な診断 Actor */
UCLASS()
class PLANETGENERATOR_API APlanetSphericalSurfacePreview : public AActor {
	GENERATED_BODY()

private:

	// 診断用の描画と不変な準備場
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	TObjectPtr<UProceduralMeshComponent> SurfaceMesh;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	TObjectPtr<UProceduralMeshComponent> WaterMesh;
	FVector PreparedAnchorDirection = FVector::ZeroVector;
	double PreparedHalfExtentMeters = 0;
	std::shared_ptr<const PlanetCore::PlanetSphericalDrainage> Surface;
	std::shared_ptr<const PlanetCore::PlanetSphericalWarp> Warp;
	std::shared_ptr<const PlanetCore::FlowChannelField> FlowPreview;
	std::shared_ptr<const PlanetCore::FlowSurfaceField> FlowNetworkPreview;
	std::shared_ptr<const FPlanetFlowPreparation> FlowPreparation;
	// 同じ不変場と初期峠に対する，確認済水系の再利用
	std::shared_ptr<const PlanetCore::SpillNetwork> ConnectedNetwork;
	std::shared_ptr<const PlanetCore::PlanetSphericalDrainage> ConnectedNetworkSource;
	FVector ConnectedInitialPass = FVector::ZeroVector;
	std::shared_ptr<const PlanetCore::SpillLakeShore> ConnectedReceivingLake;
	std::uint64_t PreparedSeed = 0;
	double PreparedRadiusMeters = 0;
	int32 PreparedReliefVertexBudget = -1;
	bool HasPreparedRefinedRelief = false;
	bool HasPreparedVertexSlopes = false;
	bool HasPreparedDrainageOnly = false;
	bool HasPreparedEdgeRefinement = false;
	bool HasPreparedRelaxation = false;
	bool HasPreparedAdaptiveDrainage = false;
	bool HasPreparedEstimatedGradients = false;
	bool IsShowingSource = false;
	static PlanetCore::TerrainSample SampleField(
		const PlanetCore::PlanetSphericalDrainage &Field,
		const PlanetCore::PlanetSphericalWarp *const Mapping,
		const PlanetCore::UnitDirection Direction,
		const bool ShouldShowSource
	);
	static std::shared_ptr<const PlanetCore::PlanetSphericalDrainage> PrepareField(
		const PlanetCore::GeologySettings &Settings,
		const bool ShouldRefineRelief,
		const bool ShouldPreserveSlopes,
		const bool ShouldConstrainDrainage,
		const bool ShouldSplitEdges,
		const bool ShouldRelaxHeights,
		const bool ShouldAdaptBeforeDrainage,
		const bool ShouldEstimateSlopes,
		const int32 ReliefBudget
	);
	bool BuildFlowChannel(
		const FVector Pass,
		const double DistanceMeters,
		const double DownstreamWaterMeters,
		const double HalfExtentMeters,
		const int32 Cells,
		const bool UseHeightOnlyQueries,
		const bool IsPositiveBranch,
		const std::shared_ptr<const PlanetCore::SpillLakeShore> &ReceivingLake
	);

public:

	// 比較する地形準備の条件
	UPROPERTY(EditAnywhere, Transient, Category="Planet|Diagnostic")
	bool ShouldLimitToDrainageEdges = false;
	UPROPERTY(EditAnywhere, Transient, Category="Planet|Diagnostic")
	bool ShouldRefineEdges = false;
	UPROPERTY(EditAnywhere, Transient, Category="Planet|Diagnostic")
	bool ShouldRelaxAddedHeights = false;
	UPROPERTY(EditAnywhere, Transient, Category="Planet|Diagnostic")
	bool ShouldAdaptBeforeDrainage = false;
	UPROPERTY(EditAnywhere, Transient, Category="Planet|Diagnostic")
	bool ShouldEstimateGradients = false;
	UPROPERTY(EditAnywhere, Transient, Category="Planet|Diagnostic")
	int32 ReliefVertexBudget = 4096;
	// 今回の準備と描画の実測
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double PreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double MeshMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double AnchorElevationMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 GeneratedVertices = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 AddedReliefVertices = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double MaximumReliefRaiseMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 RelaxedReliefVertices = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double MaximumReliefRelaxationMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	FVector MaximumReliefRaiseDirection = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double MaximumReliefCutMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	FVector MaximumReliefCutDirection = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	FString LastBuildMessage;
	// 元場に重ねる小さな湖岸候補の表示計測
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 DiagnosticWaterVertices = 0;
	// 水面が不規則な局所細分化なら０，規則的な帯なら其の列数
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 DiagnosticWaterColumns = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 DiagnosticShoreVertices = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double DiagnosticWaterAreaMetersSquared = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double DiagnosticWaterSphereErrorMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double DiagnosticShoreHeightResidualMeters = 0;
	// 視点を変えた際の河道・湖岸の再準備を検出する累積件数
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int64 DiagnosticFlowPreparations = 0;
	// 現在表示する流出口と受入水域の水位
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double DiagnosticOutletWaterMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double DiagnosticReceivingWaterMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 DiagnosticConnectedSites = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double DiagnosticRouteEndMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 DiagnosticReceivingShoreVertices = 0;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	TArray<FVector> TraceFlowRoute(const FVector Pass, const bool IsPositiveBranch, const double StopElevationMeters) const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool BuildConnectedFlowChannelPreview(
		const FVector InitialPass,
		const double DistanceMeters,
		const double HalfExtentMeters,
		const int32 Cells
	);
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool BuildFlowChannelPreview(
		const FVector Pass,
		const double DistanceMeters,
		const double DownstreamWaterMeters,
		const double HalfExtentMeters,
		const int32 Cells,
		const bool UseHeightOnlyQueries=true,
		const bool IsPositiveBranch=true
	);
	UFUNCTION(BlueprintPure, Category="Planet|Diagnostic")
	bool EvaluateFlowDirection(
		const FVector Direction,
		double &TerrainMeters,
		double &WaterMeters,
		double &Coverage,
		double &OriginalMeters
	) const;
	std::shared_ptr<const PlanetCore::FlowChannelField> GetFlowField() const;
	std::shared_ptr<const PlanetCore::FlowSurfaceField> GetFlowSurfaceField() const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool PrepareConnectedFlowNetwork();
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool RequestFlowCollision(APlanetGroundPatch *const Patch) const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool ConfigureFlowClimate(APlanetFlowTerrain *const Terrain) const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool EvaluateFlowSupport(const FVector Direction, double &SignedMeters) const;
	UFUNCTION(BlueprintPure, Category="Planet|Diagnostic")
	FVector GetDiagnosticAnchorDirection() const;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool BuildContourWaterPreview(const FVector Initial, const double WaterMeters);
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	bool BuildPreview(
		const int64 Seed,
		const double RadiusMeters,
		const FVector Direction,
		const double HalfExtentMeters,
		const int32 Cells,
		const bool ShouldRefineRelief=false,
		const bool ShouldWarpCoordinates=false,
		const bool ShouldPreserveSlopes=false,
		const bool ShouldShowSource=false
	);
	UFUNCTION(BlueprintPure, Category="Planet|Diagnostic")
	bool EvaluateDirection(const FVector Direction, double &ElevationMeters, FVector &GradientPerRadian) const;

	APlanetSphericalSurfacePreview();
};
