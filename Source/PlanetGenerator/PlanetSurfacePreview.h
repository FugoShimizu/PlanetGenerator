#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Core/PlanetCore.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include <memory>
#include "PlanetSurfacePreview.generated.h"

class UProceduralMeshComponent;

/** Bounded diagnostic of the standard-C++ sphere core, not a completed planet. */
UCLASS()
class PLANETGENERATOR_API APlanetSurfacePreview : public AActor
{
    GENERATED_BODY()
public:
    APlanetSurfacePreview();
    virtual void OnConstruction(const FTransform& Transform) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Definition")
    int64 Seed = 42;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Definition", meta=(Units="m", ClampMin="1", ClampMax="1000000000"))
    double RadiusMeters = 6371000;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Geology")
    bool bUseGeologicalBase = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Ocean")
    bool bConnectOceanBasins = false;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    int32 OceanComponentsBefore = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    int32 OceanComponentsAfter = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    int32 OceanCorridorCount = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    int32 ActiveOceanVersion = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    double ConnectedOceanFractionEstimate = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    double OceanPreparationMilliseconds = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Ocean")
    double OceanMaximumSampleLoweringMeters = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    int32 GeologicFeatureCount = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    int32 ActiveGeologyVersion = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    double InitialOceanFractionEstimate = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    int32 GeologyCalibrationProbeCount = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    int32 GeologyAreaProbeCount = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    double GeologyContinentalScale = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    double GeologyDatumShiftMeters = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    double GeologyTargetOceanFraction = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    double PreparedDefinitionMilliseconds = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Geology")
    double LastPreparationMilliseconds = 0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Diagnostic", meta=(Units="m", ClampMin="0"))
    double HeightAmplitudeMeters = 1000;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Diagnostic", meta=(ClampMin="0", ClampMax="6"))
    int32 SubdivisionLevel = 4;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Diagnostic")
    int32 GeneratedVertexCount = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Diagnostic")
    int32 GeneratedTriangleCount = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Diagnostic")
    double LastBuildMilliseconds = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Diagnostic")
    FString LastBuildMessage;

    UFUNCTION(BlueprintCallable, CallInEditor, Category="Planet|Diagnostic")
    bool RebuildPreview();
    UFUNCTION(BlueprintPure, Category="Planet|Diagnostic")
    bool EvaluateDirection(FVector Direction, double& ElevationMeters, FVector& SurfaceNormal) const;

private:
    UPROPERTY(VisibleAnywhere, Transient, Category="Components")
    TObjectPtr<UProceduralMeshComponent> PreviewMesh;
    std::optional<PlanetCore::DiagnosticField> Field;
    std::shared_ptr<const PlanetCore::GeologyField> GeologicalField;
    std::shared_ptr<const PlanetCore::OceanConnectivityField> OceanField;
};
