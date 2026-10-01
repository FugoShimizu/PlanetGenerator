#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <cstdint>
#include <memory>
#include "PlanetContinentalPreview.generated.h"

class UProceduralMeshComponent;

namespace PlanetCore {
	class SphericalContinentalField;
}

/** 解析的な大陸候補を保存せず比較する診断 Actor，完成した惑星又は歩行面ではない */
UCLASS()
class PLANETGENERATOR_API APlanetContinentalPreview : public AActor {
	GENERATED_BODY()

private:

	// 同じ不変場の再利用と，診断メッシュの所有
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	TObjectPtr<UProceduralMeshComponent> PreviewMesh;
	std::shared_ptr<const PlanetCore::SphericalContinentalField> Field;
	std::uint64_t PreparedSeed = 0;
	double PreparedRadiusMeters = 0;
	FVector AnchorDirection = FVector::ZeroVector;
	std::shared_ptr<const PlanetCore::SphericalContinentalField> PrepareDefinition(std::uint64_t Seed, double RadiusMeters);

	void PublishMesh(
		const std::shared_ptr<const PlanetCore::SphericalContinentalField> &Candidate,
		std::uint64_t Seed,
		const TArray<FVector> &Vertices,
		const TArray<int32> &Triangles,
		const TArray<FVector> &Normals,
		const TArray<FLinearColor> &Colors
	);

public:

	// 此の診断表示の結果と実測，ゲームの性能合格とは別
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	bool HasSucceededLastBuild = false;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	FString LastBuildMessage;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double PreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double MeshMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double OceanFraction = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	double AnchorElevationMeters = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 GeneratedVertices = 0;
	UPROPERTY(VisibleAnywhere, Transient, Category="Planet|Diagnostic")
	int32 GeneratedTriangles = 0;
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	void BuildGlobalPreview(int64 Seed, double RadiusMeters, int32 SubdivisionLevel, double HeightScale=1);
	UFUNCTION(BlueprintCallable, Category="Planet|Diagnostic")
	void BuildLocalPreview(int64 Seed, double RadiusMeters, FVector Direction, double HalfExtentMeters, int32 Cells);
	UFUNCTION(BlueprintPure, Category="Planet|Diagnostic")
	bool CanEvaluateDirection(FVector Direction, double &ElevationMeters, FVector &Normal) const;
	UFUNCTION(BlueprintPure, Category="Planet|Diagnostic")
	FVector GetAnchorDirection() const;

	APlanetContinentalPreview();
};
