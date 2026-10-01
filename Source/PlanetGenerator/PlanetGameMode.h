#pragma once

#include "GameFramework/GameModeBase.h"
#include "PlanetGameMode.generated.h"

class APlanetWorld;
class APlanetGroundPatch;
class APlanetExplorerCharacter;

UENUM(BlueprintType)
enum class EPlanetExplorationStage : uint8 { Planet, LandingSite, Surface, Ground, Ready, Failed };

/** 準備済みの惑星へ，地表と足場の公開後にプレイヤーを配置する */
UCLASS()
class PLANETGENERATOR_API APlanetGameMode : public AGameModeBase {
	GENERATED_BODY()

private:

	// 着地探索と操作開始迄の所有状態
	UPROPERTY(Transient)
	TObjectPtr<APlayerController> ExplorerController;
	FVector LandingDirection = FVector::UpVector;
	FVector LandingForward = FVector::ForwardVector;
	FVector LandingSunDirection = FVector::UpVector;
	double LandingElevation = 0;
	double BestLandingScore = -1;
	double BestVisibleRiseMeters = 0;
	double BestSunDot = 0;
	double BestLandingOffsetMeters = 0;
	double PreparationStartedSeconds = 0;
	double LandingSearchStartedSeconds = 0;
	double LandingSearchCpuMilliseconds = 0;
	int32 LandingCandidate = 0;
	int32 BestLandingIndex = -1;
	int32 SurfaceLeafBudget = 32768;
	int64 SurfaceRevision = 0;
	int32 AutomaticRunFramesRemaining = 0;
	int32 AutomaticRunRequestedFrames = 0;
	int64 AutomaticRunInitialCoverageMisses = 0;
	int64 AutomaticRunInitialGroundRefreshRequests = 0;
	FVector AutomaticRunOrigin = FVector::ZeroVector;
	bool bBestLandingDaylit = false;
	bool bBestSightlineBlocked = false;

	void FailPreparation(const FString &Message, const FString &Details);
	void ConsiderLandingDirection(const FVector &Direction, const int32 Index, const double OffsetMeters);
	void ConsiderLandingCandidate(const int32 Index);
	void PrepareSky();
	void PrepareLandingSurface();
	void PrepareGround();
	void EnterExploration();

protected:

	virtual void HandleStartingNewPlayer_Implementation(APlayerController *const NewPlayer) override;

public:

	// 操作画面へ公開する探索の対象と準備状態
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet")
	TObjectPtr<APlanetWorld> Planet;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet")
	TObjectPtr<APlanetGroundPatch> Ground;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet")
	TObjectPtr<APlanetExplorerCharacter> Explorer;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet")
	EPlanetExplorationStage ExplorationStage = EPlanetExplorationStage::Planet;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category = "Planet")
	FString PreparationMessage = TEXT("惑星を準備しています");

	virtual void BeginPlay() override;
	virtual void Tick(const float DeltaSeconds) override;
	APlanetGameMode();
};
