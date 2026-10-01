#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "PlanetExplorerCharacter.generated.h"

class APlanetTerrain;
class APlanetGroundPatch;
class UCameraComponent;
class UPlanetCelestialComponent;
class USpringArmComponent;

UENUM(BlueprintType)
enum class EPlanetViewMode : uint8 { FirstPerson, ThirdPerson, Overview };

/** 公開済の不変場と足場に従って移動し，三視点を提供する */
UCLASS()
class PLANETGENERATOR_API APlanetExplorerCharacter : public ACharacter {
	GENERATED_BODY()

private:

	// 身体に追従する三視点と，押下中の操作状態
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UCameraComponent> FirstCamera;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<USpringArmComponent> ThirdArm;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UCameraComponent> ThirdCamera;
	UPROPERTY(VisibleAnywhere, Category="Components")
	TObjectPtr<UCameraComponent> OverviewCamera;
	// 身体の地表を照らす共通時計への非所有参照
	UPROPERTY(Transient)
	TWeakObjectPtr<UPlanetCelestialComponent> ClockSource;
	double InputForward = 0;
	double InputRight = 0;
	double ViewPitch = 0;
	double OverviewPitch = 65;
	double OverviewYaw = 180;
	double OverviewDistance = 100;
	bool IsRunning = false;
	bool IsForwardPressed = false;
	bool IsBackwardPressed = false;
	bool IsLeftPressed = false;
	bool IsRightPressed = false;
	void UpdateCameras();
	void UpdateSceneExposure();
	void UpdateKeyboard();
	void ForwardDown();
	void ForwardUp();
	void BackwardDown();
	void BackwardUp();
	void LeftDown();
	void LeftUp();
	void RightDown();
	void RightUp();
	void RunDown();
	void RunUp();
	void JumpDown();
	void JumpUp();
	void FirstView();
	void ThirdView();
	void OverviewView();
	void MouseX(const float Value);
	void MouseY(const float Value);
	void Zoom(const float Value);

public:

	// 既存の操作・検証入口が参照する身体と公開状態
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	TObjectPtr<APlanetTerrain> Terrain;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	TObjectPtr<APlanetGroundPatch> Ground;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	EPlanetViewMode ViewMode = EPlanetViewMode::FirstPerson;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Movement")
	double WalkSpeedMetersPerSecond = 4.5;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Movement")
	double RunSpeedMetersPerSecond = 9;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Planet|Movement")
	double PreparationHorizonSeconds = 0.75;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	double TraversalSeconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	double GroundMarginMeters = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	int64 CoverageMissFrames = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	int64 GroundRefreshRequests = 0;
	UFUNCTION(BlueprintCallable, Category="Planet")
	static APlanetExplorerCharacter *SpawnOnPreparedGround(
		APlanetGroundPatch *const PreparedGround, const FVector LocalDirection, const FVector LocalForward
	);
	virtual void BeginPlay() override;
	virtual void Tick(const float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent *const Input) override;
	UFUNCTION(BlueprintCallable, Category="Planet")
	void SetMoveInput(const double Forward, const double Right, const bool bRun);
	void SetCelestialClock(UPlanetCelestialComponent *const Clock);
	UFUNCTION(BlueprintCallable, Category="Planet")
	void SetViewMode(const EPlanetViewMode Mode);
	UFUNCTION(BlueprintCallable, Category="Planet")
	void Look(const double YawDegrees, const double PitchDegrees);
	APlanetExplorerCharacter(const FObjectInitializer &ObjectInitializer);
};
