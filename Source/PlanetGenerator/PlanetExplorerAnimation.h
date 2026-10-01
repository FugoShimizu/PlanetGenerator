#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimSingleNodeInstance.h"
#include "PlanetExplorerAnimation.generated.h"

class UAnimSequence;
class UBlendSpace;

struct FPlanetExplorerAnimationProxy;

UENUM(BlueprintType)
enum class EPlanetLocomotionPhase : uint8 { Ground, Rising, Falling, Landing };

/** 身体の接線速度と実際の離着地を，既存の姿勢資産へ接続する */
UCLASS(Transient, NotBlueprintable)
class PLANETGENERATOR_API UPlanetExplorerAnimation : public UAnimSingleNodeInstance {
GENERATED_BODY()
private:

	friend struct FPlanetExplorerAnimationProxy;

	// 並列の姿勢評価中も，所有する資産を回収させない参照
	UPROPERTY(Transient)
	TObjectPtr<UBlendSpace> Locomotion;
	UPROPERTY(Transient)
	TObjectPtr<UAnimSequence> Rise;
	UPROPERTY(Transient)
	TObjectPtr<UAnimSequence> Fall;
	UPROPERTY(Transient)
	TObjectPtr<UAnimSequence> Land;

protected:

	virtual FAnimInstanceProxy *CreateAnimInstanceProxy() override;

public:

	// ゲームスレッドで取得した姿勢入力；描画中の骨を物理の入力へ戻さない
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Animation")
	double TangentialSpeedCentimetersPerSecond = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Animation")
	double VerticalSpeedCentimetersPerSecond = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Animation")
	double DirectionDegrees = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet|Animation")
	EPlanetLocomotionPhase MotionPhase = EPlanetLocomotionPhase::Ground;
	virtual void NativeInitializeAnimation() override;

	UPlanetExplorerAnimation(const FObjectInitializer &ObjectInitializer);
};
