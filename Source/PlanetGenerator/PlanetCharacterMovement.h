#pragma once

#include "CoreMinimal.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "PlanetCharacterMovement.generated.h"

/** 再生成する静的な足場と，一定の球面重力に従う身体移動 */
UCLASS()
class PLANETGENERATOR_API UPlanetCharacterMovement : public UCharacterMovementComponent {
	GENERATED_BODY()

private:

	bool IsPlanetGroundBase() const;

public:

	virtual float GetGravityZ() const override;
	virtual FVector GetImpartedMovementBaseVelocity() const override;
	virtual void UpdateBasedMovement(const float DeltaSeconds) override;
	virtual void TickComponent(
		const float DeltaTime, const ELevelTick TickType, FActorComponentTickFunction *const ThisTickFunction
	) override;
	UFUNCTION(BlueprintPure, Category="Planet")
	FVector GetPlanetGravityVector() const;
};
