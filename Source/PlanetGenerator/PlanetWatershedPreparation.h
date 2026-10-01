#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <memory>
#include "PlanetWatershedPreparation.generated.h"

namespace PlanetCore {
	struct PreparedFlowWatershed;
}

struct FPlanetWatershedPreparationState;

/** 地質から一水系までを一つのワーカーで準備し，不変な定義だけを公開する */
UCLASS()
class PLANETGENERATOR_API APlanetWatershedPreparation : public AActor {
GENERATED_BODY()
private:

	// アクターを捕捉しない作業と，完了後の共有所有
	TSharedPtr<FPlanetWatershedPreparationState> State;

public:

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	bool IsBusy = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	bool IsReady = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	double PreparationMilliseconds = 0;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Transient, Category="Planet")
	FString Status;
	UFUNCTION(BlueprintCallable, Category="Planet")
	bool RequestPreparation(const int64 Seed, const double RadiusMeters, const FVector Direction);
	UFUNCTION(BlueprintCallable, Category="Planet")
	void PollPreparation();
	UFUNCTION(BlueprintCallable, Category="Planet")
	void CancelPreparation();
	std::shared_ptr<const PlanetCore::PreparedFlowWatershed> GetPrepared() const;
	virtual void Tick(const float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	virtual void Destroyed() override;

	APlanetWatershedPreparation();
};
