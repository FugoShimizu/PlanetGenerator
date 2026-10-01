#pragma once

#include "CoreMinimal.h"

class AActor;
struct FPlanetAtmosphereState;

/** 元の大気・雲から半径に応じた寸法を決め，終了時に所有した変更だけを復元する */
class FPlanetAtmosphere {
private:

	// 初回適用前の部品値と，同じ世界の一時設定
	TSharedPtr<FPlanetAtmosphereState> State;

public:

	bool Apply(AActor &Owner, const FVector CenterWorld, const double RadiusMeters);
	void Restore();
};
