// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "PlanetGeneratorGameMode.generated.h"

/**
 *  Simple GameMode for a first person game
 */
UCLASS(abstract)
class APlanetGeneratorGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	APlanetGeneratorGameMode();
};



