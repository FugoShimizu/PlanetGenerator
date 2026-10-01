#pragma once

#include "GameFramework/HUD.h"
#include "PlanetExplorationHud.generated.h"

/** 生成中の待機画面と探索時の操作案内 */
UCLASS()
class PLANETGENERATOR_API APlanetExplorationHud : public AHUD {
	GENERATED_BODY()

public:

	virtual void DrawHUD() override;
};
