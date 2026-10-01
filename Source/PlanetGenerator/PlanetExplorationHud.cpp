#include "PlanetExplorationHud.h"
#include "PlanetGameMode.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

/** 準備中の地表を隠し，準備状態又は探索の操作案内を描く */
void APlanetExplorationHud::DrawHUD() {
	Super::DrawHUD();
	const APlanetGameMode *const Mode = GetWorld()->GetAuthGameMode<APlanetGameMode>();
	// 対象のワールド又は描画先が無い場合の終了
	if(!Mode || !Canvas || !GEngine) return;
	const float Scale = FMath::Clamp(Canvas->ClipY / 900.0F, 0.7F, 1.5F);
	if(Mode->ExplorationStage != EPlanetExplorationStage::Ready) {
		DrawRect(FLinearColor(0.025F, 0.035F, 0.05F, 1), 0, 0, Canvas->ClipX, Canvas->ClipY);
		DrawText(TEXT("手続型惑星地形生成器"), FLinearColor::White, 32, Canvas->ClipY * 0.42F, GEngine->GetMediumFont(), Scale);
		DrawText(Mode->PreparationMessage, FLinearColor(0.7F, 0.8F, 0.9F), 32, Canvas->ClipY * 0.42F + 42,
			GEngine->GetMediumFont(), Scale);
	} else {
		DrawRect(FLinearColor(0, 0, 0, 0.45F), 12, Canvas->ClipY - 40, Canvas->ClipX - 24, 28);
		DrawText(TEXT("WASD 移動　Shift 走行　Space ジャンプ　1 一人称　2 三人称　3 俯瞰　ホイール 拡大・縮小"),
			FLinearColor::White, 20, Canvas->ClipY - 35, GEngine->GetSmallFont(), Scale);
	}
	// 終了
	return;
}
