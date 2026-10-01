#pragma once

#include "Core/PlanetGeology.h"
#include "CoreMinimal.h"
#include <memory>

class UTexture2D;
namespace PlanetCore {
	class FlowSurfaceField;
	class PlanetClimate;
	class PlanetSeasonalMap;
}

/** 公開した描画と同じ不変場・気候を，植生へ一組として渡す */
struct FPlanetFlowEnvironment {
	// 不変場の寿命を共有し，季節配列の複製を避ける
	std::shared_ptr<const PlanetCore::FlowSurfaceField> Surface;
	std::shared_ptr<const PlanetCore::PlanetClimate> Climate;
	std::shared_ptr<const PlanetCore::PlanetSeasonalMap> Seasons;
	PlanetCore::GeologySettings Settings;
	// 惑星座標と，実際に公開した精密な描画範囲
	FTransform PlanetTransform;
	FVector ObserverMeters = FVector::ZeroVector;
	double SurfaceRadiusMeters = 0;
	double Phase = 0;
	TWeakObjectPtr<UTexture2D> Texture;
};
