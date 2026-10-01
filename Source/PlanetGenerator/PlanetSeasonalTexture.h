#pragma once

class UTexture2D;
namespace PlanetCore {
	class PlanetSeasonalMap;
}

UTexture2D *CreatePlanetSeasonalTexture(const PlanetCore::PlanetSeasonalMap &Map);
