#pragma once
#include "PlanetRiverTerrain.h"
namespace PlanetCore
{
inline constexpr double SurfaceTexturePeriodMeters=512;
struct SurfaceMaterialWeights { double Rock=0,Sand=0,Soil=1,Wetness=0; };
// Initial physical surface masks, not a substitute for climate/biome generation.
SurfaceMaterialWeights EvaluateSurfaceMaterial(UnitDirection Direction,double Radius,double SeaLevel,const RiverSurfaceSample& Sample);
Vec3 SurfaceMaterialTangent(UnitDirection Normal);
struct SurfaceTextureFrame
{
    Vec3 Up,Phase;
    double Radius=0,Elevation=0;
    static std::optional<SurfaceTextureFrame> Create(UnitDirection Anchor,double Radius,double Elevation);
    Vec3 Coordinates(UnitDirection Direction,double Height) const;
};
}
