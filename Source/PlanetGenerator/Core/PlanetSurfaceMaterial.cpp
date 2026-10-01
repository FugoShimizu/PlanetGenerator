#include "PlanetSurfaceMaterial.h"
#include <algorithm>
#include <cmath>
namespace PlanetCore
{
namespace
{
double Smooth(double A,double B,double X)
{
    const double T=std::clamp((X-A)/(B-A),0.0,1.0); return T*T*T*(10+T*(-15+6*T));
}
double Phase(double X)
{
    const double P=std::fmod(X,SurfaceTexturePeriodMeters); return P<0?P+SurfaceTexturePeriodMeters:P;
}
}
SurfaceMaterialWeights EvaluateSurfaceMaterial(UnitDirection Direction,double Radius,double SeaLevel,const RiverSurfaceSample& S)
{
    (void)Direction;
    const double H=S.Terrain.ElevationMeters;
    const double Slope=S.Terrain.TangentialGradientPerRadian.Length()/(Radius+H);
    SurfaceMaterialWeights W; W.Rock=Smooth(0.35,1.1,Slope);
    const double Marine=Smooth(-0.15,0.1,SeaLevel-H);
    const double River=S.ChannelCoverage*Smooth(-0.05,0.15,S.WaterElevationMeters-H);
    W.Wetness=1-(1-Marine)*(1-River);
    // Elevation-based littoral and channel sediment are provisional until
    // sediment transport supplies physically classified substrate masks.
    const double Littoral=(1-Smooth(1.5,4.0,H-SeaLevel))*(1-Smooth(20,80,SeaLevel-H));
    const double Sand=std::clamp(Littoral+(1-Littoral)*River*0.65,0.0,1.0);
    W.Sand=(1-W.Rock)*Sand; W.Soil=1-W.Rock-W.Sand; return W;
}
Vec3 SurfaceMaterialTangent(UnitDirection Direction)
{
    const auto N=Direction.Vector();
    if (N.Z < -0.999999) return UnitDirection::FromVector({N.X*N.Y,-1+N.Y*N.Y,N.Z*N.Y})->Vector();
    const double A=1/(1+N.Z);
    return UnitDirection::FromVector({1-N.X*N.X*A,-N.X*N.Y*A,-N.X})->Vector();
}
std::optional<SurfaceTextureFrame> SurfaceTextureFrame::Create(UnitDirection Anchor,double R,double H)
{
    if (!std::isfinite(R) || !std::isfinite(H) || R<1000 || R+H<=0) return {};
    SurfaceTextureFrame F; F.Up=Anchor.Vector(); F.Radius=R; F.Elevation=H;
    const auto Origin=F.Up*(R+H); F.Phase={PlanetCore::Phase(Origin.X),PlanetCore::Phase(Origin.Y),PlanetCore::Phase(Origin.Z)};
    return F;
}
Vec3 SurfaceTextureFrame::Coordinates(UnitDirection D,double H) const
{
    // Stable subtraction before float vertex storage. Rebase offsets are
    // whole periods, so 2m repeats and 64m macro variation keep their phase.
    return (D.Vector()-Up)*(Radius+H)+Up*(H-Elevation)+Phase;
}
}
