#include "Misc/AutomationTest.h"
#include "Core/PlanetSurfaceMaterial.h"
#include "Core/PlanetGround.h"
#include <cmath>
#include <algorithm>
#include <limits>
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSurfaceMaterialContracts,"PlanetGenerator.SurfaceMaterial.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPlanetSurfaceMaterialContracts::RunTest(const FString& Parameters)
{
    (void)Parameters; using namespace PlanetCore;
    const auto Anchor=*UnitDirection::FromVector({1,2,3});
    TestFalse(TEXT("Texture frame rejects unsupported radius"),SurfaceTextureFrame::Create(Anchor,999,0).has_value());
    TestFalse(TEXT("Texture frame rejects nonfinite height"),SurfaceTextureFrame::Create(Anchor,1000,std::numeric_limits<double>::quiet_NaN()).has_value());
    double MaxPhaseError=0,MaxFloatError=0,MaxShaderError=0;
    const auto ShaderPosition=[](const GroundFrame& G,const SurfaceTextureFrame& F,UnitDirection D,double H) {
        const auto L=G.SurfaceToLocal(D,H);
        const float X=static_cast<float>(L.X*100)*0.01f,Y=static_cast<float>(L.Y*100)*0.01f,Z=static_cast<float>(L.Z*100)*0.01f;
        const auto Axis=[&](double P,double A,double B,double C) { return static_cast<float>(P)+static_cast<float>(A)*X+static_cast<float>(B)*Y+static_cast<float>(C)*Z; };
        return Vec3{Axis(F.Phase.X,G.East.X,G.North.X,G.Up.X),Axis(F.Phase.Y,G.East.Y,G.North.Y,G.Up.Y),Axis(F.Phase.Z,G.East.Z,G.North.Z,G.Up.Z)};
    };
    for (const double R : {1000.0,1e6,1e9})
    {
        const auto Ground=*GroundFrame::Create(Anchor,R,3);
        const auto A=*SurfaceTextureFrame::Create(Anchor,R,3);
        const auto GroundB=*GroundFrame::Create(Ground.DirectionAt(64,7),R,3);
        const auto B=*SurfaceTextureFrame::Create(Ground.DirectionAt(64,7),R,3);
        for (int I=-16;I<=16;++I) for (int J=-8;J<=8;++J)
        {
            const auto D=Ground.DirectionAt(I*7,J*7);
            const double H=3+0.2*std::sin(I*0.3)*std::cos(J*0.7);
            const auto P=A.Coordinates(D,H),Q=B.Coordinates(D,H);
            const auto Check=[&](double X,double Y) {
                double Delta=X-Y; Delta-=std::round(Delta/SurfaceTexturePeriodMeters)*SurfaceTexturePeriodMeters;
                MaxPhaseError=std::max(MaxPhaseError,std::abs(Delta));
                double F=static_cast<double>(static_cast<float>(X))-static_cast<double>(static_cast<float>(Y));
                F-=std::round(F/SurfaceTexturePeriodMeters)*SurfaceTexturePeriodMeters;
                MaxFloatError=std::max(MaxFloatError,std::abs(F));
                return std::abs(Delta)<2e-6 && std::abs(F)<0.0002;
            };
            if (!TestTrue(TEXT("Rebased texture phase matches at metre and float precision"),Check(P.X,Q.X)&&Check(P.Y,Q.Y)&&Check(P.Z,Q.Z))) return false;
            const auto ShaderA=ShaderPosition(Ground,A,D,H),ShaderB=ShaderPosition(GroundB,B,D,H);
            MaxShaderError=std::max({MaxShaderError,(ShaderA-P).Length(),(ShaderB-Q).Length()});
            if (!TestTrue(TEXT("Float shader local-position reconstruction preserves texture phase across frames"),
                MaxShaderError<0.0002 && std::abs(std::remainder(ShaderA.X-ShaderB.X,SurfaceTexturePeriodMeters))<0.0002 &&
                std::abs(std::remainder(ShaderA.Y-ShaderB.Y,SurfaceTexturePeriodMeters))<0.0002 && std::abs(std::remainder(ShaderA.Z-ShaderB.Z,SurfaceTexturePeriodMeters))<0.0002)) return false;
            const auto E=Ground.DirectionAt(I*7+0.01,J*7);
            if (!TestTrue(TEXT("Texture coordinates stay unwrapped across neighbouring vertices"),(A.Coordinates(E,H)-P).Length()<0.011)) return false;
        }
    }
    for (const Vec3 V : {Vec3{1,2,3},Vec3{0,0,1},Vec3{0,0,-1},Vec3{1e-6,1e-6,-1},Vec3{1,1,0}})
    {
        const auto N=*UnitDirection::FromVector(V); const auto T=SurfaceMaterialTangent(N);
        if (!TestTrue(TEXT("Material tangent is orthonormal including poles"),std::abs(T.Length()-1)<1e-10 && std::abs(T.Dot(N.Vector()))<1e-10)) return false;
    }
    for (double H : {-100.0,-5.0,0.0,2.0,100.0}) for (double Slope : {0.0,0.6,1.5})
    for (double Coverage : {0.0,0.4,1.0})
    {
        RiverSurfaceSample S; S.Terrain.ElevationMeters=H; S.Terrain.TangentialGradientPerRadian={Slope*(1000+H),0,0};
        S.ChannelCoverage=Coverage; S.WaterElevationMeters=H+0.3;
        const auto W=EvaluateSurfaceMaterial(Anchor,1000,0,S);
        if (!TestTrue(TEXT("Physical masks remain normalized, bounded and finite"),std::isfinite(W.Wetness) &&
            W.Rock>=0 && W.Sand>=0 && W.Soil>=-1e-15 && W.Wetness>=0 && W.Wetness<=1 &&
            std::abs(W.Rock+W.Sand+W.Soil-1)<1e-12)) return false;
    }
    AddInfo(FString::Printf(TEXT("Surface material contracts: phase error %.9g m, float phase error %.9g m, local shader reconstruction %.9g m; three radii, 561 sites each, pole tangents and 45 physical masks"),MaxPhaseError,MaxFloatError,MaxShaderError));
    return true;
}
#endif
