#include "PlanetGeology.h"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace PlanetCore;
namespace
{
std::uint64_t Checks = 0;
void Require(bool Condition, const char* Message)
{
    ++Checks;
    if (!Condition) { std::cerr << "FAILED: " << Message << '\n'; std::exit(1); }
}
UnitDirection Direction(double Z, double Phi)
{
    const double R = std::sqrt(1 - Z * Z);
    return *UnitDirection::FromVector({R * std::cos(Phi), R * std::sin(Phi), Z});
}
void CheckDerivative(const GeologyField& Field, UnitDirection D)
{
    const auto S = Field.Evaluate(D);
    const auto V = D.Vector();
    const auto T = UnitDirection::FromVector(V.Cross(std::abs(V.Z) < 0.8 ? Vec3{0,0,1} : Vec3{1,0,0}))->Vector();
    constexpr double Epsilon = 1e-7;
    const auto P = *UnitDirection::FromVector(V * std::cos(Epsilon) + T * std::sin(Epsilon));
    const auto M = *UnitDirection::FromVector(V * std::cos(Epsilon) - T * std::sin(Epsilon));
    const double FD = (Field.Evaluate(P).Terrain.ElevationMeters - Field.Evaluate(M).Terrain.ElevationMeters) / (2 * Epsilon);
    const double Analytic = S.Terrain.TangentialGradientPerRadian.Dot(T);
    Require(std::abs(FD - Analytic) < 0.04 + 2e-6 * std::abs(Analytic), "full composed geology gradient agrees with finite differences");
    Require(std::abs(S.Terrain.TangentialGradientPerRadian.Dot(V)) < 1e-7, "geology gradient is tangential");
    const auto SurfaceTangent = (Field.SurfacePosition(P) - Field.SurfacePosition(M)) * (1.0 / (2 * Epsilon));
    Require(std::abs(Field.SurfaceNormal(D).Vector().Dot(SurfaceTangent)) / Field.Settings().RadiusMeters < 2e-6, "geology normal matches displaced surface");
}
}

int main()
{
    const auto Start = std::chrono::steady_clock::now();
    GeologySettings Settings;
    Require(Settings.IsValid(), "default geology settings valid");
    auto Bad = Settings; Bad.RadiusMeters = 0;
    Require(!GeologyField::Create(Bad), "zero radius rejected");
    Bad = Settings; Bad.Version = GeologyGeneratorVersion + 1;
    Require(!GeologyField::Create(Bad), "unknown geology version rejected");
    Bad = Settings; Bad.HistoryIntervals = 1000000;
    Require(!GeologyField::Create(Bad), "unbounded history rejected");
    Bad = Settings; Bad.PlateCount = 0;
    Require(!GeologyField::Create(Bad), "empty plate set rejected");
    Bad = Settings; Bad.HistoryMillionYears = std::numeric_limits<double>::quiet_NaN();
    Require(!GeologyField::Create(Bad), "nonfinite history rejected");
    std::stop_source Cancellation; Cancellation.request_stop();
    Require(!GeologyField::Create(Settings, Cancellation.get_token()), "cancelled build publishes nothing");

    const auto A = GeologyField::Create(Settings), B = GeologyField::Create(Settings);
    Require(A && B, "repeatable bounded preparation succeeds");
    Require(A->History().size() == Settings.HistoryIntervals + 1, "finite history frames recorded");
    Require(A->Features().size() > 20, "actual geologic boundary features generated");
    std::array<unsigned, 8> Kinds{};
    bool OldFeature = false;
    for (const auto& F : A->Features())
    {
        ++Kinds[static_cast<std::size_t>(F.Kind)];
        Require(std::abs(F.Center.Length() - 1) < 1e-14 && std::abs(F.Along.Length() - 1) < 1e-14 && std::abs(F.Across.Length() - 1) < 1e-14, "geology feature frame unit vectors");
        Require(std::abs(F.Center.Dot(F.Along)) < 1e-14 && std::abs(F.Across.Dot(F.Along)) < 1e-14 && std::abs(F.Center.Dot(F.Across)) < 1e-14, "geology feature frame orthogonal");
        Require(F.PlateA < Settings.PlateCount && F.PlateB < Settings.PlateCount, "boundary feature refers to known plates");
        Require(F.AlongSine > 0 && F.AcrossSine > 0 && F.SupportCosine > 0, "bounded nonzero compact support");
        OldFeature |= F.AgeMillionYears > 0;
    }
    Require(OldFeature, "geology uses past events, not just current Voronoi cells");
    Require(Kinds[static_cast<unsigned>(GeologicFeatureKind::Spreading)] > 0, "spreading crust history exists");
    Require(Kinds[static_cast<unsigned>(GeologicFeatureKind::Trench)] > 0, "subduction history exists");
    bool SitesMoved = false;
    for (std::uint32_t I = 0; I < Settings.PlateCount; ++I)
        SitesMoved |= (A->History().front().Sites[I] - A->History().back().Sites[I]).Length() > 0.001;
    Require(SitesMoved, "plate sites rotate during history");
    Require(A->History().front().AgeMillionYears == Settings.HistoryMillionYears && A->History().back().AgeMillionYears == 0, "history has fixed past and present epochs");

    double Minimum = 1e30, Maximum = -1e30, Checksum = 0;
    const auto EvaluationStart = std::chrono::steady_clock::now();
    for (unsigned I = 0; I < 1024; ++I)
    {
        const auto D = Direction(1 - 2 * (I + 0.5) / 1024, I * 2.399963229728653);
        const auto X = A->Evaluate(D), Y = B->Evaluate(D);
        Require(std::isfinite(X.Terrain.ElevationMeters) && std::isfinite(X.Terrain.TangentialGradientPerRadian.Length()), "finite geology sample");
        Require(std::bit_cast<std::uint64_t>(X.Terrain.ElevationMeters) == std::bit_cast<std::uint64_t>(Y.Terrain.ElevationMeters), "same seed/version/settings produce bit-identical elevation in this build");
        Require(std::abs(X.Terrain.ElevationMeters - Settings.SeaLevelMeters) <= A->AbsoluteReliefBoundMeters(), "sample stays inside the declared global relief bound");
        Require(X.OceanCrustAgeMillionYears >= 0 && X.OceanCrustAgeMillionYears <= Settings.HistoryMillionYears + 1, "finite ocean crust age range");
        Require(X.RockResistance >= 0.45 && X.RockResistance <= 0.9, "bounded continuous rock resistance");
        CheckDerivative(*A, D);
        Minimum = std::min(Minimum, X.Terrain.ElevationMeters);
        Maximum = std::max(Maximum, X.Terrain.ElevationMeters);
        Checksum += X.Terrain.ElevationMeters;
    }
    Require(Minimum < -2500 && Maximum > 500, "land and deep ocean are actual height geometry");
    Require(A->Statistics().InitialOceanFractionEstimate >= 0.60 && A->Statistics().InitialOceanFractionEstimate <= 0.70, "default geology independent ocean estimate is in the requested interval");
    Require(A->Statistics().CalibrationProbeCount != A->Statistics().AreaProbeCount, "area estimate does not reuse calibration quadrature");
    Require(std::isfinite(A->Statistics().DatumShiftMeters), "finite datum adjustment");
    const double EvaluationSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - EvaluationStart).count();
    Settings.Seed = 90210;
    const auto Different = GeologyField::Create(Settings);
    Require(Different && Different->Settings().IdentityHash() != A->Settings().IdentityHash(), "seed participates in geology identity");
    Require(Different->Evaluate(Direction(0.2, 0.3)).Terrain.ElevationMeters != A->Evaluate(Direction(0.2, 0.3)).Terrain.ElevationMeters, "seed changes geography");
    const auto BaseHash = Settings.IdentityHash();
    Settings.SeaLevelMeters = -0.0;
    Require(Settings.IdentityHash() == BaseHash, "geology identity normalizes negative zero");
    Settings.RadiusMeters = 1000;
    Require(Settings.IdentityHash() != BaseHash, "radius participates in geology identity");
    const auto Small = GeologyField::Create(Settings);
    Require(Small.has_value(), "small legal planet supported");
    Require(Small->AbsoluteReliefBoundMeters() < Settings.RadiusMeters * 0.2, "relief cannot cross the center on a small planet");
    CheckDerivative(*Small, Direction(0.6, 1.2));
    std::cout << "{\"status\":\"passed\",\"checks\":" << Checks << ",\"features\":" << A->Features().size()
        << ",\"initial_ocean_fraction_estimate\":" << A->Statistics().InitialOceanFractionEstimate
        << ",\"min_elevation_m\":" << Minimum << ",\"max_elevation_m\":" << Maximum
        << ",\"gradient_workload_seconds\":" << EvaluationSeconds << ",\"checksum\":" << Checksum
        << ",\"total_seconds\":" << std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count()
        << ",\"final_ocean_connectivity_validated\":false,\"production_terrain_validated\":false}\n";
}
