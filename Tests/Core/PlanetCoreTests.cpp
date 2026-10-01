#include "PlanetCore.h"

#include <bit>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <set>

using namespace PlanetCore;
namespace
{
std::uint64_t Checks = 0;
void Require(bool Condition, const char* Message)
{
    ++Checks;
    if (!Condition) { std::cerr << "FAILED: " << Message << '\n'; std::exit(1); }
}
void CheckEdge(TileId Tile, std::uint8_t Edge)
{
    const auto Neighbor = GetNeighbor(Tile, Edge);
    Require(Neighbor.has_value(), "neighbor exists");
    Require(Neighbor->Tile.Level == Tile.Level, "neighbor level");
    Require(Neighbor->Reversed, "outward face edges reverse");
    const auto Back = GetNeighbor(Neighbor->Tile, Neighbor->Edge);
    Require(Back && Back->Tile == Tile && Back->Edge == Edge, "directed edge round trip");
    for (std::uint8_t I = 0; I < 2; ++I)
    {
        const auto A = GetVertexKey(Tile, (Edge + 1 + I) % 3);
        const auto B = GetVertexKey(Neighbor->Tile, (Neighbor->Edge + 2 - I) % 3);
        Require(A && B && *A == *B, "shared canonical vertex");
        const auto DA = GetDirection(*A), DB = GetDirection(*B);
        Require(DA && DB, "shared directions valid");
        const auto VA = DA->Vector(), VB = DB->Vector();
        Require(std::bit_cast<std::uint64_t>(VA.X) == std::bit_cast<std::uint64_t>(VB.X) &&
            std::bit_cast<std::uint64_t>(VA.Y) == std::bit_cast<std::uint64_t>(VB.Y) &&
            std::bit_cast<std::uint64_t>(VA.Z) == std::bit_cast<std::uint64_t>(VB.Z), "shared direction bit match");
    }
}
void CheckMesh(unsigned Level)
{
    const auto Mesh = BuildUniformSphere(static_cast<std::uint8_t>(Level));
    Require(Mesh.has_value(), "mesh generated");
    const auto FourL = std::uint64_t{1} << (2 * Level);
    Require(Mesh->Directions.size() == 10 * FourL + 2 && Mesh->Triangles.size() == 20 * FourL, "icosphere counts");
    std::map<std::pair<std::uint32_t,std::uint32_t>, std::pair<unsigned,int>> Edges;
    std::vector<std::set<std::uint32_t>> Neighbors(Mesh->Directions.size());
    double Area = 0;
    for (const auto& T : Mesh->Triangles)
    {
        const Vec3 A = Mesh->Directions[T[0]].Vector(), B = Mesh->Directions[T[1]].Vector(), C = Mesh->Directions[T[2]].Vector();
        Require((B - A).Cross(C - A).Dot(A + B + C) > 0, "nondegenerate outward winding");
        Area += 2 * std::atan2(std::abs(A.Dot(B.Cross(C))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A));
        for (unsigned I = 0; I < 3; ++I)
        {
            auto U = T[I], V = T[(I + 1) % 3];
            Neighbors[U].insert(V); Neighbors[V].insert(U);
            const int Sign = U < V ? 1 : -1;
            if (U > V) std::swap(U,V);
            auto& E = Edges[{U,V}]; ++E.first; E.second += Sign;
        }
    }
    Require(Edges.size() == 30 * FourL, "Euler edge count");
    for (const auto& [Key, Edge] : Edges)
    {
        (void)Key;
        Require(Edge.first == 2 && Edge.second == 0, "closed manifold, no open/reversed seam");
    }
    unsigned FiveValent = 0;
    for (const auto& Adjacent : Neighbors)
    {
        Require(Adjacent.size() == 5 || Adjacent.size() == 6, "icosphere vertex valence");
        if (Adjacent.size() == 5) ++FiveValent;
    }
    Require(FiveValent == 12, "twelve five-valent vertices");
    Require(std::abs(Area - 4 * 3.14159265358979323846) < 1e-10, "full spherical coverage");
    for (const auto& D : Mesh->Directions) Require(std::abs(D.Vector().Length() - 1) < 5e-16, "unit directions");
}
}

int main()
{
    Require(!TileId{1,0,0}.IsValid(), "unused path bits rejected");
    Require(!TileId{0,20,0}.IsValid(), "invalid root rejected");
    Require(!TileId{0,0,33}.IsValid(), "level 33 rejected");
    Require(!TileId{}.Parent(), "root has no parent");
    Require(!TileId{}.Child(4), "child 4 rejected");
    Require(!BuildUniformSphere(7), "uniform mesh memory cap");
    Require(!UnitDirection::FromVector({0,0,0}), "zero direction rejected");
    Require(!UnitDirection::FromVector({std::numeric_limits<double>::infinity(),0,0}), "infinite direction rejected");
    Require(!UnitDirection::FromVector({std::numeric_limits<double>::quiet_NaN(),0,0}), "NaN direction rejected");
    Require(UnitDirection::FromVector({1e308,1e308,1e308}).has_value(), "overflow-safe normalization");
    Require(UnitDirection::FromVector({1e-308,1e-308,1e-308}).has_value(), "underflow-safe normalization");

    for (std::uint8_t Level = 0; Level <= 4; ++Level)
        for (std::uint8_t Root = 0; Root < 20; ++Root)
            for (std::uint64_t Path = 0; Path < (std::uint64_t{1} << (2 * Level)); ++Path)
            {
                const TileId Id{Path, Root, Level};
                Require(TileId::Decode(Id.Encode()) == Id, "explicit tile encoding round trip");
                for (std::uint8_t E = 0; E < 3; ++E) CheckEdge(Id,E);
            }
    std::uint64_t State = 90210;
    for (std::uint8_t Level = 5; Level <= 32; ++Level)
        for (std::uint8_t Root = 0; Root < 20; ++Root)
            for (unsigned Trial = 0; Trial < 20; ++Trial)
            {
                TileId Id{0,Root,0};
                for (unsigned L = 0; L < Level; ++L)
                {
                    State = State * UINT64_C(6364136223846793005) + 1;
                    const std::uint8_t Child = Trial < 4 ? static_cast<std::uint8_t>(Trial) : static_cast<std::uint8_t>(State >> 62);
                    const auto Next = Id.Child(Child);
                    Require(Next && Next->Parent() == Id, "parent/child round trip");
                    Id = *Next;
                }
                Require(TileId::Decode(Id.Encode()) == Id, "deep tile encoding round trip");
                for (std::uint8_t E = 0; E < 3; ++E) CheckEdge(Id,E);
                if (Level == 32) Require(!Id.Child(0), "split at level 32 prohibited");
            }
    for (unsigned Level = 0; Level <= 5; ++Level) CheckMesh(Level);

    const Definition Parameters;
    const auto Field = DiagnosticField::Create(Parameters), Repeat = DiagnosticField::Create(Parameters);
    Require(Field && Repeat, "valid definition");
    auto Changed = Parameters; Changed.RadiusMeters *= 2;
    Require(Changed.IdentityHash() != Parameters.IdentityHash(), "radius participates in identity");
    Changed = Parameters; ++Changed.Seed;
    const auto OtherSeed = DiagnosticField::Create(Changed);
    Require(Changed.IdentityHash() != Parameters.IdentityHash(), "seed participates in identity");
    Changed = Parameters; Changed.SeaLevelMeters = -0.0;
    Require(Changed.IdentityHash() == Parameters.IdentityHash(), "negative zero normalized in identity");
    Changed = Parameters; Changed.RadiusMeters = 0;
    Require(!DiagnosticField::Create(Changed), "zero radius rejected");
    Changed = Parameters; Changed.HeightAmplitudeMeters = Changed.RadiusMeters;
    Require(!DiagnosticField::Create(Changed), "surface through center rejected");
    Changed = Parameters; Changed.SeaLevelMeters = -Changed.RadiusMeters;
    Require(!DiagnosticField::Create(Changed), "sea through center rejected");
    Changed = Parameters; Changed.GeneratorVersion = 2;
    Require(!DiagnosticField::Create(Changed), "unknown generator version rejected");
    Changed = Parameters; Changed.RadiusMeters = std::numeric_limits<double>::max() / 2;
    Require(!DiagnosticField::Create(Changed), "unsafe floating-point range rejected");
    Changed = Parameters; Changed.RadiusMeters = std::numeric_limits<double>::denorm_min(); Changed.HeightAmplitudeMeters = 0;
    const auto TinyField = DiagnosticField::Create(Changed);
    Require(TinyField.has_value(), "tiny finite diagnostic definition");
    const auto Axis = *UnitDirection::FromVector({1,0,0});
    Require(TinyField->SurfaceNormal(Axis).Vector().X == 1, "tiny radius normal avoids reciprocal overflow");
    const auto Samples = BuildUniformSphere(3);
    bool SeedChangesField = false;
    for (const auto& D : Samples->Directions)
    {
        const auto A = Field->Evaluate(D), B = Repeat->Evaluate(D);
        Require(std::bit_cast<std::uint64_t>(A.ElevationMeters) == std::bit_cast<std::uint64_t>(B.ElevationMeters), "same definition bit repeatability");
        SeedChangesField |= A.ElevationMeters != OtherSeed->Evaluate(D).ElevationMeters;
        Require(std::abs(A.ElevationMeters) <= Parameters.HeightAmplitudeMeters, "analytic amplitude bound");
        Require(std::abs(A.TangentialGradientPerRadian.Dot(D.Vector())) < 1e-10, "gradient tangent to sphere");
        const auto Tangent = UnitDirection::FromVector(D.Vector().Cross(std::abs(D.Vector().Z) < 0.8 ? Vec3{0,0,1} : Vec3{1,0,0}))->Vector();
        constexpr double Step = 1e-6;
        const auto Plus = *UnitDirection::FromVector(D.Vector() * std::cos(Step) + Tangent * std::sin(Step));
        const auto Minus = *UnitDirection::FromVector(D.Vector() * std::cos(Step) - Tangent * std::sin(Step));
        const auto Difference = (Field->Evaluate(Plus).ElevationMeters - Field->Evaluate(Minus).ElevationMeters) / (2 * Step);
        Require(std::abs(Difference - A.TangentialGradientPerRadian.Dot(Tangent)) < 2e-6, "analytic gradient versus independent finite difference");
        const auto SurfaceTangent = (Field->SurfacePosition(Plus) - Field->SurfacePosition(Minus)) * (1.0 / (2 * Step));
        Require(std::abs(Field->SurfaceNormal(D).Vector().Dot(SurfaceTangent)) / Parameters.RadiusMeters < 5e-10, "normal perpendicular to surface tangent");
    }
    Require(SeedChangesField, "seed changes diagnostic field");
    std::cout << "{\"status\":\"passed\",\"checks\":" << Checks
        << ",\"all_directed_edges_through_level\":4,\"sampled_deep_levels\":32,\"closed_mesh_levels\":5,\"production_terrain_validated\":false}\n";
}
