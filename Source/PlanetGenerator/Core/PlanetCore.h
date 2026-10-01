#pragma once

// Standard C++ only: this header is also compiled without Unreal Engine.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace PlanetCore
{
constexpr std::uint8_t MaxTileLevel = 32;
constexpr std::uint32_t TopologyVersion = 1;

struct Vec3
{
    double X = 0, Y = 0, Z = 0;
    // Keep elementary arithmetic visible to each caller; formulas unchanged.
    Vec3 operator+(Vec3 R) const { return {X+R.X,Y+R.Y,Z+R.Z}; }
    Vec3 operator-(Vec3 R) const { return {X-R.X,Y-R.Y,Z-R.Z}; }
    Vec3 operator*(double S) const { return {X*S,Y*S,Z*S}; }
    double Dot(Vec3 R) const { return X*R.X+Y*R.Y+Z*R.Z; }
    Vec3 Cross(Vec3 R) const { return {Y*R.Z-Z*R.Y,Z*R.X-X*R.Z,X*R.Y-Y*R.X}; }
    double Length() const { return std::sqrt(Dot(*this)); }
};

class UnitDirection
{
public:
    static std::optional<UnitDirection> FromVector(Vec3 Value);
    Vec3 Vector() const { return Value; }
private:
    explicit UnitDirection(Vec3 InValue) : Value(InValue) {}
    Vec3 Value;
};

inline std::optional<UnitDirection> UnitDirection::FromVector(Vec3 V)
{
    if (!std::isfinite(V.X) || !std::isfinite(V.Y) || !std::isfinite(V.Z)) return {};
    const double Scale = std::max({std::abs(V.X), std::abs(V.Y), std::abs(V.Z)});
    if (Scale == 0) return {};
    V = {V.X / Scale, V.Y / Scale, V.Z / Scale};
    return UnitDirection(V * (1.0 / V.Length()));
}

struct TileId
{
    std::uint64_t Path = 0;
    std::uint8_t RootFace = 0;
    std::uint8_t Level = 0;
    bool operator==(const TileId&) const = default;
    bool IsValid() const;
    std::optional<TileId> Child(std::uint8_t Index) const;
    std::optional<TileId> Parent() const;
    // Fixed byte order; never serializes structure padding.
    std::array<std::uint8_t, 10> Encode() const;
    static std::optional<TileId> Decode(const std::array<std::uint8_t, 10>& Bytes);
};

enum class VertexKind : std::uint8_t { Corner, Edge, Interior };
struct VertexKey
{
    VertexKind Kind = VertexKind::Corner;
    std::uint8_t Owner = 0;
    std::uint8_t Other = 0;
    std::uint8_t Level = 0;
    std::uint64_t U = 0;
    std::uint64_t V = 0;
    bool operator==(const VertexKey&) const = default;
};

struct Neighbor
{
    TileId Tile;
    std::uint8_t Edge = 0;
    bool Reversed = false;
};

// Edge i is opposite corner i; its ordered endpoints are (i+1)%3,(i+2)%3.
std::optional<VertexKey> GetVertexKey(TileId Tile, std::uint8_t Corner);
std::optional<UnitDirection> GetDirection(VertexKey Key);
std::optional<Neighbor> GetNeighbor(TileId Tile, std::uint8_t Edge);

struct SphereMesh
{
    std::vector<UnitDirection> Directions;
    std::vector<VertexKey> Keys;
    std::vector<std::array<std::uint32_t, 3>> Triangles;
};

// Uniform meshes are bounded diagnostics, not a whole-planet streaming policy.
std::optional<SphereMesh> BuildUniformSphere(std::uint8_t Level);

struct Definition
{
    std::uint64_t Seed = 42;
    std::uint32_t GeneratorVersion = 1;
    double RadiusMeters = 6371000;
    double HeightAmplitudeMeters = 1000;
    double SeaLevelMeters = 0;
    bool IsValid() const;
    std::uint64_t IdentityHash() const;
};

struct TerrainSample
{
    double ElevationMeters = 0;
    Vec3 TangentialGradientPerRadian;
};

// Smooth deterministic fixture for topology/normal tests. This is NOT the V1
// geology, ocean connectivity, hydrology, erosion, climate or biome generator.
class DiagnosticField
{
public:
    static std::optional<DiagnosticField> Create(const Definition& Definition);
    TerrainSample Evaluate(UnitDirection Direction) const;
    Vec3 SurfacePosition(UnitDirection Direction) const;
    UnitDirection SurfaceNormal(UnitDirection Direction) const;
private:
    struct Wave { Vec3 Axis; double Frequency = 0, Phase = 0, Amplitude = 0; };
    Definition Parameters;
    std::array<Wave, 12> Waves;
};
}
