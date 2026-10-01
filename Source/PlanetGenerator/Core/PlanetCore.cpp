#include "PlanetCore.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace PlanetCore
{


bool TileId::IsValid() const
{
    if (RootFace >= 20 || Level > MaxTileLevel) return false;
    return Level == 32 || (Path >> (2 * Level)) == 0;
}
std::optional<TileId> TileId::Child(std::uint8_t Index) const
{
    if (!IsValid() || Index >= 4 || Level == MaxTileLevel) return {};
    return TileId{(Path << 2) | Index, RootFace, static_cast<std::uint8_t>(Level + 1)};
}
std::optional<TileId> TileId::Parent() const
{
    if (!IsValid() || Level == 0) return {};
    return TileId{Path >> 2, RootFace, static_cast<std::uint8_t>(Level - 1)};
}
std::array<std::uint8_t, 10> TileId::Encode() const
{
    std::array<std::uint8_t, 10> Bytes{RootFace, Level};
    for (unsigned I = 0; I < 8; ++I) Bytes[I + 2] = static_cast<std::uint8_t>(Path >> (8 * I));
    return Bytes;
}
std::optional<TileId> TileId::Decode(const std::array<std::uint8_t, 10>& Bytes)
{
    TileId Id{0, Bytes[0], Bytes[1]};
    for (unsigned I = 0; I < 8; ++I) Id.Path |= std::uint64_t(Bytes[I + 2]) << (8 * I);
    return Id.IsValid() ? std::optional<TileId>(Id) : std::nullopt;
}

namespace
{
using Barycentric = std::array<std::int64_t, 3>;
using TriangleAddress = std::array<Barycentric, 3>;
constexpr std::array<std::array<std::uint8_t, 3>, 20> Faces{{
    {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
    {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
    {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
    {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
}};

const std::array<Vec3, 12>& Corners()
{
    static const auto Values = [] {
        const double T = (1.0 + std::sqrt(5.0)) / 2.0;
        std::array<Vec3, 12> Result{{{-1,T,0},{1,T,0},{-1,-T,0},{1,-T,0},
            {0,-1,T},{0,1,T},{0,-1,-T},{0,1,-T},{T,0,-1},{T,0,1},{-T,0,-1},{-T,0,1}}};
        for (auto& V : Result) V = UnitDirection::FromVector(V)->Vector();
        return Result;
    }();
    return Values;
}

Barycentric Midpoint(Barycentric A, Barycentric B)
{
    return {(A[0] + B[0]) / 2, (A[1] + B[1]) / 2, (A[2] + B[2]) / 2};
}

template <typename T>
std::array<T, 3> SelectChild(const std::array<T, 3>& V, T AB, T BC, T CA, std::uint8_t Child)
{
    switch (Child)
    {
    case 0: return {V[0], AB, CA};
    case 1: return {AB, V[1], BC};
    case 2: return {CA, BC, V[2]};
    default: return {AB, BC, CA};
    }
}

TriangleAddress Address(TileId Id)
{
    const std::int64_t D = std::int64_t{1} << Id.Level;
    TriangleAddress V{{{D,0,0},{0,D,0},{0,0,D}}};
    for (unsigned Step = 0; Step < Id.Level; ++Step)
    {
        const auto Child = static_cast<std::uint8_t>((Id.Path >> (2 * (Id.Level - Step - 1))) & 3);
        V = SelectChild(V, Midpoint(V[0], V[1]), Midpoint(V[1], V[2]), Midpoint(V[2], V[0]), Child);
    }
    return V;
}

std::uint8_t Descend(Barycentric& P, std::int64_t Total)
{
    for (std::uint8_t I = 0; I < 3; ++I)
    {
        if (2 * P[I] >= Total)
        {
            for (auto& Value : P) Value *= 2;
            P[I] -= Total;
            return I;
        }
    }
    P = {Total - 2 * P[2], Total - 2 * P[0], Total - 2 * P[1]};
    return 3;
}

VertexKey CanonicalKey(std::uint8_t Root, std::uint8_t Level, Barycentric B)
{
    const std::int64_t D = std::int64_t{1} << Level;
    for (unsigned I = 0; I < 3; ++I)
        if (B[I] == D) return {VertexKind::Corner, Faces[Root][I], 0, 0, 0, 0};
    while (Level > 0 && ((B[0] | B[1] | B[2]) & 1) == 0)
    {
        for (auto& Value : B) Value /= 2;
        --Level;
    }
    for (unsigned I = 0; I < 3; ++I)
    {
        if (B[I] == 0)
        {
            unsigned J = (I + 1) % 3, K = (I + 2) % 3;
            if (Faces[Root][J] > Faces[Root][K]) std::swap(J, K);
            return {VertexKind::Edge, Faces[Root][J], Faces[Root][K], Level, static_cast<std::uint64_t>(B[K]), 0};
        }
    }
    return {VertexKind::Interior, Root, 0, Level, static_cast<std::uint64_t>(B[0]), static_cast<std::uint64_t>(B[1])};
}

std::optional<Neighbor> RootNeighbor(std::uint8_t Root, std::uint8_t Edge)
{
    const auto A = Faces[Root][(Edge + 1) % 3], B = Faces[Root][(Edge + 2) % 3];
    for (std::uint8_t R = 0; R < 20; ++R)
        for (std::uint8_t E = 0; E < 3; ++E)
            if (Faces[R][(E + 1) % 3] == B && Faces[R][(E + 2) % 3] == A)
                return Neighbor{TileId{0,R,0}, E, true};
    return {};
}

std::uint64_t Mix(std::uint64_t Value)
{
    Value = (Value ^ (Value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    Value = (Value ^ (Value >> 27)) * UINT64_C(0x94d049bb133111eb);
    return Value ^ (Value >> 31);
}
struct KeyHash
{
    std::size_t operator()(VertexKey K) const
    {
        const auto Meta = std::uint64_t(K.Kind) | (std::uint64_t(K.Owner) << 8) |
            (std::uint64_t(K.Other) << 16) | (std::uint64_t(K.Level) << 24);
        return static_cast<std::size_t>(Mix(Meta ^ Mix(K.U) ^ (Mix(K.V) >> 1)));
    }
};
}

std::optional<VertexKey> GetVertexKey(TileId Tile, std::uint8_t Corner)
{
    if (!Tile.IsValid() || Corner >= 3) return {};
    return CanonicalKey(Tile.RootFace, Tile.Level, Address(Tile)[Corner]);
}

std::optional<UnitDirection> GetDirection(VertexKey K)
{
    if (K.Level > MaxTileLevel) return {};
    if (K.Kind == VertexKind::Corner)
    {
        if (K.Owner >= 12 || K.Other || K.Level || K.U || K.V) return {};
        return UnitDirection::FromVector(Corners()[K.Owner]);
    }
    const std::int64_t D = std::int64_t{1} << K.Level;
    if (K.Kind == VertexKind::Edge)
    {
        if (K.Owner >= K.Other || K.Other >= 12 || K.U == 0 || K.U >= std::uint64_t(D) || K.V || !(K.U & 1)) return {};
        bool IsEdge = false;
        for (const auto& Face : Faces)
            IsEdge |= std::find(Face.begin(), Face.end(), K.Owner) != Face.end() &&
                std::find(Face.begin(), Face.end(), K.Other) != Face.end();
        if (!IsEdge) return {};
        Vec3 A = Corners()[K.Owner], B = Corners()[K.Other];
        std::int64_t Lo = 0, Hi = D;
        for (unsigned Step = 0; Step < K.Level; ++Step)
        {
            const auto Mid = (Lo + Hi) / 2;
            const auto Direction = UnitDirection::FromVector(A + B);
            if (K.U == std::uint64_t(Mid)) return Direction;
            if (K.U < std::uint64_t(Mid)) { Hi = Mid; B = Direction->Vector(); }
            else { Lo = Mid; A = Direction->Vector(); }
        }
        return {};
    }
    if (K.Kind != VertexKind::Interior || K.Owner >= 20 || K.Other || K.U == 0 || K.V == 0 ||
        K.U >= std::uint64_t(D) || K.V >= std::uint64_t(D) || K.U + K.V >= std::uint64_t(D)) return {};
    Barycentric B{static_cast<std::int64_t>(K.U), static_cast<std::int64_t>(K.V), D - static_cast<std::int64_t>(K.U + K.V)};
    if (((B[0] | B[1] | B[2]) & 1) == 0) return {};
    const auto F = Faces[K.Owner];
    std::array<Vec3, 3> V{Corners()[F[0]], Corners()[F[1]], Corners()[F[2]]};
    for (unsigned Step = 0; Step < K.Level; ++Step)
    {
        const auto Child = Descend(B, D);
        V = SelectChild(V, UnitDirection::FromVector(V[0] + V[1])->Vector(),
            UnitDirection::FromVector(V[1] + V[2])->Vector(), UnitDirection::FromVector(V[2] + V[0])->Vector(), Child);
    }
    for (unsigned I = 0; I < 3; ++I)
        if (B[I] == D) return UnitDirection::FromVector(V[I]);
    return {};
}

std::optional<Neighbor> GetNeighbor(TileId Tile, std::uint8_t Edge)
{
    if (!Tile.IsValid() || Edge >= 3) return {};
    const auto V = Address(Tile);
    const auto J = (Edge + 1) % 3, K = (Edge + 2) % 3;
    Barycentric P{};
    for (unsigned I = 0; I < 3; ++I) P[I] = 2 * (V[J][I] + V[K][I]) - V[Edge][I];
    auto Root = Tile.RootFace;
    for (std::uint8_t I = 0; I < 3; ++I)
    {
        if (P[I] >= 0) continue;
        const auto Adjacent = RootNeighbor(Root, I);
        if (!Adjacent) return {};
        const auto NextRoot = Adjacent->Tile.RootFace;
        Barycentric Mapped{};
        Mapped[Adjacent->Edge] = -P[I];
        for (unsigned A = 0; A < 3; ++A)
            for (unsigned B = 0; B < 3; ++B)
                if (A != I && Faces[Root][A] == Faces[NextRoot][B]) Mapped[B] = P[A] + P[I];
        P = Mapped;
        Root = NextRoot;
        break;
    }
    const std::int64_t Total = 3 * (std::int64_t{1} << Tile.Level);
    if (*std::min_element(P.begin(), P.end()) < 0 || P[0] + P[1] + P[2] != Total) return {};
    TileId Other{0, Root, Tile.Level};
    for (unsigned Step = 0; Step < Tile.Level; ++Step) Other.Path = (Other.Path << 2) | Descend(P, Total);
    const auto A = *GetVertexKey(Tile, static_cast<std::uint8_t>(J));
    const auto B = *GetVertexKey(Tile, static_cast<std::uint8_t>(K));
    for (std::uint8_t E = 0; E < 3; ++E)
    {
        const auto C = *GetVertexKey(Other, (E + 1) % 3), D = *GetVertexKey(Other, (E + 2) % 3);
        if (A == D && B == C) return Neighbor{Other, E, true};
        if (A == C && B == D) return Neighbor{Other, E, false};
    }
    return {};
}

std::optional<SphereMesh> BuildUniformSphere(std::uint8_t Level)
{
    if (Level > 6) return {};
    SphereMesh Mesh;
    const auto FaceChildren = std::uint64_t{1} << (2 * Level);
    Mesh.Triangles.reserve(static_cast<std::size_t>(20 * FaceChildren));
    Mesh.Directions.reserve(static_cast<std::size_t>(10 * FaceChildren + 2));
    Mesh.Keys.reserve(Mesh.Directions.capacity());
    std::unordered_map<VertexKey, std::uint32_t, KeyHash> Indices;
    Indices.reserve(Mesh.Directions.capacity());
    for (std::uint8_t Root = 0; Root < 20; ++Root)
    {
        for (std::uint64_t Path = 0; Path < FaceChildren; ++Path)
        {
            std::array<std::uint32_t, 3> Triangle{};
            for (std::uint8_t Corner = 0; Corner < 3; ++Corner)
            {
                const auto Key = *GetVertexKey({Path, Root, Level}, Corner);
                const auto [It, Inserted] = Indices.try_emplace(Key, static_cast<std::uint32_t>(Mesh.Directions.size()));
                if (Inserted)
                {
                    const auto Direction = GetDirection(Key);
                    if (!Direction) return {};
                    Mesh.Keys.push_back(Key);
                    Mesh.Directions.push_back(*Direction);
                }
                Triangle[Corner] = It->second;
            }
            Mesh.Triangles.push_back(Triangle);
        }
    }
    return Mesh;
}

bool Definition::IsValid() const
{
    return GeneratorVersion == 1 && std::isfinite(RadiusMeters) && std::isfinite(HeightAmplitudeMeters) &&
        std::isfinite(SeaLevelMeters) && RadiusMeters > 0 && HeightAmplitudeMeters >= 0 &&
        RadiusMeters > HeightAmplitudeMeters && RadiusMeters + SeaLevelMeters > 0 &&
        std::isfinite(RadiusMeters + HeightAmplitudeMeters) && std::isfinite(RadiusMeters + SeaLevelMeters) &&
        RadiusMeters + HeightAmplitudeMeters <= std::numeric_limits<double>::max() / 64.0;
}

std::uint64_t Definition::IdentityHash() const
{
    std::uint64_t Hash = UINT64_C(14695981039346656037);
    const auto Add = [&Hash](std::uint64_t Value, unsigned Bytes) {
        for (unsigned I = 0; I < Bytes; ++I) { Hash ^= (Value >> (8 * I)) & 255; Hash *= UINT64_C(1099511628211); }
    };
    const auto Bits = [](double Value) { return std::bit_cast<std::uint64_t>(Value == 0 ? 0.0 : Value); };
    Add(1, 4); // Diagnostic field family; must change for the production generator.
    Add(GeneratorVersion, 4); Add(Seed, 8); Add(Bits(RadiusMeters), 8);
    Add(Bits(HeightAmplitudeMeters), 8); Add(Bits(SeaLevelMeters), 8);
    return Hash;
}

std::optional<DiagnosticField> DiagnosticField::Create(const Definition& Definition)
{
    if (!Definition.IsValid()) return {};
    DiagnosticField Field;
    Field.Parameters = Definition;
    std::uint64_t State = Definition.Seed;
    const auto Random = [&State]() {
        State += UINT64_C(0x9e3779b97f4a7c15);
        return static_cast<double>(Mix(State) >> 11) * 0x1.0p-53;
    };
    constexpr double Pi = 3.1415926535897932384626433832795;
    for (unsigned I = 0; I < Field.Waves.size(); ++I)
    {
        const double Z = 2 * Random() - 1, Phi = 2 * Pi * Random(), R = std::sqrt(std::max(0.0, 1 - Z * Z));
        Field.Waves[I] = {{R * std::cos(Phi), R * std::sin(Phi), Z},
            2.0 + I * 1.5, 2 * Pi * Random(), Definition.HeightAmplitudeMeters / Field.Waves.size()};
    }
    return Field;
}

TerrainSample DiagnosticField::Evaluate(UnitDirection Direction) const
{
    const auto D = Direction.Vector();
    TerrainSample Sample;
    Vec3 Ambient;
    for (const auto& Wave : Waves)
    {
        const double Phase = Wave.Frequency * Wave.Axis.Dot(D) + Wave.Phase;
        Sample.ElevationMeters += Wave.Amplitude * std::sin(Phase);
        Ambient = Ambient + Wave.Axis * (Wave.Amplitude * Wave.Frequency * std::cos(Phase));
    }
    Sample.TangentialGradientPerRadian = Ambient - D * Ambient.Dot(D);
    return Sample;
}
Vec3 DiagnosticField::SurfacePosition(UnitDirection Direction) const
{
    return Direction.Vector() * (Parameters.RadiusMeters + Evaluate(Direction).ElevationMeters);
}
UnitDirection DiagnosticField::SurfaceNormal(UnitDirection Direction) const
{
    const auto Sample = Evaluate(Direction);
    // The scaled form avoids overflow from 1/radius for very small valid inputs.
    return *UnitDirection::FromVector(Direction.Vector() * (Parameters.RadiusMeters + Sample.ElevationMeters) -
        Sample.TangentialGradientPerRadian);
}
}
