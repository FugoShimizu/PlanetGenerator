#pragma once
#include "PlanetCore.h"
namespace PlanetCore
{
// Both instantiations use the same scalar operations and evaluation order.
// Height queries omit derivative arithmetic at compile time; no approximation.
template<bool Derivatives> struct TerrainJet
{
    double V=0;
    Vec3 G;
    TerrainJet operator+(TerrainJet B) const
    { if constexpr (Derivatives) return {V+B.V,G+B.G}; else return {V+B.V,{}}; }
    TerrainJet operator-(TerrainJet B) const
    { if constexpr (Derivatives) return {V-B.V,G-B.G}; else return {V-B.V,{}}; }
    TerrainJet operator*(TerrainJet B) const
    { if constexpr (Derivatives) return {V*B.V,G*B.V+B.G*V}; else return {V*B.V,{}}; }
    TerrainJet operator*(double B) const
    { if constexpr (Derivatives) return {V*B,G*B}; else return {V*B,{}}; }
    TerrainJet operator/(TerrainJet B) const
    { if constexpr (Derivatives) return {V/B.V,(G*B.V-B.G*V)*(1.0/(B.V*B.V))}; else return {V/B.V,{}}; }
};
// Scalar instantiation carries no gradient payload. Accept the same expression
// initializers so the formula remains shared with the derivative path.
template<> struct TerrainJet<false>
{
    double V=0;
    TerrainJet()=default;
    TerrainJet(double Value,Vec3) : V(Value) {}
    TerrainJet operator+(TerrainJet B) const { return {V+B.V,{}}; }
    TerrainJet operator-(TerrainJet B) const { return {V-B.V,{}}; }
    TerrainJet operator*(TerrainJet B) const { return {V*B.V,{}}; }
    TerrainJet operator*(double B) const { return {V*B,{}}; }
    TerrainJet operator/(TerrainJet B) const { return {V/B.V,{}}; }
};
}
