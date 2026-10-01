#pragma once
#include "../../Source/PlanetGenerator/Core/PlanetLod.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <string>
#include <map>
#include <numeric>
#include <set>

namespace PlanetTests
{
using namespace PlanetCore;
struct LodCheckReport { std::uint64_t Checks=0; std::string Failure; };
inline bool Verify(LodCheckReport& Report, bool Condition, const char* Message)
{
    ++Report.Checks;
    if (!Condition && Report.Failure.empty()) Report.Failure=Message;
    return Condition;
}

inline bool CheckPartition(const LodTree& Tree, LodCheckReport& Report)
{
    std::array<double, 20> Coverage{};
    for (const auto Tile : Tree.Leaves())
    {
        if (!Verify(Report, Tile.IsValid(), "valid leaf")) return false;
        Coverage[Tile.RootFace] += std::ldexp(1.0, -2 * Tile.Level);
        auto Parent = Tile.Parent();
        while (Parent)
        {
            if (!Verify(Report, !Tree.IsLeaf(*Parent), "no ancestor overlaps a leaf")) return false;
            Parent = Parent->Parent();
        }
        for (std::uint8_t Edge = 0; Edge < 3; ++Edge)
        {
            const auto Adjacent = *GetNeighbor(Tile, Edge);
            const auto Cover = Tree.CoveringLeaf(Adjacent.Tile);
            if (Cover)
            {
                if (!Verify(Report, Tile.Level - Cover->Level <= 1, "neighbor at most one level coarser")) return false;
            }
            else
            {
                if (!Verify(Report, Tile.Level < MaxTileLevel, "deepest leaf cannot have a finer neighbor")) return false;
                for (std::uint8_t I = 1; I <= 2; ++I)
                    if (!Verify(Report, Tree.IsLeaf(*Adjacent.Tile.Child((Adjacent.Edge + I) % 3)), "exactly two fine boundary leaves")) return false;
            }
        }
    }
    for (const auto Area : Coverage) if (!Verify(Report, std::abs(Area - 1) < 1e-13, "every root has complete nonoverlapping coverage")) return false;
    return true;
}

inline bool CheckMesh(const LodTree& Tree, LodCheckReport& Report)
{
    const auto Built = BuildAdaptiveSphere(Tree);
    if (!Verify(Report, Built.has_value(), "adaptive mesh generated")) return false;
    const auto& Mesh = Built->Mesh;
    if (!Verify(Report, Built->TriangleOwners.size() == Mesh.Triangles.size(), "every triangle has a leaf owner")) return false;
    if (!Verify(Report, Mesh.Keys.size() == Mesh.Directions.size(), "every vertex has a canonical key")) return false;
    if (!Verify(Report, Mesh.Triangles.size() >= Tree.Leaves().size() && Mesh.Triangles.size() <= 4 * Tree.Leaves().size(), "bounded stitch triangle count")) return false;
    if (!Verify(Report, std::accumulate(Built->StitchMaskCounts.begin(), Built->StitchMaskCounts.end(), std::size_t{0}) == Tree.Leaves().size(), "all leaves stitched")) return false;
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::pair<unsigned, int>> Edges;
    std::vector<std::vector<std::uint32_t>> Adjacency(Mesh.Directions.size());
    double Area = 0;
    for (std::size_t T = 0; T < Mesh.Triangles.size(); ++T)
    {
        const auto Triangle = Mesh.Triangles[T];
        if (!Verify(Report, Tree.IsLeaf(Built->TriangleOwners[T]), "triangle belongs to the current leaf snapshot")) return false;
        for (const auto I : Triangle) if (!Verify(Report, I < Mesh.Directions.size(), "valid mesh index")) return false;
        const auto A = Mesh.Directions[Triangle[0]].Vector();
        const auto B = Mesh.Directions[Triangle[1]].Vector();
        const auto C = Mesh.Directions[Triangle[2]].Vector();
        const double Triple = A.Dot((B - A).Cross(C - A));
        if (!Verify(Report, Triple > 0, "nondegenerate outward adaptive triangle")) return false;
        Area += 2 * std::atan2(Triple, 1 + A.Dot(B) + B.Dot(C) + C.Dot(A));
        for (unsigned I = 0; I < 3; ++I)
        {
            auto U = Triangle[I], V = Triangle[(I + 1) % 3];
            Adjacency[U].push_back(V); Adjacency[V].push_back(U);
            const int Sign = U < V ? 1 : -1;
            if (U > V) std::swap(U, V);
            auto& Entry = Edges[{U, V}]; ++Entry.first; Entry.second += Sign;
        }
    }
    for (const auto& [Endpoints, Entry] : Edges)
    {
        (void)Endpoints;
        if (!Verify(Report, Entry.first == 2 && Entry.second == 0, "no open edge, T-junction, overlap or reversed seam")) return false;
    }
    if (!Verify(Report, Mesh.Directions.size() + Mesh.Triangles.size() == Edges.size() + 2, "Euler sphere topology")) return false;
    if (!Verify(Report, std::abs(Area - 4 * 3.14159265358979323846) < 1e-10, "stitched triangles cover 4pi")) return false;
    std::vector<bool> Visited(Mesh.Directions.size(), false);
    std::vector<std::uint32_t> Queue{0}; Visited[0] = true;
    for (std::size_t I = 0; I < Queue.size(); ++I)
        for (const auto Other : Adjacency[Queue[I]])
            if (!Visited[Other]) { Visited[Other] = true; Queue.push_back(Other); }
    if (!Verify(Report, Queue.size() == Mesh.Directions.size(), "one connected surface, no unused vertices")) return false;
    return true;
}

inline bool RunLodContractChecks(LodCheckReport& Report,std::shared_ptr<LodNeighborCache> Cache={})
{
    LodTree Initial(Cache);
    if (!Verify(Report, Initial.Leaves().size() == 20, "initial coverage is twenty root leaves")) return false;
    if (!CheckPartition(Initial,Report)) return false; if (!CheckMesh(Initial,Report)) return false;
    if (!Verify(Report, Initial.Refine({0, 0, 0}, 19).Status == LodEditStatus::InvalidBudget, "budget below root coverage rejected")) return false;
    if (!Verify(Report, Initial.Refine({0, 0, 0}, LodTree::MaximumLeaves + 1).Status == LodEditStatus::InvalidBudget, "unbounded allocation rejected")) return false;
    if (!Verify(Report, Initial.Refine({0, 20, 0}, 200).Status == LodEditStatus::NotLeaf, "invalid tile rejected")) return false;
    const auto Before = Initial.Leaves();
    const auto Rejected = Initial.Refine({0, 0, 0}, 22);
    if (!Verify(Report, Rejected.Status == LodEditStatus::BudgetExceeded && Rejected.SplitParents.empty() && Initial.Leaves() == Before, "budget rejection preserves old coverage")) return false;

    // Exercise every stitch template at each root, including all five-valent
    // corners; root adjacency rotates the templates into every orientation.
    for (std::uint8_t Root = 0; Root < 20; ++Root)
        for (std::uint8_t Mask = 0; Mask < 8; ++Mask)
        {
            LodTree Tree(Cache);
            const TileId Target{0, Root, 0};
            for (std::uint8_t Edge = 0; Edge < 3; ++Edge)
                if (Mask & (1 << Edge))
                    if (!Verify(Report, Tree.Refine(GetNeighbor(Target, Edge)->Tile, 100).Status == LodEditStatus::Applied, "neighbor refinement applied")) return false;
            std::uint8_t ActualMask = 0;
            for (std::uint8_t Edge = 0; Edge < 3; ++Edge)
                if (Tree.HasFinerNeighbor(Target, Edge)) ActualMask |= static_cast<std::uint8_t>(1 << Edge);
            if (!Verify(Report, ActualMask == Mask, "requested stitch mask realized")) return false;
            if (!CheckPartition(Tree,Report)) return false; if (!CheckMesh(Tree,Report)) return false;
        }

    LodTree Deep(Cache);
    TileId Cursor{0, 0, 0};
    for (unsigned Level = 0; Level < MaxTileLevel; ++Level)
    {
        const auto Edit = Deep.Refine(Cursor, 20000);
        if (!Verify(Report, Edit.Status == LodEditStatus::Applied, "deep balanced refinement succeeds with finite budget")) return false;
        Cursor = *Cursor.Child(0);
        if (!CheckPartition(Deep,Report)) return false;
    }
    if (!Verify(Report, Deep.Refine(Cursor, 20000).Status == LodEditStatus::DepthLimit, "depth limit preserves coverage")) return false;
    if (!CheckMesh(Deep,Report)) return false;
    // Some merges next to the refinement chain must be rejected. Test that a
    // rejected transaction never deletes the children's existing coverage.
    bool SawBalanceRejection = false;
    std::set<TileId, TileOrder> Parents;
    for (const auto Tile : Deep.Leaves()) if (Tile.Parent()) Parents.insert(*Tile.Parent());
    for (const auto Parent : Parents)
    {
        LodTree Copy = Deep;
        const auto Old = Copy.Leaves();
        const auto Result = Copy.Coarsen(Parent);
        if (Result != LodEditStatus::Applied) if (!Verify(Report, Copy.Leaves() == Old, "rejected merge is atomic")) return false;
        if (Result == LodEditStatus::NeighborTooFine) SawBalanceRejection = true;
        if (!CheckPartition(Copy,Report)) return false;
    }
    if (!Verify(Report, SawBalanceRejection, "unbalanced merge case exercised")) return false;

    // A deterministic sequence mixes refinements, budget failures and merges.
    LodTree Moving(Cache);
    std::uint64_t State = 29403;
    unsigned Applied = 0, Merged = 0;
    for (unsigned Step = 0; Step < 500; ++Step)
    {
        State = State * UINT64_C(6364136223846793005) + 1;
        auto It = Moving.Leaves().begin();
        std::advance(It, static_cast<std::ptrdiff_t>((State >> 16) % Moving.Leaves().size()));
        const auto Tile = *It;
        const auto Old = Moving.Leaves();
        if (Step % 3 == 0 && Tile.Parent())
        {
            const auto Result = Moving.Coarsen(*Tile.Parent());
            if (Result == LodEditStatus::Applied) ++Merged;
            else if (!Verify(Report, Moving.Leaves() == Old, "moving merge failure retains previous partition")) return false;
        }
        else
        {
            const auto Budget = Step % 5 == 0 ? Moving.Leaves().size() : std::size_t{2400};
            const auto Edit = Moving.Refine(Tile, Budget);
            if (Edit.Status == LodEditStatus::Applied) ++Applied;
            else if (!Verify(Report, Moving.Leaves() == Old, "moving refinement failure retains previous partition")) return false;
        }
        if (!CheckPartition(Moving,Report)) return false;
        if (Step % 25 == 0) if (!CheckMesh(Moving,Report)) return false;
    }
    if (!Verify(Report, Applied > 100 && Merged > 10, "mixed refinement/coarsening workload exercised")) return false;
    if (!CheckMesh(Moving,Report)) return false;
    const auto A = BuildAdaptiveSphere(Moving), B = BuildAdaptiveSphere(Moving);
    if (!Verify(Report, A->Mesh.Keys == B->Mesh.Keys && A->Mesh.Triangles == B->Mesh.Triangles, "snapshot output order is deterministic")) return false;
    for (std::size_t I = 0; I < A->Mesh.Directions.size(); ++I)
    {
        const auto VA = A->Mesh.Directions[I].Vector(), VB = B->Mesh.Directions[I].Vector();
        if (!Verify(Report, std::bit_cast<std::uint64_t>(VA.X) == std::bit_cast<std::uint64_t>(VB.X) &&
                std::bit_cast<std::uint64_t>(VA.Y) == std::bit_cast<std::uint64_t>(VB.Y) &&
                std::bit_cast<std::uint64_t>(VA.Z) == std::bit_cast<std::uint64_t>(VB.Z), "adaptive directions repeat bit exactly")) return false;
    }
    return true;
}
}
