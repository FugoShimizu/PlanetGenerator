#include "PlanetOceanSurface.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
namespace PlanetCore {
	double SphericalTriangleChordDistance(UnitDirection Focus, const std::array<UnitDirection, 3> &Triangle) {
		const auto F = Focus.Vector();
		double Distance = 2;
		bool Inside = true;
		for(unsigned I = 0; I < 3; ++I) {
			const auto A = Triangle[I].Vector(), B = Triangle[(I + 1) % 3].Vector(), C = Triangle[(I + 2) % 3].Vector();
			Distance = std::min(Distance, (F - A).Length());
			const auto Normal = UnitDirection::FromVector(A.Cross(B - A));
			if(!Normal) return Distance;
			const auto N = Normal->Vector();
			if((F - A).Dot(N) * (C - A).Dot(N) < 0) Inside = false;
			const auto Projected = UnitDirection::FromVector(F - N * (F - A).Dot(N));
			if(!Projected) continue;
			const auto Q = Projected->Vector();
			if(Q.Dot(A + B) > 0 && (Q - A).Dot(B - A) >= 0 && (Q - B).Dot(A - B) >= 0)
				Distance = std::min(Distance, (F - Q).Length());
		}
		return Inside ? 0 : Distance;
	}
	namespace {
		using OceanSplitRequest = std::pair<std::size_t, unsigned>;
		unsigned PredictedSplitLevels(double SquaredEdgeOrErrorRatio) {
			unsigned Levels = 1;
			while(SquaredEdgeOrErrorRatio > 4 && Levels < 4) {
				SquaredEdgeOrErrorRatio *= 0.25;
				++Levels;
			}
			return Levels;
		}
		std::optional<OceanSurfaceMesh> ProjectOceanSurface(const LodView &Input, double SeaLevel, SphereMesh Mesh,
			const std::optional<GroundFrame> &Previous, std::stop_token Stop, bool Reuse,
			OceanReuseDiagnostics *Diagnostics = nullptr, std::vector<OceanSplitRequest> *FailedTriangles = nullptr) {
			if(Diagnostics) *Diagnostics = {};
			if(FailedTriangles) FailedTriangles->clear();
			if(!Input.IsValid() || !std::isfinite(SeaLevel) || std::abs(SeaLevel) >= Input.RadiusMeters ||
				Stop.stop_requested())
				return {};
			auto Focus = Input.SurfaceObserverMeters.value_or(Input.ObserverMeters);
			if(std::abs(Focus.Length() - Input.RadiusMeters - SeaLevel) > 256) Focus = Input.ObserverMeters;
			const auto Anchor = *UnitDirection::FromVector(Focus);
			if(Mesh.Directions.size() < 12 || Mesh.Triangles.size() < 20) return {};
			OceanSurfaceMesh Result;
			Result.Sphere = std::move(Mesh);
			Result.Frame = *GroundFrame::Create(Anchor, Input.RadiusMeters, SeaLevel);
			if(Previous && Previous->RadiusMeters == Input.RadiusMeters && Previous->AnchorElevationMeters == SeaLevel &&
				Previous->SurfaceToLocal(Anchor, SeaLevel).Length() <= 128)
				Result.Frame = *Previous;
			Result.NearRadiusMeters = std::abs(Focus.Length() - Input.RadiusMeters - SeaLevel) <= 256 ? 24 : 0;
			Result.ReusedTopology = Reuse;
			Result.Positions.reserve(Result.Sphere.Directions.size());
			for(std::size_t I = 0; I < Result.Sphere.Directions.size(); ++I) {
				if((I & 127) == 0 && Stop.stop_requested()) return {};
				const auto D = Result.Sphere.Directions[I];
				const auto P = Result.Frame.SurfaceToLocal(D, SeaLevel);
				Result.Positions.push_back(P);
				if((D.Vector() - Anchor.Vector()).Length() * Input.RadiusMeters <= 32) {
					const Vec3 F{static_cast<float>(P.X * 100) / 100.0, static_cast<float>(P.Y * 100) / 100.0,
						static_cast<float>(P.Z * 100) / 100.0};
					Result.MaximumFloatErrorMeters = std::max(Result.MaximumFloatErrorMeters, (P - F).Length());
				}
			}
			if(Result.NearRadiusMeters > 0 && Result.MaximumFloatErrorMeters > 0.001) return {};
			if(Reuse) {
				const double SeaRadius = Input.RadiusMeters + SeaLevel;
				const double ObserverLength = Input.ObserverMeters.Length();
				const auto ObserverDirection = Input.ObserverMeters * (1 / ObserverLength);
				const double Horizon =
					ObserverLength > SeaRadius ? std::acos(SeaRadius / ObserverLength) : 3.14159265358979323846;
				for(std::size_t I = 0; I < Result.Sphere.Triangles.size(); ++I) {
					if((I & 127) == 0 && Stop.stop_requested()) return {};
					const auto T = Result.Sphere.Triangles[I];
					std::array<Vec3, 3> D;
					double FloatError = 0;
					for(unsigned J = 0; J < 3; ++J) {
						if(T[J] >= Result.Positions.size()) return {};
						D[J] = Result.Sphere.Directions[T[J]].Vector();
						const auto P = Result.Positions[T[J]];
						const Vec3 F{static_cast<float>(P.X * 100) / 100.0, static_cast<float>(P.Y * 100) / 100.0,
							static_cast<float>(P.Z * 100) / 100.0};
						FloatError = std::max(FloatError, (P - F).Length());
					}
					const auto CenterDirection = UnitDirection::FromVector(D[0] + D[1] + D[2]);
					if(!CenterDirection) return {};
					const auto C = CenterDirection->Vector();
					const double Cap = std::max({(D[0] - C).Length(), (D[1] - C).Length(), (D[2] - C).Length()});
					if(Cap >= std::sqrt(2.0)) return {};
					const double Edge =
						std::max({(D[0] - D[1]).Length(), (D[1] - D[2]).Length(), (D[2] - D[0]).Length()}) * SeaRadius;
					// Every convex combination of these sphere vertices has C dot P
					// at least R*cos(cap). Thus the radial deficit is <= R*capChord^2/2.
					const double ChordError = SeaRadius * 0.5 * Cap * Cap;
					if(Result.NearRadiusMeters > 0 &&
						(C - Anchor.Vector()).Length() * Input.RadiusMeters <= Cap * Input.RadiusMeters +
								Result.NearRadiusMeters + 64 * std::numeric_limits<double>::epsilon() * Input.RadiusMeters) {
						if((Edge > 2 || ChordError + FloatError > Input.SurfaceErrorMeters || FloatError > 0.001) &&
							SphericalTriangleChordDistance(Anchor,
								{Result.Sphere.Directions[T[0]], Result.Sphere.Directions[T[1]],
									Result.Sphere.Directions[T[2]]}) *
									Input.RadiusMeters <=
								Result.NearRadiusMeters + 64 * std::numeric_limits<double>::epsilon() * Input.RadiusMeters) {
							if(Diagnostics) {
								Diagnostics->RejectedNear = true;
								++Diagnostics->RejectedNearTriangles;
								Diagnostics->NearEdgeMeters = std::max(Diagnostics->NearEdgeMeters, Edge);
								Diagnostics->NearErrorMeters = std::max(Diagnostics->NearErrorMeters, ChordError + FloatError);
								Diagnostics->FloatErrorMeters = std::max(Diagnostics->FloatErrorMeters, FloatError);
							}
							if(FailedTriangles) {
								const double Ratio =
									std::max(Edge * Edge / 4, (ChordError + FloatError) / Input.SurfaceErrorMeters);
								FailedTriangles->emplace_back(I, PredictedSplitLevels(Ratio));
								continue;
							}
							return {};
						}
					}
					const double Angle = 2 * std::asin(Cap * 0.5);
					if(Horizon + Angle < 3.14159265358979323846 && C.Dot(ObserverDirection) < std::cos(Horizon + Angle))
						continue;
					const double Distance =
						std::max(1.0, (C * SeaRadius - Input.ObserverMeters).Length() - Cap * SeaRadius - FloatError);
					const double Pixels = (ChordError + FloatError) * Input.FocalLengthPixels / Distance;
					// A bounding cap can cross the horizon while the triangle is wholly
					// hidden. Only in the potential rejection case do the exact arc test.
					if(Pixels > 1 && ObserverLength > SeaRadius &&
						SphericalTriangleChordDistance(*UnitDirection::FromVector(ObserverDirection),
							{Result.Sphere.Directions[T[0]], Result.Sphere.Directions[T[1]], Result.Sphere.Directions[T[2]]}) >
							2 * std::sin(Horizon * 0.5) + 64 * std::numeric_limits<double>::epsilon())
						continue;
					Result.MaximumProjectionErrorPixels = std::max(Result.MaximumProjectionErrorPixels, Pixels);
					if(!std::isfinite(Pixels) || Pixels > 1) {
						if(Diagnostics) {
							Diagnostics->RejectedProjection = true;
							++Diagnostics->RejectedProjectionTriangles;
							Diagnostics->ProjectionErrorPixels = std::max(Diagnostics->ProjectionErrorPixels, Pixels);
						}
						if(FailedTriangles && std::isfinite(Pixels)) {
							FailedTriangles->emplace_back(I, PredictedSplitLevels(Pixels));
							continue;
						}
						return {};
					}
				}
			}
			if(FailedTriangles && !FailedTriangles->empty()) return {};
			return Result;
		}
	}
	OceanSurfaceCache::OceanSurfaceCache(double R, double H)
		: Radius(R), SeaLevel(H), Tiles(R, [H](UnitDirection) { return H; }) {
	}
	bool OceanSurfaceCache::Uses(double R, double H) const {
		return Radius == R && SeaLevel == H;
	}
	std::optional<OceanSurfaceMesh> BuildOceanSurface(const LodView &Input, double SeaLevel,
		const std::optional<GroundFrame> &Previous, std::stop_token Stop, OceanSurfaceCache *Cache) {
		if(!Input.IsValid() || !std::isfinite(SeaLevel) || std::abs(SeaLevel) >= Input.RadiusMeters || Stop.stop_requested())
			return {};
		if(Cache && !Cache->Uses(Input.RadiusMeters, SeaLevel)) return {};
		LodView View = Input;
		View.ReliefBoundMeters = std::abs(SeaLevel);
		auto Focus = Input.SurfaceObserverMeters.value_or(Input.ObserverMeters);
		if(std::abs(Focus.Length() - Input.RadiusMeters - SeaLevel) > 256) Focus = Input.ObserverMeters;
		View.SurfaceObserverMeters = Focus;
		const double Altitude = std::abs(Focus.Length() - Input.RadiusMeters - SeaLevel);
		View.SurfaceRadiusMeters = Altitude <= 256 ? 24 : 0;
		View.SurfaceEdgeMeters = 2;
		auto Selection = Cache ? SelectLodForView(View, Stop, {}, &Cache->Tiles)
							   : SelectLodForView(View, Stop, [SeaLevel](UnitDirection) { return SeaLevel; });
		if(!Selection || !Selection->SurfaceResolutionSatisfied || Stop.stop_requested()) return {};
		auto Mesh = BuildAdaptiveSphere(Selection->Tree);
		if(!Mesh || Stop.stop_requested()) return {};
		auto Result = ProjectOceanSurface(Input, SeaLevel, std::move(Mesh->Mesh), Previous, Stop, false);
		if(!Result) return {};
		Result->CachedTiles = Selection->CachedTiles;
		Result->CachedProbeHits = Selection->SurfaceProbeCacheHits;
		Result->LeafCount = Selection->Tree.Leaves().size();
		return Result;
	}
	std::optional<OceanSurfaceMesh> ReuseOceanSurface(const LodView &Input, double SeaLevel, const SphereMesh &Candidate,
		const std::optional<GroundFrame> &Previous, std::stop_token Stop, OceanReuseDiagnostics *Diagnostics) {
		return ProjectOceanSurface(Input, SeaLevel, Candidate, Previous, Stop, true, Diagnostics);
	}
	namespace {
		std::optional<OceanSurfaceMesh> RefineOceanPartition(const LodView &Input, double SeaLevel, const LodTree &TerrainTree,
			const AdaptiveSphereMesh &Candidate, const std::optional<GroundFrame> &Previous, std::stop_token Stop,
			OceanReuseDiagnostics *InitialDiagnostics, LodTree &UpdatedTree, AdaptiveSphereMesh &UpdatedMesh,
			std::size_t PartitionBudget) {
			if(InitialDiagnostics) *InitialDiagnostics = {};
			if(!Input.IsValid() || Stop.stop_requested() || TerrainTree.Leaves().size() > PartitionBudget ||
				PartitionBudget < 20 || PartitionBudget > LodTree::MaximumLeaves ||
				Candidate.TriangleOwners.size() != Candidate.Mesh.Triangles.size())
				return {};
			std::optional<LodTree> WorkingTree;
			AdaptiveSphereMesh WorkingMesh;
			const auto *Current = &Candidate;
			std::vector<OceanSplitRequest> Failed;
			std::uint32_t RefinedParents = 0;
			for(std::uint32_t Pass = 0; Pass <= 4; ++Pass) {
				auto Projected = ProjectOceanSurface(
					Input, SeaLevel, Current->Mesh, Previous, Stop, true, Pass == 0 ? InitialDiagnostics : nullptr, &Failed);
				if(Projected) {
					Projected->RefinementPasses = Pass;
					Projected->RefinedTerrainParents = RefinedParents;
					Projected->LeafCount = WorkingTree ? WorkingTree->Leaves().size() : TerrainTree.Leaves().size();
					if(WorkingTree) {
						UpdatedTree = std::move(*WorkingTree);
						UpdatedMesh = std::move(WorkingMesh);
					}
					return Projected;
				}
				if(Stop.stop_requested() || Failed.empty() || Pass == 4) return {};
				if(!WorkingTree) WorkingTree = TerrainTree;
				std::map<TileId, unsigned, TileOrder> Owners;
				for(const auto &Request : Failed) {
					const auto Owner = Current->TriangleOwners[Request.first];
					const unsigned Target = std::min<unsigned>(Input.MaximumLevel, Owner.Level + Request.second);
					auto &Level = Owners[Owner];
					Level = std::max(Level, Target);
				}
				bool Changed = false;
				for(const auto &Request : Owners) {
					std::vector<TileId> Pending{Request.first};
					if(Request.second <= Request.first.Level) return {};
					while(!Pending.empty()) {
						if(Stop.stop_requested()) return {};
						const auto Tile = Pending.back();
						Pending.pop_back();
						if(Tile.Level >= Request.second) continue;
						if(WorkingTree->IsLeaf(Tile)) {
							const auto Edit = WorkingTree->Refine(Tile, PartitionBudget);
							if(Edit.Status != LodEditStatus::Applied) return {};
							RefinedParents += static_cast<std::uint32_t>(Edit.SplitParents.size());
							Changed = true;
						}
						// Balancing can already have split an owner: walk its children
						// until the requested depth is met, then validate actual geometry.
						for(unsigned Child = 0; Child < 4; ++Child)
							Pending.push_back(*Tile.Child(static_cast<std::uint8_t>(Child)));
					}
				}
				if(!Changed) return {};
				auto Built = UpdateAdaptiveSphere(*WorkingTree, *Current, Stop);
				if(!Built || Stop.stop_requested()) return {};
				WorkingMesh = std::move(*Built);
				Current = &WorkingMesh;
			}
			return {};
		}

	}
	/**
	 * 固定骨格を含む大きな共通分割を，地表の視点選択予算と分離して検証する
	 * @param Input 有効な視点と精度条件
	 * @param
	 * SeaLevel 一定海面の標高
	 * @param SkeletonTree 全球被覆と平衡を持つ水面骨格
	 * @param Candidate
	 * 同じ木から作った整合する三角形
	 * @param Previous 前回の局所原点
	 * @param Stop 中断要求
	 * @param InitialDiagnostics
	 * 初回の精度不足の記録
	 * @return 既存の分割上限内で精度を満たす候補，失敗時は空
	 */
	std::optional<OceanSurfaceMesh> BuildOceanSurfaceFromSkeleton(const LodView &Input, double SeaLevel,
		const LodTree &SkeletonTree, const AdaptiveSphereMesh &Candidate, const std::optional<GroundFrame> &Previous,
		std::stop_token Stop, OceanReuseDiagnostics *InitialDiagnostics) {
		LodTree UpdatedTree;
		AdaptiveSphereMesh UpdatedMesh;
		// 視点の有効性を保ち，骨格と精度の追加分割にだけ既存の全球分割上限を使う返戻
		std::optional<OceanSurfaceMesh> Result = RefineOceanPartition(Input, SeaLevel, SkeletonTree, Candidate, Previous, Stop,
			InitialDiagnostics, UpdatedTree, UpdatedMesh, LodTree::MaximumLeaves);
		if(Result) {
			if(Result->RefinementPasses)
				Result->Hierarchy = SphereRefinementHierarchy{
					std::move(UpdatedTree), std::move(UpdatedMesh.TriangleOwners), UpdatedMesh.StitchMaskCounts};
			else
				Result->Hierarchy =
					SphereRefinementHierarchy{SkeletonTree, Candidate.TriangleOwners, Candidate.StitchMaskCounts};
		}
		return Result;
	}
	std::optional<OceanSurfaceMesh> BuildOceanSurfaceFromTerrain(const LodView &Input, double SeaLevel,
		const LodTree &TerrainTree, const AdaptiveSphereMesh &Candidate, const std::optional<GroundFrame> &Previous,
		std::stop_token Stop, OceanReuseDiagnostics *InitialDiagnostics, OceanSurfaceCache *Cache) {
		if(InitialDiagnostics) *InitialDiagnostics = {};
		if(!Input.IsValid() || Stop.stop_requested() || (Cache && !Cache->Uses(Input.RadiusMeters, SeaLevel))) return {};
		// A new terrain partition that already satisfies the sea needs no
		// refinement. Prefer it over growing an older water partition.
		if(Cache && TerrainTree.Leaves().size() <= Input.LeafBudget &&
			Candidate.TriangleOwners.size() == Candidate.Mesh.Triangles.size() &&
			std::all_of(TerrainTree.Leaves().begin(), TerrainTree.Leaves().end(),
				[&](TileId T) { return T.Level <= Input.MaximumLevel; })) {
			auto Direct = ProjectOceanSurface(Input, SeaLevel, Candidate.Mesh, Previous, Stop, true, InitialDiagnostics);
			if(Direct && !Stop.stop_requested()) {
				Direct->LeafCount = TerrainTree.Leaves().size();
				Cache->TopologyTree = TerrainTree;
				Cache->TopologyMesh = Candidate;
				return Direct;
			}
		}
		LodTree UpdatedTree;
		AdaptiveSphereMesh UpdatedMesh;
		if(Cache && Cache->TopologyTree && Cache->TopologyMesh && Cache->TopologyTree->Leaves().size() <= Input.LeafBudget &&
			std::all_of(Cache->TopologyTree->Leaves().begin(), Cache->TopologyTree->Leaves().end(),
				[&](TileId Tile) { return Tile.Level <= Input.MaximumLevel; })) {
			auto Result = RefineOceanPartition(Input, SeaLevel, *Cache->TopologyTree, *Cache->TopologyMesh, Previous, Stop,
				InitialDiagnostics, UpdatedTree, UpdatedMesh, Input.LeafBudget);
			if(Result && !Stop.stop_requested()) {
				if(Result->RefinementPasses > 0) {
					Cache->TopologyTree = std::move(UpdatedTree);
					Cache->TopologyMesh = std::move(UpdatedMesh);
				}
				Result->ReusedPreviousTopology = true;
				return Result;
			}
		}
		if(Stop.stop_requested()) return {};
		auto Result = RefineOceanPartition(Input, SeaLevel, TerrainTree, Candidate, Previous, Stop, InitialDiagnostics,
			UpdatedTree, UpdatedMesh, Input.LeafBudget);
		if(Result && Cache && !Stop.stop_requested()) {
			// A failed cache attempt (including budget exhaustion) leaves the old
			// complete snapshot intact until a fresh bounded partition succeeds.
			if(Result->RefinementPasses > 0) {
				Cache->TopologyTree = std::move(UpdatedTree);
				Cache->TopologyMesh = std::move(UpdatedMesh);
			} else {
				Cache->TopologyTree = TerrainTree;
				Cache->TopologyMesh = Candidate;
			}
		}
		if(Stop.stop_requested()) return {};
		return Result;
	}

}
