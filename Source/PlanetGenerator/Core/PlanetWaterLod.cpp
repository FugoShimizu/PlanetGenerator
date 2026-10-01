#include "PlanetWaterLod.h"

namespace PlanetCore {
	namespace {
		/** 深い親から順に戻し，同じ段階の順序を固定する */
		struct WaterParentOrder {
			bool operator()(const TileId Left, const TileId Right) const noexcept {
				// 深さ優先，同じ深さは既存の葉順の返戻
				return Left.Level != Right.Level ? Left.Level > Right.Level : TileOrder{}(Left, Right);
			}
		};
	}

	std::optional<WaterLodTrim> TrimWaterLod(const LodTree &Previous, const LodView &View, const UnitDirection Focus,
		const double NearRadiusMeters, const std::uint8_t MinimumLevel, const std::stop_token Stop) {
		// 不正な視点・骨格・検査領域，又は中断の拒否
		if(!View.IsValid() || MinimumLevel > 6 || !std::isfinite(NearRadiusMeters) || NearRadiusMeters < 0 ||
			Stop.stop_requested())
			return {};
		WaterLodTrim Result{Previous};
		std::set<TileId, WaterParentOrder> Pending;
		for(const TileId Leaf : Previous.Leaves()) {
			// 元の分割が固定骨格を満たさない場合の拒否
			if(Leaf.Level < MinimumLevel) return {};
			if(Leaf.Level > MinimumLevel) Pending.insert(*Leaf.Parent());
		}
		while(!Pending.empty()) {
			// 中断後の候補を公開しない返戻
			if(Stop.stop_requested()) return {};
			const TileId Parent = *Pending.begin();
			Pending.erase(Pending.begin());
			bool Children = true;
			for(std::uint8_t Index = 0; Index < 4; ++Index) Children &= Result.Tree.IsLeaf(*Parent.Child(Index));
			if(!Children) continue;
			std::array<Vec3, 3> Corners;
			for(std::uint8_t Index = 0; Index < 3; ++Index)
				Corners[Index] = GetDirection(*GetVertexKey(Parent, Index))->Vector();
			const Vec3 Center = UnitDirection::FromVector(Corners[0] + Corners[1] + Corners[2])->Vector();
			double Cap = 0, Edge = 0;
			for(std::uint8_t Index = 0; Index < 3; ++Index) {
				Cap = std::max(Cap, (Corners[Index] - Center).Length());
				Edge = std::max(Edge, (Corners[Index] - Corners[(Index + 1) % 3]).Length());
			}
			const double Radius = View.RadiusMeters;
			if((Center - Focus.Vector()).Length() * Radius <= NearRadiusMeters + Cap * Radius) continue;
			const double Distance = (Center * Radius - View.ObserverMeters).Length() - Cap * Radius;
			// ８画素の受入れ値より十分細かい親だけを候補にし，再細分化との往復を抑える
			if(Distance <= 0 || Edge * Radius * View.FocalLengthPixels / Distance > 2) continue;
			if(Result.Tree.Coarsen(Parent) != LodEditStatus::Applied) continue;
			++Result.CoarsenedParents;
			if(Parent.Level > MinimumLevel) {
				Pending.insert(*Parent.Parent());
				// 隣接側の平衡制約が緩んだ候補も再訪する
				for(std::uint8_t Index = 0; Index < 3; ++Index) Pending.insert(*GetNeighbor(Parent, Index)->Tile.Parent());
			}
		}
		// 後段で水面精度を再検査する候補の返戻
		return Result;
	}
}
