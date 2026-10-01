#include "PlanetView.h"
#include "PlanetGround.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <stop_token>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		struct VertexOrder {
			/**
			 * 共有頂点の識別子を固定した辞書順で比較する
			 * @param A 比較する第１要素
			 * @param B 比較する第２要素
			 * @return 第１頂点が先に並ぶ場合に真
			 */
			bool operator()(const VertexKey &A, const VertexKey &B) const {
				// 同じ定義から得た評価結果の返戻
				return std::tie(A.Kind, A.Owner, A.Other, A.Level, A.U, A.V) < std::tie(B.Kind, B.Owner, B.Other, B.Level, B.U, B.V);
			}
		};

		struct TileHash {
			/**
			 * 区画識別子を固定幅の整数ハッシュへ写す
			 * @param Tile 対象の区画識別子
			 * @return 区画のハッシュ
			 */
			std::size_t operator()(const TileId Tile) const noexcept {
				std::uint64_t Value =
				Tile.Path ^ static_cast<std::uint64_t>(Tile.RootFace) << 56 ^ static_cast<std::uint64_t>(Tile.Level) << 48;
				Value = (Value ^ (Value >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
				Value = (Value ^ (Value >> 27)) * UINT64_C(0X94D049BB133111EB);
				// 同じ定義から得た評価結果の返戻
				return static_cast<std::size_t>(Value ^ Value >> 31);
			}
		};

		struct SurfaceTileData {
			// 不変な方向・中心と，必要時に準備する標高と補間誤差
			std::array<Vec3, 3> Directions;
			Vec3 Center;
			double CapChord = 0;
			double EdgeChord = 0;
			double CapAngle = 0;
			std::array<Vec3, 4> ProbeDirections;
			std::array<double, 4> ProbeErrors {};
			double AnchorHeight = 0;
			std::array<double, 3> VertexHeights {};
			bool HeightsReady = false;
			bool ProbesReady = false;
		};

		struct StitchProbeData {
			// 最大４接合面の中点と重心
			std::array<Vec3, 16> Directions;
			std::array<double, 16> Errors {};
			std::uint8_t Count = 0;
		};

		struct StitchKey {
			TileId Tile;
			std::uint8_t Mask = 0;
		};

		struct StitchOrder {
			/**
			 * 接合方式を区画と境界マスクで比較する
			 * @param A 比較する第１要素
			 * @param B 比較する第２要素
			 * @return 第１接合が先に並ぶ場合に真
			 */
			bool operator()(const StitchKey &A, const StitchKey &B) const {
				// 同じ定義から得た評価結果の返戻
				return A.Tile == B.Tile ? A.Mask < B.Mask : TileOrder{}(A.Tile, B.Tile);
			}
		};

		/**
		 * 区画の方向と角度範囲を構築する
		 * @param Tile 対象の区画識別子
		 * @return 標高未評価の区画幾何
		 */
		SurfaceTileData TileGeometry(TileId Tile) {
			SurfaceTileData Data;
			for(std::uint8_t K = 0; K < 3; ++K) Data.Directions[K] = GetDirection(*GetVertexKey(Tile, K))->Vector();
			Data.Center = UnitDirection::FromVector(Data.Directions[0] + Data.Directions[1] + Data.Directions[2])->Vector();
			for(unsigned K = 0; K < 3; ++K) {
				Data.CapChord = std::max(Data.CapChord, (Data.Center - Data.Directions[K]).Length());
				Data.EdgeChord = std::max(Data.EdgeChord, (Data.Directions[K] - Data.Directions[(K + 1) % 3]).Length());
			}
			Data.CapAngle = 2 * std::asin(std::min(1.0, Data.CapChord * 0.5));
			// 同じ定義から得た評価結果の返戻
			return Data;
		}

		/**
		 * 同じ区画の中心と頂点の標高を，表示視点と独立に保持する
		 * @param Data 区画固定の幾何と標高標本
		 * @param Tile 対象の区画識別子
		 * @param Radius 惑星半径，単位 m
		 * @param Evaluate 同じ不変場の標高照会
		 * @param VertexHeight 共有頂点番号に依る標高照会
		 * @param Evaluations 新規標高照会の累積数
		 * @return 全ての標高が有効で準備済の場合に真
		 */
		template<class VertexEvaluator> bool PrepareTileHeights(
			SurfaceTileData &Data,
			const TileId Tile,
			const double Radius,
			const SurfaceHeightEvaluator &Evaluate,
			const VertexEvaluator &VertexHeight,
			std::uint64_t &Evaluations
		) {
			// 必要な準備が完了した場合の返戻
			if(Data.HeightsReady) return true;
			Data.AnchorHeight = Evaluate(*UnitDirection::FromVector(Data.Center));
			++Evaluations;
			// 標本の準備を継続出来ない場合の返戻
			if(!std::isfinite(Data.AnchorHeight) || Radius + Data.AnchorHeight <= 0.0) return false;
			for(std::uint8_t K = 0; K < 3; ++K) {
				const double Height = VertexHeight(*GetVertexKey(Tile, K));
				// 標本の準備を継続出来ない場合の返戻
				if(!std::isfinite(Height) || Radius + Height <= 0.0) return false;
				Data.VertexHeights[K] = Height;
			}
			Data.HeightsReady = true;
			// 全頂点を確認した後の返戻
			return true;
		}

		/**
		 * 現在の三角形を包む箱への距離で描画密度を順位付けする
		 * @param Data 区画固定の幾何と標高標本
		 * @param View 視点と分割予算
		 * @param ErrorPixels 区画中心の標高残差を投影した画素数
		 * @param ProjectionScale 箱迄の距離に基づく，メートル当りの画素数
		 * @return 描画分割の順位付けに用いる辺の画素数
		 * 元関数全体の誤差上限ではない
		 */
		double ProjectedTilePixels(const SurfaceTileData &Data, const LodView &View, double &ErrorPixels, double &ProjectionScale) {
			const Vec3 Observer = View.ObserverMeters - Data.Center * (View.RadiusMeters + Data.AnchorHeight);
			std::array<Vec3, 3> Positions;
			Vec3 Low, High;
			for(std::size_t K = 0; K < Positions.size(); ++K) {
				const Vec3 Point = (Data.Directions[K] - Data.Center) * View.RadiusMeters + Data.Directions[K] * Data.VertexHeights[K] -
				Data.Center * Data.AnchorHeight;
				Positions[K] = Point;
				if(!K) Low = High = Point;
				else {
					Low = { std::min(Low.X, Point.X), std::min(Low.Y, Point.Y), std::min(Low.Z, Point.Z) };
					High = { std::max(High.X, Point.X), std::max(High.Y, Point.Y), std::max(High.Z, Point.Z) };
				}
			}
			const Vec3 Closest {
				std::clamp(Observer.X, Low.X, High.X),
				std::clamp(Observer.Y, Low.Y, High.Y),
				std::clamp(Observer.Z, Low.Z, High.Z)
			};
			double Edge = 0;
			for(std::size_t K = 0; K < Positions.size(); ++K) {
				Edge = std::max(Edge, (Positions[K] - Positions[(K + 1) % Positions.size()]).Length());
			}
			ProjectionScale = View.FocalLengthPixels / std::max(1.0, (Observer - Closest).Length());
			// 評価済の中心標高と三角形平面の放射方向残差を，辺長とは独立に順位へ反映する
			const Vec3 Normal = ((Positions[1] - Positions[0]) * (1 / Edge)).Cross((Positions[2] - Positions[0]) * (1 / Edge));
			ErrorPixels = std::abs(Positions[0].Dot(Normal) / Data.Center.Dot(Normal)) * ProjectionScale;
			// 近景の精度制約と全球の遮蔽上限を別に維持した，描画分割の優先度の返戻
			return Edge * ProjectionScale;
		}

		/**
		 * 区画固定の局所座標で，中点と重心の補間誤差を評価する
		 * @param Data 区画固定の幾何と標高標本
		 * @param Tile 対象の区画識別子
		 * @param Radius 惑星半径，単位 m
		 * @param Evaluate 同じ不変場の標高照会
		 * @param VertexHeight 共有頂点番号に依る標高照会
		 * @param Evaluations 新規標高照会の累積数
		 * @return 全ての標本が有限で準備済の場合に真
		 */
		template<class VertexEvaluator> bool PrepareTileProbes(
			SurfaceTileData &Data,
			TileId Tile,
			double Radius,
			const SurfaceHeightEvaluator &Evaluate,
			const VertexEvaluator &VertexHeight,
			std::uint64_t &Evaluations
		) {
			// 標本の準備を継続出来ない場合の返戻
			if(!PrepareTileHeights(Data, Tile, Radius, Evaluate, VertexHeight, Evaluations)) return false;
			const UnitDirection Anchor = *UnitDirection::FromVector(Data.Center);
			const std::optional<GroundFrame> Frame = GroundFrame::Create(Anchor, Radius, Data.AnchorHeight);
			// 標本の準備を継続出来ない場合の返戻
			if(!Frame) return false;
			const GroundFrame &F = *Frame;
			std::array<Vec3, 3> Local;
			for(std::uint8_t K = 0; K < 3; ++K) {
				Local[K] = F.SurfaceToLocal(*UnitDirection::FromVector(Data.Directions[K]), Data.VertexHeights[K]);
			}
			const double R0 = F.RadiusMeters + F.AnchorElevationMeters;
			const std::array<Vec3, 4> Qs {
				(Local[0] + Local[1]) * 0.5,
				(Local[1] + Local[2]) * 0.5,
				(Local[2] + Local[0]) * 0.5,
				(Local[0] + Local[1] + Local[2]) * (1.0 / 3)
			};
			for(unsigned K = 0; K < Qs.size(); ++K) {
				const Vec3 Q = Qs[K];
				const std::optional<UnitDirection> D = UnitDirection::FromVector(F.LocalToPlanet(Q));
				// 標本の準備を継続出来ない場合の返戻
				if(!D) return false;
				const double Increment = 2 * R0 * Q.Z + Q.Dot(Q), Distance = std::sqrt(R0 * R0 + Increment);
				const double Interpolated = F.AnchorElevationMeters + Increment / (Distance + R0);
				const double Error = std::abs(Evaluate(*D) - Interpolated);
				++Evaluations;
				// 標本の準備を継続出来ない場合の返戻
				if(!std::isfinite(Error)) return false;
				Data.ProbeDirections[K] = D->Vector();
				Data.ProbeErrors[K] = Error;
			}
			Data.ProbesReady = true;
			// 必要な準備が完了した場合の返戻
			return true;
		}
	} // namespace

	struct LodSurfaceCache::Impl {
		struct Entry {
			SurfaceTileData Data;
			std::list<TileId>::iterator Recent;
		};

		struct StitchEntry {
			StitchProbeData Data;
			std::list<StitchKey>::iterator Recent;
		};

		// 同じ不変場の所有権と，区画・接合標本の有界な使用順
		double Radius;
		std::size_t Capacity;
		SurfaceHeightEvaluator Height;
		std::shared_ptr<LodNeighborCache> Neighbors = std::make_shared<LodNeighborCache>();
		std::unordered_map<TileId, Entry, TileHash> Tiles;
		std::list<TileId> Recent;
		std::map<StitchKey, StitchEntry, StitchOrder> Variants;
		std::list<StitchKey> RecentVariants;

		/**
		 * 境界の接合標本を使用順に保持する
		 * @param Key 区画と接合方式
		 * @return 指定した区画と境界マスクの保持領域
		 */
		StitchProbeData &GetVariant(StitchKey Key) {
			if(const std::map<StitchKey, StitchEntry, StitchOrder>::iterator It = Variants.find(Key); It != Variants.end()) {
				RecentVariants.splice(RecentVariants.begin(), RecentVariants, It->second.Recent);
				// 同じ定義から得た評価結果の返戻
				return It->second.Data;
			}
			if(Variants.size() >= std::min(Capacity, LodSurfaceCache::MaximumStitchVariants)) {
				Variants.erase(RecentVariants.back());
				RecentVariants.pop_back();
			}
			RecentVariants.push_front(Key);
			// 同じ定義から得た評価結果の返戻
			return Variants.emplace(Key, StitchEntry{ {}, RecentVariants.begin() }).first->second.Data;
		}

		/**
		 * 不変の標高場と保持上限を束縛する
		 * @param R 惑星半径，単位 m
		 * @param H 所有する不変場の標高照会
		 * @param C 保持上限又は共有する標本保持器
		 */
		Impl(double R, SurfaceHeightEvaluator H, std::size_t C) : Radius(R), Capacity(C), Height(std::move(H)) {}

		/**
		 * 区画の幾何と標高標本を使用順に保持する
		 * @param Tile 対象の区画識別子
		 * @return 指定した区画の保持領域
		 */
		SurfaceTileData &Get(TileId Tile) {
			if(const std::unordered_map<TileId, Entry, TileHash>::iterator It = Tiles.find(Tile); It != Tiles.end()) {
				Recent.splice(Recent.begin(), Recent, It->second.Recent);
				// 同じ定義から得た評価結果の返戻
				return It->second.Data;
			}
			if(Tiles.size() >= Capacity) {
				Tiles.erase(Recent.back());
				Recent.pop_back();
			}
			Recent.push_front(Tile);
			// 同じ定義から得た評価結果の返戻
			return Tiles.emplace(Tile, Entry{ TileGeometry(Tile), Recent.begin() }).first->second.Data;
		}
	};

	struct LodSurfacePreparation::Impl {
		// 先読み対象と，次回へ引継ぐ未処理の区画
		LodView View;
		std::shared_ptr<LodSurfaceCache> Cache;
		std::vector<TileId> Pending;
		std::map<VertexKey, double, VertexOrder> Heights;
		SurfacePreparationProgress Progress;
		SurfaceCorridor Region;
		std::optional<SurfaceCorridor> FoliageRegion;

		/**
		 * 先読み範囲と深さ優先の未処理列を初期化する
		 * @param V 近景の先読み条件
		 * @param C 保持上限又は共有する標本保持器
		 */
		Impl(const LodView &V, std::shared_ptr<LodSurfaceCache> C) : View(V),
		Cache(std::move(C)),
		Region(
			*SurfaceCorridor::Create(
				V.SurfaceObserverMeters.value_or(V.ObserverMeters),
				V.SurfacePathEndMeters.value_or(V.SurfaceObserverMeters.value_or(V.ObserverMeters)),
				V.RadiusMeters,
				V.SurfaceRadiusMeters
			)
		),
		FoliageRegion(
			V.FoliageRadiusMeters > 0.0 ?
			SurfaceCorridor::Create(
				V.SurfaceObserverMeters.value_or(V.ObserverMeters),
				V.SurfacePathEndMeters.value_or(V.SurfaceObserverMeters.value_or(V.ObserverMeters)),
				V.RadiusMeters,
				V.FoliageRadiusMeters
			) :
			std::nullopt
		) {
			Pending.reserve(20 + 3 * MaxTileLevel);
			for(unsigned Face = 20; Face > 0; --Face) Pending.push_back({ 0, static_cast<std::uint8_t>(Face - 1), 0 });
			// 有効な処理を完了する終了
			return;
		}
	};

	/**
	 * 一つの不変場専用の有界な標本保持器を作る
	 * @param Radius 惑星半径，単位 m
	 * @param Height 同じ不変場の標高照会
	 * @param Capacity 保持する区画数の上限
	 */
	LodSurfaceCache::LodSurfaceCache(double Radius, SurfaceHeightEvaluator Height, std::size_t Capacity) : State(
		std::make_unique<Impl>(Radius, std::move(Height), Capacity)
	) {}

	LodSurfaceCache::~LodSurfaceCache() = default;

	/**
	 * 実際に保持中の区画数を返す
	 * @return 現在の区画数
	 */
	std::size_t LodSurfaceCache::TileCount() const {
		// 同じ定義から得た評価結果の返戻
		return State->Tiles.size();
	}

	/**
	 * 分割可能な先読み処理の状態を所有する
	 * @param S 所有する準備状態
	 */
	LodSurfacePreparation::LodSurfacePreparation(std::unique_ptr<Impl> S) : State(std::move(S)) {}

	LodSurfacePreparation::~LodSurfacePreparation() = default;

	/**
	 * 同じ半径の保持器へ，有限な先読み要求を結び付ける
	 * @param V 近景の先読み条件
	 * @param C 保持上限又は共有する標本保持器
	 * @return 有効な先読み処理，不正な条件では空
	 */
	std::shared_ptr<LodSurfacePreparation> LodSurfacePreparation::Create(const LodView &V, std::shared_ptr<LodSurfaceCache> C) {
		// 不正又は中断した分割を公開しない返戻
		const bool Invalid =
		!V.IsValid() || V.SurfaceRadiusMeters <= 0.0 || !C || C->State->Radius != V.RadiusMeters || !C->State->Height ||
		!C->State->Capacity || C->State->Capacity > LodSurfaceCache::MaximumTiles;
		// 先読み条件と保持器が対応しない場合の返戻
		if(Invalid) return {};
		const Vec3 Start = V.SurfaceObserverMeters.value_or(V.ObserverMeters);
		// 不正又は中断した分割を公開しない返戻
		const bool InvalidRegion =
		!SurfaceCorridor::Create(Start, V.SurfacePathEndMeters.value_or(Start), V.RadiusMeters, V.SurfaceRadiusMeters) ||
		V.FoliageRadiusMeters > 0.0 &&
		!SurfaceCorridor::Create(Start, V.SurfacePathEndMeters.value_or(Start), V.RadiusMeters, V.FoliageRadiusMeters);
		// 身体又は植生の先読み範囲を定義出来ない場合の返戻
		if(InvalidRegion) return {};
		// 同じ定義から得た評価結果の返戻
		return std::shared_ptr<LodSurfacePreparation>(new LodSurfacePreparation(std::make_unique<Impl>(V, std::move(C))));
	}

	/**
	 * 先読みの元の視点条件を参照する
	 * @return 変更しない視点条件
	 */
	const LodView &LodSurfacePreparation::TargetView() const {
		// 同じ定義から得た評価結果の返戻
		return State->View;
	}

	/**
	 * 指定した標本保持器を共有しているか確認する
	 * @param C 保持上限又は共有する標本保持器
	 * @return 同じ保持器の場合に真
	 */
	bool LodSurfacePreparation::Uses(const std::shared_ptr<LodSurfaceCache> &C) const {
		// 同じ定義から得た評価結果の返戻
		return State->Cache == C;
	}

	/**
	 * 中断と区画予算を守り，近景の不変な標本を段階的に準備する
	 * @param TileBudget 今回処理する区画数の上限
	 * @param Stop 中断要求
	 * @return 完了・拒否・予算と実評価数
	 */
	SurfacePreparationProgress LodSurfacePreparation::Advance(std::size_t TileBudget, std::stop_token Stop) {
		Impl &S = *State;
		const LodView &V = S.View;
		LodSurfaceCache::Impl &C = *S.Cache->State;
		// 現在迄の準備結果の返戻
		if(S.Progress.Complete || !TileBudget) return S.Progress;
		const auto VertexHeight = [&](const VertexKey Key) -> double {
			// 同じ定義から得た評価結果の返戻
			if(const std::map<VertexKey, double, VertexOrder>::const_iterator It = S.Heights.find(Key); It != S.Heights.end()) {
				// 保持済の評価結果の返戻
				return It->second;
			}
			const double H = C.Height(*GetDirection(Key));
			++S.Progress.HeightEvaluations;
			if(S.Heights.size() < LodSurfaceCache::MaximumTiles) S.Heights.emplace(Key, H);
			// 同じ定義から得た評価結果の返戻
			return H;
		};
		// 深さ優先の未処理列は２０＋３×深度以内
		// 呼出単位と準備全体の双方に処理上限を置く
		const std::uint64_t MaximumVisits = 4 * V.LeafBudget + 20;
		for(std::size_t Step = 0; Step < TileBudget && !S.Pending.empty(); ++Step) {
			// 現在迄の準備結果の返戻
			if(Stop.stop_requested()) return S.Progress;
			if(S.Progress.VisitedTiles >= MaximumVisits) {
				S.Progress.BudgetExhausted = true;
				S.Progress.Complete = true;
				// 現在迄の準備結果の返戻
				return S.Progress;
			}
			const TileId Tile = S.Pending.back();
			S.Pending.pop_back();
			++S.Progress.VisitedTiles;
			SurfaceTileData &Data = C.Get(Tile);
			const bool Near = S.Region.Intersects(Data.Center, Data.CapChord * V.RadiusMeters);
			const bool Foliage = S.FoliageRegion && S.FoliageRegion->Intersects(Data.Center, Data.CapChord * V.RadiusMeters);
			if(!Near && !Foliage) continue;
			const double Edge = Data.EdgeChord * (V.RadiusMeters + V.ReliefBoundMeters);
			bool Split = Near && Edge > V.SurfaceEdgeMeters || Foliage && Edge > V.FoliageEdgeMeters;
			if(!Split) {
				if(Data.ProbesReady) ++S.Progress.ProbeHits;
				else if(!PrepareTileProbes(Data, Tile, V.RadiusMeters, C.Height, VertexHeight, S.Progress.HeightEvaluations)) {
					S.Progress.Valid = false;
					S.Progress.Complete = true;
					// 現在迄の準備結果の返戻
					return S.Progress;
				}
				for(unsigned K = 0; K < Data.ProbeErrors.size(); ++K) {
					if(Near && S.Region.Contains(Data.ProbeDirections[K]) && Data.ProbeErrors[K] > V.SurfaceErrorMeters) Split = true;
					if(Foliage && S.FoliageRegion->Contains(Data.ProbeDirections[K]) && Data.ProbeErrors[K] > V.FoliageErrorMeters) Split = true;
				}
			}
			if(Split && Tile.Level < V.MaximumLevel) {
				for(unsigned K = 4; K > 0; --K) S.Pending.push_back(*Tile.Child(static_cast<std::uint8_t>(K - 1)));
			}
		}
		S.Progress.Complete = S.Pending.empty();
		// 現在迄の準備結果の返戻
		return S.Progress;
	}

	/**
	 * 半径・視点・精度・保持量の入力範囲を確認する
	 * @return 全ての入力条件を満たす場合に真
	 */
	bool LodView::IsValid() const {
		// 有効な処理を完了する終了
		return std::isfinite(RadiusMeters) && RadiusMeters >= 1000.0 && RadiusMeters <= 1E9 && std::isfinite(ReliefBoundMeters) &&
		ReliefBoundMeters >= 0.0 && ReliefBoundMeters < RadiusMeters && std::isfinite(ObserverMeters.Length()) &&
		ObserverMeters.Length() > 0 &&
		(!SurfaceObserverMeters || std::isfinite(SurfaceObserverMeters->Length()) && SurfaceObserverMeters->Length() > 0) &&
		(!SurfacePathEndMeters || std::isfinite(SurfacePathEndMeters->Length()) && SurfacePathEndMeters->Length() > 0) &&
		std::isfinite(FocalLengthPixels) && FocalLengthPixels >= 1.0 && FocalLengthPixels <= 16384.0 &&
		std::isfinite(TargetEdgePixels) && TargetEdgePixels >= 1.0 && TargetEdgePixels <= 1024.0 && LeafBudget > 19 &&
		LeafBudget <= LodView::MaximumViewLeaves && PixelLeafBudget > 19 && PixelLeafBudget <= LodView::MaximumViewLeaves &&
		MaximumLevel > 0 && MaximumLevel <= MaxTileLevel && std::isfinite(SurfaceRadiusMeters) && SurfaceRadiusMeters >= 0.0 &&
		SurfaceRadiusMeters <= 256.0 && std::isfinite(SurfaceEdgeMeters) && SurfaceEdgeMeters >= 0.25 && SurfaceEdgeMeters <= 16.0 &&
		std::isfinite(SurfaceErrorMeters) && SurfaceErrorMeters > 0.0 && SurfaceErrorMeters <= 0.02 &&
		std::isfinite(FoliageRadiusMeters) && FoliageRadiusMeters >= 0.0 && FoliageRadiusMeters <= 256.0 && (
			!FoliageRadiusMeters ||
			FoliageRadiusMeters >= SurfaceRadiusMeters && SurfaceRadiusMeters > 0.0 && std::isfinite(FoliageEdgeMeters) &&
			FoliageEdgeMeters >= SurfaceEdgeMeters && FoliageEdgeMeters <= 16.0 && std::isfinite(FoliageErrorMeters) &&
			FoliageErrorMeters >= SurfaceErrorMeters && FoliageErrorMeters <= 0.1
		);
	}

	/**
	 * 足元と植生の精度を先に予約し，残る予算で全球の閉じた分割を選ぶ
	 * @param View 視点と分割予算
	 * @param Stop 中断要求
	 * @param Height 同じ不変場の標高照会
	 * @param Cache 同じ場を束縛した標本保持器，省略可
	 * @return 被覆を維持した選択結果，不正な標高又は中断時は空
	 */
	std::optional<LodSelection> SelectLodForView(
		const LodView &View,
		std::stop_token Stop,
		const SurfaceHeightEvaluator &Height,
		LodSurfaceCache *Cache
	) {
		// 不正又は中断した分割を公開しない返戻
		if(!View.IsValid() || Stop.stop_requested()) return {};
		// 不正又は中断した分割を公開しない返戻
		const bool InvalidCache = Cache && (
			Height || Cache->State->Radius != View.RadiusMeters || !Cache->State->Height || !Cache->State->Capacity ||
			Cache->State->Capacity > LodSurfaceCache::MaximumTiles
		);
		// 異なる元場又は半径の標本を混在させない返戻
		if(InvalidCache) return {};
		const SurfaceHeightEvaluator &Evaluate = Cache ? Cache->State->Height : Height;
		LodSelection Result;
		if(Cache) Result.Tree = LodTree(Cache->State->Neighbors);
		const std::uint64_t PriorNeighborHits = Cache ? Cache->State->Neighbors->Hits() : 0;
		const double ObserverDistance = View.ObserverMeters.Length();
		const Vec3 ObserverDirection = View.ObserverMeters * (1 / ObserverDistance);
		const double InnerRadius = View.RadiusMeters - View.ReliefBoundMeters, OuterRadius = View.RadiusMeters + View.ReliefBoundMeters;
		const bool CanOcclude = ObserverDistance > InnerRadius;
		const double HorizonAngle =
		CanOcclude ? std::acos(InnerRadius / ObserverDistance) + std::acos(InnerRadius / OuterRadius) : 3.14159265358979323846;
		const Vec3 SurfaceStart = View.SurfaceObserverMeters.value_or(View.ObserverMeters);
		const std::optional<SurfaceCorridor> Region = SurfaceCorridor::Create(
			SurfaceStart,
			View.SurfacePathEndMeters.value_or(SurfaceStart),
			View.RadiusMeters,
			View.SurfaceRadiusMeters
		);
		// 不正又は中断した分割を公開しない返戻
		if(!Region) return {};
		const std::optional<SurfaceCorridor> FoliageRegion = View.FoliageRadiusMeters > 0.0 ?
		SurfaceCorridor::Create(
			SurfaceStart,
			View.SurfacePathEndMeters.value_or(SurfaceStart),
			View.RadiusMeters,
			View.FoliageRadiusMeters
		) :
		std::nullopt;
		// 不正又は中断した分割を公開しない返戻
		if(View.FoliageRadiusMeters > 0.0 && !FoliageRegion) return {};
		std::map<VertexKey, double, VertexOrder> Heights;
		const auto VertexHeight = [&](const VertexKey Key) -> double {
			// 同じ定義から得た評価結果の返戻
			if(const std::map<VertexKey, double, VertexOrder>::const_iterator It = Heights.find(Key); It != Heights.end()) {
				// 保持済の評価結果の返戻
				return It->second;
			}
			const double H = Evaluate(*GetDirection(Key));
			++Result.SurfaceHeightEvaluations;
			Heights.emplace(Key, H);
			// 同じ定義から得た評価結果の返戻
			return H;
		};
		std::map<TileId, SurfaceTileData, TileOrder> LocalData;
		const auto DataFor = [&](const TileId Tile) -> SurfaceTileData & {
			// 同じ定義から得た評価結果の返戻
			if(Cache) return Cache->State->Get(Tile);
			const std::map<TileId, SurfaceTileData, TileOrder>::iterator It = LocalData.find(Tile);
			// 同じ定義から得た評価結果の返戻
			if(It != LocalData.end()) return It->second;
			// 同じ定義から得た評価結果の返戻
			return LocalData.emplace(Tile, TileGeometry(Tile)).first->second;
		};
		bool InvalidHeight = false;
		struct TileMetric {
			double Pixels = 0;
			double ErrorPixels = 0;
			double SurfaceEdge = 0;
			double SurfaceError = 0;
			double FoliageEdge = 0;
			double FoliageError = 0;
			bool HeightChecked = false;
			bool PixelHeightChecked = false;
		};
		std::unordered_map<TileId, TileMetric, TileHash> Metrics;
		const auto Metric = [&](const TileId Tile, const bool ForceHeight=false, const bool ProjectedProbes=false) -> TileMetric {
			// 同じ定義から得た評価結果の返戻
			const std::unordered_map<TileId, TileMetric, TileHash>::const_iterator It = Metrics.find(Tile);
			const bool Ready = It != Metrics.end() &&
			(!ForceHeight || It->second.HeightChecked || !It->second.SurfaceEdge && !It->second.FoliageEdge || !Evaluate) &&
			(!ProjectedProbes || It->second.PixelHeightChecked || !Evaluate);
			// 現在必要な精度の照会が済んだ指標の返戻
			if(Ready) return It->second;
			SurfaceTileData &Data = DataFor(Tile);
			const Vec3 Center = Data.Center;
			const double CapChord = Data.CapChord, Edge = Data.EdgeChord * OuterRadius;
			TileMetric M;
			if(View.SurfaceRadiusMeters > 0.0 && Region->Intersects(Center, CapChord * View.RadiusMeters)) M.SurfaceEdge = Edge;
			if(FoliageRegion && FoliageRegion->Intersects(Center, CapChord * View.RadiusMeters)) M.FoliageEdge = Edge;
			// 辺長だけで細分化が確定する粗い区画の中間誤差を省き，最終区画と接合境界では全標本を検査する
			if(
				Evaluate && (M.SurfaceEdge > 0.0 || M.FoliageEdge > 0.0) && (
					ForceHeight || !View.DeferCoarseSurfaceProbes || M.SurfaceEdge > 0.0 && M.SurfaceEdge <= View.SurfaceEdgeMeters ||
					M.FoliageEdge > 0.0 && M.FoliageEdge <= View.FoliageEdgeMeters
				)
			) {
				if(Data.ProbesReady) ++Result.SurfaceProbeCacheHits;
				else if(!PrepareTileProbes(Data, Tile, View.RadiusMeters, Evaluate, VertexHeight, Result.SurfaceHeightEvaluations)) {
					InvalidHeight = true;
					// 同じ定義から得た評価結果の返戻
					return M;
				}
				M.HeightChecked = true;
				for(unsigned K = 0; K < Data.ProbeErrors.size(); ++K) {
					if(Region->Contains(Data.ProbeDirections[K])) M.SurfaceError = std::max(M.SurfaceError, Data.ProbeErrors[K]);
					if(FoliageRegion && FoliageRegion->Contains(Data.ProbeDirections[K])) {
						M.FoliageError = std::max(M.FoliageError, Data.ProbeErrors[K]);
					}
				}
			}
			const double Cap = Data.CapAngle;
			const bool Hidden =
			CanOcclude && HorizonAngle + Cap < 3.14159265358979323846 && Center.Dot(ObserverDirection) < std::cos(HorizonAngle + Cap);
			if(!Hidden && Evaluate) {
				if(!PrepareTileHeights(Data, Tile, View.RadiusMeters, Evaluate, VertexHeight, Result.SurfaceHeightEvaluations)) {
					InvalidHeight = true;
					// 同じ定義から得た評価結果の返戻
					return M;
				}
				double ProjectionScale = 0;
				M.Pixels = ProjectedTilePixels(Data, View, M.ErrorPixels, ProjectionScale);
				if(ProjectedProbes) {
					if(
						!Data.ProbesReady && !PrepareTileProbes(Data, Tile, View.RadiusMeters, Evaluate, VertexHeight, Result.SurfaceHeightEvaluations)
					) {
						InvalidHeight = true;
						// 遠景の中間標本が不正な候補の返戻
						return M;
					}
					for(const double Error : Data.ProbeErrors) M.ErrorPixels = std::max(M.ErrorPixels, Error * ProjectionScale);
					M.PixelHeightChecked = true;
				}
				if(!std::isfinite(M.Pixels) || !std::isfinite(M.ErrorPixels)) {
					InvalidHeight = true;
					// 座標又は投影値が浮動小数点の範囲を超えた候補の返戻
					return M;
				}
			} else if(!Hidden) {
				const double Bound = OuterRadius * CapChord + View.ReliefBoundMeters;
				const double Distance = std::max(1.0, (Center * View.RadiusMeters - View.ObserverMeters).Length() - Bound);
				M.Pixels = Edge * View.FocalLengthPixels / Distance;
			}
			Metrics.insert_or_assign(Tile, M);
			// 同じ定義から得た評価結果の返戻
			return M;
		};
		struct Entry {
			double Priority;
			TileId Tile;
		};
		struct Less {
			/**
			 * 同じ優先度の区画を識別子順で安定に並べる
			 * @param A 比較する第１要素
			 * @param B 比較する第２要素
			 * @return 第１候補の優先度が低い場合に真
			 */
			bool operator()(const Entry &A, const Entry &B) const {
				// 同じ定義から得た評価結果の返戻
				return A.Priority != B.Priority ? A.Priority < B.Priority : TileOrder{}(B.Tile, A.Tile);
			}
		};
		/**
		 * 足元又は植生の必須分割へ，遠景の予算を再配分する
		 * @param Tile 必須の細分化区画
		 * @return 同じ総葉数上限での原子的な分割結果
		 */
		const auto RefineMandatory = [&](const TileId Tile) -> LodEdit {
			LodEdit Edit = Result.Tree.Refine(Tile, View.LeafBudget);
			// 空間精度を満たせる場合，又は予算以外の拒否の返戻
			if(Edit.Status != LodEditStatus::BudgetExceeded) return Edit;
			// 遠景だけを縮約し，身体・通路・植生の必須領域を維持する
			for(unsigned Pass = 0; Pass < MaxTileLevel; ++Pass) {
				std::set<TileId, TileOrder> Parents;
				for(const TileId Leaf : Result.Tree.Leaves()) if(const std::optional<TileId> Parent = Leaf.Parent()) Parents.insert(*Parent);
				std::vector<Entry> Candidates;
				for(const TileId Parent : Parents) {
					bool Complete = true;
					for(std::uint8_t K = 0; K < 4; ++K) if(!Result.Tree.IsLeaf(*Parent.Child(K))) Complete = false;
					if(!Complete) continue;
					const TileMetric M = Metric(Parent, false, true);
					if(M.SurfaceEdge > 0.0 || M.FoliageEdge > 0.0) continue;
					Candidates.push_back({ std::max(M.Pixels / View.TargetEdgePixels, M.ErrorPixels), Parent });
				}
				std::sort(
					Candidates.begin(),
					Candidates.end(),
					[](const Entry &A, const Entry &B) -> bool {
						return A.Priority != B.Priority ? A.Priority < B.Priority : TileOrder{}(A.Tile, B.Tile);
					}
				);
				bool Changed = false;
				for(const Entry &Candidate : Candidates) {
					// 中断又は不正な標高を含む未達の分割結果の返戻
					if(Stop.stop_requested() || InvalidHeight) return Edit;
					if(Result.Tree.Coarsen(Candidate.Tile) != LodEditStatus::Applied) continue;
					++Result.ReclaimedDistantParents;
					Changed = true;
					Edit = Result.Tree.Refine(Tile, View.LeafBudget);
					// 必須分割を確保した結果又は予算以外の拒否の返戻
					if(Edit.Status != LodEditStatus::BudgetExceeded) return Edit;
				}
				if(!Changed) break;
			}
			// 遠景を縮約しても正当に確保出来ない分割結果の返戻
			return Edit;
		};
		unsigned Steps = 0;
		enum class RefineKind { Surface, Foliage, PixelError, Pixels };
		const auto RefinePass = [&](const RefineKind Kind) -> bool {
			const bool PixelPass = Kind == RefineKind::Pixels || Kind == RefineKind::PixelError;
			const std::size_t Budget = Kind == RefineKind::Pixels ? std::min(View.LeafBudget, View.PixelLeafBudget) : View.LeafBudget;
			// 必要な準備が完了した場合の返戻
			if(PixelPass && Result.Tree.Leaves().size() >= Budget) return true;
			std::priority_queue<Entry, std::vector<Entry>, Less> Queue;
			const auto Score = [&](const TileId Tile) -> double {
				const TileMetric M = Metric(Tile, false, PixelPass);
				if(Kind == RefineKind::Surface) {
					// 同じ定義から得た評価結果の返戻
					return std::max(M.SurfaceEdge / View.SurfaceEdgeMeters, M.SurfaceError / View.SurfaceErrorMeters);
				}
				if(Kind == RefineKind::Foliage) {
					// 同じ定義から得た評価結果の返戻
					return std::max(M.FoliageEdge / View.FoliageEdgeMeters, M.FoliageError / View.FoliageErrorMeters);
				}
				// 同じ定義から得た評価結果の返戻
				return Kind == RefineKind::PixelError ? M.ErrorPixels : M.Pixels / View.TargetEdgePixels;
			};
			for(const TileId Tile : Result.Tree.Leaves()) Queue.push({ Score(Tile), Tile });
			constexpr double Target = 1.0;
			while(!Queue.empty()) {
				// 標本の準備を継続出来ない場合の返戻
				if(InvalidHeight || !(Steps++ & 63) && Stop.stop_requested()) return false;
				const Entry Current = Queue.top();
				Queue.pop();
				if(!Result.Tree.IsLeaf(Current.Tile)) continue;
				if(Current.Priority <= Target) break;
				if(Current.Tile.Level >= View.MaximumLevel) continue;
				const LodEdit Edit = PixelPass ? Result.Tree.Refine(Current.Tile, Budget) : RefineMandatory(Current.Tile);
				if(Edit.Status != LodEditStatus::Applied) continue;
				Result.RefinedParents += static_cast<std::uint32_t>(Edit.SplitParents.size());
				for(const TileId Parent : Edit.SplitParents) for(std::uint8_t K = 0; K < 4; ++K) {
					const TileId Child = *Parent.Child(K);
					Queue.push({ Score(Child), Child });
				}
			}
			// 必要な準備が完了した場合の返戻
			return true;
		};
		// 同じ閉じた分割で足元を先に予約し，残る予算を遠景へ配分する
		if(View.SurfaceRadiusMeters > 0.0 && !RefinePass(RefineKind::Surface)) return {};
		// 不正又は中断した分割を公開しない返戻
		if(View.FoliageRadiusMeters > 0.0 && !RefinePass(RefineKind::Foliage)) return {};
		// 辺長に依る見掛けの密度より先に，山体の投影誤差へ精度確保用の総予算を配分する
		if(Evaluate && !RefinePass(RefineKind::PixelError)) return {};
		// 不正又は中断した分割を公開しない返戻
		if(InvalidHeight || !RefinePass(RefineKind::Pixels)) return {};
		// 遠景と植生の分割に依り隣接面が変わる為，足元の精度を再検査する
		if(View.SurfaceRadiusMeters > 0.0 && !RefinePass(RefineKind::Surface)) return {};
		// 不正又は中断した分割を公開しない返戻
		if(View.FoliageRadiusMeters > 0.0 && !RefinePass(RefineKind::Foliage)) return {};
		// 親の４標本に含まれない接合中点と対角線を，実際の赤緑分割面で検査する
		if(Evaluate && (View.SurfaceRadiusMeters > 0.0 || View.FoliageRadiusMeters > 0.0)) while(true) {
			// 不正又は中断した分割を公開しない返戻
			if(Stop.stop_requested()) return {};
			std::vector<Entry> Bad;
			Result.MaximumStitchErrorMeters = 0;
			Result.MaximumFoliageStitchErrorMeters = 0;
			for(const TileId Tile : Result.Tree.Leaves()) {
				// 不正又は中断した分割を公開しない返戻
				if(!(Steps++ & 63) && Stop.stop_requested()) return {};
				const TileMetric M = Metric(Tile, true);
				// 不正又は中断した分割を公開しない返戻
				if(InvalidHeight) return {};
				if(!M.SurfaceEdge && !M.FoliageEdge) continue;
				std::uint8_t Mask = 0;
				for(std::uint8_t E = 0; E < 3; ++E) if(Result.Tree.HasFinerNeighbor(Tile, E)) Mask |= static_cast<std::uint8_t>(1 << E);
				if(!Mask) continue;
				StitchProbeData Scratch;
				StitchProbeData &Probes = Cache ? Cache->State->GetVariant({ Tile, Mask }) : Scratch;
				if(Probes.Count) ++Result.StitchProbeCacheHits;
				else {
					SurfaceTileData &Data = DataFor(Tile);
					// 保持器から追出された標本を，同じ不変な式で再評価する
					if(!Data.ProbesReady) {
						Metrics.erase(Tile);
						Metric(Tile, true);
					}
					// 不正又は中断した分割を公開しない返戻
					if(InvalidHeight || !Data.ProbesReady) return {};
					const std::optional<GroundFrame> Frame =
					GroundFrame::Create(*UnitDirection::FromVector(Data.Center), View.RadiusMeters, Data.AnchorHeight);
					// 不正又は中断した分割を公開しない返戻
					if(!Frame) return {};
					const GroundFrame &F = *Frame;
					std::array<Vec3, 6> Positions;
					for(std::uint8_t K = 0; K < 3; ++K) {
						Positions[K] = F.SurfaceToLocal(*UnitDirection::FromVector(Data.Directions[K]), Data.VertexHeights[K]);
					}
					const std::optional<TileId> Child = Tile.Child(3);
					// 不正又は中断した分割を公開しない返戻
					if(!Child) return {};
					for(std::uint8_t K = 0; K < 3; ++K) if(Mask & 1 << K) {
						const VertexKey Key = *GetVertexKey(*Child, (K + 1) % 3);
						const UnitDirection D = *GetDirection(Key);
						const double H = VertexHeight(Key);
						// 不正又は中断した分割を公開しない返戻
						if(!std::isfinite(H) || View.RadiusMeters + H <= 0.0) return {};
						Positions[3 + K] = F.SurfaceToLocal(D, H);
					}
					const StitchPattern Pattern = *GetStitchPattern(Mask);
					StitchProbeData Prepared;
					const double R0 = F.RadiusMeters + F.AnchorElevationMeters;
					for(unsigned I = 0; I < Pattern.Count; ++I) {
						const std::array<std::uint8_t, 3> T = Pattern.Triangles[I];
						const Vec3 A = Positions[T[0]], B = Positions[T[1]], C = Positions[T[2]];
						for(const Vec3 Q : { (A + B) * 0.5, (B + C) * 0.5, (C + A) * 0.5, (A + B + C) * (1.0 / 3) }) {
							const std::optional<UnitDirection> D = UnitDirection::FromVector(F.LocalToPlanet(Q));
							// 不正又は中断した分割を公開しない返戻
							if(!D) return {};
							const double Increment = 2 * R0 * Q.Z + Q.Dot(Q);
							const double H = F.AnchorElevationMeters + Increment / (std::sqrt(R0 * R0 + Increment) + R0);
							const double Error = std::abs(Evaluate(*D) - H);
							++Result.SurfaceHeightEvaluations;
							// 不正又は中断した分割を公開しない返戻
							if(!std::isfinite(Error)) return {};
							Prepared.Directions[Prepared.Count] = D->Vector();
							Prepared.Errors[Prepared.Count++] = Error;
						}
					}
					Probes = Prepared;
				}
				double Error = 0, FoliageError = 0;
				for(unsigned K = 0; K < Probes.Count; ++K) {
					if(Region->Contains(Probes.Directions[K])) Error = std::max(Error, Probes.Errors[K]);
					if(FoliageRegion && FoliageRegion->Contains(Probes.Directions[K])) FoliageError = std::max(FoliageError, Probes.Errors[K]);
				}
				Result.MaximumStitchErrorMeters = std::max(Result.MaximumStitchErrorMeters, Error);
				Result.MaximumFoliageStitchErrorMeters = std::max(Result.MaximumFoliageStitchErrorMeters, FoliageError);
				if(Error > View.SurfaceErrorMeters || FoliageError > View.FoliageErrorMeters) {
					Bad.push_back({ std::max(Error / View.SurfaceErrorMeters, FoliageError / View.FoliageErrorMeters), Tile });
				}
			}
			if(Bad.empty() || Result.StitchRefinementPasses >= MaxTileLevel) break;
			std::sort(
				Bad.begin(),
				Bad.end(),
				[](const Entry &A, const Entry &B) -> bool {
					// 同じ定義から得た評価結果の返戻
					return A.Priority != B.Priority ? A.Priority > B.Priority : TileOrder{}(A.Tile, B.Tile);
				}
			);
			std::uint32_t Changed = 0;
			for(const Entry &Item : Bad) {
				if(Item.Tile.Level >= View.MaximumLevel) continue;
				const LodEdit Edit = RefineMandatory(Item.Tile);
				if(Edit.Status == LodEditStatus::Applied) Changed += static_cast<std::uint32_t>(Edit.SplitParents.size());
			}
			if(!Changed) break;
			Result.RefinedParents += Changed;
			Result.StitchRefinedParents += Changed;
			++Result.StitchRefinementPasses;
			// 不正又は中断した分割を公開しない返戻
			if(!RefinePass(RefineKind::Surface)) return {};
			// 不正又は中断した分割を公開しない返戻
			if(View.FoliageRadiusMeters > 0.0 && !RefinePass(RefineKind::Foliage)) return {};
		}
		for(const TileId Tile : Result.Tree.Leaves()) {
			const TileMetric M = Metric(Tile, true);
			Result.DeepestLevel = std::max(Result.DeepestLevel, Tile.Level);
			Result.MaximumRemainingEdgePixels = std::max(Result.MaximumRemainingEdgePixels, M.Pixels);
			Result.MaximumSurfaceEdgeMeters = std::max(Result.MaximumSurfaceEdgeMeters, M.SurfaceEdge);
			Result.MaximumSurfaceErrorMeters = std::max(Result.MaximumSurfaceErrorMeters, M.SurfaceError);
			Result.MaximumFoliageEdgeMeters = std::max(Result.MaximumFoliageEdgeMeters, M.FoliageEdge);
			Result.MaximumFoliageErrorMeters = std::max(Result.MaximumFoliageErrorMeters, M.FoliageError);
		}
		// 不正又は中断した分割を公開しない返戻
		if(InvalidHeight) return {};
		Result.SurfaceResolutionSatisfied =
		Result.MaximumSurfaceEdgeMeters <= View.SurfaceEdgeMeters && Result.MaximumSurfaceErrorMeters <= View.SurfaceErrorMeters &&
		Result.MaximumStitchErrorMeters <= View.SurfaceErrorMeters;
		Result.FoliageResolutionSatisfied = !View.FoliageRadiusMeters ||
		Result.MaximumFoliageEdgeMeters <= View.FoliageEdgeMeters && Result.MaximumFoliageErrorMeters <= View.FoliageErrorMeters &&
		Result.MaximumFoliageStitchErrorMeters <= View.FoliageErrorMeters;
		Result.CachedTiles = Cache ? Cache->TileCount() : 0;
		Result.NeighborCacheHits = Cache ? Cache->State->Neighbors->Hits() - PriorNeighborHits : 0;
		Result.CachedNeighborTiles = Cache ? Cache->State->Neighbors->TileCount() : 0;
		// 同じ定義から得た評価結果の返戻
		return Result;
	}
} // namespace PlanetCore
