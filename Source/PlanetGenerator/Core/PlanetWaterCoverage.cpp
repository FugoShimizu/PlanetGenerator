#include "PlanetWaterCoverage.h"
#include "PlanetOceanSurface.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <stop_token>
#include <unordered_map>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		constexpr std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		constexpr std::size_t MaximumVertices = 262144;
		constexpr std::size_t MaximumTriangles = 524288;

		/** 正規頂点の各属性を混合し，番号の挿入順序とは独立に探索する */
		struct WaterVertexHash {
			/**
			 * 等しい正規鍵に同じ探索値を与える
			 * @param Key 共有頂点の正規鍵
			 * @return 全属性を混合した探索値
			 */
			std::size_t operator()(const VertexKey &Key) const noexcept {
				const auto Mix = [](std::uint64_t Value) -> std::uint64_t {
					Value = (Value ^ (Value >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
					Value = (Value ^ (Value >> 27)) * UINT64_C(0X94D049BB133111EB);
					// 上位と下位の偏りを混合した値の返戻
					return Value ^ Value >> 31;
				};
				const std::uint64_t Meta = static_cast<std::uint64_t>(Key.Kind) | static_cast<std::uint64_t>(Key.Owner) << 8 |
				static_cast<std::uint64_t>(Key.Other) << 16 | static_cast<std::uint64_t>(Key.Level) << 24;
				// 一致判定自体は VertexKey の全属性に依る等値比較へ委ねる
				return static_cast<std::size_t>(Mix(Meta ^ Mix(Key.U) ^ Mix(Key.V) >> 1));
			}
		};

		/** 同じ要求内で不変な元三角形に対する検査結果 */
		struct WaterSourceCheck {
			bool Failed = false;
			bool FeatureFailed = false;
			double WaterErrorMeters = 0;
		};

		/** 同じ要求内で位置が変わらない水面頂点の精度検査結果 */
		struct WaterVertexCheck {
			double WaterErrorMeters = 0;
			double ShoreErrorMeters = 0;
			double FloatErrorMeters = 0;
			bool Bad = false;
		};

		/** 頂点順序も含めた参照元の識別子を探索する */
		struct WaterTriangleHash {
			/**
			 * 三頂点の順序を保持した探索値を作る
			 * @param Vertices 同じ要求内で不変な頂点識別子
			 * @return 順序を含む探索値
			 */
			std::size_t operator()(const std::array<std::uint32_t, 3> &Vertices) const noexcept {
				std::uint64_t Hash = 1469598103934665603ULL;
				for(const std::uint32_t Vertex : Vertices) Hash = (Hash ^ Vertex) * 1099511628211ULL;
				// 出力順序には使用しない探索値の返戻
				return static_cast<std::size_t>(Hash);
			}
		};

		/**
		 * 頂点番号の向きに依らない共有辺の識別子を作る
		 * @param A 第１の頂点番号
		 * @param B 第２の頂点番号
		 * @return 小さい番号を上位にした識別子
		 */
		std::uint64_t EdgeKey(const std::uint32_t A, const std::uint32_t B) {
			// 隣接面から同じ交点を参照する識別子の返戻
			return static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
		}

		/**
		 * 局所座標で三角形が有限な面積を持つかを調べる
		 * @param A 第１の頂点
		 * @param B 第２の頂点
		 * @param C 第３の頂点
		 * @return 倍精度で表現可能な正の面積を持つ場合に真
		 */
		bool HasArea(const Vec3 A, const Vec3 B, const Vec3 C) {
			const Vec3 Cross = (B - A).Cross(C - A);
			const double AreaSquared = Cross.Dot(Cross);
			// 微小な面を任意の閾値で削らず，表現出来ない候補だけを拒否する判定の返戻
			return std::isfinite(AreaSquared) && AreaSquared > 0;
		}

		/**
		 * 辺への最短視距離から保守的な投影長を求める
		 * @param A 第１の惑星座標
		 * @param B 第２の惑星座標
		 * @param View 視点と内部遮蔽球
		 * @return 画素長，確実に遮蔽される辺は零，退化又は非有限は NaN
		 */
		double ProjectedWaterEdgePixels(const Vec3 A, const Vec3 B, const WaterCoverageView &View) {
			const Vec3 AB = B - A;
			const double Length = AB.Length();
			// 潰れた辺を正常な水際として扱わない返戻
			if(!(Length > 0) || !std::isfinite(Length)) return std::numeric_limits<double>::quiet_NaN();
			const Vec3 Center = (A + B) * 0.5, Ray = Center - View.ObserverMeters;
			const double RaySquared = Ray.Dot(Ray);
			const double T = RaySquared > 0 ? std::clamp(-View.ObserverMeters.Dot(Ray) / RaySquared, 0.0, 1.0) : 0;
			// 中点への線分を辺の半長だけ太らせても遮られる辺の返戻
			if(T > 0 && T < 1 && (View.ObserverMeters + Ray * T).Length() + Length * 0.5 < View.OccluderRadiusMeters) return 0;
			const double Along = std::clamp((View.ObserverMeters - A).Dot(AB) / (Length * Length), 0.0, 1.0);
			const double Distance = std::max(0.01, (A + AB * Along - View.ObserverMeters).Length());
			// 視方向に依らない保守的な投影長の返戻
			return Length * View.FocalLengthPixels / Distance;
		}
	}

	/**
	 * 近景の分割を保持しつつ，排水骨格と同じ固定段階迄の共通頂点を確保する
	 * @param NearTree 現在の整合した近景分割
	 * @param MinimumLevel 排水骨格の最低段階，最大６
	 * @param Stop 中断要求
	 * @return 全球被覆と平衡を保つ私有コピー，予算超過・不正・中断は空
	 */
	std::optional<LodTree> PrepareWaterSkeletonTree(
		const LodTree &NearTree,
		const std::uint8_t MinimumLevel,
		const std::stop_token Stop
	) {
		// 有界な全球骨格の段階を超える要求又は中断に対する返戻
		if(MinimumLevel > 6 || Stop.stop_requested()) return {};
		LodTree Result = NearTree;
		for(std::uint8_t Level = 0; Level < MinimumLevel; ++Level) {
			std::vector<TileId> Pending;
			for(const TileId Tile : Result.Leaves()) if(Tile.Level == Level) Pending.push_back(Tile);
			for(const TileId Tile : Pending) {
				// 中断又は元の近景を保持出来ない予算超過に対する返戻
				if(Stop.stop_requested()) return {};
				const bool Rejected = Result.IsLeaf(Tile) && Result.Refine(Tile, LodTree::MaximumLeaves).Status != LodEditStatus::Applied;
				// 完全な骨格と近景を保てない分割の返戻
				if(Rejected) return {};
			}
		}
		// 河道の固定節点を持ち，近景も維持した全球分割の返戻
		return Result;
	}

	/**
	 * 各辺の水際を一度だけ作り，隣接する濡れた多角形から同じ頂点を参照する
	 * @param Samples 共通分割の水面位置と符号付き水深
	 * @param Triangles 有限且つ整合した三角形分割
	 * @param Stop 中断要求
	 * @return 切出した水面，無効又は未完成の場合は空
	 */
	std::optional<WaterCoverageMesh> BuildWaterCoverage(
		std::span<const WaterCoverageSample> Samples,
		std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop
	) {
		// 無制限な配列確保又は開始前の中断を拒否する返戻
		if(Samples.size() > MaximumVertices || Triangles.size() > MaximumTriangles || Stop.stop_requested()) return {};
		for(const WaterCoverageSample &Sample : Samples) {
			const bool Invalid =
			!std::isfinite(Sample.Position.X) || !std::isfinite(Sample.Position.Y) || !std::isfinite(Sample.Position.Z) ||
			!std::isfinite(Sample.DepthMeters);
			// 非有限な位置や水深を乾いた地点として隠さない返戻
			if(Invalid) return {};
		}
		std::vector<std::uint32_t> Original(Samples.size(), Missing);
		std::map<std::uint64_t, std::uint32_t> Crossings;
		for(const std::array<std::uint32_t, 3> &T : Triangles) {
			const bool Invalid = Stop.stop_requested() || T[0] >= Samples.size() || T[1] >= Samples.size() || T[2] >= Samples.size() ||
			!HasArea(Samples[T[0]].Position, Samples[T[1]].Position, Samples[T[2]].Position);
			// 元の分割が不正な場合又は中断された場合の返戻
			if(Invalid) return {};
			const double FirstDepth = Samples[T[0]].DepthMeters, SecondDepth = Samples[T[1]].DepthMeters;
			const double ThirdDepth = Samples[T[2]].DepthMeters;
			if(FirstDepth <= 0 && SecondDepth <= 0 && ThirdDepth <= 0) continue;
			for(unsigned k = 0; k < 3; ++k) {
				const std::uint32_t A = T[k], B = T[(k + 1) % 3];
				const double X = Samples[A].DepthMeters, Y = Samples[B].DepthMeters;
				if(X >= 0) Original[A] = 0;
				if(X < 0 && Y > 0 || X > 0 && Y < 0) Crossings.try_emplace(EdgeKey(A, B), Missing);
			}
		}
		const std::size_t OriginalCount = static_cast<std::size_t>(std::count(Original.begin(), Original.end(), 0U));
		// 水際を加えると頂点予算を超える候補の返戻
		if(OriginalCount + Crossings.size() > MaximumVertices) return {};
		WaterCoverageMesh Result;
		Result.Vertices.reserve(OriginalCount + Crossings.size());
		for(std::uint32_t i = 0; i < Samples.size(); ++i) if(Original[i] != Missing) {
			Original[i] = static_cast<std::uint32_t>(Result.Vertices.size());
			Result.Vertices.push_back({ Samples[i].Position, i, i, 0 });
		}
		for(std::pair<const std::uint64_t, std::uint32_t> &Entry : Crossings) {
			// 水際構築中の中断に対する返戻
			if(Stop.stop_requested()) return {};
			const std::uint32_t A = static_cast<std::uint32_t>(Entry.first >> 32), B = static_cast<std::uint32_t>(Entry.first);
			const double Scale = std::max(std::abs(Samples[A].DepthMeters), std::abs(Samples[B].DepthMeters));
			const double X = Samples[A].DepthMeters / Scale, Y = Samples[B].DepthMeters / Scale, T = X / (X - Y);
			const Vec3 P = Samples[A].Position, Q = Samples[B].Position;
			Entry.second = static_cast<std::uint32_t>(Result.Vertices.size());
			Result.Vertices.push_back({ { std::lerp(P.X, Q.X, T), std::lerp(P.Y, Q.Y, T), std::lerp(P.Z, Q.Z, T) }, A, B, T });
		}
		for(std::uint32_t i = 0; i < Triangles.size(); ++i) {
			// 完成していない面を公開しない中断時の返戻
			if(Stop.stop_requested()) return {};
			const std::array<std::uint32_t, 3> &T = Triangles[i];
			const double FirstDepth = Samples[T[0]].DepthMeters, SecondDepth = Samples[T[1]].DepthMeters;
			const double ThirdDepth = Samples[T[2]].DepthMeters;
			if(FirstDepth <= 0 && SecondDepth <= 0 && ThirdDepth <= 0) continue;
			std::array<std::uint32_t, 4> Polygon;
			unsigned Count = 0;
			for(unsigned k = 0; k < 3; ++k) {
				const std::uint32_t A = T[(k + 2) % 3], B = T[k];
				const double X = Samples[A].DepthMeters, Y = Samples[B].DepthMeters;
				if(X < 0 && Y > 0 || X > 0 && Y < 0) Polygon[Count++] = Crossings.at(EdgeKey(A, B));
				if(Y >= 0) Polygon[Count++] = Original[B];
			}
			for(unsigned k = 1; k + 1 < Count; ++k) {
				const std::array<std::uint32_t, 3> Face { Polygon[0], Polygon[k], Polygon[k + 1] };
				const bool Invalid = Result.Triangles.size() == MaximumTriangles ||
				!HasArea(Result.Vertices[Face[0]].Position, Result.Vertices[Face[1]].Position, Result.Vertices[Face[2]].Position);
				// 丸めで潰れた面や予算超過を黙って穴として公開しない返戻
				if(Invalid) return {};
				Result.Triangles.push_back(Face);
				Result.SourceTriangles.push_back(i);
			}
		}
		// 同じ辺の水際を共有した水面の返戻
		return Result;
	}

	namespace {
		/**
		 * 三辺の半空間で弦を切取り，球面三角形を通る短い弧の区間を求める
		 * @param Start 弧の始点
		 * @param End 弧の終点
		 * @param Triangle 同じ半球の非退化な三角形
		 * @return 弦上の交差区間，交差しない場合は空
		 */
		std::optional<std::array<double, 2>> ClipWaterArc(
			const UnitDirection Start,
			const UnitDirection End,
			const std::array<UnitDirection, 3> &Triangle
		) {
			double Low = 0, High = 1;
			for(unsigned k = 0; k < 3; ++k) {
				const Vec3 A = Triangle[k].Vector(), B = Triangle[(k + 1) % 3].Vector(), C = Triangle[(k + 2) % 3].Vector();
				const std::optional<UnitDirection> Normal = UnitDirection::FromVector(A.Cross(B - A));
				// 退化した判定面から必要な流路を除外しない保守的な返戻
				if(!Normal) return std::array<double, 2>{ 0, 1 };
				const Vec3 N = Normal->Vector();
				const double Sign = (C - A).Dot(N) >= 0 ? 1 : -1, First = (Start.Vector() - A).Dot(N) * Sign + 1E-14;
				const double Last = (End.Vector() - A).Dot(N) * Sign + 1E-14;
				// 両端が同じ外側に有る場合の返戻
				if(First < 0 && Last < 0) return {};
				if(First >= 0 && Last >= 0) continue;
				const double Fraction = First / (First - Last);
				if(First < 0) Low = std::max(Low, Fraction);
				else High = std::min(High, Fraction);
				// 三辺の内側へ同時に入る区間が無い場合の返戻
				if(Low > High) return {};
			}
			// 正規化した弦と同じ大円弧が通る区間の返戻
			return std::array<double, 2>{ Low, High };
		}
	}

	/**
	 * 短い大円弧と球面三角形の交差を判定する
	 * @param Start 弧の始点
	 * @param End 弧の終点
	 * @param Triangle 同じ半球の非退化な三角形
	 * @return 内側又は保守的な境界と交差する場合に真
	 */
	bool ArcIntersectsSphericalTriangle(
		const UnitDirection Start,
		const UnitDirection End,
		const std::array<UnitDirection, 3> &Triangle
	) {
		// 三辺で切取った区間の有無の返戻
		return ClipWaterArc(Start, End, Triangle).has_value();
	}

	/**
	 * 三角形を通る弧の両端を弦面へ投影し，線形水深が全区間で正か調べる
	 * @param Start 弧の始点
	 * @param End 弧の終点
	 * @param Triangle 同じ半球の非退化な三角形
	 * @param Depths 頂点の水深を実半径で割り，共通の正定数を掛けた射影水深
	 * @return 交差しない，又は交差区間の全体が線形な水域内の場合に真
	 */
	bool ArcCoveredByLinearWater(
		const UnitDirection Start,
		const UnitDirection End,
		const std::array<UnitDirection, 3> &Triangle,
		const std::array<double, 3> &Depths
	) {
		const std::optional<std::array<double, 2>> Interval = ClipWaterArc(Start, End, Triangle);
		// 三角形外の流路は此の面の分割条件としない返戻
		if(!Interval) return true;
		const Vec3 A = Triangle[0].Vector(), U = Triangle[1].Vector() - A, V = Triangle[2].Vector() - A;
		const Vec3 N = U.Cross(V);
		const double Squared = N.Dot(N);
		// 退化又は非有限な面を被覆済みとしない返戻
		if(!(Squared > 0) || !std::isfinite(Squared)) return false;
		for(const double T : *Interval) {
			const Vec3 P = Start.Vector() * (1 - T) + End.Vector() * T;
			const double Denominator = N.Dot(P);
			// 射影出来ない区間を被覆済みとしない返戻
			if(!Denominator || !std::isfinite(Denominator)) return false;
			const Vec3 Q = P * (N.Dot(A) / Denominator) - A;
			const double B = Q.Cross(V).Dot(N) / Squared, C = U.Cross(Q).Dot(N) / Squared;
			const double Depth = Depths[0] * (1 - B - C) + Depths[1] * B + Depths[2] * C;
			// 途中を結ぶ線分の片端でも水域外なら，発見用の細分化を継続する返戻
			if(!(Depth > 0) || !std::isfinite(Depth)) return false;
		}
		// 凸な線形水域に両端が入る為，途中にも欠落が無い判定の返戻
		return true;
	}

	/**
	 * 有界な反復で水面の曲率と水際を追従させ，共有辺を同時に分割する
	 * @param Partition 元の球面分割
	 * @param Frame 共通の局所座標
	 * @param Focus 近景の中心
	 * @param NearRadiusMeters 近景の検査半径
	 * @param Evaluate 水面と水深の地点評価
	 * @param Diagnostics 最後の精度検査値
	 * @param Stop 中断要求
	 * @param View 投影寸法の制限，空は局所精度だけを検査
	 * @param RequiredEdge 同じ要求内で不変な流路等の形状から求める必要な辺長
	 * @param Hierarchy 元の平衡分割と所有葉
	 * @param ReuseSourceChecks 同じ要求内の同一面の検査結果を再利用，比較検査時だけ無効化可能
	 * @return 検査済の候補，又は不正・未完成を表す空
	 */
	std::optional<AdaptiveWaterCoverage> BuildAdaptiveWaterCoverage(
		SphereMesh Partition,
		const GroundFrame &Frame,
		const UnitDirection Focus,
		const double NearRadiusMeters,
		const std::function<WaterCoverageSample(UnitDirection)> &Evaluate,
		WaterRefinementDiagnostics &Diagnostics,
		const std::stop_token Stop,
		const WaterCoverageView *View,
		const WaterDetailEvaluator &RequiredEdge,
		std::optional<SphereRefinementHierarchy> Hierarchy,
		const bool ReuseSourceChecks
	) {
		const auto Milliseconds = []() -> double {
			return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
		};
		double StageStart = Milliseconds();
		Diagnostics = {};
		const double RadiusMeters = Frame.RadiusMeters;
		const bool InvalidInput =
		!Evaluate || !std::isfinite(NearRadiusMeters) || NearRadiusMeters < 0 || !std::isfinite(RadiusMeters) || RadiusMeters <= 0 ||
		Partition.Directions.size() > MaximumVertices || Partition.Triangles.size() > MaximumTriangles || Stop.stop_requested();
		// 不正な検査条件や初期予算超過を拒否する返戻
		if(InvalidInput) return {};
		if(View) {
			const double FocalLength = View->FocalLengthPixels, ShoreEdge = View->MaximumShoreEdgePixels;
			const double OccluderRadius = View->OccluderRadiusMeters;
			const bool InvalidView =
			!std::isfinite(View->ObserverMeters.Length()) || !std::isfinite(FocalLength) || FocalLength <= 0 || !std::isfinite(ShoreEdge) ||
			ShoreEdge <= 0 || !std::isfinite(OccluderRadius) || OccluderRadius < 0;
			// 不正な視点を使って精度検査を省略しない返戻
			if(InvalidView) return {};
		}
		if(Hierarchy) {
			const bool InvalidHierarchy =
			Hierarchy->TriangleOwners.size() != Partition.Triangles.size() || Partition.Keys.size() != Partition.Directions.size();
			// 参照元の面と識別子が一致しない階層の返戻
			if(InvalidHierarchy) return {};
			// 分割後の葉を参照しない面の返戻
			for(const TileId Owner : Hierarchy->TriangleOwners) if(!Hierarchy->Tree.IsLeaf(Owner)) return {};
		} else Partition.Keys.clear();
		std::vector<WaterCoverageSample> Samples;
		Samples.reserve(Partition.Directions.size());
		for(const UnitDirection D : Partition.Directions) {
			// 評価を中断した候補の返戻
			if(Stop.stop_requested()) return {};
			Samples.push_back(Evaluate(D));
		}
		// 広い可視範囲の細い河道は局所円より深い分割を要する為，投影検査時だけ反復予算を拡張する
		Diagnostics.StageMilliseconds[0] = Milliseconds() - StageStart;
		const std::uint32_t MaximumPasses = View ? 24 : 8;
		Diagnostics.Passes.reserve(MaximumPasses + 1);
		// 識別子は同じ要求内の頂点に固定し，再組立で番号が変わっても標本の同一性を保つ
		std::vector<std::uint32_t> RefinementLevel(Partition.Triangles.size(), 0), VertexIdentities(Partition.Directions.size());
		for(std::uint32_t i = 0; i < VertexIdentities.size(); ++i) VertexIdentities[i] = i;
		std::uint32_t NextIdentity = static_cast<std::uint32_t>(VertexIdentities.size());
		std::unordered_map<std::array<std::uint32_t, 3>, WaterSourceCheck, WaterTriangleHash> PreviousChecks;
		std::unordered_map<std::uint64_t, WaterVertexCheck> VertexChecks;
		for(std::uint32_t Pass = 0; Pass <= MaximumPasses; ++Pass) {
			std::vector<WaterRefinementPass> History = std::move(Diagnostics.Passes);
			const std::array<double, 4> Timings = Diagnostics.StageMilliseconds;
			const std::size_t CheckHits = Diagnostics.SourceCheckHits, CheckEvaluations = Diagnostics.SourceCheckEvaluations;
			const std::size_t VertexHits = Diagnostics.VertexCheckHits, VertexEvaluations = Diagnostics.VertexCheckEvaluations;
			Diagnostics = {};
			Diagnostics.StageMilliseconds = Timings;
			Diagnostics.SourceCheckHits = CheckHits;
			Diagnostics.SourceCheckEvaluations = CheckEvaluations;
			Diagnostics.VertexCheckHits = VertexHits;
			Diagnostics.VertexCheckEvaluations = VertexEvaluations;
			Diagnostics.Passes = std::move(History);
			Diagnostics.Pass = Pass;
			Diagnostics.SourceVertices = Partition.Directions.size();
			Diagnostics.SourceTriangles = Partition.Triangles.size();
			StageStart = Milliseconds();
			std::optional<WaterCoverageMesh> Wet = BuildWaterCoverage(Samples, Partition.Triangles, Stop);
			Diagnostics.StageMilliseconds[1] += Milliseconds() - StageStart;
			StageStart = Milliseconds();
			// 不正な元形状や評価値を公開しない返戻
			if(!Wet) return {};
			std::vector<bool> Failed(Partition.Triangles.size(), false), BadVertex(Wet->Vertices.size(), false);
			std::unordered_map<std::array<std::uint32_t, 3>, WaterSourceCheck, WaterTriangleHash> NextChecks;
			if(ReuseSourceChecks) NextChecks.reserve(Partition.Triangles.size());
			for(std::size_t i = 0; i < Partition.Triangles.size(); ++i) {
				// 検査中の中断時の返戻
				if(Stop.stop_requested()) return {};
				const std::array<std::uint32_t, 3> &T = Partition.Triangles[i];
				const std::array<std::uint32_t, 3> Identity { VertexIdentities[T[0]], VertexIdentities[T[1]], VertexIdentities[T[2]] };
				if(ReuseSourceChecks) {
					if(
						const std::unordered_map<std::array<std::uint32_t, 3>, WaterSourceCheck, WaterTriangleHash>::const_iterator Found =
						PreviousChecks.find(Identity);
						Found != PreviousChecks.end()
					) {
						const WaterSourceCheck &Check = Found->second;
						Failed[i] = Check.Failed;
						Diagnostics.FeatureTriangles += Check.FeatureFailed ? 1 : 0;
						Diagnostics.WaterErrorMeters = std::max(Diagnostics.WaterErrorMeters, Check.WaterErrorMeters);
						++Diagnostics.SourceCheckHits;
						NextChecks.emplace(Identity, Check);
						continue;
					}
				}
				++Diagnostics.SourceCheckEvaluations;
				WaterSourceCheck Uncached;
				WaterSourceCheck &Check = ReuseSourceChecks ? NextChecks[Identity] : Uncached;
				const std::array<UnitDirection, 3> D { Partition.Directions[T[0]], Partition.Directions[T[1]], Partition.Directions[T[2]] };
				if(RequiredEdge) {
					const UnitDirection Center = *UnitDirection::FromVector(D[0].Vector() + D[1].Vector() + D[2].Vector());
					const double Cap = std::max(
						{
							(D[0].Vector() - Center.Vector()).Length(),
							(D[1].Vector() - Center.Vector()).Length(),
							(D[2].Vector() - Center.Vector()).Length()
						}
					);
					std::array<double, 3> ProjectedDepths;
					for(unsigned k = 0; k < 3; ++k) {
						const double Radius = Frame.LocalToPlanet(Samples[T[k]].Position).Length();
						// 異なる水位の頂点を共通球面へ射影出来ない場合の返戻
						if(!(Radius > 0) || !std::isfinite(Radius)) return {};
						ProjectedDepths[k] = Samples[T[k]].DepthMeters * (Frame.RadiusMeters / Radius);
					}
					const double Limit = RequiredEdge(Center, Cap, D, ProjectedDepths);
					// 無効な分割条件を精度達成として扱わない返戻
					if(!(Limit > 0) || std::isnan(Limit)) return {};
					const double Edge = Frame.RadiusMeters * std::max(
						{ (D[0].Vector() - D[1].Vector()).Length(), (D[1].Vector() - D[2].Vector()).Length(), (D[2].Vector() - D[0].Vector()).Length() }
					);
					if(Edge > Limit) {
						Failed[i] = true;
						++Diagnostics.FeatureTriangles;
						Check.FeatureFailed = true;
					}
				}
				if(!NearRadiusMeters || SphericalTriangleChordDistance(Focus, D) * Frame.RadiusMeters > NearRadiusMeters) {
					Check.Failed = Failed[i];
					continue;
				}
				for(
					const std::array<double, 3> Weight :
					std::array<std::array<double, 3>, 4>{ { { 0.5, 0.5, 0 }, { 0, 0.5, 0.5 }, { 0.5, 0, 0.5 }, { 1.0 / 3, 1.0 / 3, 1.0 / 3 } } }
				) {
					const Vec3 P = Samples[T[0]].Position * Weight[0] + Samples[T[1]].Position * Weight[1] + Samples[T[2]].Position * Weight[2];
					const double Depth =
					Samples[T[0]].DepthMeters * Weight[0] + Samples[T[1]].DepthMeters * Weight[1] + Samples[T[2]].DepthMeters * Weight[2];
					const std::optional<UnitDirection> Direction = UnitDirection::FromVector(Frame.LocalToPlanet(P));
					// 無効な方向を持つ候補の返戻
					if(!Direction) return {};
					const WaterCoverageSample Actual = Evaluate(*Direction);
					const double Error = (P - Actual.Position).Length();
					// 非有限な場を乾いた部分として扱わない返戻
					if(!std::isfinite(Error) || !std::isfinite(Actual.DepthMeters)) return {};
					const double ActualDepth = Actual.DepthMeters;
					if(ActualDepth <= 0 && Depth <= 0) continue;
					Diagnostics.WaterErrorMeters = std::max(Diagnostics.WaterErrorMeters, Error);
					Check.WaterErrorMeters = std::max(Check.WaterErrorMeters, Error);
					Failed[i] = Failed[i] || Error > 0.01 || Actual.DepthMeters > 0.01 && Depth <= 0 || Actual.DepthMeters < -0.01 && Depth > 0;
				}
				Check.Failed = Failed[i];
			}
			PreviousChecks = std::move(NextChecks);
			for(std::size_t i = 0; i < Wet->Vertices.size(); ++i) {
				// 水際検査中の中断時の返戻
				if(Stop.stop_requested()) return {};
				const WaterCoverageVertex &Vertex = Wet->Vertices[i];
				const Vec3 P = Vertex.Position;
				const std::optional<UnitDirection> D = UnitDirection::FromVector(Frame.LocalToPlanet(P));
				// 無効な水際方向を拒否する返戻
				if(!D) return {};
				if(!NearRadiusMeters || (D->Vector() - Focus.Vector()).Length() * Frame.RadiusMeters > NearRadiusMeters + 8) continue;
				const std::uint64_t Identity = EdgeKey(VertexIdentities[Vertex.SourceA], VertexIdentities[Vertex.SourceB]);
				if(ReuseSourceChecks) {
					if(
						const std::unordered_map<std::uint64_t, WaterVertexCheck>::const_iterator Found = VertexChecks.find(Identity);
						Found != VertexChecks.end()
					) {
						const WaterVertexCheck &Check = Found->second;
						Diagnostics.WaterErrorMeters = std::max(Diagnostics.WaterErrorMeters, Check.WaterErrorMeters);
						Diagnostics.ShoreErrorMeters = std::max(Diagnostics.ShoreErrorMeters, Check.ShoreErrorMeters);
						Diagnostics.FloatErrorMeters = std::max(Diagnostics.FloatErrorMeters, Check.FloatErrorMeters);
						BadVertex[i] = Check.Bad;
						++Diagnostics.VertexCheckHits;
						continue;
					}
				}
				++Diagnostics.VertexCheckEvaluations;
				const WaterCoverageSample Actual = Evaluate(*D);
				const Vec3 Float {
					static_cast<float>(P.X * 100) / 100.0,
					static_cast<float>(P.Y * 100) / 100.0,
					static_cast<float>(P.Z * 100) / 100.0
				};
				const double Error = (P - Actual.Position).Length();
				const double ShoreError = Vertex.SourceA != Vertex.SourceB ? std::abs(Actual.DepthMeters) : 0;
				// 非有限な水際評価を拒否する返戻
				if(!std::isfinite(Error) || !std::isfinite(Actual.DepthMeters)) return {};
				const WaterVertexCheck Check { Error, ShoreError, (P - Float).Length(), Error > 0.01 || ShoreError > 0.02 };
				Diagnostics.WaterErrorMeters = std::max(Diagnostics.WaterErrorMeters, Check.WaterErrorMeters);
				Diagnostics.ShoreErrorMeters = std::max(Diagnostics.ShoreErrorMeters, Check.ShoreErrorMeters);
				Diagnostics.FloatErrorMeters = std::max(Diagnostics.FloatErrorMeters, Check.FloatErrorMeters);
				BadVertex[i] = Check.Bad;
				if(ReuseSourceChecks && VertexChecks.size() < MaximumVertices) VertexChecks.emplace(Identity, Check);
			}
			// 座標基準の精度不足は細分化では改善しない為，此処で拒否する返戻
			if(Diagnostics.FloatErrorMeters > 0.001) return {};
			for(std::size_t i = 0; i < Wet->Triangles.size(); ++i) {
				const std::array<std::uint32_t, 3> &T = Wet->Triangles[i];
				if(BadVertex[T[0]] || BadVertex[T[1]] || BadVertex[T[2]]) Failed[Wet->SourceTriangles[i]] = true;
			}
			if(View) {
				// 水深零の頂点を結ぶ辺だけを数え，外周の水際と内部共有辺を区別する
				struct ShoreEdge {
					std::uint32_t First = 0;
					std::uint32_t Second = 0;
					std::uint32_t SourceTriangle = 0;
					std::uint32_t Incidence = 0;
				};
				std::map<std::uint64_t, ShoreEdge> ShoreEdges;
				for(std::size_t i = 0; i < Wet->Triangles.size(); ++i) {
					const std::array<std::uint32_t, 3> &T = Wet->Triangles[i];
					for(unsigned k = 0; k < 3; ++k) {
						const std::uint32_t A = T[k], B = T[(k + 1) % 3];
						const WaterCoverageVertex &U = Wet->Vertices[A], &V = Wet->Vertices[B];
						if(U.SourceA == U.SourceB && Samples[U.SourceA].DepthMeters || V.SourceA == V.SourceB && Samples[V.SourceA].DepthMeters) {
							continue;
						}
						ShoreEdge &Edge = ShoreEdges[EdgeKey(A, B)];
						Edge.First = A;
						Edge.Second = B;
						Edge.SourceTriangle = Wet->SourceTriangles[i];
						++Edge.Incidence;
					}
				}
				for(const std::pair<const std::uint64_t, ShoreEdge> &Entry : ShoreEdges) {
					// 遠景の検査中も中断を受け付ける返戻
					if(Stop.stop_requested()) return {};
					const ShoreEdge &Edge = Entry.second;
					if(Edge.Incidence != 1) continue;
					const Vec3 A = Frame.LocalToPlanet(Wet->Vertices[Edge.First].Position);
					const Vec3 B = Frame.LocalToPlanet(Wet->Vertices[Edge.Second].Position);
					const double Pixels = ProjectedWaterEdgePixels(A, B, *View);
					// 不正な辺を投影条件の達成として扱わない返戻
					if(!std::isfinite(Pixels)) return {};
					if(Pixels > Diagnostics.MaximumShoreEdgePixels) {
						Diagnostics.MaximumShoreEdgePixels = Pixels;
						Diagnostics.WorstShoreStartMeters = A;
						Diagnostics.WorstShoreEndMeters = B;
						Diagnostics.WorstShoreRefinementLevel =
						Hierarchy ? Hierarchy->TriangleOwners[Edge.SourceTriangle].Level : RefinementLevel[Edge.SourceTriangle];
						const std::array<std::uint32_t, 3> &Source = Partition.Triangles[Edge.SourceTriangle];
						for(unsigned k = 0; k < 3; ++k) Diagnostics.WorstShoreSourceDirections[k] = Partition.Directions[Source[k]].Vector();
					}
					if(Pixels > View->MaximumShoreEdgePixels) {
						Failed[Edge.SourceTriangle] = true;
						++Diagnostics.ProjectedEdges;
					}
				}
			}
			Diagnostics.StageMilliseconds[2] += Milliseconds() - StageStart;
			Diagnostics.FailedTriangles = static_cast<std::size_t>(std::count(Failed.begin(), Failed.end(), true));
			Diagnostics.Passes.push_back(
				{
					Pass,
					Diagnostics.WorstShoreRefinementLevel,
					Partition.Directions.size(),
					Diagnostics.FailedTriangles,
					Diagnostics.MaximumShoreEdgePixels,
					(Diagnostics.WorstShoreStartMeters + Diagnostics.WorstShoreEndMeters) * 0.5
				}
			);
			// 全検査を満たした水面と参照元を纏めた返戻
			if(!Diagnostics.FailedTriangles) {
				Diagnostics.RefinementStage = WaterRefinementStage::Complete;
				// 全検査を満たした同じ水面の返戻
				return AdaptiveWaterCoverage{ std::move(Partition), std::move(*Wet), std::move(Hierarchy), Pass, Diagnostics.FloatErrorMeters };
			}
			// 有界な反復回数を使切った未完成候補の返戻
			if(Pass == MaximumPasses) return {};
			StageStart = Milliseconds();
			if(Hierarchy) {
				Diagnostics.RefinementStage = WaterRefinementStage::OwnerRefinement;
				std::set<TileId, TileOrder> Owners;
				for(std::size_t i = 0; i < Failed.size(); ++i) if(Failed[i]) Owners.insert(Hierarchy->TriangleOwners[i]);
				for(const TileId Owner : Owners) {
					// 所有葉の編集途中で中断された未完成候補の返戻
					if(Stop.stop_requested()) return {};
					if(!Hierarchy->Tree.IsLeaf(Owner)) continue;
					const LodEdit Edit = Hierarchy->Tree.Refine(Owner, LodTree::MaximumLeaves);
					Diagnostics.LastLodEditStatus = Edit.Status;
					// 拒否された階層編集の種類を保持する返戻
					if(Edit.Status != LodEditStatus::Applied) return {};
				}
				const AdaptiveSphereMesh PreviousMesh {
					std::move(Partition),
					std::move(Hierarchy->TriangleOwners),
					Hierarchy->StitchMaskCounts
				};
				Diagnostics.RefinementStage = WaterRefinementStage::MeshRebuild;
				std::optional<AdaptiveSphereMesh> Refined = UpdateAdaptiveSphere(Hierarchy->Tree, PreviousMesh, Stop);
				// 再構築の失敗又は終了直後の中断時の返戻
				if(!Refined || Stop.stop_requested()) return {};
				Diagnostics.PlannedVertices = Refined->Mesh.Directions.size();
				Diagnostics.RefinementStage = WaterRefinementStage::OutputBudget;
				const bool OverBudget = Refined->Mesh.Directions.size() > MaximumVertices || Refined->Mesh.Triangles.size() > MaximumTriangles;
				// 公開頂点と三角形の既存予算を超えた候補の返戻
				if(OverBudget) return {};
				std::unordered_map<VertexKey, std::size_t, WaterVertexHash> Previous;
				Previous.reserve(PreviousMesh.Mesh.Keys.size());
				for(std::size_t i = 0; i < PreviousMesh.Mesh.Keys.size(); ++i) Previous.emplace(PreviousMesh.Mesh.Keys[i], i);
				std::vector<WaterCoverageSample> RefinedSamples;
				std::vector<std::uint32_t> RefinedIdentities;
				RefinedIdentities.reserve(Refined->Mesh.Directions.size());
				RefinedSamples.reserve(Refined->Mesh.Directions.size());
				for(std::size_t i = 0; i < Refined->Mesh.Directions.size(); ++i) {
					// 再評価中に中断された未完成候補の返戻
					if(Stop.stop_requested()) return {};
					const std::unordered_map<VertexKey, std::size_t, WaterVertexHash>::const_iterator Found = Previous.find(Refined->Mesh.Keys[i]);
					RefinedSamples.push_back(Found != Previous.end() ? Samples[Found->second] : Evaluate(Refined->Mesh.Directions[i]));
					RefinedIdentities.push_back(Found != Previous.end() ? VertexIdentities[Found->second] : NextIdentity++);
				}
				Samples = std::move(RefinedSamples);
				VertexIdentities = std::move(RefinedIdentities);
				Partition = std::move(Refined->Mesh);
				Hierarchy->TriangleOwners = std::move(Refined->TriangleOwners);
				Hierarchy->StitchMaskCounts = Refined->StitchMaskCounts;
				Diagnostics.StageMilliseconds[3] += Milliseconds() - StageStart;
				continue;
			}
			std::map<std::uint64_t, std::uint32_t> Midpoints;
			for(std::size_t i = 0; i < Failed.size(); ++i) if(Failed[i]) {
				const std::array<std::uint32_t, 3> &T = Partition.Triangles[i];
				for(unsigned k = 0; k < 3; ++k) Midpoints.try_emplace(EdgeKey(T[k], T[(k + 1) % 3]), Missing);
			}
			Diagnostics.PlannedVertices = Partition.Directions.size() + Midpoints.size();
			// 共有中点の追加前に頂点予算を検査する返戻
			if(Partition.Directions.size() + Midpoints.size() > MaximumVertices) return {};
			for(std::pair<const std::uint64_t, std::uint32_t> &Entry : Midpoints) {
				// 中点評価中の中断時の返戻
				if(Stop.stop_requested()) return {};
				const std::uint32_t A = static_cast<std::uint32_t>(Entry.first >> 32), B = static_cast<std::uint32_t>(Entry.first);
				const std::optional<UnitDirection> D =
				UnitDirection::FromVector(Partition.Directions[A].Vector() + Partition.Directions[B].Vector());
				// 対蹠点を結ぶ曖昧な辺の返戻
				if(!D) return {};
				Entry.second = static_cast<std::uint32_t>(Partition.Directions.size());
				Partition.Directions.push_back(*D);
				VertexIdentities.push_back(NextIdentity++);
				Samples.push_back(Evaluate(*D));
			}
			std::vector<std::array<std::uint32_t, 3>> Refined;
			Refined.reserve(std::min(MaximumTriangles, Partition.Triangles.size() + Midpoints.size() * 2));
			std::vector<std::uint32_t> RefinedDepth;
			RefinedDepth.reserve(Refined.capacity());
			for(std::size_t Parent = 0; Parent < Partition.Triangles.size(); ++Parent) {
				const std::array<std::uint32_t, 3> &T = Partition.Triangles[Parent];
				// 隣接面を含む整合化中の中断時の返戻
				if(Stop.stop_requested()) return {};
				std::array<std::uint32_t, 6> Local { T[0], T[1], T[2], Missing, Missing, Missing };
				std::uint8_t Mask = 0;
				for(unsigned k = 0; k < 3; ++k) {
					const std::map<std::uint64_t, std::uint32_t>::const_iterator Found = Midpoints.find(EdgeKey(T[(k + 1) % 3], T[(k + 2) % 3]));
					if(Found != Midpoints.end()) {
						Mask |= static_cast<std::uint8_t>(1U << k);
						Local[k + 3] = Found->second;
					}
				}
				const StitchPattern Pattern = *GetStitchPattern(Mask);
				// 面予算を超える分割を途中迄公開しない返戻
				if(Refined.size() + Pattern.Count > MaximumTriangles) return {};
				for(unsigned k = 0; k < Pattern.Count; ++k) {
					const std::array<std::uint8_t, 3> &Face = Pattern.Triangles[k];
					Refined.push_back({ Local[Face[0]], Local[Face[1]], Local[Face[2]] });
					RefinedDepth.push_back(RefinementLevel[Parent] + (Mask == 7 ? 1 : 0));
				}
			}
			Partition.Triangles = std::move(Refined);
			RefinementLevel = std::move(RefinedDepth);
			Diagnostics.StageMilliseconds[3] += Milliseconds() - StageStart;
		}
		// 未完成候補を公開しない返戻
		return {};
	}
}
