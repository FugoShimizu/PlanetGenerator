#include "PlanetOcean.h"
#include "PlanetLandRelief.h"
#include "PlanetMesoRelief.h"
#include "PlanetOrogenicRidges.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <unordered_map>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		struct DisjointSet {
			std::vector<std::uint32_t> Parents;
			std::uint32_t Root(std::uint32_t I);
			void Join(std::uint32_t A, std::uint32_t B);

			explicit DisjointSet(const std::size_t N);
		};

		/** 海域経路だけに使う球面方向と三角形 */
		struct OceanGraphMesh {
			std::vector<UnitDirection> Directions;
			std::vector<std::array<std::uint32_t, 3>> Triangles;
		};

		constexpr double Pi = 3.14159265358979323846;
		constexpr std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		constexpr std::uint64_t FineOceanGraphVersion = 1;

		/**
		 * 球面方向を単位ベクトルへ正規化する
		 * @param V 有効な評価方向
		 * @return 正規化した方向
		 */
		Vec3 Unit(const Vec3 V) {
			return UnitDirection::FromVector(V)->Vector();
		}

		/**
		 * 指定した軸の座標を取得する
		 * @param V 対象ベクトル
		 * @param I ０～２の軸番号
		 * @return 軸に対応する成分
		 */
		double Axis(const Vec3 V, const unsigned I) {
			return !I ? V.X : I == 1 ? V.Y : V.Z;
		}

		/**
		 * 二つの境界の成分毎の下限を取得する
		 * @param A 第１境界
		 * @param B 第２境界
		 * @return 成分毎の最小値
		 */
		Vec3 Low(const Vec3 A, const Vec3 B) {
			return { std::min(A.X, B.X), std::min(A.Y, B.Y), std::min(A.Z, B.Z) };
		}

		/**
		 * 二つの境界の成分毎の上限を取得する
		 * @param A 第１境界
		 * @param B 第２境界
		 * @return 成分毎の最大値
		 */
		Vec3 High(const Vec3 A, const Vec3 B) {
			return { std::max(A.X, B.X), std::max(A.Y, B.Y), std::max(A.Z, B.Z) };
		}

		/**
		 * 経路圧縮しながら代表節点を求める
		 * @param I 節点番号
		 * @return 所属集合の代表節点
		 */
		std::uint32_t DisjointSet::Root(std::uint32_t I) {
			while(I != Parents[I]) {
				Parents[I] = Parents[Parents[I]];
				I = Parents[I];
			}
			// 圧縮した経路の代表番号の返戻
			return I;
		}

		/**
		 * 二つの節点を同じ連結成分へ統合する
		 * @param A 第１節点
		 * @param B 第２節点
		 */
		void DisjointSet::Join(std::uint32_t A, std::uint32_t B) {
			A = Root(A);
			B = Root(B);
			if(A != B) Parents[std::max(A, B)] = std::min(A, B);
			// 終了
			return;
		}

		/**
		 * 各節点を独立集合として初期化する
		 * @param N 節点数
		 */
		DisjointSet::DisjointSet(const std::size_t N) : Parents(N) {
			std::iota(Parents.begin(), Parents.end(), 0U);
			// 終了
			return;
		}

		/**
		 * 単位球上の三角形の面積を求める
		 * @param A 第１頂点
		 * @param B 第２頂点
		 * @param C 第３頂点
		 * @return 球面面積（ステラジアン）
		 */
		double TriangleArea(const Vec3 A, const Vec3 B, const Vec3 C) {
			return 2 * std::atan2(std::abs(A.Dot(B.Cross(C))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A));
		}

		/**
		 * 海峡からの角距離を有限幅の連続マスクへ写す
		 * @param C 海峡の幾何情報
		 * @param D 球面上の評価方向
		 * @return 被覆率と要求時の解析勾配
		 */
		template<bool Derivatives> TerrainSample CorridorMask(const OceanCorridor &C, const Vec3 D) {
			const double X = D.Dot(C.Start), Y = D.Dot(C.Tangent), Angle = std::atan2(Y, X);
			Vec3 Closest;
			double DotClosest;
			if(Angle > 0 && Angle < C.ArcAngle) {
				DotClosest = std::sqrt(X * X + Y * Y);
				// 海峡幅の外に有る地点の返戻
				if(2 * (1 - DotClosest) >= C.OuterChordSquared) return {};
				if constexpr(Derivatives) Closest = (C.Start * X + C.Tangent * Y) * (1 / DotClosest);
			} else {
				const double EndDot = D.Dot(C.End);
				if constexpr(Derivatives) Closest = X >= EndDot ? C.Start : C.End;
				DotClosest = std::max(X, EndDot);
			}
			const double Q = 2 * (1 - DotClosest);
			// 外側の支持幅を越える地点の返戻
			if(Q >= C.OuterChordSquared) return {};
			// 内側の完全被覆の返戻
			if(Q <= C.InnerChordSquared) return { 1, {} };
			const double Width = C.OuterChordSquared - C.InnerChordSquared, T = (Q - C.InnerChordSquared) / Width;
			const double Smooth = T * T * T * (10 + T * (-15 + 6 * T));
			if constexpr(Derivatives) {
				const double Derivative = 30 * T * T * (1 - T) * (1 - T);
				// 被覆率と方向微分の返戻
				return { 1 - Smooth, Closest * (2 * Derivative / Width) };
			// 微分を省いた同じ被覆率の返戻
			} else return { 1 - Smooth, {} };
		}

		/**
		 * 必要な生成定義だけで球面辺を一段細分化し，細い閉鎖海域を接続候補へ含める
		 * @param Fine 排水前の中規模起伏を含む場合に真
		 * @param Stop 準備の中断要求
		 * @return 海域経路用の全球三角網，準備失敗又は中断時は空
		 */
		std::optional<OceanGraphMesh> BuildOceanGraph(const bool Fine, const std::stop_token Stop) {
			// 中断済みの全球準備の返戻
			if(Stop.stop_requested()) return {};
			std::optional<SphereMesh> Coarse = BuildUniformSphere(6);
			// 基準球面網を準備出来なかった場合の返戻
			if(!Coarse) return {};
			OceanGraphMesh Mesh { std::move(Coarse->Directions), std::move(Coarse->Triangles) };
			// 通常の海陸定義では既存の網の返戻
			if(!Fine) return Mesh;
			std::unordered_map<std::uint64_t, std::uint32_t> Midpoints;
			Midpoints.reserve(Mesh.Triangles.size() * 3 / 2);
			std::vector<std::array<std::uint32_t, 3>> Refined;
			Refined.reserve(Mesh.Triangles.size() * 4);
			const auto Midpoint = [&Mesh, &Midpoints](const std::uint32_t A, const std::uint32_t B) -> std::optional<std::uint32_t> {
				// 範囲外又は退化した辺を追加しない返戻
				if(A >= Mesh.Directions.size() || B >= Mesh.Directions.size() || A == B) return {};
				const std::uint64_t Key = static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
				const std::unordered_map<std::uint64_t, std::uint32_t>::const_iterator Found = Midpoints.find(Key);
				// 共有辺の既存中点の返戻
				if(Found != Midpoints.end()) return Found->second;
				// 地点番号が表現出来ない場合の返戻
				if(Mesh.Directions.size() >= std::numeric_limits<std::uint32_t>::max()) return {};
				const std::optional<UnitDirection> Direction =
				UnitDirection::FromVector(Mesh.Directions[A].Vector() + Mesh.Directions[B].Vector());
				// 無効な球面方向を追加しない返戻
				if(!Direction) return {};
				const std::uint32_t Index = static_cast<std::uint32_t>(Mesh.Directions.size());
				Mesh.Directions.push_back(*Direction);
				Midpoints.emplace(Key, Index);
				// 新しい共有中点番号の返戻
				return Index;
			};
			std::uint32_t Visited = 0;
			for(const std::array<std::uint32_t, 3> &Triangle : Mesh.Triangles) {
				// 全球準備を中断した場合の返戻
				if(!(Visited++ & 255) && Stop.stop_requested()) return {};
				const std::uint32_t A = Triangle[0], B = Triangle[1], C = Triangle[2];
				const std::optional<std::uint32_t> AB = Midpoint(A, B), BC = Midpoint(B, C), CA = Midpoint(C, A);
				// 三角形の中点を準備出来なかった場合の返戻
				if(!AB || !BC || !CA) return {};
				Refined.push_back({ A, *AB, *CA });
				Refined.push_back({ *AB, B, *BC });
				Refined.push_back({ *CA, *BC, C });
				Refined.push_back({ *AB, *BC, *CA });
			}
			Mesh.Triangles = std::move(Refined);
			// 一段細分化した海域接続網の返戻
			return Mesh;
		}
	}

	/**
	 * 排水・侵食に使う海陸と造山及び中規模起伏の定義を識別する
	 * @return 海域の算法版と，地質及び追加場を含む識別子
	 */
	std::uint64_t OceanConnectivityField::DefinitionHash() const {
		const std::uint64_t BaseHash = Geology->Settings().IdentityHash();
		std::uint64_t Hash = (BaseHash ^ OceanConnectivityVersion) * UINT64_C(0X100000001B3);
		if(Orogenic) Hash = (Hash ^ UINT64_C(0X4F524F47454E4943) ^ OrogenicRidgePrototypeVersion) * UINT64_C(0X100000001B3);
		if(Meso) {
			Hash = (Hash ^ UINT64_C(0X5052454D45534F52) ^ PredrainageMesoReliefVersion) * UINT64_C(0X100000001B3);
			if(HasFineOceanGraph) Hash = (Hash ^ UINT64_C(0X46494E454F434541) ^ FineOceanGraphVersion) * UINT64_C(0X100000001B3);
			else Hash = (Hash ^ UINT64_C(0X5348415245444F43) ^ UINT64_C(1)) * UINT64_C(0X100000001B3);
		}
		if(PredrainageLandRelief) Hash = (Hash ^ UINT64_C(0X5052454C414E4452) ^ PredrainageLandReliefVersion) * UINT64_C(0X100000001B3);
		if(AdaptiveSamplingHash) Hash = (Hash ^ UINT64_C(0X41444150544F4345) ^ AdaptiveSamplingHash) * UINT64_C(0X100000001B3);
		// 起伏の追加順を区別する識別子の返戻
		return Hash;
	}

	/**
	 * 中規模起伏の追加が排水前の海陸場に含まれるか調べる
	 * @return 含まれる場合に真
	 */
	bool OceanConnectivityField::HasPredrainageMesoRelief() const {
		return static_cast<bool>(Meso);
	}

	/**
	 * 細部起伏を排水前の海陸場へ含めたか調べる
	 * @return 含まれる場合に真
	 */
	bool OceanConnectivityField::HasPredrainageLandRelief() const {
		return PredrainageLandRelief;
	}

	/**
	 * 同じ惑星の元地質を取得する
	 * @return 不変な地質場
	 */
	const GeologyField &OceanConnectivityField::Base() const {
		return *Geology;
	}

	/**
	 * 海域接続の準備統計を取得する
	 * @return 保持している統計
	 */
	const OceanConnectivityStatistics &OceanConnectivityField::Statistics() const {
		return Stats;
	}

	/**
	 * 接続海峡を読取専用で取得する
	 * @return 保持している海峡の範囲
	 */
	std::span<const OceanCorridor> OceanConnectivityField::Corridors() const {
		return Channels;
	}

	/**
	 * 大洋接続の前に使う地質・造山・中規模起伏の連続な標高を評価する
	 * @param Direction 球面上の評価方向
	 * @return 元の海岸線を保つ標高と，要求時の解析接線勾配
	 */
	template<bool Derivatives> TerrainSample OceanConnectivityField::EvaluateSourceImpl(const UnitDirection Direction) const {
		TerrainSample Result = [&] {
			// 元地質の標高と解析微分の返戻
			if constexpr(Derivatives) return Geology->Evaluate(Direction).Terrain;
			// 元地質の標高だけの返戻
			else return TerrainSample{ Geology->EvaluateElevation(Direction), {} };
		}();
		// 追加起伏を持たない元地質の返戻
		if(!Orogenic && !Meso) return Result;
		const double Sea = Geology->Settings().SeaLevelMeters;
		if(Orogenic) {
			const double T = std::clamp((Result.ElevationMeters - Sea) / 500.0, 0.0, 1.0);
			if(T > 0) {
				const double Weight = T * T * T * (10 + T * (-15 + 6 * T));
				const TerrainSample Ridge = [&] {
					// 造山場の標高と解析微分の返戻
					if constexpr(Derivatives) return Orogenic->Evaluate(Direction);
					// 造山場の標高だけの返戻
					else return TerrainSample{ Orogenic->EvaluateElevation(Direction), {} };
				}();
				Vec3 Gradient;
				if constexpr(Derivatives) {
					const double Derivative = T < 1 ? 30 * T * T * (1 - T) * (1 - T) / 500.0 : 0;
					Gradient =
					Result.TangentialGradientPerRadian * (1 + Derivative * Ridge.ElevationMeters) + Ridge.TangentialGradientPerRadian * Weight;
				}
				Result = { Result.ElevationMeters + Weight * Ridge.ElevationMeters, Gradient };
			}
		}
		if(Meso) {
			const double T = std::clamp((Result.ElevationMeters - Sea - 150.0) / 850.0, 0.0, 1.0);
			if(T > 0) {
				const double Weight = T * T * T * (10 + T * (-15 + 6 * T));
				const TerrainSample Relief = [&] {
					// 中規模起伏と解析微分の返戻
					if constexpr(Derivatives) return Meso->Evaluate(Direction);
					// 中規模起伏の標高だけの返戻
					else return TerrainSample{ Meso->EvaluateElevation(Direction), {} };
				}();
				const double Scale = Geology->Settings().RadiusMeters / (Geology->Settings().RadiusMeters + 250000);
				Vec3 Gradient;
				if constexpr(Derivatives) {
					const double Derivative = T < 1 ? 30 * T * T * (1 - T) * (1 - T) / 850.0 : 0;
					Gradient = Result.TangentialGradientPerRadian * (1 + Scale * Derivative * Relief.ElevationMeters) +
					Relief.TangentialGradientPerRadian * (Scale * Weight);
				}
				Result = { Result.ElevationMeters + Scale * Weight * Relief.ElevationMeters, Gradient };
			}
		}
		if(PredrainageLandRelief) {
			constexpr double Width = PredrainageLandReliefFullMeters - PredrainageLandReliefStartMeters;
			const double T = std::clamp((Result.ElevationMeters - Sea - PredrainageLandReliefStartMeters) / Width, 0.0, 1.0);
			if(T > 0) {
				const double Weight = T * T * T * (10 + T * (-15 + 6 * T));
				const TerrainJet<Derivatives> Detail = SampleLandRelief<Derivatives>(
					Direction.Vector(),
					Geology->Settings().RadiusMeters,
					Geology->Settings().Seed ^ UINT64_C(0X5355424752494401)
				);
				Vec3 Gradient;
				if constexpr(Derivatives) {
					const double Derivative = T < 1 ? 30 * T * T * (1 - T) * (1 - T) / Width : 0;
					const Vec3 D = Direction.Vector(), Tangent = Detail.G - D * Detail.G.Dot(D);
					Gradient = Result.TangentialGradientPerRadian * (1 + Derivative * Detail.V) + Tangent * Weight;
				}
				Result = { Result.ElevationMeters + Weight * Detail.V, Gradient };
			}
		}
		// 大洋接続前の標高と要求した微分の返戻
		return Result;
	}

	/**
	 * 空間索引で海峡補正を評価して海陸へ合成する
	 * @param Direction 球面上の評価方向
	 * @return 標高と要求時の解析勾配
	 */
	template<bool Derivatives> TerrainSample OceanConnectivityField::EvaluateImpl(const UnitDirection Direction) const {
		const TerrainSample Original = EvaluateSourceImpl<Derivatives>(Direction);
		const double Excess = Original.ElevationMeters - FloorMeters;
		// 海峡補正が不要な標高の返戻
		if(Index.empty() || Excess <= 0) return Original;
		const Vec3 D = Direction.Vector();
		double Remaining = 1;
		Vec3 RemainingGradient;
		// 有界な全球網の平衡木は，探索配列の深さ64を下回る
		std::array<std::uint32_t, 64> Stack {};
		unsigned Count = 1;
		while(Count) {
			const IndexNode &Node = Index[Stack[--Count]];
			if(D.X < Node.Min.X || D.Y < Node.Min.Y || D.Z < Node.Min.Z || D.X > Node.Max.X || D.Y > Node.Max.Y || D.Z > Node.Max.Z) {
				continue;
			}
			if(!Node.Count) {
				Stack[Count++] = Node.Right;
				Stack[Count++] = Node.Left;
				continue;
			}
			for(std::uint32_t I = Node.Begin; I < Node.Begin + Node.Count; ++I) {
				const TerrainSample Mask = CorridorMask<Derivatives>(Channels[Order[I]], D);
				if constexpr(Derivatives) {
					RemainingGradient = RemainingGradient * (1 - Mask.ElevationMeters) - Mask.TangentialGradientPerRadian * Remaining;
				}
				Remaining *= 1 - Mask.ElevationMeters;
			}
		}
		// ５次接続の積分で両端を２階迄合わせる
		// 遷移幅以上を全被覆した地点は最低水深へ制限する
		double Positive = Excess - 0.5 * TransitionMeters, Derivative = 1;
		if(Excess < TransitionMeters) {
			const double T = Excess / TransitionMeters;
			Positive = TransitionMeters * T * T * T * T * (2.5 + T * (-3 + T));
			Derivative = T * T * T * (10 + T * (-15 + 6 * T));
		}
		Vec3 Gradient;
		if constexpr(Derivatives) {
			Gradient = Original.TangentialGradientPerRadian * (1 - (1 - Remaining) * Derivative) + RemainingGradient * Positive;
		}
		// 海峡補正後の標高と接線勾配の返戻
		if constexpr(Derivatives) return { Original.ElevationMeters - (1 - Remaining) * Positive, Gradient - D * Gradient.Dot(D) };
		// 同じ海峡補正後の標高だけの返戻
		else return { Original.ElevationMeters - (1 - Remaining) * Positive, {} };
	}

	/**
	 * 同じ海陸場の標高と接線勾配を評価する
	 * @param D 球面上の評価方向
	 * @return 標高と解析勾配
	 */
	TerrainSample OceanConnectivityField::Evaluate(const UnitDirection D) const {
		return EvaluateImpl<true>(D);
	}

	/**
	 * 微分を省いて同じ海陸場の標高を評価する
	 * @param D 球面上の評価方向
	 * @return 海峡を含む標高
	 */
	double OceanConnectivityField::EvaluateElevation(const UnitDirection D) const {
		return EvaluateImpl<false>(D).ElevationMeters;
	}

	/**
	 * 有限幅の全海峡を空間索引へ格納する
	 */
	void OceanConnectivityField::BuildIndex() {
		Index.clear();
		Order.clear();
		// 海峡が無い場合の終了
		if(Channels.empty()) return;
		Order.resize(Channels.size());
		std::iota(Order.begin(), Order.end(), 0U);
		Index.reserve(Channels.size() * 2);
		const auto Build = [&](auto &&Self, const std::uint32_t Begin, const std::uint32_t End) -> std::uint32_t {
			IndexNode Node;
			Node.Min = Channels[Order[Begin]].BoundsMin;
			Node.Max = Channels[Order[Begin]].BoundsMax;
			for(std::uint32_t I = Begin + 1; I < End; ++I) {
				Node.Min = Low(Node.Min, Channels[Order[I]].BoundsMin);
				Node.Max = High(Node.Max, Channels[Order[I]].BoundsMax);
			}
			const std::uint32_t Id = static_cast<std::uint32_t>(Index.size());
			Index.push_back(Node);
			if(End - Begin < 5) {
				std::sort(Order.begin() + Begin, Order.begin() + End);
				Index[Id].Begin = Begin;
				Index[Id].Count = End - Begin;
				// 構築した索引節点の番号の返戻
				return Id;
			}
			const Vec3 Extent = Node.Max - Node.Min;
			const unsigned Split = Extent.X >= Extent.Y && Extent.X >= Extent.Z ? 0 : Extent.Y >= Extent.Z ? 1 : 2;
			const std::uint32_t Middle = Begin + (End - Begin) / 2;
			std::nth_element(
				Order.begin() + Begin,
				Order.begin() + Middle,
				Order.begin() + End,
				[&](const std::uint32_t A, const std::uint32_t B) {
					const double X = Axis(Channels[A].Center, Split), Y = Axis(Channels[B].Center, Split);
					// 中心の座標順と同値時の安定な番号順の返戻
					return X != Y ? X < Y : A < B;
				}
			);
			const std::uint32_t Left = Self(Self, Begin, Middle), Right = Self(Self, Middle, End);
			Index[Id].Left = Left;
			Index[Id].Right = Right;
			// 構築した索引節点の番号の返戻
			return Id;
		};
		Build(Build, 0, static_cast<std::uint32_t>(Order.size()));
		// 終了
		return;
	}

	/**
	 * 固定球面網で大洋を結ぶ海峡を準備する
	 * @param Base 同じ惑星定義の地質場
	 * @param Ridges 任意の造山場
	 * @param Meso 任意の排水前中規模起伏
	 * @param Stop 中断要求
	 * @param IncludeLandRelief 細部起伏を排水準備前へ含める場合に真
	 * @param Sampling 海域接続に使う網
	 * @return 接続済み海域場，失敗又は中断時は空
	 */
	std::optional<OceanConnectivityField> OceanConnectivityField::CreateImpl(
		std::shared_ptr<const GeologyField> Base,
		std::shared_ptr<const PlanetOrogenicRidges> Ridges,
		std::shared_ptr<const PlanetMesoRelief> Meso,
		const std::stop_token Stop,
		const bool IncludeLandRelief,
		const OceanSamplingMode Sampling
	) {
		// 地質が無い場合又は中断要求時の返戻
		if(!Base || Stop.stop_requested()) return {};
		OceanConnectivityField Field;
		Field.Geology = std::move(Base);
		Field.Orogenic = std::move(Ridges);
		Field.Meso = std::move(Meso);
		Field.PredrainageLandRelief = IncludeLandRelief;
		Field.HasFineOceanGraph = Field.Meso && Sampling == OceanSamplingMode::Fine;
		const GeologySettings &S = Field.Geology->Settings();
		// 表示の詳細度と独立した暫定の物理幅
		// 小惑星では遷移幅を広げ，起伏の狭い壁への圧縮を避ける
		const double Sea = S.SeaLevelMeters, ReliefScale = S.RadiusMeters / (S.RadiusMeters + 250000);
		constexpr double InnerAngle = 0.004;
		const double OuterAngle = std::max(0.014, 12000.0 / (S.RadiusMeters + 250000));
		Field.Stats.InnerHalfWidthMeters = InnerAngle * S.RadiusMeters;
		Field.Stats.OuterHalfWidthMeters = OuterAngle * S.RadiusMeters;
		Field.Stats.MinimumDepthMeters = 100 * ReliefScale;
		Field.TransitionMeters = 10 * ReliefScale;
		Field.FloorMeters = Sea - Field.Stats.MinimumDepthMeters - 0.5 * Field.TransitionMeters;
		const std::optional<OceanGraphMesh> Mesh = BuildOceanGraph(Field.HasFineOceanGraph, Stop);
		// 全球網を準備出来ない場合の返戻
		if(!Mesh) return {};
		// 固定網の準備経路と適応網の準備経路で，同じ連続海峡を使う
		if(!ConnectOnMesh(Field, Mesh->Directions, Mesh->Triangles, Stop)) return {};
		// 海峡と検査統計を保持した場の返戻
		return Field;
	}

	/**
	 * 指定した閉じた網で未接続の海域を結び，同じ連続海峡場を再評価する
	 * @param Field 元地質・既存の海峡と物理幅，成功時は追加海峡を保持する
	 * @param Directions 検証済の球面節点
	 * @param Triangles 検証済の外向き共有三角形
	 * @param Stop 中断要求
	 * @return 網上の補正後統計迄を計算出来た場合に真
	 */
	bool OceanConnectivityField::ConnectPreparedMesh(
		OceanConnectivityField &Field,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop
	) {
		const GeologySettings &S = Field.Geology->Settings();
		const double Sea = S.SeaLevelMeters, ReliefScale = S.RadiusMeters / (S.RadiusMeters + 250000);
		constexpr double InnerAngle = 0.004;
		const double OuterAngle = std::max(0.014, 12000.0 / (S.RadiusMeters + 250000));
		const std::uint32_t N = static_cast<std::uint32_t>(Directions.size());
		Field.Stats.GraphNodes = N;
		Field.Stats.ComponentsBefore = 0;
		Field.Stats.ComponentsAfter = 0;
		Field.Stats.BlockedWetEdgesBefore = Field.Stats.BlockedWetEdgesAfter = 0;
		Field.Stats.PassageEvaluations = 0;
		Field.Stats.GraphOceanAreaFraction = 0;
		Field.Stats.MaximumSampleLoweringMeters = 0;
		std::vector<double> Heights(N), Areas(N, 0);
		std::vector<TerrainSample> Samples(N);
		std::vector<std::vector<std::uint32_t>> Neighbors(N);
		DisjointSet Components(N);
		for(std::uint32_t I = 0; I < N; ++I) {
			// 全球評価中の中断要求に従う返戻
			if(!(I & 255) && Stop.stop_requested()) return false;
			Samples[I] = Field.Evaluate(Directions[I]);
			Heights[I] = Samples[I].ElevationMeters;
			// 不正な元場を連結判定から落として受理しない返戻
			if(!std::isfinite(Heights[I]) || !std::isfinite(Samples[I].TangentialGradientPerRadian.Length())) return false;
		}
		for(const std::array<std::uint32_t, 3> &T : Triangles) {
			const double Share = TriangleArea(Directions[T[0]].Vector(), Directions[T[1]].Vector(), Directions[T[2]].Vector()) / 3;
			for(unsigned K = 0; K < 3; ++K) {
				const std::uint32_t A = T[K], B = T[(K + 1) % 3];
				Areas[A] += Share;
				Neighbors[A].push_back(B);
				Neighbors[B].push_back(A);
			}
		}
		// 共有辺の途中の地形も，接続判定と開削費用へ同時に使う
		std::vector<std::vector<double>> Crests(N);
		for(std::uint32_t I = 0; I < N; ++I) {
			std::vector<std::uint32_t> &Adjacent = Neighbors[I];
			std::sort(Adjacent.begin(), Adjacent.end());
			Adjacent.erase(std::unique(Adjacent.begin(), Adjacent.end()), Adjacent.end());
			Crests[I].resize(Adjacent.size());
		}
		// 標本間の極大を含む通過高度の計測
		const auto MeasurePassage = [&](const std::uint32_t A, const std::uint32_t B) -> std::optional<double> {
			const Vec3 Start = Directions[A].Vector(), End = Directions[B].Vector(), Delta = End - Start;
			double Crest = std::max(Samples[A].ElevationMeters, Samples[B].ElevationMeters);
			bool HasFiniteSource = true;
			const auto Sample = [&](const double Fraction) -> std::array<double, 2> {
				const UnitDirection D = *UnitDirection::FromVector(Start * (1 - Fraction) + End * Fraction);
				const TerrainSample Value = Fraction == 1 ? Samples[B] : Field.Evaluate(D);
				if(Fraction != 1) ++Field.Stats.PassageEvaluations;
				HasFiniteSource =
				HasFiniteSource && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
				// 正の射影尺度を省いて符号だけを使う通路方向の微分の返戻
				return { Value.ElevationMeters, Value.TangentialGradientPerRadian.Dot(Delta) };
			};
			double PreviousDerivative = Samples[A].TangentialGradientPerRadian.Dot(Delta);
			for(unsigned Probe = 1; Probe < 9; ++Probe) {
				// 有界な通路探索中の中断への返戻
				if(Stop.stop_requested()) return {};
				const double Fraction = Probe * 0.125;
				const std::array<double, 2> Value = Sample(Fraction);
				Crest = std::max(Crest, Value[0]);
				if(PreviousDerivative > 0 && Value[1] < 0) {
					double Low = (Probe - 1) * 0.125, High = Fraction;
					for(unsigned Iteration = 0; Iteration < 24; ++Iteration) {
						const double Middle = std::midpoint(Low, High);
						const std::array<double, 2> Peak = Sample(Middle);
						Crest = std::max(Crest, Peak[0]);
						if(Peak[1] > 0) Low = Middle;
						else High = Middle;
					}
				}
				PreviousDerivative = Value[1];
			}
			// 欠落した場又は中断済の判定を公開しない返戻
			if(!HasFiniteSource || Stop.stop_requested()) return {};
			// 元場を変更しない通過高度の返戻
			return Crest;
		};
		// 濡れた端点間の通路の照合
		for(std::uint32_t I = 0; I < N; ++I) for(std::size_t Edge = 0; Edge < Neighbors[I].size(); ++Edge) {
			const std::uint32_t J = Neighbors[I][Edge];
			if(J < I) continue;
			const std::optional<double> Crest = MeasurePassage(I, J);
			// 通過高度を取得出来ない辺を水路としない返戻
			if(!Crest) return false;
			Crests[I][Edge] = *Crest;
			const std::size_t Reverse =
			static_cast<std::size_t>(std::lower_bound(Neighbors[J].begin(), Neighbors[J].end(), I) - Neighbors[J].begin());
			Crests[J][Reverse] = *Crest;
			if(Heights[I] < Sea && Heights[J] < Sea) {
				if(*Crest < Sea) Components.Join(I, J);
				else ++Field.Stats.BlockedWetEdgesBefore;
			}
		}
		// 通過可能な辺だけからの主大洋の選択
		std::vector<double> ComponentAreas(N, 0);
		for(std::uint32_t I = 0; I < N; ++I) if(Heights[I] < Sea) ComponentAreas[Components.Root(I)] += Areas[I];
		const std::uint32_t Main =
		static_cast<std::uint32_t>(std::max_element(ComponentAreas.begin(), ComponentAreas.end()) - ComponentAreas.begin());
		// 接続先の大洋が無い場合の返戻
		if(ComponentAreas[Main] <= 0.0) return false;
		for(const double A : ComponentAreas) if(A > 0) ++Field.Stats.ComponentsBefore;
		// 開削量を費用へ含める多始点最短経路で海域を接続する
		// 河川水位・侵食は別の処理で拘束する
		std::vector<double> Distance(N, std::numeric_limits<double>::infinity());
		std::vector<std::uint32_t> Parent(N, Missing);
		using Entry = std::pair<double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Queue;
		for(std::uint32_t I = 0; I < N; ++I) if(Heights[I] < Sea && Components.Root(I) == Main) {
			Distance[I] = 0;
			Parent[I] = I;
			Queue.emplace(0, I);
		}
		std::uint32_t Visits = 0;
		while(!Queue.empty()) {
			// 最短経路探索中の中断要求に従う返戻
			if(!(Visits++ & 255) && Stop.stop_requested()) return false;
			const double Cost = Queue.top().first;
			const std::uint32_t I = Queue.top().second;
			Queue.pop();
			if(Cost != Distance[I]) continue;
			for(std::size_t Edge = 0; Edge < Neighbors[I].size(); ++Edge) {
				const std::uint32_t J = Neighbors[I][Edge];
				const double Length = std::acos(std::clamp(Directions[I].Vector().Dot(Directions[J].Vector()), -1.0, 1.0));
				const double Cut = std::max(0.0, Crests[I][Edge] - Sea) / (500 * ReliefScale), Candidate = Cost + Length * (1 + Cut * Cut);
				if(Candidate < Distance[J]) {
					Distance[J] = Candidate;
					Parent[J] = I;
					Queue.emplace(Candidate, J);
				}
			}
		}
		std::vector<std::uint32_t> Representatives(N, Missing);
		for(std::uint32_t I = 0; I < N; ++I) if(Heights[I] < Sea) {
			const std::uint32_t Root = Components.Root(I);
			const std::uint32_t Previous = Representatives[Root];
			if(Previous == Missing || Distance[I] < Distance[Previous]) Representatives[Root] = I;
		}
		std::vector<Entry> Targets;
		for(std::uint32_t I = 0; I < N; ++I) {
			if(Representatives[I] != Missing && I != Main) Targets.emplace_back(Distance[Representatives[I]], Representatives[I]);
		}
		std::sort(Targets.begin(), Targets.end());
		std::vector<bool> Connected(N, false);
		Connected[Main] = true;
		std::vector<bool> Routed(N, false);
		for(const Entry &Target : Targets) {
			// 中断要求が有る場合の返戻
			if(Stop.stop_requested()) return false;
			std::uint32_t I = Target.second;
			std::vector<std::uint32_t> TouchedComponents;
			for(std::uint32_t Steps = 0; Steps < N; ++Steps) {
				const bool Wet = Heights[I] < Sea;
				const std::uint32_t Component = Components.Root(I);
				if(Routed[I] || Wet && Connected[Component]) break;
				if(Wet) TouchedComponents.push_back(Component);
				const std::uint32_t J = Parent[I];
				// 正しい接続経路を復元出来ない場合の返戻
				if(J == Missing || J == I || !(Distance[J] < Distance[I])) return false;
				const Vec3 A = Directions[I].Vector(), B = Directions[J].Vector();
				const double Dot = std::clamp(A.Dot(B), -1.0, 1.0);
				OceanCorridor C;
				C.Start = A;
				C.End = B;
				C.Center = Unit(A + B);
				C.Tangent = Unit(B - A * Dot);
				C.ArcAngle = std::acos(Dot);
				C.InnerChordSquared = 2 * (1 - std::cos(InnerAngle));
				C.OuterChordSquared = 2 * (1 - std::cos(OuterAngle));
				const double Bound = std::sqrt(2 * (1 - std::cos(0.5 * C.ArcAngle + OuterAngle)));
				C.BoundsMin = C.Center - Vec3{ Bound, Bound, Bound };
				C.BoundsMax = C.Center + Vec3{ Bound, Bound, Bound };
				Field.Channels.push_back(C);
				Field.Stats.CorridorLengthMeters += C.ArcAngle * S.RadiusMeters;
				Routed[I] = true;
				I = J;
			}
			for(const std::uint32_t C : TouchedComponents) Connected[C] = true;
		}
		Field.BuildIndex();
		Field.Stats.CorridorCount = static_cast<std::uint32_t>(Field.Channels.size());
		Field.Stats.IndexNodeCount = static_cast<std::uint32_t>(Field.Index.size());
		// 合成後の実地形を再評価する
		// 網上の接続と連続海峡内の水深は別々に検査する
		DisjointSet After(N);
		std::vector<double> Corrected(N);
		for(std::uint32_t I = 0; I < N; ++I) {
			// 全球評価中の中断要求に従う返戻
			if(!(I & 255) && Stop.stop_requested()) return false;
			Samples[I] = Field.Evaluate(Directions[I]);
			Corrected[I] = Samples[I].ElevationMeters;
			Field.Stats.MaximumSampleLoweringMeters = std::max(Field.Stats.MaximumSampleLoweringMeters, Heights[I] - Corrected[I]);
			if(Corrected[I] < Sea) Field.Stats.GraphOceanAreaFraction += Areas[I] / (4 * Pi);
		}
		for(std::uint32_t I = 0; I < N; ++I) {
			if(!(Corrected[I] < Sea)) continue;
			for(std::size_t Edge = 0; Edge < Neighbors[I].size(); ++Edge) {
				const std::uint32_t J = Neighbors[I][Edge];
				if(J < I || !(Corrected[J] < Sea)) continue;
				// 海峡補正は標高を上げない為，元から通れた辺の証拠を再利用する
				if(Crests[I][Edge] < Sea) {
					After.Join(I, J);
					continue;
				}
				const std::optional<double> Crest = MeasurePassage(I, J);
				// 補正後の通路を再評価出来ない場合の返戻
				if(!Crest) return false;
				if(*Crest < Sea) After.Join(I, J);
				else ++Field.Stats.BlockedWetEdgesAfter;
			}
		}
		std::vector<bool> Seen(N, false);
		for(std::uint32_t I = 0; I < N; ++I) if(Corrected[I] < Sea && !Seen[After.Root(I)]) {
			Seen[After.Root(I)] = true;
			++Field.Stats.ComponentsAfter;
		}
		// 接続を実測した網の代表方向を保持し，適応網でも其の証拠を引継ぐ
		Field.OceanAnchors.clear();
		if(Field.Stats.ComponentsAfter == 1) {
			for(std::uint32_t I = 0; I < N; ++I) if(Corrected[I] < Sea) Field.OceanAnchors.push_back(Directions[I]);
		}
		constexpr unsigned ProbeCount = 16384;
		unsigned Ocean = 0;
		for(unsigned I = 0; I < ProbeCount; ++I) {
			// 全球評価中の中断要求に従う返戻
			if(!(I & 255) && Stop.stop_requested()) return false;
			const double Z = 1 - 2 * (I + 0.5) / ProbeCount, P = I * Pi * (3 - std::sqrt(5.0)), R = std::sqrt(1 - Z * Z);
			if(Field.Evaluate(*UnitDirection::FromVector({ R * std::cos(P), R * std::sin(P), Z })).ElevationMeters < Sea) ++Ocean;
		}
		Field.Stats.IndependentOceanFractionEstimate = static_cast<double>(Ocean) / ProbeCount;
		// 同じ通路検査でも全海域が接続した場合だけ成功する返戻
		return Field.Stats.ComponentsAfter == 1;
	}

	/**
	 * 地質場から接続した大洋と海底海峡を準備する
	 * @param Base 惑星の地質場
	 * @param Stop 中断要求
	 * @return 接続済み海域場，入力不正又は中断時は空
	 */
	std::optional<OceanConnectivityField> OceanConnectivityField::Create(
		std::shared_ptr<const GeologyField> Base,
		const std::stop_token Stop
	) {
		// 元地質を使う接続済み海域の返戻
		return CreateImpl(std::move(Base), {}, {}, Stop);
	}

	/**
	 * 地質起伏に造山稜線を加えた海陸を，排水準備より前に構築する
	 * @param Base 同じ惑星定義の地質場
	 * @param Ridges 地質履歴から準備した不変の造山場
	 * @param Stop 中断要求
	 * @return 接続済み海域場，入力不正又は中断時は空
	 */
	std::optional<OceanConnectivityField> OceanConnectivityField::CreateWithOrogeny(
		std::shared_ptr<const GeologyField> Base,
		std::shared_ptr<const PlanetOrogenicRidges> Ridges,
		const std::stop_token Stop
	) {
		// 惑星定義が一致しない入力の返戻
		if(!Base || !Ridges || Ridges->DefinitionHash() != Base->Settings().IdentityHash()) return {};
		// 造山を含む接続済み海域の返戻
		return CreateImpl(std::move(Base), std::move(Ridges), {}, Stop);
	}

	/**
	 * 地質方向の山稜と中規模起伏を，排水・侵食より前へ共有して準備する
	 * @param Base 同じ惑星定義の地質場
	 * @param Ridges 地質履歴から準備した不変の造山場
	 * @param Stop 中断要求
	 * @param Mode 細部起伏も排水前へ含めるか，既定は従来通り
	 * @param Sampling 河口と補間面を同じ網へ揃える場合の明示指定
	 * @return 起伏を前置した接続済み海域場，入力不正又は中断時は空
	 */
	std::optional<OceanConnectivityField> OceanConnectivityField::CreateWithPredrainageRelief(
		std::shared_ptr<const GeologyField> Base,
		std::shared_ptr<const PlanetOrogenicRidges> Ridges,
		const std::stop_token Stop,
		const PredrainageReliefMode Mode,
		const OceanSamplingMode Sampling
	) {
		// 未定義の接続網を採用しない返戻
		if(Sampling != OceanSamplingMode::Fine && Sampling != OceanSamplingMode::SharedDrainage) return {};
		// 未定義の起伏順序を採用しない返戻
		if(Mode != PredrainageReliefMode::MesoOnly && Mode != PredrainageReliefMode::MesoAndLand) return {};
		// 定義不一致又は中断要求が有る場合の返戻
		if(!Base || !Ridges || Ridges->DefinitionHash() != Base->Settings().IdentityHash() || Stop.stop_requested()) return {};
		std::optional<PlanetMesoRelief> Prepared =
		PlanetMesoRelief::CreateForDrainage(Base->Settings().Seed, Base->Settings().RadiusMeters);
		// 中規模起伏の準備に失敗した場合の返戻
		if(!Prepared) return {};
		const std::shared_ptr<const PlanetMesoRelief> Meso = std::make_shared<const PlanetMesoRelief>(std::move(*Prepared));
		// 指定した起伏順序で準備した海域の返戻
		return CreateImpl(std::move(Base), std::move(Ridges), Meso, Stop, Mode == PredrainageReliefMode::MesoAndLand, Sampling);
	}

	/**
	 * 最新の接続検査で同じ大洋に属した水没節点を取得する
	 * @return 当該不変場と同じ寿命の検査済方向列
	 */
	std::span<const UnitDirection> OceanConnectivityField::ConnectedOceanAnchors() const {
		// 標高だけから推測した任意地点とは区別した接続証拠の返戻
		return OceanAnchors;
	}
}
