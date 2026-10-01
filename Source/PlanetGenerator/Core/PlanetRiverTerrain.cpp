#include "PlanetRiverTerrain.h"
#include "PlanetBasinFill.h"
#include "PlanetErosionPreparation.h"
#include "PlanetJet.h"
#include "PlanetLandRelief.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		// ± K で値・１階・２階微分を合わせる４次式を用い，有限幅の接続の為に K 自身の勾配も含める
		/**
		 * ２値の最大値を有限の遷移幅で滑らかに結合する
		 * @param A 第１の値と勾配
		 * @param B 第２の値と勾配
		 * @param K 遷移幅と其の勾配，値が正でない場合は厳密な最大値
		 * @return 結合後の値と勾配
		 */
		template<bool Derivatives> TerrainJet<Derivatives> SmoothMax(
			const TerrainJet<Derivatives> A,
			const TerrainJet<Derivatives> B,
			const TerrainJet<Derivatives> K
		) {
			using Jet = TerrainJet<Derivatives>;
			const Jet D = A - B;
			// 遷移域の外で厳密な最大値を選んだ結果の返戻
			if(K.V <= 0.0 || std::abs(D.V) >= K.V) return A.V >= B.V ? A : B;
			const double T = D.V / K.V, T2 = T * T, P = 0.375 + 0.75 * T2 - 0.125 * T2 * T2, Q = 1.5 * T - 0.5 * T * T2;
			// 同じ地点に対する値と指定した微分の返戻
			if constexpr(Derivatives) return { (A.V + B.V + K.V * P) * 0.5, (A.G + B.G + D.G * Q + K.G * (P - T * Q)) * 0.5 };
			// 同じ地点に対する値と指定した微分の返戻
			else return { (A.V + B.V + K.V * P) * 0.5, {} };
		}

		/**
		 * 範囲外を固定し，内側は端で２階微分まで零へ接続する
		 * @param Input 入力値と勾配
		 * @param Low 遷移の下限
		 * @param High 遷移の上限
		 * @return ０～１の重みと勾配
		 */
		template<bool Derivatives> TerrainJet<Derivatives> ReliefGate(
			const TerrainJet<Derivatives> Input,
			const double Low,
			const double High
		) {
			// 下側の支持領域外の返戻
			if(Input.V <= Low) return {};
			// 上側の支持領域外の返戻
			if(Input.V >= High) return { 1, {} };
			const double T = (Input.V - Low) / (High - Low), Value = T * T * T * (10 + T * (-15 + 6 * T));
			// 値と解析勾配の返戻
			if constexpr(Derivatives) return { Value, Input.G * (30 * T * T * (1 - T) * (1 - T) / (High - Low)) };
			// 値だけの返戻
			else return { Value, {} };
		}

		/**
		 * 方向が閉じた軸平行境界内に有るかを判定する
		 * @param D 評価方向
		 * @param Min 各軸の下限
		 * @param Max 各軸の上限
		 * @return 全ての閉区間内に有る場合に真
		 */
		bool Contains(const Vec3 D, const Vec3 Min, const Vec3 Max) {
			// 三軸の閉区間との包含判定の返戻
			return D.X >= Min.X && D.Y >= Min.Y && D.Z >= Min.Z && D.X <= Max.X && D.Y <= Max.Y && D.Z <= Max.Z;
		}

		/**
		 * 指定した軸の座標を取得する
		 * @param V 対象ベクトル
		 * @param A 軸番号，０：X，１：Y，其れ以外：Z
		 * @return 指定軸の座標
		 */
		double Axis(Vec3 V, unsigned A) {
			// 指定された軸の成分の返戻
			return !A ? V.X : A == 1 ? V.Y : V.Z;
		}

		/**
		 * ２つの境界から成分毎の下限を求める
		 * @param A 第１の境界
		 * @param B 第２の境界
		 * @return 各軸の小さい方の値
		 */
		Vec3 Low(Vec3 A, Vec3 B) {
			// 各軸の小さい境界の返戻
			return { std::min(A.X, B.X), std::min(A.Y, B.Y), std::min(A.Z, B.Z) };
		}

		/**
		 * ２つの境界から成分毎の上限を求める
		 * @param A 第１の境界
		 * @param B 第２の境界
		 * @return 各軸の大きい方の値
		 */
		Vec3 High(Vec3 A, Vec3 B) {
			// 各軸の大きい境界の返戻
			return { std::max(A.X, B.X), std::max(A.Y, B.Y), std::max(A.Z, B.Z) };
		}

		/**
		 * 河床・谷・起伏抑制で同じ曲線座標と有限区間を使う
		 * @param C 共有する河道形状
		 * @param D 評価方向
		 * @param AlongAngle 公称弧角の出力先
		 * @param AlongGradient 公称弧角の接線勾配の出力先
		 * @return 有限河道への二乗距離と解析勾配
		 */
		template<bool Derivatives> TerrainJet<Derivatives> ChannelDistance(
			const RiverTerrainChannel &C,
			const Vec3 D,
			double &AlongAngle,
			Vec3 &AlongGradient
		) {
			const double X = D.Dot(C.Start), Y = D.Dot(C.Tangent), Den = X * X + Y * Y;
			AlongAngle = std::atan2(Y, X);
			Vec3 ParameterGradient;
			if constexpr(Derivatives) {
				ParameterGradient = (C.Tangent * X - C.Start * Y) * (1 / Den);
				AlongGradient = ParameterGradient;
			}
			if(C.Planform) {
				const RiverPlanformSample Curve = C.Planform->EvaluateAngle(AlongAngle);
				const Vec3 Difference = D - Curve.Direction.Vector();
				const double Outside = AlongAngle < 0 ? AlongAngle : AlongAngle > C.ArcAngle ? AlongAngle - C.ArcAngle : 0;
				const double Squared = Difference.Dot(Difference) + Outside * Outside;
				Vec3 Gradient;
				if constexpr(Derivatives) {
					const Vec3 CurveDerivative = Curve.DerivativePerFraction * (1 / C.Planform->ArcRadians());
					Gradient = Difference * 2 + ParameterGradient * (2 * (Outside - Difference.Dot(CurveDerivative)));
					Gradient = Gradient - D * D.Dot(Gradient);
				}
				// 端で一階微分が接続する区間罰則を含む曲線座標の返戻
				return { Squared, Gradient };
			}
			const Vec3 Closest = AlongAngle > 0 && AlongAngle < C.ArcAngle ?
			(C.Start * X + C.Tangent * Y) * (1 / std::sqrt(Den)) :
			AlongAngle > 0 ? C.End : C.Start;
			const Vec3 Difference = D - Closest;
			Vec3 Gradient;
			if constexpr(Derivatives) Gradient = (Difference - D * Difference.Dot(D)) * 2;
			// 直線の比較経路を変えない二乗弦距離の返戻
			return { Difference.Dot(Difference), Gradient };
		}

		/**
		 * 有限幅の谷の影響と河道に沿う位置を評価する
		 * @param C 河道の形状
		 * @param D 評価方向
		 * @param AlongAngle 上流端からの角距離の出力先
		 * @param AlongGradient 角距離の接線勾配の出力先
		 * @param CoreDistance 内側の幅で正規化した二乗弦距離の出力先
		 * @return 谷の影響率と其の勾配
		 */
		template<bool Derivatives> TerrainJet<Derivatives> ChannelMask(
			const RiverTerrainChannel &C,
			const Vec3 D,
			double &AlongAngle,
			Vec3 &AlongGradient,
			TerrainJet<Derivatives> &CoreDistance
		) {
			const TerrainJet<Derivatives> Distance = ChannelDistance<Derivatives>(C, D, AlongAngle, AlongGradient);
			const double Q = Distance.V;
			Vec3 QGradient;
			if constexpr(Derivatives) QGradient = Distance.G;
			if constexpr(Derivatives) CoreDistance = { Q / C.InnerChordSquared, QGradient * (1 / C.InnerChordSquared) };
			else CoreDistance = { Q / C.InnerChordSquared, {} };
			// 有効な候補又は支持領域が無い場合の返戻
			if(Q >= C.OuterChordSquared) return {};
			// 河道の内側では形状を完全に適用する返戻
			if(Q <= C.InnerChordSquared) return { 1, {} };
			const double Width = C.OuterChordSquared - C.InnerChordSquared, T = (Q - C.InnerChordSquared) / Width;
			const double Smooth = T * T * T * (10 + T * (-15 + 6 * T)), Derivative = 30 * T * T * (1 - T) * (1 - T);
			// 同じ地点に対する値と指定した微分の返戻
			if constexpr(Derivatives) return { 1 - Smooth, QGradient * (-Derivative / Width) };
			// 同じ地点に対する値と指定した微分の返戻
			else return { 1 - Smooth, {} };
		}

		/**
		 * 河道端を含む有限弧までの距離から，中距離起伏の C2 緩衝率を返す
		 * @param C 河道の形状
		 * @param D 評価方向
		 * @param Radius 惑星半径，単位は m
		 * @return 河道を避ける緩衝率と接線勾配
		 */
		template<bool Derivatives> TerrainJet<Derivatives> MesoCorridorGate(
			const RiverTerrainChannel &C,
			const Vec3 D,
			const double Radius
		) {
			using Jet = TerrainJet<Derivatives>;
			double AlongAngle = 0;
			Vec3 AlongGradient;
			const Jet Geometry = ChannelDistance<Derivatives>(C, D, AlongAngle, AlongGradient);
			const double Chord = std::sqrt(Geometry.V), Distance = Radius * Chord;
			// 内側緩衝域の零寄与の返戻
			if(Distance <= C.MesoInnerMeters) return {};
			// 外側緩衝域の完全寄与の返戻
			if(Distance >= C.MesoOuterMeters) return { 1, {} };
			Vec3 Gradient;
			if constexpr(Derivatives) Gradient = Geometry.G * (Radius / (2 * Chord));
			// 支持域内を連続に接続した緩衝率の返戻
			return ReliefGate(Jet{ Distance, Gradient }, C.MesoInnerMeters, C.MesoOuterMeters);
		}
	} // namespace

	/**
	 * 有限幅の全河道を地点評価の空間索引へ登録する
	 */
	void RiverTerrainField::BuildIndex() {
		// 現在の準備条件を満たさない場合の終了
		if(Valleys.empty()) return;
		Order.resize(Valleys.size());
		std::iota(Order.begin(), Order.end(), 0U);
		Index.reserve(Valleys.size() * 2);
		const auto Build = [&](auto &&Self, std::uint32_t Begin, std::uint32_t End) -> std::uint32_t {
			IndexNode Node;
			Node.Min = Valleys[Order[Begin]].MesoBoundsMin;
			Node.Max = Valleys[Order[Begin]].MesoBoundsMax;
			for(std::uint32_t i = Begin + 1; i < End; ++i) {
				Node.Min = Low(Node.Min, Valleys[Order[i]].MesoBoundsMin);
				Node.Max = High(Node.Max, Valleys[Order[i]].MesoBoundsMax);
			}
			const std::uint32_t NodeIndex = static_cast<std::uint32_t>(Index.size());
			Index.push_back(Node);
			if(End - Begin < 5) {
				Index[NodeIndex].Begin = Begin;
				Index[NodeIndex].Count = End - Begin;
				// 構築した索引節点の返戻
				return NodeIndex;
			}
			const Vec3 Extent = Node.Max - Node.Min;
			const unsigned SplitAxis = Extent.X >= Extent.Y && Extent.X >= Extent.Z ? 0 : Extent.Y >= Extent.Z ? 1 : 2;
			const std::uint32_t Middle = Begin + (End - Begin) / 2;
			std::nth_element(
				Order.begin() + Begin,
				Order.begin() + Middle,
				Order.begin() + End,
				[&](const std::uint32_t LeftIndex, const std::uint32_t RightIndex) {
					const double X = Axis(Valleys[LeftIndex].Center, SplitAxis), Y = Axis(Valleys[RightIndex].Center, SplitAxis);
					// 同じ中心では特徴番号順に固定した比較結果の返戻
					return X != Y ? X < Y : LeftIndex < RightIndex;
				}
			);
			const std::uint32_t LeftChild = Self(Self, Begin, Middle), RightChild = Self(Self, Middle, End);
			Index[NodeIndex].Left = LeftChild;
			Index[NodeIndex].Right = RightChild;
			// 構築した索引節点の返戻
			return NodeIndex;
		};
		Build(Build, 0, static_cast<std::uint32_t>(Order.size()));
		// 終了
		return;
	}

	/**
	 * 海陸と準備済みの排水から，比較基準の河道合成場を構築する
	 * @param Source 同じ生成定義の海陸
	 * @param Drainage 河道と河床の縦断
	 * @param Stop 中断要求
	 * @return 不変な地点評価場，準備失敗又は中断時は空
	 */
	std::optional<RiverTerrainField> RiverTerrainField::Create(
		std::shared_ptr<const OceanConnectivityField> Source,
		std::shared_ptr<const DrainageNetwork> Drainage,
		const std::stop_token Stop
	) {
		// 必須の準備済み基盤が無い場合の返戻
		if(!Source || !Drainage) return {};
		// 盛土未統合の縦断を開削専用の形状として公開しない返戻
		if(Drainage->RequiresBalancedTerrain()) return {};
		// 中断した場を公開しない返戻
		if(Stop.stop_requested()) return {};
		// 異なる惑星定義の排水網を混ぜない返戻
		if(Drainage->SourceDefinitionHash() != Source->DefinitionHash()) return {};
		RiverTerrainField Result;
		Result.Ocean = std::move(Source);
		Result.Network = std::move(Drainage);
		const double Radius = Result.Base().Settings().RadiusMeters;
		const std::optional<PlanetMesoRelief> Meso = PlanetMesoRelief::Create(Result.Base().Settings().Seed, Radius);
		// 中規模起伏の準備失敗の返戻
		if(!Meso) return {};
		Result.Meso = *Meso;
		Result.ReliefBoundMeters = Result.Base().AbsoluteReliefBoundMeters();
		Result.Valleys.reserve(Result.Network->Reaches().size());
		for(std::size_t i = 0; i < Result.Network->Reaches().size(); ++i) {
			// 有効な候補又は支持領域が無い場合の返戻
			if(Stop.stop_requested()) return {};
			const RiverReach &Reach = Result.Network->Reaches()[i];
			const std::optional<RiverProfileSample> First = Result.Network->SampleReach(i, 0), Last = Result.Network->SampleReach(i, 1);
			// 有効な候補又は支持領域が無い場合の返戻
			if(!First || !Last) return {};
			RiverTerrainChannel Channel;
			Channel.ReachIndex = static_cast<std::uint32_t>(i);
			Channel.Start = First->Direction;
			Channel.End = Last->Direction;
			Channel.Tangent = First->DownstreamTangent;
			Channel.ArcAngle = Reach.ProfileLengthMeters() / Radius;
			Channel.Planform = Reach.Planform;
			Channel.Center = Channel.Planform ?
			UnitDirection::FromVector(Channel.Start * std::cos(Channel.ArcAngle * 0.5) + Channel.Tangent * std::sin(Channel.ArcAngle * 0.5))
			->Vector() :
			UnitDirection::FromVector(Channel.Start + Channel.End)->Vector();
			Channel.StartBedMeters = First->BedMeters;
			Channel.EndBedMeters = Last->BedMeters;
			const std::optional<RiverCrossSectionWidths> Widths = Result.Network->ChannelWidths(i);
			// 水位の準備と共有する横断寸法が不正な場合の返戻
			if(!Widths) return {};
			Channel.BedHalfWidthMeters = Widths->BedHalfWidthMeters;
			Channel.InnerHalfWidthMeters = Widths->InnerHalfWidthMeters;
			Channel.BedToBankChordRatio =
			std::sin(Channel.BedHalfWidthMeters / (2 * Radius)) / std::sin(Channel.InnerHalfWidthMeters / (2 * Radius));
			double MaximumCut = 0;
			if(!Result.Network->Profiles().empty()) MaximumCut = Result.Network->Profiles()[i].MaximumSampledBedCutMeters;
			else for(unsigned k = 0; k < 17; ++k) {
				const std::optional<RiverProfileSample> Sample = Result.Network->SampleReach(i, k / 16.0);
				const double Height = Result.Ocean->Evaluate(*UnitDirection::FromVector(Sample->Direction)).ElevationMeters;
				MaximumCut = std::max(MaximumCut, Height - Sample->BedMeters);
			}
			// ５次の遷移式に広い幅を与えて人工的な急壁を抑えるが，有限個の検査地点を連続的な勾配上限の証明とは扱わない
			Channel.OuterHalfWidthMeters = Channel.InnerHalfWidthMeters + std::max(4 * Channel.InnerHalfWidthMeters, 12.5 * MaximumCut);
			const double OuterAngle = Channel.OuterHalfWidthMeters / Radius;
			// 有効な候補又は支持領域が無い場合の返戻
			if(!std::isfinite(OuterAngle) || OuterAngle >= 0.25 || Channel.ArcAngle <= 0.0) return {};
			Channel.InnerChordSquared = 4 * std::pow(std::sin(Channel.InnerHalfWidthMeters / (2 * Radius)), 2);
			Channel.OuterChordSquared = 4 * std::pow(std::sin(OuterAngle / 2), 2);
			const double Deviation = Channel.Planform ? Channel.Planform->MaximumDeviationRadians() : 0;
			const double Bound = std::nextafter(
				2 * std::sin((Channel.ArcAngle * 0.5 + OuterAngle + Deviation) * 0.5) + 1E-14,
				std::numeric_limits<double>::infinity()
			);
			Channel.BoundsMin = Channel.Center - Vec3{ Bound, Bound, Bound };
			Channel.BoundsMax = Channel.Center + Vec3{ Bound, Bound, Bound };
			Channel.MesoInnerMeters = Channel.OuterHalfWidthMeters + std::min(500.0, Radius * 0.01);
			Channel.MesoOuterMeters = Channel.OuterHalfWidthMeters + std::min(5000.0, Radius * 0.05);
			const double MesoAngle = Channel.MesoOuterMeters / Radius;
			const double MesoBound = std::nextafter(
				2 * std::sin((Channel.ArcAngle * 0.5 + MesoAngle + Deviation) * 0.5) + 1E-14,
				std::numeric_limits<double>::infinity()
			);
			Channel.MesoBoundsMin = Channel.Center - Vec3{ MesoBound, MesoBound, MesoBound };
			Channel.MesoBoundsMax = Channel.Center + Vec3{ MesoBound, MesoBound, MesoBound };
			const std::optional<RiverHeights> Before = Result.Network->SampleHeights(i, -OuterAngle / Channel.ArcAngle);
			const std::optional<RiverHeights> After = Result.Network->SampleHeights(i, 1 + OuterAngle / Channel.ArcAngle);
			const double MinimumBed = std::min(Before->BedMeters, After->BedMeters);
			Result.ReliefBoundMeters = std::max(Result.ReliefBoundMeters, std::abs(MinimumBed));
			const double ValleyHalfWidth = Channel.OuterHalfWidthMeters;
			Result.Stats.MaximumValleyHalfWidthMeters = std::max(Result.Stats.MaximumValleyHalfWidthMeters, ValleyHalfWidth);
			Result.Stats.MaximumPreparedCutMeters = std::max(Result.Stats.MaximumPreparedCutMeters, MaximumCut);
			Result.Valleys.push_back(Channel);
		}
		Result.ReliefBoundMeters += 1 + LandReliefAmplitudeMeters + 444 * Radius / (Radius + 250000) + Result.Valleys.size() * 0.05;
		Result.Stats.Channels = static_cast<std::uint32_t>(Result.Valleys.size());
		Result.BuildIndex();
		Result.Stats.IndexNodes = static_cast<std::uint32_t>(Result.Index.size());
		// 全河道を準備した不変な合成場の返戻
		return Result;
	}

	/**
	 * 気候・流量・侵食と，侵食後の河床を同じ不変な場へ結合する
	 * @param Ocean 元の海陸と海底
	 * @param Drainage 同じ海陸から準備した侵食前の排水
	 * @param Stop 中断要求
	 * @return 侵食と河床が整合した地点評価場，失敗又は中断時は空
	 */
	std::optional<RiverTerrainField> RiverTerrainField::CreateWithErosion(
		const std::shared_ptr<const OceanConnectivityField> &Ocean,
		const std::shared_ptr<const DrainageNetwork> &Drainage,
		const std::stop_token Stop
	) {
		// 盛土未統合の縦断を侵食経由で描画へ迂回させない返戻
		if(Drainage && Drainage->RequiresBalancedTerrain()) return {};
		std::optional<PlanetErosionPreparation> Prepared = PlanetErosionPreparation::Create(Ocean, Drainage, Stop);
		// 侵食基盤の準備失敗の返戻
		if(!Prepared) return {};
		std::optional<RiverTerrainField> Surface = Create(Ocean, Prepared->GetDrainage(), Stop);
		// 河道の合成失敗又は公開前の中断の返戻
		if(!Surface || Stop.stop_requested()) return {};
		Surface->Erosion = std::make_shared<const PlanetErosionPreparation>(std::move(*Prepared));
		Surface->ReliefBoundMeters += PlanetAeolianField::MaximumAddedHeightMeters;
		// 表示と衝突に共通の最終評価場の返戻
		return Surface;
	}

	/**
	 * 同じ河道索引を広い領域の分割条件にも使い，細い流路の事前発見を可能にする
	 * @param Min 照会範囲の下限
	 * @param Max 照会範囲の上限
	 * @param Candidates 交差する河道番号の出力先
	 * @return 照会条件が有効な場合に真
	 */
	bool RiverTerrainField::GatherChannelCandidates(const Vec3 Min, const Vec3 Max, std::vector<std::uint32_t> &Candidates) const {
		Candidates.clear();
		// 不正な範囲を候補無と誤認しない返戻
		if(!std::isfinite(Min.Length()) || !std::isfinite(Max.Length()) || Min.X > Max.X || Min.Y > Max.Y || Min.Z > Max.Z) {
			// 不正な照会範囲の返戻
			return false;
		}
		const std::function<bool(Vec3, Vec3)> Overlaps = [&](const Vec3 Low, const Vec3 High) {
			// 接する境界も含めた包囲箱の交差判定の返戻
			return Min.X <= High.X && Max.X >= Low.X && Min.Y <= High.Y && Max.Y >= Low.Y && Min.Z <= High.Z && Max.Z >= Low.Z;
		};
		// 索引は３２ビット個数を中央値で二分する為，此の深さで全ての保留枝を保持出来る
		std::array<std::uint32_t, 64> Stack {};
		unsigned Count = Index.empty() ? 0 : 1;
		while(Count) {
			const IndexNode &Node = Index[Stack[--Count]];
			if(!Overlaps(Node.Min, Node.Max)) continue;
			if(!Node.Count) {
				Stack[Count++] = Node.Right;
				Stack[Count++] = Node.Left;
				continue;
			}
			for(std::uint32_t k = Node.Begin; k < Node.Begin + Node.Count; ++k) {
				const std::uint32_t Candidate = Order[k];
				if(Overlaps(Valleys[Candidate].BoundsMin, Valleys[Candidate].BoundsMax)) Candidates.push_back(Candidate);
			}
		}
		// 保守的な候補集合を収集した結果の返戻
		return true;
	}

	/**
	 * 同じ基盤へ河道と氾濫原を合成し，地点の地面を評価する
	 * @param Direction 評価方向
	 * @param Indexed 空間索引を使う場合に真，比較時は全河道を調べる
	 * @param CandidateCount 任意の候補数の出力先
	 * @param Surface 任意の河川被覆と水位の出力先
	 * @param Stages 任意の段階別標高の出力先
	 * @return 合成後の地形，勾配は指定時だけ算出
	 */
	template<bool Derivatives> TerrainSample RiverTerrainField::EvaluateImpl(
		const UnitDirection Direction,
		const bool Indexed,
		std::size_t *CandidateCount,
		RiverSurfaceSample *const Surface,
		RiverElevationStages *const Stages
	) const {
		using Jet = TerrainJet<Derivatives>;
		TerrainSample Original = [&] {
			if(Erosion) {
				// 表示と衝突のどちらも，準備済みの砂丘後の地形を基盤にする
				const PlanetAeolianField &Terrain = *Erosion->GetAeolianField();
				// 選択した不変な基盤の評価結果の返戻
				if constexpr(Derivatives) return Terrain.Evaluate(Direction);
				// 同じ地点に対する値と指定した微分の返戻
				else return TerrainSample{ Terrain.EvaluateElevation(Direction), {} };
			}
			// 選択した不変な基盤の評価結果の返戻
			if constexpr(Derivatives) return Ocean->Evaluate(Direction);
			// 同じ地点に対する値と指定した微分の返戻
			else return TerrainSample{ Ocean->EvaluateElevation(Direction), {} };
		}();
		if(Stages) Stages->BeforeBasinMeters = Original.ElevationMeters;
		if(Basins) {
			if constexpr(Derivatives) Original = Basins->Apply(Direction, Original);
			else Original.ElevationMeters = Basins->ApplyElevation(Direction, Original.ElevationMeters);
		}
		if(Stages) Stages->AfterBasinMeters = Original.ElevationMeters;
		const Vec3 Point = Direction.Vector();
		// 通常の照会はスタック上で完結し，河道が密集する合流点でも寄与を切捨てず全河道数以下の配列へ退避する
		std::array<std::uint32_t, 128> Small {};
		std::size_t Size = 0;
		std::vector<std::uint32_t> Large;
		const auto Add = [&](std::uint32_t i) {
			if(Size < Small.size()) Small[Size] = i;
			else {
				if(Large.empty()) Large.assign(Small.begin(), Small.end());
				Large.push_back(i);
			}
			++Size;
			// 終了
			return;
		};
		if(Indexed && !Index.empty()) {
			std::array<std::uint32_t, 64> Stack {};
			unsigned Count = 1;
			while(Count) {
				const IndexNode &Node = Index[Stack[--Count]];
				if(!Contains(Point, Node.Min, Node.Max)) continue;
				if(!Node.Count) {
					Stack[Count++] = Node.Right;
					Stack[Count++] = Node.Left;
					continue;
				}
				for(std::uint32_t k = Node.Begin; k < Node.Begin + Node.Count; ++k) {
					const std::uint32_t i = Order[k];
					if(Contains(Point, Valleys[i].MesoBoundsMin, Valleys[i].MesoBoundsMax)) Add(i);
				}
			}
		} else if(!Indexed) for(std::uint32_t i = 0; i < Valleys.size(); ++i) Add(i);
		std::uint32_t *const CandidateIndices = Large.empty() ? Small.data() : Large.data();
		std::sort(CandidateIndices, CandidateIndices + Size);
		if(CandidateCount) *CandidateCount = Size;
		const Jet H { Original.ElevationMeters, Original.TangentialGradientPerRadian };
		Jet Numerator, WeightSum, CoreCut, CoreSupport;
		Jet MesoClearance { 1, {} };
		double WaterWeight = 0, WeightedWater = 0, Coverage = 0, WaterSupport = 0;
		double MinimumWaterDistance = std::numeric_limits<double>::infinity();
		for(std::size_t k = 0; k < Size; ++k) {
			const RiverTerrainChannel &ChannelDefinition = Valleys[CandidateIndices[k]];
			// 支持領域の解析的な判定を両経路で共有し，比較経路は索引と其の境界を使わず全河道を独立に調べる
			double Angle = 0;
			Vec3 AlongGradient;
			// 対向半球は支持領域に入らない為，角距離の特異点を評価する前に除外する
			if(Point.Dot(ChannelDefinition.Center) <= 0.0) continue;
			if(MesoClearance.V > 0 && !Ocean->HasPredrainageMesoRelief()) {
				MesoClearance = MesoClearance * MesoCorridorGate<Derivatives>(ChannelDefinition, Point, Base().Settings().RadiusMeters);
			}
			Jet CoreDistance;
			const Jet Mask = ChannelMask(ChannelDefinition, Point, Angle, AlongGradient, CoreDistance);
			if(Mask.V <= 0.0) continue;
			const std::optional<RiverHeights> Profile =
			Network->SampleHeights(ChannelDefinition.ReachIndex, Angle / ChannelDefinition.ArcAngle);
			Vec3 BedGradient, FloodplainGradient;
			if constexpr(Derivatives) {
				const double Slope = Profile->BedSlope * Base().Settings().RadiusMeters;
				BedGradient = AlongGradient * Slope;
				FloodplainGradient = AlongGradient * (Profile->WaterSlope * Base().Settings().RadiusMeters);
			}
			const Jet Bed { Profile->BedMeters, BedGradient };
			// 広い谷底は水面より高い氾濫原とし，其の内側の有限幅だけを掘る事で幅広い谷全体の水没を避ける
			const Jet Floodplain { Profile->WaterMeters + Network->Settings().BankFreeboardMeters, FloodplainGradient };
			const Jet ValleyCut = SmoothMax(H - Floodplain, {}, { 0.02, {} }) * Mask;
			Jet Channel;
			if(CoreDistance.V <= ChannelDefinition.BedToBankChordRatio * ChannelDefinition.BedToBankChordRatio) Channel = { 1, {} };
			else if(CoreDistance.V < 1) {
				// 正の幅を持つ平坦な河床の外だけで平方根を評価し，中心の特異微分を避けて値・勾配・曲率を氾濫原へ接続する
				const double Distance = std::sqrt(CoreDistance.V), Width = 1 - ChannelDefinition.BedToBankChordRatio;
				const double T = (Distance - ChannelDefinition.BedToBankChordRatio) / Width;
				Vec3 Gradient;
				if constexpr(Derivatives) Gradient = CoreDistance.G * (-30 * T * T * (1 - T) * (1 - T) / (2 * Distance * Width));
				Channel = { 1 - T * T * T * (10 + T * (-15 + 6 * T)), Gradient };
			}
			if(Surface) {
				const double BedCoverage = std::clamp(Channel.V, 0.0, 1.0), Wet = std::clamp(Mask.V, 0.0, 1.0);
				Coverage += (1 - Coverage) * BedCoverage;
				// 狭い川底の支持端で水位を切らず，同じ谷の範囲で隣接河道と共有する
				WaterSupport += (1 - WaterSupport) * Wet;
				if(Wet > 0) {
					// 河道中心の拘束を補間する為，二乗距離の逆二乗で重み付けする
					// 最短距離を単位に再尺度化し，中心近傍でも無限大や固定の微小幅を持込まない
					const double Distance = CoreDistance.V;
					if(Distance < MinimumWaterDistance) {
						const double Ratio = Distance / MinimumWaterDistance;
						WaterWeight *= Ratio * Ratio;
						WeightedWater *= Ratio * Ratio;
						MinimumWaterDistance = Distance;
					}
					const double Ratio = !Distance ? 1 : MinimumWaterDistance / Distance, Weight = Wet * Ratio * Ratio;
					WaterWeight += Weight;
					WeightedWater += Weight * Profile->WaterMeters;
				}
			}
			const Jet ChannelCut = SmoothMax(H - ValleyCut - Bed, {}, { 0.02, {} }) * Channel;
			const Jet LocalCut = ValleyCut + ChannelCut;
			// 遠くの深い谷が全河床を支配する事を避ける為，正則化した逆距離の重みで自身の河道を滑らかに優先する
			const double Inv = 1 / (1 + CoreDistance.V);
			Vec3 WeightGradient;
			if constexpr(Derivatives) WeightGradient = CoreDistance.G * (-2 * Inv * Inv * Inv);
			const Jet Weight = Mask * Jet{ Inv * Inv, WeightGradient };
			Numerator = Numerator + LocalCut * Weight;
			WeightSum = WeightSum + Weight;
			if(CoreDistance.V < 1) {
				const double T = CoreDistance.V, Smooth = T * T * T * (10 + T * (-15 + 6 * T));
				Vec3 Gradient;
				if constexpr(Derivatives) Gradient = CoreDistance.G * (-30 * T * T * (1 - T) * (1 - T));
				const Jet Core { 1 - Smooth, Gradient };
				// 合流点でも河道中心の河床上限を保ち，重みで分離した広域の開削とは別に補間残差を補正する
				CoreCut = SmoothMax(CoreCut, LocalCut * Core, CoreSupport * Core * 0.02);
				CoreSupport = CoreSupport + Core - CoreSupport * Core;
			}
		}
		Jet Cut;
		if(WeightSum.V > 0) {
			const double Value = Numerator.V / WeightSum.V;
			if constexpr(Derivatives) Cut = { Value, (Numerator.G - WeightSum.G * Value) * (1 / WeightSum.V) };
			else Cut = { Value, {} };
			Cut = SmoothMax(Cut, CoreCut, CoreSupport * 0.02);
		}
		if(Surface) {
			Surface->ChannelCoverage = std::clamp(Coverage, 0.0, 1.0);
			const double Sea = Network->Settings().SeaLevelMeters, LandHeight = std::max(0.0, Original.ElevationMeters - Sea);
			Surface->WaterSupport = LandHeight > 0 ? std::clamp(WaterSupport, 0.0, 1.0) : 1;
			if(LandHeight > 0) {
				const double Freeboard = Network->Settings().BankFreeboardMeters;
				const double RawHead = WaterWeight > 0 ? std::max(0.0, WeightedWater / WaterWeight - Sea) : 0;
				// 河道中心の拘束と大洋の拘束を距離で補間する，局所上限で中心の縦断を切下げない
				// 中心からの二乗距離で上限を再尺度化し，中心では元の水位，海岸では一定海面へ接続する
				// 内側の河岸に達したら上限を完全に適用し，遠方で元地形より高い水位を残さない
				// 中心では０，河岸では１として両端の１階・２階微分を零へ接続する
				const double Q = WaterWeight > 0 ? MinimumWaterDistance : 1, Distance = Q < 1 ? Q * Q * Q * (10 + Q * (-15 + 6 * Q)) : 1;
				const double Ceiling = LandHeight * (LandHeight / (LandHeight + 0.5 * Freeboard));
				const double Width = Ceiling * (0.5 * Freeboard / (LandHeight + Freeboard)), ScaledHead = RawHead * Distance;
				const double Blend = Width > 0 ? std::max(0.0, 1 - std::abs(ScaledHead - Ceiling) / Width) : 0;
				Surface->WaterElevationMeters =
				Distance > 0 ? Sea + (std::min(ScaledHead, Ceiling) - 0.25 * Width * Blend * Blend) / Distance : Sea + RawHead;
			} else Surface->WaterElevationMeters = Sea;
		}
		Jet Result = H - Cut;
		if(Stages) {
			Stages->AfterValleyMeters = Result.V;
			Stages->AfterFineReliefMeters = Result.V;
		}
		// 海岸と河道の拘束を維持し，陸上の非河道部だけへ同じ解析的起伏を加える
		const Jet Land = ReliefGate(Result - Jet{ Network->Settings().SeaLevelMeters, {} }, 2, 8);
		const Jet Valley = Jet{ 1, {} } - ReliefGate(WeightSum, 0.001, 0.05);
		const Jet Mask = Land * Valley;
		if(Mask.V > 0) {
			if(!Ocean->HasPredrainageLandRelief()) {
				const Jet Relief =
				SampleLandRelief<Derivatives>(Point, Base().Settings().RadiusMeters, Base().Settings().Seed ^ UINT64_C(0X5355424752494401));
				Result = Result + Relief * Mask;
			}
			if(Stages) Stages->AfterFineReliefMeters = Result.V;
			const Jet Highland = ReliefGate(Result - Jet{ Network->Settings().SeaLevelMeters, {} }, 150, 1000);
			const Jet MesoMask = Mask * Highland * MesoClearance;
			if(MesoMask.V > 0 && !Ocean->HasPredrainageMesoRelief()) {
				Jet Detail;
				if constexpr(Derivatives) {
					const TerrainSample Sample = Meso.Evaluate(Direction);
					Detail = Jet{ Sample.ElevationMeters, Sample.TangentialGradientPerRadian };
				} else Detail = Jet{ Meso.EvaluateElevation(Direction), {} };
				const double Scale = Base().Settings().RadiusMeters / (Base().Settings().RadiusMeters + 250000);
				Result = Result + Detail * MesoMask * Scale;
			}
		}
		if(Stages) Stages->FinalMeters = Result.V;
		// 同じ地点に対する値と指定した微分の返戻
		if constexpr(Derivatives) return { Result.V, Result.G - Point * Result.G.Dot(Point) };
		// 同じ地点に対する値と指定した微分の返戻
		else return { Result.V, {} };
	}

	/**
	 * 水面の位置に必要な地形と水位の値だけを評価する
	 * @param Direction 評価方向
	 * @return 勾配を含まない標高・水位・支持範囲
	 */
	RiverWaterHeights RiverTerrainField::EvaluateWater(const UnitDirection Direction) const {
		RiverSurfaceSample Detail;
		const TerrainSample Ground = EvaluateImpl<false>(Direction, true, nullptr, &Detail);
		// 勾配を求めず，同じ地形・水位・支持範囲を返す
		return { Ground.ElevationMeters, Detail.WaterElevationMeters, Detail.WaterSupport };
	}

	/**
	 * 地形と同じ河道から地表被覆と水位を取得する
	 * @param Direction 評価方向
	 * @return 標高・勾配と河川被覆・水位
	 */
	RiverSurfaceSample RiverTerrainField::EvaluateSurface(const UnitDirection Direction) const {
		RiverSurfaceSample Result;
		Result.Terrain = EvaluateImpl<true>(Direction, true, nullptr, &Result);
		// 同じ河道から求めた地面と水位の返戻
		return Result;
	}

	/**
	 * 表示と衝突で共有する標高だけを評価する
	 * @param Direction 評価方向
	 * @return 合成後の標高
	 */
	double RiverTerrainField::EvaluateElevation(const UnitDirection Direction) const {
		// 微分を省いた同じ最終標高の返戻
		return EvaluateImpl<false>(Direction, true).ElevationMeters;
	}

	/**
	 * 同じ地点評価から各合成段階の標高を取得する
	 * @param Direction 球面上の正規方向
	 * @return 元地形・盛土・河谷・細部起伏・最終地形の標高
	 */
	RiverElevationStages RiverTerrainField::EvaluateElevationStages(const UnitDirection Direction) const {
		RiverElevationStages Stages;
		EvaluateImpl<false>(Direction, true, nullptr, nullptr, &Stages);
		// 同じ評価順で求めた各段階の標高の返戻
		return Stages;
	}

	/**
	 * 盛土の上包絡に入る前の補間面を読取専用で診断する
	 * @param Direction 球面上の評価方向
	 * @return 補間面の値と解析勾配，盛土を持たない場では空
	 */
	std::optional<TerrainSample> RiverTerrainField::EvaluateBasinReconstruction(const UnitDirection Direction) const {
		// 通常の盛土未使用経路を別の地形へ置換えない返戻
		if(!Basins) return {};
		// 合成前の補間面の返戻
		return Basins->EvaluateReconstruction(Direction);
	}

	/**
	 * 地形の標高と解析勾配を評価する
	 * @param Direction 評価方向
	 * @return 合成後の標高と接線勾配
	 */
	TerrainSample RiverTerrainField::Evaluate(const UnitDirection Direction) const {
		// 索引付きの最終標高と解析勾配の返戻
		return EvaluateImpl<true>(Direction, true);
	}

	/**
	 * 空間索引を使わず全河道を調べて地形を評価する
	 * @param Direction 評価方向
	 * @return 索引検査に用いる標高と接線勾配
	 */
	TerrainSample RiverTerrainField::EvaluateReference(const UnitDirection Direction) const {
		// 全河道を調べる独立比較用の地形の返戻
		return EvaluateImpl<true>(Direction, false);
	}

	/**
	 * 空間索引が列挙する河道の候補数を取得する
	 * @param Direction 評価方向
	 * @return 支持領域の精密判定前の候補数
	 */
	std::size_t RiverTerrainField::CandidateCount(const UnitDirection Direction) const {
		std::size_t Count = 0;
		EvaluateImpl<false>(Direction, true, &Count);
		// 索引が列挙した候補数の返戻
		return Count;
	}
}
