#include "PlanetDrainage.h"
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
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		constexpr double Pi = 3.14159265358979323846;
		constexpr std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();

		/**
		 * 単位方向間の中心角を求める
		 * @param A 始点方向
		 * @param B 終点方向
		 * @return ラジアン単位の非負の中心角
		 */
		double Arc(const Vec3 A, const Vec3 B) {
			// 中心角の返戻
			return std::atan2(A.Cross(B).Length(), A.Dot(B));
		}
	}

	/**
	 * 陸から海へ向かう弧で最初に検出した交差を絞込む
	 * @param Start 陸上の始点
	 * @param End 弧の終点，陸上の場合も途中の海岸を探索する
	 * @param Radius 惑星半径
	 * @param Sea 一定の海面高度
	 * @param Height 連続な標高関数
	 * @param Stop 中断要求
	 * @return 海岸の挟込み，不正入力又は中断時は空，６４区間より狭い入り江の不在保証ではない
	 */
	std::optional<ShoreCrossing> FindShoreCrossing(
		const UnitDirection Start,
		const UnitDirection End,
		const double Radius,
		const double Sea,
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop
	) {
		// 海岸探索の入力が不正な場合の返戻
		if(!Height || !std::isfinite(Radius) || Radius < 1000 || Radius > 1E9 || !std::isfinite(Sea) || Stop.stop_requested()) {
			// 海岸探索を開始出来ない場合の返戻
			return {};
		}
		const Vec3 A = Start.Vector(), B = End.Vector();
		const double Angle = Arc(A, B);
		const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(A.Cross(B - A).Cross(A));
		// 弧の接線を定義出来ない場合の返戻
		if(!Tangent || Angle <= 0 || Angle >= Pi - 1E-9) return {};
		const auto At = [&](const double T) {
			// 端点を保持した大円上の方向の返戻
			return !T ?
			Start :
			T == 1 ? End : *UnitDirection::FromVector(A * std::cos(T * Angle) + Tangent->Vector() * std::sin(T * Angle));
		};
		// 大円弧と曲線で共通の有限な海岸探索を使う返戻
		return FindShoreCrossingOnCurve(At, Angle * Radius, 1, Radius, Sea, Height, Stop);
	}

	/**
	 * 同じ河道曲線の最初に検出した海岸を実距離上限で絞込む
	 * @param Curve 公称位置０から１に対応する連続な球面方向
	 * @param ParameterLengthMeters 曲線の公称長
	 * @param MaximumStretch 公称距離に対する実接線長の保守上限
	 * @param Radius 惑星の半径
	 * @param Sea 一定海面
	 * @param Height 同じ連続な地形の標高
	 * @param Stop 中断要求
	 * @return 検出した海岸の実距離上限付き挟込み，全微小湾の不在保証ではない
	 */
	std::optional<ShoreCrossing> FindShoreCrossingOnCurve(
		const std::function<UnitDirection(double)> &Curve,
		const double ParameterLengthMeters,
		const double MaximumStretch,
		const double Radius,
		const double Sea,
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop
	) {
		const bool Valid =
		Curve && Height && std::isfinite(ParameterLengthMeters) && ParameterLengthMeters > 0 && std::isfinite(MaximumStretch) &&
		MaximumStretch >= 1 && std::isfinite(ParameterLengthMeters * MaximumStretch) && std::isfinite(Radius) && Radius >= 1000 &&
		(Radius < 1E9 || Radius == 1E9) && std::isfinite(Sea) && !Stop.stop_requested();
		// 不正な曲線又は実距離上限の返戻
		if(!Valid) return {};
		const double First = Height(Curve(0)) - Sea, Last = Height(Curve(1)) - Sea;
		// 陸から海へ向かう有効な挟込みでない場合の返戻
		if(!std::isfinite(First) || !std::isfinite(Last) || First <= 0) return {};
		double Land = 0, Wet = 1, Residual = Last;
		bool Found = false;
		for(unsigned k = 1; k < 65; ++k) {
			// 準備が中断された場合の返戻
			if(Stop.stop_requested()) return {};
			const double T = k / 64.0, H = k == 64 ? Last : Height(Curve(T)) - Sea;
			// 標高が非有限な場合の返戻
			if(!std::isfinite(H)) return {};
			if(H <= 0) {
				Wet = T;
				Found = true;
				Residual = H;
				break;
			}
			Land = T;
		}
		// 全区間で海岸を検出しなかった場合の返戻
		if(!Found) return {};
		const double Length = ParameterLengthMeters * MaximumStretch, Tolerance = std::max(1E-5, Radius * 1E-12);
		for(unsigned k = 0; k < 64 && (Wet - Land) * Length > Tolerance; ++k) {
			// 準備が中断された場合の返戻
			if(Stop.stop_requested()) return {};
			const double Mid = std::midpoint(Land, Wet), H = Height(Curve(Mid)) - Sea;
			// 二分探索を進められない場合の返戻
			if(!std::isfinite(H) || Mid == Land || Mid == Wet) return {};
			if(H <= 0) {
				Wet = Mid;
				Residual = H;
			} else Land = Mid;
		}
		// 海岸の挟込みが指定精度へ達しない場合の返戻
		if((Wet - Land) * Length > Tolerance || Wet <= 0 || Wet > 1) return {};
		// 最初に検出した海岸の挟込みの返戻
		return ShoreCrossing{ Curve(Wet).Vector(), Land, Wet, (Wet - Land) * Length, Residual };
	}

	/**
	 * 半径，海面，水位と流路選択の定義を検査する
	 * @return 有限且つ対応範囲内の場合に真
	 */
	bool DrainageSettings::IsValid() const {
		// 対応する有限な定義の返戻
		return std::isfinite(RadiusMeters) && !(RadiusMeters < 1000) && RadiusMeters <= 1E9 && std::isfinite(SeaLevelMeters) &&
		std::abs(SeaLevelMeters) < RadiusMeters * 0.5 && std::isfinite(MinimumSlope) && MinimumSlope >= 1E-10 && MinimumSlope <= 0.01 &&
		std::isfinite(BankFreeboardMeters) && BankFreeboardMeters >= 0 && !(BankFreeboardMeters > 10) &&
		std::isfinite(ChannelAreaFraction) && ChannelAreaFraction > 0 && !(ChannelAreaFraction > 1);
	}

	/**
	 * 固定グラフから大洋へ到達する排水木と共有水位を準備する
	 * @param Sites 方向，標高，局所面積と海の判定
	 * @param Edges 無向の接続辺
	 * @param S 半径と水位拘束
	 * @param Stop 中断要求
	 * @return 全陸域が大洋へ到達する排水，不正又は中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::Solve(
		std::span<const DrainageSite> Sites,
		std::span<const std::array<std::uint32_t, 2>> Edges,
		const DrainageSettings &S,
		const std::stop_token Stop
	) {
		// 任意入力では此のグラフ自身の大洋接続を要求する
		return SolveImpl(Sites, Edges, S, OceanConnectivityEvidence::Graph, Stop);
	}

	/**
	 * 接続証拠を持つ大洋出口から固定網の排水を解く
	 * @param Sites 方向，標高，局所面積と海の判定
	 * @param Edges 無向の排水接続辺
	 * @param S 半径と水位拘束
	 * @param Evidence 同じ網又は不変な元海域場による接続証拠
	 * @param Stop 中断要求
	 * @return 全陸域が大洋へ到達する排水，不正又は中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::SolveImpl(
		std::span<const DrainageSite> Sites,
		std::span<const std::array<std::uint32_t, 2>> Edges,
		const DrainageSettings &S,
		const OceanConnectivityEvidence Evidence,
		const std::stop_token Stop
	) {
		// 公開する適応網は６５５３６点迄とし，海域接続の検査を省略せずに受理する
		const std::size_t MaximumSites = Evidence == OceanConnectivityEvidence::Graph ? 65536 : 163842;
		const bool HasValidInput =
		S.IsValid() && !Sites.empty() && Sites.size() <= MaximumSites && Edges.size() <= Sites.size() * 8 && !Stop.stop_requested();
		// 非対応の排水設定又は入力規模の返戻
		if(!HasValidInput) return {};
		DrainageNetwork N;
		N.Definition = S;
		N.Stats.Nodes = static_cast<std::uint32_t>(Sites.size());
		const std::uint32_t Count = N.Stats.Nodes;
		N.Graph.resize(Count);
		struct Link {
			std::uint32_t To;
			double Length;
		};
		std::vector<std::vector<Link>> Neighbors(Count);
		std::uint32_t OceanRoot = Missing;
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 排水準備の中断時の返戻
			if(!(i & 255) && Stop.stop_requested()) return {};
			const DrainageSite &Site = Sites[i];
			const std::optional<UnitDirection> D = UnitDirection::FromVector(Site.Direction);
			const bool HasValidSite =
			D && std::isfinite(Site.ElevationMeters) && std::isfinite(Site.AreaSquareMeters) && Site.AreaSquareMeters > 0 &&
			(!Site.Ocean || Site.ElevationMeters <= S.SeaLevelMeters);
			// 地点の方向，標高又は局所面積が不正な場合の返戻
			if(!HasValidSite) return {};
			DrainageNode &Node = N.Graph[i];
			Node.Direction = D->Vector();
			Node.OriginalElevationMeters = Site.ElevationMeters;
			Node.Ocean = Site.Ocean;
			Node.Downstream = Missing;
			Node.Outlet = Missing;
			Node.WaterMeters = std::numeric_limits<double>::infinity();
			if(Site.Ocean) {
				++N.Stats.OceanNodes;
				// 有効な大洋又は有限な陸域面積が無い場合の返戻
				if(OceanRoot == Missing) OceanRoot = i;
			} else {
				++N.Stats.LandNodes;
				Node.ContributingAreaSquareMeters = Site.AreaSquareMeters;
				N.Stats.LandAreaSquareMeters += Site.AreaSquareMeters;
			}
		}
		// 有効な大洋又は有限な陸域面積が無い場合の返戻
		if(OceanRoot == Missing || !std::isfinite(N.Stats.LandAreaSquareMeters)) return {};
		for(const std::array<std::uint32_t, 2> E : Edges) {
			// 範囲外又は自己循環の辺の返戻
			if(E[0] >= Count || E[1] >= Count || E[0] == E[1]) return {};
			const double Angle = Arc(N.Graph[E[0]].Direction, N.Graph[E[1]].Direction);
			// 有効な長さを持たない辺の返戻
			if(!std::isfinite(Angle) || Angle <= 0 || Angle >= Pi - 1E-9) return {};
			const double Length = Angle * S.RadiusMeters;
			Neighbors[E[0]].push_back({ E[1], Length });
			Neighbors[E[1]].push_back({ E[0], Length });
		}
		for(std::vector<Link> &Adjacent : Neighbors) {
			std::sort(
				Adjacent.begin(),
				Adjacent.end(),
				[](const Link A, const Link B) {
					return A.To < B.To;
				}
			);
			Adjacent.erase(
				std::unique(
					Adjacent.begin(),
					Adjacent.end(),
					[](const Link A, const Link B) {
						return A.To == B.To;
					}
				),
				Adjacent.end()
			);
		}
		// 任意グラフの孤立水域を正当な出口としない
		// 海域場の証拠は内部の CreateOnMesh だけが渡す
		if(Evidence == OceanConnectivityEvidence::Graph) {
			std::vector<bool> OceanSeen(Count, false);
			std::vector<std::uint32_t> WetQueue { OceanRoot };
			OceanSeen[OceanRoot] = true;
			for(std::size_t i = 0; i < WetQueue.size(); ++i) {
				for(const Link E : Neighbors[WetQueue[i]]) if(N.Graph[E.To].Ocean && !OceanSeen[E.To]) {
					OceanSeen[E.To] = true;
					WetQueue.push_back(E.To);
				}
			}
			// 同じ入力網で接続を証明出来ない水域の返戻
			if(WetQueue.size() != N.Stats.OceanNodes) return {};
		}
		using Entry = std::pair<double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Queue;
		std::vector<bool> Settled(Count, false);
		std::vector<std::uint32_t> Order;
		Order.reserve(Count);
		std::vector<double> ParentLength(Count, 0);
		for(std::uint32_t i = 0; i < Count; ++i) if(N.Graph[i].Ocean) {
			DrainageNode &Node = N.Graph[i];
			Node.WaterMeters = S.SeaLevelMeters;
			Node.Downstream = i;
			Node.Outlet = i;
			Queue.emplace(Node.WaterMeters, i);
		}
		std::uint32_t Iterations = 0;
		while(!Queue.empty()) {
			// 水位伝播の中断時の返戻
			if(!(Iterations++ & 255) && Stop.stop_requested()) return {};
			const auto [Height, i] = Queue.top();
			Queue.pop();
			DrainageNode &From = N.Graph[i];
			if(Settled[i] || Height != From.WaterMeters) continue;
			Settled[i] = true;
			Order.push_back(i);
			if(!From.Ocean) {
				const DrainageNode &Down = N.Graph[From.Downstream];
				// 確定済みの下流へ下降しない結果の返戻
				if(!Settled[From.Downstream] || !(Down.WaterMeters < Height)) return {};
				From.Outlet = Down.Outlet;
				From.DistanceToOceanMeters = Down.DistanceToOceanMeters + ParentLength[i];
			}
			for(const Link E : Neighbors[i]) {
				DrainageNode &To = N.Graph[E.To];
				if(Settled[E.To] || To.Ocean) continue;
				// Priority-Flood の最小水位を伝播し，実辺長で最低勾配，nextafter で丸めによる平坦部の発生を防ぐ
				const double Rise = std::nextafter(Height + S.MinimumSlope * E.Length, std::numeric_limits<double>::infinity());
				const double Candidate = std::max(To.OriginalElevationMeters - S.BankFreeboardMeters, Rise);
				// 非有限な水位候補の返戻
				if(!std::isfinite(Candidate)) return {};
				if(Candidate < To.WaterMeters || Candidate == To.WaterMeters && i < To.Downstream) {
					const bool Changed = Candidate < To.WaterMeters;
					To.WaterMeters = Candidate;
					To.Downstream = i;
					ParentLength[E.To] = E.Length;
					if(Changed) Queue.emplace(Candidate, E.To);
				}
			}
		}
		// 全地点を大洋へ接続出来なかった場合の返戻
		if(Order.size() != Count) return {};
		for(std::vector<std::uint32_t>::reverse_iterator it = Order.rbegin(); it != Order.rend(); ++it) {
			const DrainageNode &Node = N.Graph[*it];
			if(!Node.Ocean) N.Graph[Node.Downstream].ContributingAreaSquareMeters += Node.ContributingAreaSquareMeters;
		}
		if(S.PreferBreaching) {
			// 排水木を保持し，窪地全体の盛土に代えて障壁を開削する側の水位を選ぶ，最低勾配を除いた上限を下流へ伝播する
			// 固定した木の水位拘束であり，実際の開削面，全球で最小の開削量，土砂の保存を保証する物ではない
			std::vector<double> Headroom(Count, 0);
			for(std::uint32_t i = 0; i < Count; ++i) {
				const DrainageNode &Node = N.Graph[i];
				if(!Node.Ocean) {
					Headroom[i] = std::max(
						0.0,
						Node.OriginalElevationMeters - S.BankFreeboardMeters - S.SeaLevelMeters - S.MinimumSlope * Node.DistanceToOceanMeters
					);
				}
			}
			for(std::vector<std::uint32_t>::reverse_iterator it = Order.rbegin(); it != Order.rend(); ++it) {
				const DrainageNode &Node = N.Graph[*it];
				if(!Node.Ocean && !N.Graph[Node.Downstream].Ocean) {
					Headroom[Node.Downstream] = std::min(Headroom[Node.Downstream], Headroom[*it]);
				}
			}
			for(const std::uint32_t i : Order) {
				// 排水準備の中断時の返戻
				if(!(i & 255) && Stop.stop_requested()) return {};
				DrainageNode &Node = N.Graph[i];
				if(Node.Ocean) continue;
				const double Grade = S.SeaLevelMeters + S.MinimumSlope * Node.DistanceToOceanMeters;
				const double Rise = std::nextafter(
					N.Graph[Node.Downstream].WaterMeters + S.MinimumSlope * ParentLength[i],
					std::numeric_limits<double>::infinity()
				);
				Node.WaterMeters = std::max(Grade + Headroom[i], Rise);
				// 非有限な開削側水位の返戻
				if(!std::isfinite(Node.WaterMeters)) return {};
			}
		}
		const double Threshold = 4 * Pi * S.RadiusMeters * S.RadiusMeters * S.ChannelAreaFraction;
		for(std::uint32_t i = 0; i < Count; ++i) {
			DrainageNode &Node = N.Graph[i];
			// 非有限な集水面積の返戻
			if(!std::isfinite(Node.ContributingAreaSquareMeters)) return {};
			// 暫定の共有水深を集水面積から求める，集水面積は下流へ減らないため，合流点に逆勾配の河床を作らない
			Node.DepthMeters = std::min(12.0, 0.5 + 0.2 * std::log1p(Node.ContributingAreaSquareMeters / 1E6));
			if(Node.Ocean) {
				N.Stats.OutletAreaSquareMeters += Node.ContributingAreaSquareMeters;
				continue;
			}
			const double Raise = std::max(0.0, Node.WaterMeters + S.BankFreeboardMeters - Node.OriginalElevationMeters);
			const double Cut = std::max(0.0, Node.OriginalElevationMeters - S.BankFreeboardMeters - Node.WaterMeters);
			N.Stats.MaximumRequiredWaterCutMeters = std::max(N.Stats.MaximumRequiredWaterCutMeters, Cut);
			N.Stats.MaximumRequiredBankRaiseMeters = std::max(N.Stats.MaximumRequiredBankRaiseMeters, Raise);
			N.Stats.AreaWeightedBankRaiseMeters += Raise * Sites[i].AreaSquareMeters;
			N.Stats.LongestRouteMeters = std::max(N.Stats.LongestRouteMeters, Node.DistanceToOceanMeters);
			if(Node.ContributingAreaSquareMeters >= Threshold) {
				N.Rivers.push_back({ i, Node.Downstream, ParentLength[i], 1, N.Graph[Node.Downstream].Direction });
			}
		}
		if(N.Stats.LandAreaSquareMeters > 0) N.Stats.AreaWeightedBankRaiseMeters /= N.Stats.LandAreaSquareMeters;
		N.Stats.Reaches = static_cast<std::uint32_t>(N.Rivers.size());
		// 準備が中断された場合の返戻
		if(Stop.stop_requested()) return {};
		// 準備した固定排水と水位の返戻
		return N;
	}

	/**
	 * 接続済みの海陸標高から固定球面網の排水と河口を準備する
	 * @param Ocean 地質と接続済みの海域
	 * @param Stop 中断要求
	 * @return 固定排水と連続な水位，準備失敗時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::Create(const OceanConnectivityField &Ocean, const std::stop_token Stop) {
		// 海岸と同じ標高を排水へ渡す
		const auto Height = [&Ocean](const UnitDirection D) {
			return Ocean.EvaluateElevation(D);
		};
		// 元の海陸に基づく排水と縦断の返戻
		return CreateImpl(Ocean, Height, true, Stop);
	}

	/**
	 * 指定した連続標高で流路だけを解き直し，海岸と侵食前の縦断は元の海陸から準備する
	 * @param Ocean 元の海陸分類と定義識別
	 * @param Height 再経路化する標高関数
	 * @param Stop 中断要求
	 * @return 新しい排水木と海岸・水位，不正値又は中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateForHeight(
		const OceanConnectivityField &Ocean,
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop
	) {
		// 標高関数が無い又は中断済みの場合の返戻
		if(!Height || Stop.stop_requested()) return {};
		// 別の経路標高から準備した排水の返戻
		return CreateImpl(Ocean, Height, false, Stop);
	}

	/**
	 * 海岸を保ったまま指定標高で排水木と河川縦断を準備する
	 * @param Ocean 海陸分類と最初の海岸を定義する場
	 * @param Height 排水を作る有限な標高関数
	 * @param SharedSourceHeight 標高関数が海陸場そのものの場合に真
	 * @param Stop 中断要求
	 * @return 固定排水と連続な水位，不正値又は中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateImpl(
		const OceanConnectivityField &Ocean,
		const std::function<double(UnitDirection)> &Height,
		const bool SharedSourceHeight,
		const std::stop_token Stop
	) {
		// 準備が中断された場合の返戻
		if(!Height || Stop.stop_requested()) return {};
		const std::optional<SphereMesh> Mesh = BuildUniformSphere(6);
		// 海域準備の網は此の粗網の全頂点を保持する
		// 補正後の同じ標高で接続を再評価済みである
		if(!Mesh || Ocean.Statistics().GraphNodes < Mesh->Directions.size() || Ocean.Statistics().ComponentsAfter != 1) return {};
		// 既存の固定網と局所網で面積・海岸の処理を共有する返戻
		return CreateOnMesh(Ocean, Mesh->Directions, Mesh->Triangles, Height, SharedSourceHeight, Stop);
	}

	/**
	 * 閉じた球面網から面積・下流・河口・縦断を共通の処理で準備する
	 * @param Ocean 元の海陸と接続証拠
	 * @param Directions 内部の網生成器が作った地点
	 * @param Triangles 同じ生成器が作った閉じた面
	 * @param Height 排水経路の標高関数
	 * @param SharedSourceHeight 元の海陸と同じ標高を使う場合に真
	 * @param Stop 中断要求
	 * @return 排水網，不正値・準備失敗・中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateOnMesh(
		const OceanConnectivityField &Ocean,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::function<double(UnitDirection)> &Height,
		const bool SharedSourceHeight,
		const std::stop_token Stop
	) {
		const bool HasValidSource =
		Height && Directions.size() > 40961 && !Triangles.empty() && Ocean.Statistics().ComponentsAfter == 1 && !Stop.stop_requested();
		// 共通の海陸証拠や正規の球面網が無い場合の返戻
		if(!HasValidSource) return {};
		DrainageSettings S;
		S.RadiusMeters = Ocean.Base().Settings().RadiusMeters;
		S.SeaLevelMeters = Ocean.Base().Settings().SeaLevelMeters;
		S.PreferBreaching = true;
		S.UseCurvedPlanforms = true;
		S.PlanformSeed = Ocean.Base().Settings().Seed;
		// 大きな半径でも最低勾配による累積上昇を有界にする，物理的な縦断形状は地域の侵食で扱う
		S.MinimumSlope = 1E-7 * std::min(1.0, 1E6 / S.RadiusMeters);
		std::vector<DrainageSite> Sites(Directions.size());
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(Triangles.size() * 3);
		for(std::size_t i = 0; i < Sites.size(); ++i) {
			// 排水準備の中断時の返戻
			if(!(i & 255) && Stop.stop_requested()) return {};
			Sites[i].Direction = Directions[i].Vector();
			Sites[i].ElevationMeters = Height(Directions[i]);
			const double SourceHeight = SharedSourceHeight ? Sites[i].ElevationMeters : Ocean.EvaluateElevation(Directions[i]);
			Sites[i].Ocean = SourceHeight < S.SeaLevelMeters;
		}
		for(const std::array<std::uint32_t, 3> T : Triangles) {
			const Vec3 A = Sites[T[0]].Direction, B = Sites[T[1]].Direction, C = Sites[T[2]].Direction;
			const double Share =
			2 * std::atan2(std::abs(A.Dot((B - A).Cross(C - A))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A)) * S.RadiusMeters * S.RadiusMeters / 3;
			for(unsigned k = 0; k < 3; ++k) {
				Sites[T[k]].AreaSquareMeters += Share;
				const std::uint32_t U = T[k], V = T[(k + 1) % 3];
				Edges.push_back({ std::min(U, V), std::max(U, V) });
			}
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		std::optional<DrainageNetwork> Network = SolveImpl(Sites, Edges, S, OceanConnectivityEvidence::ConnectedSourceField, Stop);
		// 大洋へ向かう排水を解けなかった場合の返戻
		if(!Network) return {};
		Network->SourceHash = Ocean.DefinitionHash();
		const auto ShoreHeight = [&Ocean](const UnitDirection D) {
			return Ocean.EvaluateElevation(D);
		};
		Network = Network->WithCoastalOutlets(ShoreHeight, Stop);
		// 海岸に排水を接続出来なかった場合の返戻
		if(!Network) return {};
		// 経路だけを再選択し，侵食と谷を元の海陸から再構築する縦断曲線の返戻
		return Network->WithTerrainProfiles(ShoreHeight, Stop);
	}

	/**
	 * 窪地から既存河道までの指定辺だけを河道に昇格し，共有水位を再構築する
	 * 計算量：O(n log n + m p)（n は指定辺数，m は全河道数，p は縦断の固定標本数）
	 * @param UpstreamNodes 昇格する排水辺の上流地点番号
	 * @param Ocean 元の排水と同じ海陸場
	 * @param Stop 準備の中断要求
	 * @return 追加河道と縦断を持つ排水，不正な定義・地点・中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::WithPromotedTributaries(
		std::span<const std::uint32_t> UpstreamNodes,
		const OceanConnectivityField &Ocean,
		const std::stop_token Stop
	) const {
		const bool HasSharedDefinition =
		!Graph.empty() && Stats.CoastalOutletNodes && SourceHash == Ocean.DefinitionHash() && Longitudinal.size() == Rivers.size() &&
		!Stop.stop_requested();
		// 海岸調整前の網又は異なる惑星定義を混ぜない返戻
		if(!HasSharedDefinition) return {};
		// 追加すべき辺が無い場合の返戻
		if(UpstreamNodes.empty()) return *this;
		DrainageNetwork Result = *this;
		std::vector<std::uint8_t> Existing(Graph.size());
		for(const RiverReach &Reach : Rivers) Existing[Reach.Upstream] = 1;
		std::vector<std::uint32_t> Ordered(UpstreamNodes.begin(), UpstreamNodes.end());
		std::sort(Ordered.begin(), Ordered.end());
		Ordered.erase(std::unique(Ordered.begin(), Ordered.end()), Ordered.end());
		Result.Rivers.reserve(Rivers.size() + Ordered.size());
		for(const std::uint32_t Upstream : Ordered) {
			// 不正な辺や中断済みの生成を公開しない返戻
			if(Stop.stop_requested() || Upstream >= Graph.size() || Graph[Upstream].Ocean || Graph[Upstream].Downstream >= Graph.size()) {
				// 不正な昇格指定の返戻
				return {};
			}
			if(Existing[Upstream]) continue;
			const DrainageNode &Start = Graph[Upstream], &End = Graph[Start.Downstream];
			const double Length = Arc(Start.Direction, End.Direction) * Definition.RadiusMeters;
			// 退化した河道の返戻
			if(!std::isfinite(Length) || Length <= 0) return {};
			const std::uint32_t Reach = static_cast<std::uint32_t>(Result.Rivers.size());
			Result.Rivers.push_back({ Upstream, Start.Downstream, Length, 1, End.Direction });
			if(End.Ocean) {
				// 既に絞った海岸地点を新しい河口として再利用する
				Result.CoastalMouths.push_back({ Reach, { End.Direction, 1, 1, 0, End.OriginalElevationMeters - Definition.SeaLevelMeters } });
			}
		}
		Result.Stats.Reaches = static_cast<std::uint32_t>(Result.Rivers.size());
		// 新しい合流点と河口の水位を同一の海陸場で調整する
		const auto Height = [&Ocean](const UnitDirection Direction) {
			return Ocean.EvaluateElevation(Direction);
		};
		// 追加河道を含む縦断の返戻
		return Result.WithTerrainProfiles(Height, Stop);
	}

	/**
	 * 選択した河道上の方向，水位，河床と勾配を評価する
	 * @param Index 河道番号
	 * @param T 河口迄の長さで正規化した下流方向の位置
	 * @return 有効範囲の評価結果，範囲外又は不正時は空
	 */
	std::optional<RiverProfileSample> DrainageNetwork::SampleReach(const std::size_t Index, const double T) const {
		// 河道番号又は位置が不正な場合の返戻
		if(Index >= Rivers.size() || !std::isfinite(T) || T < 0 || T > 1) return {};
		const RiverReach &Reach = Rivers[Index];
		const DrainageNode &A = Graph[Reach.Upstream];
		const DrainageNode &B = Graph[Reach.Downstream];
		const std::optional<RiverHeights> CurvedHeights = Reach.Planform ? SampleHeights(Index, T) : std::nullopt;
		if(Reach.Planform) {
			// 曲線を持つ河道の縦断が不正な場合の返戻
			if(!CurvedHeights) return {};
			const RiverPlanformSample Shape = Reach.Planform->Evaluate(T * Reach.PlanformEndFraction);
			const double Stretch = Shape.DerivativePerFraction.Length() / Reach.Planform->ArcRadians();
			// 接線が消える曲線を公開しない返戻
			if(!(std::isfinite(Stretch) && Stretch > 0)) return {};
			const Vec3 Point = !T ? A.Direction : T == 1 ? Reach.EndDirection : Shape.Direction.Vector();
			// 実曲線上の接線長で水位勾配を換算した返戻
			return RiverProfileSample{
				Point,
				Shape.DerivativePerFraction * (1 / Shape.DerivativePerFraction.Length()),
				CurvedHeights->WaterMeters,
				CurvedHeights->BedMeters,
				CurvedHeights->WaterSlope / Stretch
			};
		}
		const double Angle = Reach.ProfileLengthMeters() / Definition.RadiusMeters;
		const std::optional<UnitDirection> Tangent =
		UnitDirection::FromVector(A.Direction.Cross(B.Direction - A.Direction).Cross(A.Direction));
		// 河道の接線が不正な場合の返戻
		if(!Tangent) return {};
		RiverProfileSample Sample;
		Sample.Direction = !T ? A.Direction : T == 1 ?
		Reach.EndDirection :
		UnitDirection::FromVector(A.Direction * std::cos(T * Angle) + Tangent->Vector() * std::sin(T * Angle))->Vector();
		Sample.DownstreamTangent = Tangent->Vector() * std::cos(T * Angle) - A.Direction * std::sin(T * Angle);
		const std::optional<RiverHeights> Heights = SampleHeights(Index, T);
		// 縦断標高を評価出来なかった場合の返戻
		if(!Heights) return {};
		Sample.WaterMeters = Heights->WaterMeters;
		Sample.BedMeters = Heights->BedMeters;
		Sample.DownstreamWaterSlope = Heights->WaterSlope;
		// 方向と水位，河床の返戻
		return Sample;
	}

	/**
	 * 単調な余裕量と最低勾配から水位と河床を評価する
	 * @param T 正規化した河道内の位置，範囲外では端点接線で延長する
	 * @return 水位，河床，実距離に対する各勾配
	 */
	RiverHeights RiverLongitudinalProfile::Evaluate(const double T) const {
		// 端点の接線による延長で，有限幅の谷の端と C1 接続を保つ
		if(T < 0 || T > 1) {
			const double End = T < 0 ? 0.0 : 1.0;
			RiverHeights H = Evaluate(End);
			H.WaterMeters += (T - End) * LengthMeters * H.WaterSlope;
			H.BedMeters += (T - End) * LengthMeters * H.BedSlope;
			// 端点の接線で延長した水位と河床の返戻
			return H;
		}
		const double Position = T * Segments;
		const unsigned i = std::min(static_cast<unsigned>(Position), Segments - 1);
		const double U = Position - i, A = Headroom[i], B = Headroom[i + 1], M = Derivative[i] / Segments;
		// 差分形式で，大きな Hermite 基底同士の相殺を避ける
		const double N = Derivative[i + 1] / Segments, Delta = B - A, C = 3 * Delta - 2 * M - N, D = M + N - 2 * Delta;
		const double H = A + U * (M + U * (C + U * D)), DH = Segments * (M + U * (2 * C + 3 * U * D));
		RiverHeights Result;
		Result.WaterMeters = !T ? StartWater : T == 1 ? EndWater : std::lerp(GradeStart, GradeEnd, T) + H;
		Result.WaterSlope = (GradeEnd - GradeStart + DH) / LengthMeters;
		Result.BedMeters = Result.WaterMeters - std::lerp(StartDepth, EndDepth, T);
		Result.BedSlope = Result.WaterSlope - (EndDepth - StartDepth) / LengthMeters;
		// 区間の水位，河床と勾配の返戻
		return Result;
	}

	/**
	 * 準備済みの縦断曲線，又は未準備時の直線水位を評価する
	 * @param Index 河道番号
	 * @param T 正規化した下流方向の位置
	 * @return 標高と勾配，不正な番号又は非有限な位置は空
	 */
	std::optional<RiverHeights> DrainageNetwork::SampleHeights(const std::size_t Index, const double T) const {
		// 河道番号又は位置が不正な場合の返戻
		if(Index >= Rivers.size() || !std::isfinite(T)) return {};
		// 該当河道に準備済みの曲線が有る場合の返戻
		if(Index < Longitudinal.size()) return Longitudinal[Index].Evaluate(T);
		const RiverReach &R = Rivers[Index];
		const DrainageNode &A = Graph[R.Upstream];
		const DrainageNode &B = Graph[R.Downstream];
		const double Water = std::lerp(A.WaterMeters, B.WaterMeters, T);
		const double Slope = (B.WaterMeters - A.WaterMeters) / R.ProfileLengthMeters();
		// 未準備の曲線に代わる直線の縦断標高の返戻
		return RiverHeights{
			Water,
			Water - std::lerp(A.DepthMeters, B.DepthMeters, T),
			Slope,
			Slope - (B.DepthMeters - A.DepthMeters) / R.ProfileLengthMeters()
		};
	}

	/**
	 * 集水面積と水深から河床・岸の共通半幅を求める
	 * @param Index 河道番号
	 * @return 有効な河道の半幅，範囲外又は不正な形状値では空
	 */
	std::optional<RiverCrossSectionWidths> DrainageNetwork::ChannelWidths(const std::size_t Index) const {
		// 存在しない河道に対する返戻
		if(Index >= Rivers.size()) return {};
		const RiverReach &Reach = Rivers[Index];
		const DrainageNode &Start = Graph[Reach.Upstream], &End = Graph[Reach.Downstream];
		const double Area = Start.ContributingAreaSquareMeters;
		const double Depth = std::max(Start.DepthMeters, End.DepthMeters) + Definition.BankFreeboardMeters;
		// 非有限又は負の入力に対する返戻
		if(!std::isfinite(Area) || !std::isfinite(Depth) || Area < 0 || Depth < 0) return {};
		RiverCrossSectionWidths Result;
		Result.BedHalfWidthMeters = 0.5 * std::min(256.0, 2 + 0.12 * std::sqrt(Area / 1E6));
		// ５次の遷移式の最大微分 1.875 から，河岸の横断勾配 0.6 を目安に幅を決める
		Result.InnerHalfWidthMeters = Result.BedHalfWidthMeters + std::max(Result.BedHalfWidthMeters, 1.875 * (Depth + 0.02) / 0.6);
		// 算出した幅が非有限な場合の返戻
		if(!std::isfinite(Result.InnerHalfWidthMeters)) return {};
		// 水位準備と開削の双方で使う横断寸法の返戻
		return Result;
	}

	/**
	 * 現在の地形から始点・合流点・途中地点の上限を読み直し，下降する水位を共有する
	 * @param Height 再構成した連続な標高，既存の海域境界は維持する
	 * @param Stop 中断要求
	 * @return 地形と縦断形状を更新した排水，入力不正又は中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::WithTerrainProfiles(
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop
	) const {
		// 標高関数が未指定又は中断済みの場合の返戻
		if(!Height || Stop.stop_requested()) return {};
		DrainageNetwork N = *this;
		N.BalancedProfiles = false;
		N.Longitudinal.clear();
		N.Longitudinal.resize(Rivers.size());
		N.Stats.ProfileKnots = 0;
		N.Stats.ProfileTerrainEvaluations = 0;
		N.Stats.MaximumUnresolvedKnotBedGapMeters = 0;
		N.Stats.MaximumSampledWaterAboveTerrainMeters = 0;
		N.Stats.MaximumProfileWaterLoweringMeters = 0;
		std::vector<std::array<double, RiverLongitudinalProfile::Segments + 1>> TerrainHeights(Rivers.size());
		const std::size_t Count = Graph.size();
		const DrainageSettings &S = Definition;
		/**
		 * 河岸の地面より低い水位上限を求め，海岸では余裕高を滑らかに０へ近付ける
		 * @param Height 元の地面の標高
		 * @return 海面以上で，陸上では元の地面を越えない水位上限
		 */
		const auto BankCeiling = [&S](const double Height) {
			const double Relief = std::max(0.0, Height - S.SeaLevelMeters);
			// 海面以下に別の正の河川水位を設定しない為の返戻
			if(!Relief) return S.SeaLevelMeters;
			// 深さを地面の上へ足すのでなく，水面の上に河岸の余裕を残した上限の返戻
			return S.SeaLevelMeters + Relief * (Relief / (Relief + S.BankFreeboardMeters));
		};
		for(DrainageNode &Node : N.Graph) {
			// 準備が中断された場合の返戻
			if(Stop.stop_requested()) return {};
			if(Node.Ocean) continue;
			Node.OriginalElevationMeters = Height(*UnitDirection::FromVector(Node.Direction));
			++N.Stats.ProfileTerrainEvaluations;
			// 更新した始点又は合流点の標高が不正な場合の返戻
			if(!std::isfinite(Node.OriginalElevationMeters)) return {};
		}
		std::vector<std::uint32_t> Order(Count), Selected(Count, Missing);
		std::iota(Order.begin(), Order.end(), 0U);
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				const double X = Graph[A].DistanceToOceanMeters, Y = Graph[B].DistanceToOceanMeters;
				// 同じ距離では地点番号で確定する処理順序の返戻
				return X != Y ? X < Y : A < B;
			}
		);
		for(std::uint32_t i = 0; i < Rivers.size(); ++i) Selected[Rivers[i].Upstream] = i;
		std::vector<double> Distance(Count, 0), Grade(Count, S.SeaLevelMeters), Headroom(Count, 0);
		// 切詰めた河口への公称経路長を用いる，公開済みの粗い経路距離は維持し，全陸上地点で同じ最低勾配を使う
		for(const std::uint32_t i : Order) {
			// 準備が中断された場合の返戻
			if(Stop.stop_requested()) return {};
			const DrainageNode &A = N.Graph[i];
			if(A.Ocean) continue;
			const std::uint32_t Down = A.Downstream;
			const double Length =
			Selected[i] == Missing ? Arc(A.Direction, Graph[Down].Direction) * S.RadiusMeters : Rivers[Selected[i]].ProfileLengthMeters();
			Distance[i] = Distance[Down] + Length;
			Grade[i] = S.SeaLevelMeters + S.MinimumSlope * Distance[i];
			Headroom[i] = std::max(0.0, std::min(A.WaterMeters, BankCeiling(A.OriginalElevationMeters)) - Grade[i]);
		}
		// 水源から大洋へ向かい，途中地形の上限を共有合流点へ伝播する，単独の水位だけを下げず，非選択の支流も下流より高く保つ
		for(std::vector<std::uint32_t>::reverse_iterator it = Order.rbegin(); it != Order.rend(); ++it) {
			// 準備が中断された場合の返戻
			if(Stop.stop_requested()) return {};
			const std::uint32_t i = *it;
			if(Graph[i].Ocean) continue;
			const std::uint32_t Down = Graph[i].Downstream, R = Selected[i];
			double Previous = Headroom[i];
			if(R != Missing) {
				RiverLongitudinalProfile &P = N.Longitudinal[R];
				P.GradeStart = Grade[i];
				P.GradeEnd = Grade[Down];
				P.LengthMeters = Rivers[R].ProfileLengthMeters();
				P.StartDepth = Graph[i].DepthMeters;
				P.EndDepth = Graph[Down].DepthMeters;
				const std::optional<RiverCrossSectionWidths> Widths = ChannelWidths(R);
				// 河岸の標高制約を評価出来ない場合の返戻
				if(!Widths) return {};
				const double BankAngle = Widths->InnerHalfWidthMeters / S.RadiusMeters, BankCosine = std::cos(BankAngle);
				const double BankSine = std::sin(BankAngle);
				std::array<double, RiverLongitudinalProfile::Segments + 1> Caps {};
				/**
				 * 区間内の水位余裕の上限を評価し，必要時だけ標高を節点へ残す
				 * @param T 正規化した河道内の位置
				 * @param KnotHeight 節点標高の格納先，又は空
				 * @return 最低勾配上の非負の余裕量，不正又は中断時は空
				 */
				const auto SampleCap = [&](const double T, double *KnotHeight=nullptr) -> std::optional<double> {
					// 中断又は無効な経路の場合の返戻
					if(Stop.stop_requested()) return {};
					const std::optional<RiverProfileSample> Sample = SampleReach(R, T);
					// 河道位置を求められない場合の返戻
					if(!Sample) return {};
					const double H = !T ? N.Graph[i].OriginalElevationMeters : Height(*UnitDirection::FromVector(Sample->Direction));
					if(T) ++N.Stats.ProfileTerrainEvaluations;
					// 非有限な標高の場合の返戻
					if(!std::isfinite(H)) return {};
					if(KnotHeight) *KnotHeight = H;
					double CeilingHeight = H;
					// 曲線の各地点の接線に直交する両岸を，同じ水位制約へ渡す
					const Vec3 Across = UnitDirection::FromVector(Sample->Direction.Cross(Sample->DownstreamTangent))->Vector();
					// 斜面上の河道で，中心だけが地面の下でも横側へ溢れる状態を避ける
					for(const double Sign : { -1.0, 1.0 }) {
						const UnitDirection Bank = *UnitDirection::FromVector(Sample->Direction * BankCosine + Across * (Sign * BankSine));
						const double BankHeight = Height(Bank);
						++N.Stats.ProfileTerrainEvaluations;
						// 河岸標高が不正な場合の返戻
						if(!std::isfinite(BankHeight)) return {};
						CeilingHeight = std::min(CeilingHeight, BankHeight);
					}
					// 海面からの最低勾配を維持した水位余裕の返戻
					return std::max(0.0, std::min(Sample->WaterMeters, BankCeiling(CeilingHeight)) - std::lerp(P.GradeStart, P.GradeEnd, T));
				};
				for(unsigned k = 0; k <= RiverLongitudinalProfile::Segments; ++k) {
					const std::optional<double> Cap = SampleCap(static_cast<double>(k) / RiverLongitudinalProfile::Segments, &TerrainHeights[R][k]);
					// 節点を準備出来なかった場合の返戻
					if(!Cap) return {};
					Caps[k] = *Cap;
				}
				for(unsigned k = 0; k < RiverLongitudinalProfile::Segments; ++k) {
					std::array<double, 5> Values { Caps[k], 0, 0, 0, Caps[k + 1] };
					for(unsigned j = 1; j < 4; ++j) {
						const std::optional<double> Cap = SampleCap((k + 0.25 * j) / RiverLongitudinalProfile::Segments);
						// 区間内部を準備出来なかった場合の返戻
						if(!Cap) return {};
						Values[j] = *Cap;
					}
					double IntervalCap = *std::min_element(Values.begin(), Values.end());
					// 検出した谷底だけを追加探索する，任意の微小な窪地の不存在を保証する物ではない
					for(unsigned j = 1; j < 4; ++j) {
						if(!IntervalCap || Values[j] >= Values[j - 1] || Values[j] >= Values[j + 1]) continue;
						double Left = (k + 0.25 * (j - 1)) / RiverLongitudinalProfile::Segments;
						double Right = (k + 0.25 * (j + 1)) / RiverLongitudinalProfile::Segments;
						constexpr double Ratio = 0.6180339887498948482;
						double A = Right - Ratio * (Right - Left), B = Left + Ratio * (Right - Left);
						std::optional<double> CA = SampleCap(A), CB = SampleCap(B);
						// 探索を開始出来なかった場合の返戻
						if(!CA || !CB) return {};
						for(unsigned Step = 0; Step < 16; ++Step) {
							IntervalCap = std::min({ IntervalCap, *CA, *CB });
							if(*CA < *CB) {
								Right = B;
								B = A;
								CB = CA;
								A = Right - Ratio * (Right - Left);
								CA = SampleCap(A);
							} else {
								Left = A;
								A = B;
								CA = CB;
								B = Left + Ratio * (Right - Left);
								CB = SampleCap(B);
							}
							// 探索途中で中断又は不正値を検出した場合の返戻
							if(!CA || !CB) return {};
						}
						IntervalCap = std::min({ IntervalCap, *CA, *CB });
					}
					// 単調補間の両端を区間内の上限以下へ置き，補間の途中で上限を越えない様にする
					Previous = std::min(Previous, IntervalCap);
					P.Headroom[k] = Previous;
				}
				Headroom[i] = P.Headroom.front();
				P.Headroom.back() = Previous;
			}
			if(!Graph[Down].Ocean) Headroom[Down] = std::min(Headroom[Down], Previous);
		}
		for(const std::uint32_t i : Order) {
			if(Graph[i].Ocean) continue;
			N.Graph[i].WaterMeters = Grade[i] + Headroom[i];
			// 更新した合流水位が下流へ下降しない場合の返戻
			if(!(N.Graph[i].WaterMeters > N.Graph[Graph[i].Downstream].WaterMeters)) return {};
			N.Stats.MaximumProfileWaterLoweringMeters =
			std::max(N.Stats.MaximumProfileWaterLoweringMeters, Graph[i].WaterMeters - N.Graph[i].WaterMeters);
		}
		for(std::size_t i = 0; i < Rivers.size(); ++i) {
			// 準備が中断された場合の返戻
			if(Stop.stop_requested()) return {};
			const RiverReach &R = Rivers[i];
			RiverLongitudinalProfile &P = N.Longitudinal[i];
			P.Headroom.front() = Headroom[R.Upstream];
			P.Headroom.back() = Headroom[R.Downstream];
			P.StartWater = N.Graph[R.Upstream].WaterMeters;
			P.EndWater = N.Graph[R.Downstream].WaterMeters;
			// 等間隔の PCHIP に調和平均の内部微分を使う，合流点の余裕量の微分は０とし，実水位には厳密な下り勾配を残す
			// 区間内部を含め，三次補間による水位の逆転を許さない
			for(unsigned k = 1; k < RiverLongitudinalProfile::Segments; ++k) {
				const double A = P.Headroom[k] - P.Headroom[k - 1], B = P.Headroom[k + 1] - P.Headroom[k];
				// 余裕量が下流へ増える不正な区間の返戻
				if(A > 0 || B > 0) return {};
				P.Derivative[k] = A < 0 && B < 0 ? RiverLongitudinalProfile::Segments * 2 * A / (1 + A / B) : 0;
			}
			N.Stats.ProfileKnots += RiverLongitudinalProfile::Segments + 1;
			// 準備済みの標高標本を谷幅の算出へ再利用する，開削量だけを残し，地形の再評価を避けて一時配列を解放する
			for(unsigned k = 0; k <= RiverLongitudinalProfile::Segments; ++k) {
				const RiverHeights Heights = P.Evaluate(static_cast<double>(k) / RiverLongitudinalProfile::Segments);
				const double Gap = Heights.BedMeters - TerrainHeights[i][k];
				const double OriginalWater = SampleHeights(i, static_cast<double>(k) / RiverLongitudinalProfile::Segments)->WaterMeters;
				N.Stats.MaximumProfileWaterLoweringMeters =
				std::max(N.Stats.MaximumProfileWaterLoweringMeters, OriginalWater - Heights.WaterMeters);
				N.Stats.MaximumSampledWaterAboveTerrainMeters =
				std::max(N.Stats.MaximumSampledWaterAboveTerrainMeters, Heights.WaterMeters - TerrainHeights[i][k]);
				P.MaximumSampledBedCutMeters = std::max(P.MaximumSampledBedCutMeters, -Gap);
				N.Stats.MaximumUnresolvedKnotBedGapMeters = std::max(N.Stats.MaximumUnresolvedKnotBedGapMeters, Gap);
			}
		}
		// 更新した地形に対する統計を再計算する，面積重みには累積集水面積でなく各地点の局所面積を使う
		std::vector<double> Area(Count);
		for(std::size_t i = 0; i < Count; ++i) Area[i] = Graph[i].ContributingAreaSquareMeters;
		for(std::size_t i = 0; i < Count; ++i) if(!Graph[i].Ocean) Area[Graph[i].Downstream] -= Graph[i].ContributingAreaSquareMeters;
		N.Stats.MaximumRequiredBankRaiseMeters = 0;
		N.Stats.MaximumRequiredWaterCutMeters = 0;
		N.Stats.AreaWeightedBankRaiseMeters = 0;
		for(std::size_t i = 0; i < Count; ++i) if(!Graph[i].Ocean) {
			const DrainageNode &A = N.Graph[i];
			const double Raise = std::max(0.0, A.WaterMeters + S.BankFreeboardMeters - A.OriginalElevationMeters);
			N.Stats.MaximumRequiredBankRaiseMeters = std::max(N.Stats.MaximumRequiredBankRaiseMeters, Raise);
			N.Stats.MaximumRequiredWaterCutMeters =
			std::max(N.Stats.MaximumRequiredWaterCutMeters, A.OriginalElevationMeters - S.BankFreeboardMeters - A.WaterMeters);
			N.Stats.AreaWeightedBankRaiseMeters += Raise * std::max(0.0, Area[i]);
		}
		if(N.Stats.LandAreaSquareMeters > 0) N.Stats.AreaWeightedBankRaiseMeters /= N.Stats.LandAreaSquareMeters;
		// 準備した固定排水と水位の返戻
		return N;
	}
}
