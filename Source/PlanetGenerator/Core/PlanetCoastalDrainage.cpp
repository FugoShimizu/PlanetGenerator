#include "PlanetDrainage.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace PlanetCore {
	/**
	 * 排水辺が最初に海へ入る地点を流出口にし，面積・下流参照・水位を再計算する
	 * @param Height 接続済みの大洋を含む連続な標高関数
	 * @param Stop 中断要求
	 * @return 元の標本番号を保持して海岸の流出口だけを末尾へ追加した排水，不正又は中断時は空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::WithCoastalOutlets(
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop
	) const {
		// 二重の経路修正や未定義の入力に対する返戻
		if(!Height || Graph.empty() || Stats.CoastalOutletNodes || Stop.stop_requested()) return {};
		DrainageNetwork N = *this;
		N.Longitudinal.clear();
		N.Rivers.clear();
		N.CoastalMouths.clear();
		N.Stats = {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Graph.size());
		N.Graph.reserve(Count * 2);
		std::vector<double> Areas(Count);
		std::vector<std::optional<ShoreCrossing>> Shores(Count);
		std::vector<std::optional<RiverPlanform>> Curves(Definition.UseCurvedPlanforms ? Count : 0);
		std::vector<double> CurveEnds(Definition.UseCurvedPlanforms ? Count : 0, 1);
		std::vector<std::uint32_t> Order(Count);
		std::iota(Order.begin(), Order.end(), 0U);
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				// 元の木の下流から上流へ向かう確定順序の返戻
				return Graph[A].DistanceToOceanMeters != Graph[B].DistanceToOceanMeters ?
				Graph[A].DistanceToOceanMeters < Graph[B].DistanceToOceanMeters :
				A < B;
			}
		);
		for(std::uint32_t i = 0; i < Count; ++i) Areas[i] = Graph[i].ContributingAreaSquareMeters;
		for(const DrainageNode &Node : Graph) if(!Node.Ocean) Areas[Node.Downstream] -= Node.ContributingAreaSquareMeters;
		for(std::uint32_t i = 0; i < Count; ++i) {
			// 経路探索を中断した場合の返戻
			if(Stop.stop_requested()) return {};
			DrainageNode &Node = N.Graph[i];
			const double LocalArea = Areas[i];
			Node.ContributingAreaSquareMeters = Node.Ocean ? 0 : LocalArea;
			if(Node.Ocean) continue;
			// 元の標本に正の局所面積が無い場合の返戻
			if(!std::isfinite(LocalArea) || LocalArea <= 0) return {};
			bool HasFiniteHeights = true;
			const auto CheckedHeight = [&](const UnitDirection Direction) {
				const double Value = Height(Direction);
				HasFiniteHeights &= std::isfinite(Value);
				// 入力の有限性を記録した標高の返戻
				return Value;
			};
			const UnitDirection Start = *UnitDirection::FromVector(Node.Direction);
			const UnitDirection End = *UnitDirection::FromVector(Graph[Node.Downstream].Direction);
			if(Definition.UseCurvedPlanforms) {
				const double Width = 0.5 * std::min(256.0, 2 + 0.12 * std::sqrt(Graph[i].ContributingAreaSquareMeters / 1E6));
				const std::uint64_t Seed = Definition.PlanformSeed ^ static_cast<std::uint64_t>(i) << 32 ^ Graph[i].Downstream;
				Curves[i] = RiverPlanform::Create(Start, End, Definition.RadiusMeters, Width, Seed);
				// 準備出来ない曲線を直線へ置換えて隠さない返戻
				if(!Curves[i]) return {};
				const auto At = [&](const double T) {
					return Curves[i]->Evaluate(T).Direction;
				};
				Shores[i] = FindShoreCrossingOnCurve(
					At,
					Curves[i]->ArcRadians() * Definition.RadiusMeters,
					Curves[i]->MaximumTangentStretch(),
					Definition.RadiusMeters,
					Definition.SeaLevelMeters,
					CheckedHeight,
					Stop
				);
				if(Shores[i]) CurveEnds[i] = Shores[i]->WetFraction;
			} else Shores[i] = FindShoreCrossing(Start, End, Definition.RadiusMeters, Definition.SeaLevelMeters, CheckedHeight, Stop);
			// 元から海へ向かう辺の交差を失った場合や，評価の失敗を陸上辺と誤認しない返戻
			if(!HasFiniteHeights || Stop.stop_requested() || !Shores[i] && Graph[Node.Downstream].Ocean) return {};
			if(!Shores[i]) continue;
			const std::uint32_t Outlet = static_cast<std::uint32_t>(N.Graph.size());
			DrainageNode Coast;
			Coast.Direction = Shores[i]->Direction;
			Coast.OriginalElevationMeters = Definition.SeaLevelMeters + Shores[i]->TerrainResidualMeters;
			Coast.WaterMeters = Definition.SeaLevelMeters;
			Coast.Downstream = Coast.Outlet = Outlet;
			Coast.Ocean = true;
			Node.Downstream = Outlet;
			N.Graph.push_back(Coast);
			++N.Stats.CoastalOutletNodes;
			// 公開する辺自体を河口迄に短縮する為，挟込みの比率も其の辺へ換算する
			Shores[i]->LandFraction /= Shores[i]->WetFraction;
			Shores[i]->WetFraction = 1;
		}
		for(const std::uint32_t i : Order) {
			DrainageNode &Node = N.Graph[i];
			if(Node.Ocean) continue;
			const DrainageNode &Down = N.Graph[Node.Downstream];
			const double Length = Definition.RadiusMeters * (
				Definition.UseCurvedPlanforms ?
				Curves[i]->ArcRadians() * CurveEnds[i] :
				std::atan2(Node.Direction.Cross(Down.Direction).Length(), Node.Direction.Dot(Down.Direction))
			);
			// 海岸への辺が消滅した場合の返戻
			if(!std::isfinite(Length) || Length <= 0) return {};
			Node.DistanceToOceanMeters = Down.DistanceToOceanMeters + Length;
			Node.Outlet = Down.Outlet;
			const double Rise =
			std::nextafter(Down.WaterMeters + Definition.MinimumSlope * Length, std::numeric_limits<double>::infinity());
			Node.WaterMeters = std::max(Rise, std::min(Node.WaterMeters, Node.OriginalElevationMeters - Definition.BankFreeboardMeters));
		}
		for(std::vector<std::uint32_t>::reverse_iterator i = Order.rbegin(); i != Order.rend(); ++i) {
			const DrainageNode &Node = N.Graph[*i];
			if(!Node.Ocean) N.Graph[Node.Downstream].ContributingAreaSquareMeters += Node.ContributingAreaSquareMeters;
		}
		constexpr double Pi = 3.14159265358979323846;
		const double Threshold = 4 * Pi * Definition.RadiusMeters * Definition.RadiusMeters * Definition.ChannelAreaFraction;
		N.Stats.Nodes = static_cast<std::uint32_t>(N.Graph.size());
		for(std::uint32_t i = 0; i < N.Graph.size(); ++i) {
			// 集水の再計算を中断した場合の返戻
			if(Stop.stop_requested()) return {};
			DrainageNode &Node = N.Graph[i];
			Node.DepthMeters = std::min(12.0, 0.5 + 0.2 * std::log1p(Node.ContributingAreaSquareMeters / 1E6));
			if(Node.Ocean) {
				++N.Stats.OceanNodes;
				N.Stats.OutletAreaSquareMeters += Node.ContributingAreaSquareMeters;
				continue;
			}
			++N.Stats.LandNodes;
			N.Stats.LandAreaSquareMeters += Areas[i];
			N.Stats.LongestRouteMeters = std::max(N.Stats.LongestRouteMeters, Node.DistanceToOceanMeters);
			const double Raise = std::max(0.0, Node.WaterMeters + Definition.BankFreeboardMeters - Node.OriginalElevationMeters);
			N.Stats.MaximumRequiredBankRaiseMeters = std::max(N.Stats.MaximumRequiredBankRaiseMeters, Raise);
			N.Stats.AreaWeightedBankRaiseMeters += Raise * Areas[i];
			N.Stats.MaximumRequiredWaterCutMeters = std::max(
				N.Stats.MaximumRequiredWaterCutMeters,
				Node.OriginalElevationMeters - Definition.BankFreeboardMeters - Node.WaterMeters
			);
			if(Node.ContributingAreaSquareMeters < Threshold) continue;
			const DrainageNode &Down = N.Graph[Node.Downstream];
			const double Length = Definition.RadiusMeters * (
				Definition.UseCurvedPlanforms ?
				Curves[i]->ArcRadians() * CurveEnds[i] :
				std::atan2(Node.Direction.Cross(Down.Direction).Length(), Node.Direction.Dot(Down.Direction))
			);
			const std::uint32_t Reach = static_cast<std::uint32_t>(N.Rivers.size());
			N.Rivers.push_back({ i, Node.Downstream, Length, 1, Down.Direction });
			if(Definition.UseCurvedPlanforms) {
				N.Rivers.back().Planform = Curves[i];
				N.Rivers.back().PlanformEndFraction = CurveEnds[i];
			}
			if(Shores[i]) {
				N.CoastalMouths.push_back({ Reach, *Shores[i] });
				N.Stats.MaximumMouthBracketWidthMeters = std::max(N.Stats.MaximumMouthBracketWidthMeters, Shores[i]->BracketWidthMeters);
				N.Stats.MaximumMouthHeightResidualMeters =
				std::max(N.Stats.MaximumMouthHeightResidualMeters, std::abs(Shores[i]->TerrainResidualMeters));
			}
		}
		if(N.Stats.LandAreaSquareMeters > 0) N.Stats.AreaWeightedBankRaiseMeters /= N.Stats.LandAreaSquareMeters;
		N.Stats.Reaches = static_cast<std::uint32_t>(N.Rivers.size());
		// 最初の海岸で終端し，流量と侵食も同じ流出口を参照する排水の返戻
		return N;
	}
}
