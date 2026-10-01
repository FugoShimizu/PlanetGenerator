#include "PlanetDrainage.h"
#include "PlanetDrainageFit.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <numeric>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 河道内部を含む一つの排水木で開削・盛土を配分し，合流水位を共有する
	 * 計算量：O(n log² n)，記憶量：O(n)，n は陸上地点と河道内部標本の合計
	 * @param Height 河岸を含む再構成前の標高，元の海域境界を維持する
	 * @param Stop 中断要求
	 * @return 下降する共有縦断，入力不正又は中断時は空，盛土を含む地形合成が別途必要
	 */
	std::optional<DrainageNetwork> DrainageNetwork::WithBalancedTerrainProfiles(
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop
	) const {
		// 元の固定排水又は標高が無い場合の返戻
		if(!Height || Graph.empty() || Stop.stop_requested()) return {};
		constexpr std::uint32_t Missing = NoDrainageFitDownstream, Segments = RiverLongitudinalProfile::Segments;
		const std::size_t Count = Graph.size();
		// 細分化後の標本番号が表現範囲を越える場合の返戻
		if(Count >= Missing || Rivers.size() > (Missing - Count) / (Segments - 1)) return {};
		DrainageNetwork Result = *this;
		Result.BalancedProfiles = true;
		Result.Longitudinal.assign(Rivers.size(), {});
		Result.Stats.ProfileKnots = 0;
		Result.Stats.ProfileTerrainEvaluations = 0;
		Result.Stats.MaximumUnresolvedKnotBedGapMeters = 0;
		Result.Stats.MaximumSampledWaterAboveTerrainMeters = 0;
		Result.Stats.MaximumProfileWaterLoweringMeters = 0;
		Result.Stats.MaximumRequiredBankRaiseMeters = 0;
		Result.Stats.MaximumRequiredWaterCutMeters = 0;
		Result.Stats.AreaWeightedBankRaiseMeters = 0;
		std::vector<std::uint32_t> Order(Count), Selected(Count, Missing), Mapping(Count, Missing), Interior(Rivers.size());
		std::vector<double> Distance(Count), Length(Count), Area(Count);
		std::vector<DrainageFitSite> Sites;
		Sites.reserve(Count + Rivers.size() * (Segments - 1));
		std::iota(Order.begin(), Order.end(), 0U);
		// 元の排水距離で下流を先に処理し，河口の切詰め後の距離を再構成する
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				const double X = Graph[A].DistanceToOceanMeters, Y = Graph[B].DistanceToOceanMeters;
				// 同距離では地点番号に依る順序の返戻
				return X != Y ? X < Y : A < B;
			}
		);
		for(std::uint32_t Index = 0; Index < Rivers.size(); ++Index) Selected[Rivers[Index].Upstream] = Index;
		// 海岸で余裕高を０へ近付ける，既存水位を上限にせず開削と盛土を共に許す
		const auto Target = [&](const double Elevation) {
			const double Relief = std::max(0.0, Elevation - Definition.SeaLevelMeters);
			// 余裕高０と海面一致を含む正規入力の返戻
			if(!Relief) return Definition.SeaLevelMeters;
			// 固定海面に連続な目標水位の返戻
			return Definition.SeaLevelMeters + Relief * (Relief / (Relief + Definition.BankFreeboardMeters));
		};
		for(const std::uint32_t Index : Order) {
			// 地点準備中の中断時の返戻
			if(Stop.stop_requested()) return {};
			DrainageNode &Node = Result.Graph[Index];
			Area[Index] = Node.ContributingAreaSquareMeters;
			if(Node.Ocean) continue;
			const DrainageNode &Down = Graph[Node.Downstream];
			const Vec3 A = Node.Direction, B = Down.Direction;
			Length[Index] = Selected[Index] == Missing ?
			std::atan2(A.Cross(B).Length(), A.Dot(B)) * Definition.RadiusMeters :
			Rivers[Selected[Index]].ProfileLengthMeters();
			Distance[Index] = Distance[Node.Downstream] + Length[Index];
			Node.OriginalElevationMeters = Height(*UnitDirection::FromVector(Node.Direction));
			++Result.Stats.ProfileTerrainEvaluations;
			// 正の実長と有限な地形を用意出来ない場合の返戻
			if(!std::isfinite(Node.OriginalElevationMeters) || !std::isfinite(Distance[Index]) || Length[Index] <= 0) return {};
			Mapping[Index] = static_cast<std::uint32_t>(Sites.size());
			Sites.push_back({ Missing, Target(Node.OriginalElevationMeters) - Definition.MinimumSlope * Distance[Index], 0 });
		}
		// 陸上地点が無い場合には再構成対象が無い為の返戻
		if(Sites.empty()) return {};
		for(std::uint32_t Index = 0; Index < Rivers.size(); ++Index) {
			Interior[Index] = static_cast<std::uint32_t>(Sites.size());
			Sites.resize(Sites.size() + Segments - 1);
		}
		// 一区間の半分ずつを端点へ配分し，支流の合流点を重複した未知数にしない
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(Graph[Index].Ocean) continue;
			const std::uint32_t Down = Mapping[Graph[Index].Downstream], Reach = Selected[Index];
			const double Step = Length[Index] / (Reach == Missing ? 1 : Segments);
			DrainageFitSite &Site = Sites[Mapping[Index]];
			Site.Downstream = Reach == Missing ? Down : Interior[Reach];
			Site.Weight += Step * 0.5;
			if(Down != Missing) Sites[Down].Weight += Step * 0.5;
			Area[Graph[Index].Downstream] -= Graph[Index].ContributingAreaSquareMeters;
			if(Reach == Missing) continue;
			for(std::uint32_t Knot = 0; Knot < Segments - 1; ++Knot) {
				DrainageFitSite &Inside = Sites[Interior[Reach] + Knot];
				Inside.Downstream = Knot + 1 == Segments - 1 ? Down : Interior[Reach] + Knot + 1;
				Inside.Weight = Step;
			}
		}
		std::vector<std::array<double, Segments + 1>> TerrainHeights(Rivers.size());
		for(std::uint32_t Index = 0; Index < Rivers.size(); ++Index) {
			const RiverReach &Reach = Rivers[Index];
			RiverLongitudinalProfile &Profile = Result.Longitudinal[Index];
			Profile.GradeStart = Definition.SeaLevelMeters + Definition.MinimumSlope * Distance[Reach.Upstream];
			Profile.GradeEnd = Definition.SeaLevelMeters + Definition.MinimumSlope * Distance[Reach.Downstream];
			Profile.LengthMeters = Reach.ProfileLengthMeters();
			Profile.StartDepth = Graph[Reach.Upstream].DepthMeters;
			Profile.EndDepth = Graph[Reach.Downstream].DepthMeters;
			const std::optional<RiverCrossSectionWidths> Widths = ChannelWidths(Index);
			const std::optional<RiverProfileSample> Start = SampleReach(Index, 0);
			// 河岸の位置を算出出来ない場合の返戻
			if(!Widths || !Start) return {};
			const std::optional<UnitDirection> Across = UnitDirection::FromVector(Start->Direction.Cross(Start->DownstreamTangent));
			// 横断方向を算出出来ない場合の返戻
			if(!Across) return {};
			const double BankAngle = Widths->InnerHalfWidthMeters / Definition.RadiusMeters, BankCosine = std::cos(BankAngle);
			const double BankSine = std::sin(BankAngle);
			for(std::uint32_t Knot = 0; Knot <= Segments; ++Knot) {
				// 河道内部の準備中の中断時の返戻
				if(Stop.stop_requested()) return {};
				const double T = static_cast<double>(Knot) / Segments;
				const std::optional<RiverProfileSample> Sample = SampleReach(Index, T);
				// 河道内部の方向を取得出来ない場合の返戻
				if(!Sample) return {};
				const double H = Height(*UnitDirection::FromVector(Sample->Direction));
				++Result.Stats.ProfileTerrainEvaluations;
				// 不正な河床標高を拒否する返戻
				if(!std::isfinite(H)) return {};
				TerrainHeights[Index][Knot] = H;
				const std::uint32_t SiteIndex =
				!Knot ? Mapping[Reach.Upstream] : Knot == Segments ? Mapping[Reach.Downstream] : Interior[Index] + Knot - 1;
				if(SiteIndex == Missing) continue;
				double BankHeight = H;
				for(const double Side : { -1.0, 1.0 }) {
					const UnitDirection Bank = *UnitDirection::FromVector(Sample->Direction * BankCosine + Across->Vector() * (Side * BankSine));
					const double BankH = Height(Bank);
					++Result.Stats.ProfileTerrainEvaluations;
					// 非有限な河岸標高を拒否する返戻
					if(!std::isfinite(BankH)) return {};
					BankHeight = std::min(BankHeight, BankH);
				}
				const double Remaining = std::lerp(Distance[Reach.Upstream], Distance[Reach.Downstream], T);
				const double Value = Target(BankHeight) - Definition.MinimumSlope * Remaining;
				DrainageFitSite &Site = Sites[SiteIndex];
				Site.TargetMeters = !Knot || Knot == Segments ? std::min(Site.TargetMeters, Value) : Value;
			}
		}
		const std::optional<DrainageFitResult> Fitted = FitDrainageTree(Sites, Definition.SeaLevelMeters, Stop);
		// 全河道と共有合流の同時最適化に失敗した場合の返戻
		if(!Fitted) return {};
		for(const std::uint32_t Index : Order) {
			DrainageNode &Node = Result.Graph[Index];
			if(Node.Ocean) continue;
			Node.WaterMeters = Fitted->Levels[Mapping[Index]] + Definition.MinimumSlope * Distance[Index];
			// 丸めを含め，下降する水位を表現出来ない場合の返戻
			if(!(Node.WaterMeters > Result.Graph[Node.Downstream].WaterMeters)) return {};
			const double Raise = std::max(0.0, Node.WaterMeters + Definition.BankFreeboardMeters - Node.OriginalElevationMeters);
			Result.Stats.MaximumRequiredBankRaiseMeters = std::max(Result.Stats.MaximumRequiredBankRaiseMeters, Raise);
			Result.Stats.AreaWeightedBankRaiseMeters += Raise * std::max(0.0, Area[Index]);
			Result.Stats.MaximumRequiredWaterCutMeters = std::max(
				Result.Stats.MaximumRequiredWaterCutMeters,
				Node.OriginalElevationMeters - Definition.BankFreeboardMeters - Node.WaterMeters
			);
			Result.Stats.MaximumProfileWaterLoweringMeters =
			std::max(Result.Stats.MaximumProfileWaterLoweringMeters, Graph[Index].WaterMeters - Node.WaterMeters);
		}
		if(Result.Stats.LandAreaSquareMeters > 0) Result.Stats.AreaWeightedBankRaiseMeters /= Result.Stats.LandAreaSquareMeters;
		for(std::uint32_t Index = 0; Index < Rivers.size(); ++Index) {
			// 補間準備中の中断時の返戻
			if(Stop.stop_requested()) return {};
			const RiverReach &Reach = Rivers[Index];
			RiverLongitudinalProfile &Profile = Result.Longitudinal[Index];
			Profile.StartWater = Result.Graph[Reach.Upstream].WaterMeters;
			Profile.EndWater = Result.Graph[Reach.Downstream].WaterMeters;
			for(std::uint32_t Knot = 0; Knot <= Segments; ++Knot) {
				const std::uint32_t SiteIndex =
				!Knot ? Mapping[Reach.Upstream] : Knot == Segments ? Mapping[Reach.Downstream] : Interior[Index] + Knot - 1;
				Profile.Headroom[Knot] = SiteIndex == Missing ? 0 : Fitted->Levels[SiteIndex] - Definition.SeaLevelMeters;
			}
			for(std::uint32_t Knot = 1; Knot < Segments; ++Knot) {
				const double A = Profile.Headroom[Knot] - Profile.Headroom[Knot - 1], B = Profile.Headroom[Knot + 1] - Profile.Headroom[Knot];
				// 順序を失った補間節点を拒否する返戻
				if(A > 0 || B > 0) return {};
				Profile.Derivative[Knot] = A < 0 && B < 0 ? Segments * 2 * A / (1 + A / B) : 0;
			}
			Result.Stats.ProfileKnots += Segments + 1;
			for(std::uint32_t Knot = 0; Knot <= Segments; ++Knot) {
				const RiverHeights Sample = Profile.Evaluate(static_cast<double>(Knot) / Segments);
				const double Gap = Sample.BedMeters - TerrainHeights[Index][Knot];
				Profile.MaximumSampledBedCutMeters = std::max(Profile.MaximumSampledBedCutMeters, -Gap);
				Result.Stats.MaximumUnresolvedKnotBedGapMeters = std::max(Result.Stats.MaximumUnresolvedKnotBedGapMeters, Gap);
				Result.Stats.MaximumSampledWaterAboveTerrainMeters =
				std::max(Result.Stats.MaximumSampledWaterAboveTerrainMeters, Sample.WaterMeters - TerrainHeights[Index][Knot]);
				Result.Stats.MaximumProfileWaterLoweringMeters = std::max(
					Result.Stats.MaximumProfileWaterLoweringMeters,
					SampleHeights(Index, static_cast<double>(Knot) / Segments)->WaterMeters - Sample.WaterMeters
				);
			}
		}
		// 地形側の盛土再構成を必要とする共有縦断の返戻
		return Result;
	}
}
