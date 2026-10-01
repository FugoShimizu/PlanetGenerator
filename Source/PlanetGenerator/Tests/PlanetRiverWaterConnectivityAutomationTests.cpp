#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetRiverTerrain.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetRiverDetachedWater, "PlanetGenerator.RiverWater.DetachedWater",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 実描画で見付かった小水域を連続な場から再標本化し，河道への接続を検査する
 * @param Parameters 未使用の検査引数
 * @return 局所の閉鎖水域が検出されなかった場合に真，全域の連結保証とは別の回帰検査
 */
bool FPlanetRiverDetachedWater::RunTest(const FString &Parameters) {
	(void)Parameters;
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000;
	std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
	// 地質を準備出来ない場合の返戻
	if(!Geology) return false;
	std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(std::move(*Geology)));
	// 海陸を準備出来ない場合の返戻
	if(!PreparedOcean) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*PreparedOcean));
	std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 排水を準備出来ない場合の返戻
	if(!PreparedDrainage) return false;
	const std::optional<PlanetCore::RiverTerrainField> Field = PlanetCore::RiverTerrainField::CreateWithErosion(
		Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(std::move(*PreparedDrainage)));
	// 実描画と同じ侵食地形を準備出来ない場合の返戻
	if(!Field) return false;
	const PlanetCore::UnitDirection Center =
		*PlanetCore::UnitDirection::FromVector({901.2986062224002, 429.1526340361705, -86.98759298481397});
	const PlanetCore::Vec3 Normal = Center.Vector();
	const PlanetCore::Vec3 U = PlanetCore::UnitDirection::FromVector(Normal.Cross({0, 0, 1}))->Vector();
	const PlanetCore::Vec3 V = Normal.Cross(U);
	const PlanetCore::RiverSurfaceSample Surface = Field->EvaluateSurface(Center);
	const double Original = Field->GetErosionPreparation()->GetAeolianField()->EvaluateElevation(Center);
	AddInfo(FString::Printf(TEXT("Detached source: original=%.17g H=%.17g W=%.17g channel=%.9g support=%.9g"), Original,
		Surface.Terrain.ElevationMeters, Surface.WaterElevationMeters, Surface.ChannelCoverage, Surface.WaterSupport));
	for(const PlanetCore::RiverTerrainChannel &Channel : Field->Channels()) {
		if(Normal.Dot(Channel.Center) <= 0) continue;
		const double X = Normal.Dot(Channel.Start), Y = Normal.Dot(Channel.Tangent), Angle = std::atan2(Y, X);
		const PlanetCore::Vec3 Closest = Angle > 0 && Angle < Channel.ArcAngle
			? (Channel.Start * X + Channel.Tangent * Y) * (1 / std::sqrt(X * X + Y * Y))
			: (Angle <= 0 ? Channel.Start : Channel.End);
		const double Distance = (Normal - Closest).Length();
		if(Distance * Distance >= Channel.OuterChordSquared) continue;
		const PlanetCore::RiverHeights Profile = *Field->Drainage().SampleHeights(Channel.ReachIndex, Angle / Channel.ArcAngle);
		AddInfo(FString::Printf(TEXT("Detached contributor: reach=%u fraction=%.9g distance=%.9g inner=%.9g "
									 "outer=%.9g q=%.9g W=%.17g bed=%.17g"),
			Channel.ReachIndex, Angle / Channel.ArcAngle, Distance * Settings.RadiusMeters, Channel.InnerHalfWidthMeters,
			Channel.OuterHalfWidthMeters, Distance * Distance / Channel.InnerChordSquared, Profile.WaterMeters,
			Profile.BedMeters));
	}
	for(const std::uint32_t HalfCells : {60u, 120u}) {
		const std::uint32_t Side = 2 * HalfCells + 1;
		const double Step = 15.0 / HalfCells;
		std::vector<double> Depths;
		Depths.reserve(Side * Side);
		double MaximumOutsideBankOverflow = 0;
		for(std::uint32_t Row = 0; Row < Side; ++Row)
			for(std::uint32_t Column = 0; Column < Side; ++Column) {
				const double A = (static_cast<double>(Column) - HalfCells) * Step;
				const double B = (static_cast<double>(Row) - HalfCells) * Step;
				const PlanetCore::UnitDirection Direction =
					*PlanetCore::UnitDirection::FromVector(Normal * Settings.RadiusMeters + U * A + V * B);
				const PlanetCore::RiverSurfaceSample Sample = Field->EvaluateSurface(Direction);
				const double Water = Sample.WaterSupport > 0 ? Sample.WaterElevationMeters : Settings.SeaLevelMeters;
				Depths.push_back(Water - Sample.Terrain.ElevationMeters);
				if(Sample.ChannelCoverage == 0) {
					const double BaseHeight = Field->GetErosionPreparation()->GetAeolianField()->EvaluateElevation(Direction);
					if(BaseHeight > Settings.SeaLevelMeters)
						MaximumOutsideBankOverflow = std::max(MaximumOutsideBankOverflow, Water - BaseHeight);
				}
			}
		const std::uint32_t Start = HalfCells * Side + HalfCells;
		std::deque<std::uint32_t> Pending;
		std::vector<bool> Visited(Depths.size(), false);
		if(Depths[Start] > 0) {
			Pending.push_back(Start);
			Visited[Start] = true;
		}
		bool Boundary = false;
		std::uint32_t Count = 0;
		double MaximumDepth = 0;
		while(!Pending.empty()) {
			const std::uint32_t Index = Pending.front();
			Pending.pop_front();
			const std::uint32_t Row = Index / Side, Column = Index % Side;
			Boundary |= Row == 0 || Column == 0 || Row == Side - 1 || Column == Side - 1;
			MaximumDepth = std::max(MaximumDepth, Depths[Index]);
			++Count;
			for(const std::uint32_t Neighbor : {Row ? Index - Side : Index, Row + 1 < Side ? Index + Side : Index,
					Column ? Index - 1 : Index, Column + 1 < Side ? Index + 1 : Index})
				if(!Visited[Neighbor] && Depths[Neighbor] > 0) {
					Visited[Neighbor] = true;
					Pending.push_back(Neighbor);
				}
		}
		// 区域内の主河道1658の中点を，独立な排水の縦断から求める
		const PlanetCore::Vec3 River = Field->Drainage().SampleReach(1658, 0.5)->Direction;
		const double Scale = Settings.RadiusMeters / River.Dot(Normal);
		const double RiverX = HalfCells + River.Dot(U) * Scale / Step;
		const double RiverY = HalfCells + River.Dot(V) * Scale / Step;
		// 再現領域から主河道が外れた場合は検査前提の失敗として返戻
		if(!TestTrue(TEXT("Reference river remains inside the diagnostic patch"),
			   RiverX >= 0 && RiverY >= 0 && RiverX < Side && RiverY < Side))
			return false;
		const std::uint32_t RiverIndex =
			static_cast<std::uint32_t>(std::round(RiverY)) * Side + static_cast<std::uint32_t>(std::round(RiverX));
		const bool Connected = Visited[RiverIndex];
		std::vector<bool> Classified(Depths.size(), false);
		std::uint32_t DetachedComponents = 0;
		for(std::uint32_t Seed = 0; Seed < Depths.size(); ++Seed) {
			if(Classified[Seed] || Depths[Seed] <= 0) continue;
			Pending.push_back(Seed);
			Classified[Seed] = true;
			bool ReachesBoundary = false, ReachesRiver = false;
			while(!Pending.empty()) {
				const std::uint32_t Index = Pending.front();
				Pending.pop_front();
				const std::uint32_t Row = Index / Side, Column = Index % Side;
				ReachesBoundary |= Row == 0 || Column == 0 || Row == Side - 1 || Column == Side - 1;
				ReachesRiver |= Index == RiverIndex;
				for(const std::uint32_t Neighbor : {Row ? Index - Side : Index, Row + 1 < Side ? Index + Side : Index,
						Column ? Index - 1 : Index, Column + 1 < Side ? Index + 1 : Index})
					if(!Classified[Neighbor] && Depths[Neighbor] > 0) {
						Classified[Neighbor] = true;
						Pending.push_back(Neighbor);
					}
			}
			if(!ReachesBoundary && !ReachesRiver) ++DetachedComponents;
		}
		TestEqual(TEXT("No detached wet component elsewhere in the sampled patch"), DetachedComponents, 0u);
		TestTrue(TEXT("Outside banks the water ceiling does not exceed original land"), MaximumOutsideBankOverflow <= 1E-10);
		AddInfo(FString::Printf(TEXT("Detached patch: spacing=%.9g closedComponents=%u outsideBankOverflow=%.9g"), Step,
			DetachedComponents, MaximumOutsideBankOverflow));
		AddInfo(FString::Printf(TEXT("Detached grid: spacing=%.9g samples=%llu component=%u maxDepth=%.9g "
									 "boundary=%d referenceRiver=%d centerDepth=%.9g"),
			Step, static_cast<unsigned long long>(Depths.size()), Count, MaximumDepth, Boundary, Connected, Depths[Start]));
		// 領域端へ続く場合は局所的な閉鎖を確定出来ない為，別の接続検査が必要
		TestTrue(TEXT("Known wet region is dry or connected to the reference river, without relying on patch escape"),
			!Count || Connected);
	}
	// 検出した接続不良を含む結果の返戻
	return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFinalSeaGrid, "PlanetGenerator.RiverWater.GlobalSeaGrid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 固定全球格子で最終 H の水没成分を列挙し，元の大洋経路が埋まらない事を検査する
 * @param Parameters 未使用の検査引数
 * @return 全球格子上の大洋接続を保つ場合に真．格子間の全点保証とは区別する
 */
bool FPlanetFinalSeaGrid::RunTest(const FString &Parameters) {
	(void)Parameters;
	for(const std::uint64_t Seed : {7ULL, 42ULL, 2026ULL}) {
		for(const double RadiusMeters : {1000.0, 1000000.0, 6371000.0}) {
			AddInfo(FString::Printf(TEXT("Global sea case seed=%llu radius=%.0f"),
				static_cast<unsigned long long>(Seed), RadiusMeters));
			PlanetCore::GeologySettings Settings;
			Settings.Seed = Seed;
			Settings.RadiusMeters = RadiusMeters;
			const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
			if(!TestTrue(TEXT("全球水域用の地質を準備"), Geology.has_value())) return false;
			const std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
				PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
			if(!TestTrue(TEXT("全球水域用の大洋を準備"), PreparedOcean.has_value())) return false;
			const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
				std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedOcean);
			const std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
			if(!TestTrue(TEXT("全球水域用の排水を準備"), PreparedDrainage.has_value())) return false;
			const std::optional<PlanetCore::RiverTerrainField> Terrain = PlanetCore::RiverTerrainField::CreateWithErosion(
				Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedDrainage));
			if(!TestTrue(TEXT("侵食と河道を含む最終地形を準備"), Terrain.has_value())) return false;
			const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
			if(!TestTrue(TEXT("全球の固定三角格子を準備"), Mesh.has_value())) return false;
			const std::uint32_t Count = static_cast<std::uint32_t>(Mesh->Directions.size());
			std::vector<std::uint32_t> Parent(Count), ComponentSizes(Count, 0);
			std::vector<std::uint8_t> Wet(Count, 0);
			std::uint32_t OriginalWet = 0, FinalWet = 0, ClosedOriginal = 0;
			for(std::uint32_t i = 0; i < Count; ++i) {
				Parent[i] = i;
				const PlanetCore::UnitDirection Direction = Mesh->Directions[i];
				const bool WasWet = Ocean->EvaluateElevation(Direction) < Settings.SeaLevelMeters;
				const bool IsWet = Terrain->EvaluateElevation(Direction) < Settings.SeaLevelMeters;
				OriginalWet += WasWet ? 1u : 0u;
				FinalWet += IsWet ? 1u : 0u;
				ClosedOriginal += WasWet && !IsWet ? 1u : 0u;
				Wet[i] = IsWet ? 1 : 0;
			}
			const auto Root = [&Parent](std::uint32_t Index) {
				while(Parent[Index] != Index) {
					Parent[Index] = Parent[Parent[Index]];
					Index = Parent[Index];
				}
				// 経路圧縮後の親を返す
				return Index;
			};
			for(const std::array<std::uint32_t, 3> Triangle : Mesh->Triangles) {
				for(unsigned Edge = 0; Edge < 3; ++Edge) {
					const std::uint32_t A = Triangle[Edge], B = Triangle[(Edge + 1) % 3];
					if(!Wet[A] || !Wet[B]) continue;
					const std::uint32_t ARoot = Root(A), BRoot = Root(B);
					if(ARoot != BRoot) Parent[BRoot] = ARoot;
				}
			}
			std::uint32_t Components = 0, Largest = 0, LargestRoot = 0;
			for(std::uint32_t i = 0; i < Count; ++i) {
				if(!Wet[i]) continue;
				const std::uint32_t Index = Root(i);
				const std::uint32_t Size = ++ComponentSizes[Index];
				if(Size == 1) ++Components;
				if(Size > Largest) { Largest = Size; LargestRoot = Index; }
			}
			if(Components > 1) {
				for(std::uint32_t i = 0; i < Count; ++i) {
					if(!Wet[i] || Root(i) == LargestRoot) continue;
					const PlanetCore::Vec3 D = Mesh->Directions[i].Vector();
					AddInfo(FString::Printf(TEXT("Detached final sea node=%u component=%u direction=(%.9g,%.9g,%.9g)"),
						i, ComponentSizes[Root(i)], D.X, D.Y, D.Z));
					break;
				}
			}
			AddInfo(FString::Printf(TEXT("Final sea grid: nodes=%u originalWet=%u finalWet=%u ")
				TEXT("closedOriginal=%u components=%u largest=%u detached=%u"),
				Count, OriginalWet, FinalWet, ClosedOriginal, Components, Largest, FinalWet - Largest));
			TestTrue(TEXT("全球格子で海域を検出"), FinalWet > 0);
			TestEqual(TEXT("既存の大洋経路を最終 H が埋めない"), ClosedOriginal, 0u);
			TestEqual(TEXT("全球格子の最終 H の水没成分は一つ"), Components, 1u);
		}
	}
	// 有限解像度の検査結果の返戻
	return !HasAnyErrors();
}
#endif
