#include "Core/PlanetDrainage.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSphericalRefinement.h"
#include "Core/PlanetSphericalSpline.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	using PlanetCore::DrainageNetwork;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::PlanetSphericalRefinement;
	using PlanetCore::SphericalReliefSamplingMesh;
	using PlanetCore::SphericalReliefSettings;
	using PlanetCore::SphericalSplineField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;

	/**
	 * 追加海峡の中心線・再準備・入力拒否を，実際の公開場で検査する
	 * @param Test 検査結果
	 * @param Original 変更前の海域
	 * @param Connected 同じ網を再接続した場
	 * @param Mesh 検査する適応網
	 * @return 再接続が既存地形を上げず，公開した契約を守る場合に真
	 */
	bool CheckSamplingOcean(
		FAutomationTestBase &Test,
		const OceanConnectivityField &Original,
		const OceanConnectivityField &Connected,
		const SphericalReliefSamplingMesh &Mesh
	) {
		// 公開入力の範囲・向き・中断の検査
		std::stop_source Cancelled;
		Cancelled.request_stop();
		Test.TestFalse(
			TEXT("中断した適応海域の拒否"),
			Original.WithSamplingMesh(Mesh.Directions, Mesh.Triangles, Cancelled.get_token()).has_value()
		);
		Test.TestFalse(
			TEXT("面の欠落の拒否"),
			Original.WithSamplingMesh(Mesh.Directions, std::span(Mesh.Triangles).first(Mesh.Triangles.size() - 1)).has_value()
		);
		std::vector<std::array<std::uint32_t, 3>> Invalid = Mesh.Triangles;
		Invalid[0][0] = static_cast<std::uint32_t>(Mesh.Directions.size());
		Test.TestFalse(TEXT("頂点番号の範囲外の拒否"), Original.WithSamplingMesh(Mesh.Directions, Invalid).has_value());
		Invalid[0] = Mesh.Triangles[0];
		std::swap(Invalid[0][0], Invalid[0][1]);
		Test.TestFalse(TEXT("裏向きの面の拒否"), Original.WithSamplingMesh(Mesh.Directions, Invalid).has_value());
		Invalid[0] = Mesh.Triangles[1];
		Test.TestFalse(TEXT("重複した面の拒否"), Original.WithSamplingMesh(Mesh.Directions, Invalid).has_value());
		// 既存索引の内容を残さず，同じ網で再準備しても場を変えない検査
		const std::optional<OceanConnectivityField> Again = Connected.WithSamplingMesh(Mesh.Directions, Mesh.Triangles);
		// 同じ網の再準備が成立しない場合の返戻
		if(!Test.TestTrue(TEXT("同じ網の海域を再準備"), Again.has_value())) return false;
		Test.TestTrue(TEXT("網を含む識別子の再現"), Connected.DefinitionHash() == Again->DefinitionHash());
		Test.TestTrue(TEXT("既存定義と適応定義の区別"), Original.DefinitionHash() != Connected.DefinitionHash());
		Test.TestTrue(TEXT("再準備で海峡を重複追加しない"), Connected.Corridors().size() == Again->Corridors().size());
		Test.TestTrue(TEXT("再構築した索引の容量"), Again->Statistics().IndexNodeCount <= 2 * Again->Statistics().CorridorCount);
		// 再準備の一致と地形の変化量
		std::uint32_t Changed = 0, CenterChecks = 0;
		double LargestRaise = 0;
		for(const UnitDirection Direction : Mesh.Directions) {
			const PlanetCore::TerrainSample A = Connected.Evaluate(Direction), B = Again->Evaluate(Direction);
			const double Previous = Original.EvaluateElevation(Direction);
			LargestRaise = std::max(LargestRaise, A.ElevationMeters - Previous);
			if(A.ElevationMeters != Previous) ++Changed;
			const bool HasSameSample =
			A.ElevationMeters == B.ElevationMeters && (A.TangentialGradientPerRadian - B.TangentialGradientPerRadian).Length() == 0;
			// 再準備に依る地形又は接線の変化を認めない返戻
			if(!Test.TestTrue(TEXT("再接続した標高と勾配の一致"), HasSameSample)) return false;
		}
		Test.TestTrue(TEXT("海峡の追加で地形を持上げない"), LargestRaise < 1E-8);
		// 追加海峡の最低水深と独立な区間内試料
		const double Ceiling = Connected.Base().Settings().SeaLevelMeters - Connected.Statistics().MinimumDepthMeters;
		for(std::size_t Index = Original.Corridors().size(); Index < Connected.Corridors().size(); ++Index) {
			const PlanetCore::OceanCorridor &Channel = Connected.Corridors()[Index];
			for(unsigned Along = 0; Along < 41; ++Along) {
				const double Angle = Channel.ArcAngle * Along / 40;
				const std::optional<UnitDirection> Direction =
				UnitDirection::FromVector(Channel.Start * std::cos(Angle) + Channel.Tangent * std::sin(Angle));
				// 海峡内の有効な球面方向を準備出来ない場合の返戻
				if(!Direction) return false;
				// 端点の接続だけで区間途中の水深を代用しない返戻
				if(!Test.TestTrue(TEXT("追加海峡の区間途中の最低水深"), Connected.EvaluateElevation(*Direction) <= Ceiling + 1E-7)) return false;
				++CenterChecks;
			}
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Adaptive ocean contracts: nodes=%llu changed=%u centerChecks=%u largestRaise=%.12g hash=%llu"),
				static_cast<unsigned long long>(Mesh.Directions.size()),
				Changed,
				CenterChecks,
				LargestRaise,
				static_cast<unsigned long long>(Connected.DefinitionHash())
			)
		);
		// 全ての公開契約の検査結果の返戻
		return !Test.HasAnyErrors();
	}

	/**
	 * 排水前に細分化した元場から，厳密な海域接続検査付きの排水網と共有縦断を解く
	 * @param Test 検査結果の記録先
	 * @param Ocean 海陸と造山を含む元場
	 * @param Base 比較する元の固定網
	 * @param Budget 追加節点の上限
	 * @return 有限な元場・海洋接続・下降水位を確認出来た場合に真
	 */
	bool CompareAdaptiveDrainage(
		FAutomationTestBase &Test,
		const OceanConnectivityField &Ocean,
		const PlanetCore::SphereMesh &Base,
		const std::uint32_t Budget
	) {
		const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
		const double Radius = Ocean.Base().Settings().RadiusMeters, Sea = Ocean.Base().Settings().SeaLevelMeters;
		const auto Original = [&Ocean](const UnitDirection Direction) -> double {
			// 同じ元地形の標高の返戻
			return Ocean.EvaluateElevation(Direction);
		};
		std::vector<double> Heights;
		for(const UnitDirection Direction : Base.Directions) Heights.push_back(Original(Direction));
		SphericalReliefSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = Sea;
		Settings.TargetErrorMeters *= Radius / (Radius + 250000);
		Settings.MaximumAddedVertices = Budget;
		std::optional<SphericalReliefSamplingMesh> Mesh =
		PlanetSphericalRefinement::CreateSamplingMesh(Base.Directions, Base.Triangles, Heights, Original, Settings);
		// 元標高を保つ網を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("排水前の適応網を準備"), Mesh.has_value())) return false;
		const std::optional<OceanConnectivityField> Connected = Ocean.WithSamplingMesh(Mesh->Directions, Mesh->Triangles);
		// 追加した水没点を分類から除外せず，同じ連続場で接続する
		if(!Test.TestTrue(TEXT("追加節点を含む同じ場で海域を再接続"), Connected.has_value())) return false;
		// 再接続処理の不合格を水位検査で隠さない返戻
		if(Budget && !CheckSamplingOcean(Test, Ocean, *Connected, *Mesh)) return false;
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			Mesh->ElevationsMeters[Index] = Connected->EvaluateElevation(Mesh->Directions[Index]);
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Adaptive ocean: radius=%.0f budget=%u before=%u after=%u corridors=%u added=%u lowering=%.12g"),
				Radius,
				Budget,
				Connected->Statistics().ComponentsBefore,
				Connected->Statistics().ComponentsAfter,
				Connected->Statistics().CorridorCount,
				Connected->Statistics().CorridorCount - Ocean.Statistics().CorridorCount,
				Connected->Statistics().MaximumSampleLoweringMeters
			)
		);
		Settings.MaximumAddedVertices = 0;
		const std::optional<PlanetSphericalRefinement> Scaffold =
		PlanetSphericalRefinement::Create(Mesh->Directions, Mesh->Triangles, Mesh->ElevationsMeters, Original, Settings);
		// 接線制限用の共有網を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("同じ元標高の補間台を準備"), Scaffold.has_value())) return false;
		std::vector<Vec3> Gradients;
		std::vector<std::uint32_t> Unconstrained(Mesh->Directions.size());
		std::iota(Unconstrained.begin(), Unconstrained.end(), 0U);
		for(const UnitDirection Direction : Mesh->Directions) {
			Gradients.push_back(Connected->Evaluate(Direction).TangentialGradientPerRadian);
		}
		const std::optional<std::vector<Vec3>> Limited = Scaffold->ConstrainDrainageGradients(Gradients, Sea, Unconstrained);
		// 排水方向を仮定せず海陸符号だけを守る接線を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("元場の接線と海陸符号を準備"), Limited.has_value())) return false;
		const std::optional<SphericalSplineField> Surface =
		SphericalSplineField::CreateOnMeshWithGradients(6, Mesh->Directions, Mesh->Triangles, Mesh->ElevationsMeters, *Limited);
		// 細分化した同じ元場を補間出来ない場合の返戻
		if(!Test.TestTrue(TEXT("適応元場を連続化"), Surface.has_value())) return false;
		const auto Height = [&Surface](const UnitDirection Direction) -> double {
			// 分類・河口・縦断で共有する同じ補間面の返戻
			return Surface->Evaluate(Direction).ElevationMeters;
		};
		std::vector<PlanetCore::DrainageSite> Sites(Mesh->Directions.size());
		std::vector<std::array<std::uint32_t, 2>> Edges;
		for(std::size_t Index = 0; Index < Sites.size(); ++Index) {
			Sites[Index].Direction = Mesh->Directions[Index].Vector();
			Sites[Index].ElevationMeters = Height(Mesh->Directions[Index]);
			Sites[Index].Ocean = Sites[Index].ElevationMeters < Sea;
		}
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) {
			const Vec3 A = Sites[Triangle[0]].Direction, B = Sites[Triangle[1]].Direction, C = Sites[Triangle[2]].Direction;
			const double Area = 2 * std::atan2(std::abs(A.Dot((B - A).Cross(C - A))), 1 + A.Dot(B) + B.Dot(C) + C.Dot(A));
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t First = Triangle[Side], Second = Triangle[(Side + 1) % 3];
				Sites[First].AreaSquareMeters += Area * Radius * Radius / 3;
				Edges.push_back({ std::min(First, Second), std::max(First, Second) });
			}
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		std::vector<std::vector<std::uint32_t>> Neighbors(Sites.size());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			Neighbors[Edge[0]].push_back(Edge[1]);
			Neighbors[Edge[1]].push_back(Edge[0]);
		}
		std::vector<bool> Visited(Sites.size(), false);
		std::uint32_t OceanComponents = 0, WetNodes = 0;
		for(std::uint32_t Index = 0; Index < Sites.size(); ++Index) {
			if(!Sites[Index].Ocean) continue;
			++WetNodes;
			if(Visited[Index]) continue;
			++OceanComponents;
			std::vector<std::uint32_t> Pending { Index };
			Visited[Index] = true;
			while(!Pending.empty()) {
				const std::uint32_t Current = Pending.back();
				Pending.pop_back();
				for(const std::uint32_t Neighbor : Neighbors[Current]) if(Sites[Neighbor].Ocean && !Visited[Neighbor]) {
					Visited[Neighbor] = true;
					Pending.push_back(Neighbor);
				}
			}
		}
		Test.AddInfo(
			FString::Printf(
				TEXT("Adaptive hydrology input: radius=%.0f budget=%u vertices=%llu oceanComponents=%u wetNodes=%u"),
				Radius,
				Budget,
				static_cast<unsigned long long>(Sites.size()),
				OceanComponents,
				WetNodes
			)
		);
		PlanetCore::DrainageSettings DrainageSettings;
		DrainageSettings.RadiusMeters = Radius;
		DrainageSettings.SeaLevelMeters = Sea;
		DrainageSettings.PreferBreaching = true;
		DrainageSettings.MinimumSlope = 1E-7 * std::min(1.0, 1E6 / Radius);
		std::optional<DrainageNetwork> Network = DrainageNetwork::Solve(Sites, Edges, DrainageSettings);
		// 元の海域証拠で代用せず，実際の適応網で大洋が連結している必要が有る
		if(!Test.TestTrue(TEXT("適応網自身の海域接続と排水を解決"), Network.has_value())) return false;
		Network = Network->WithCoastalOutlets(Height);
		// 適応元場と同じ海岸に河口を置けない場合の返戻
		if(!Test.TestTrue(TEXT("適応元場の海岸へ接続"), Network.has_value())) return false;
		Network = Network->WithTerrainProfiles(Height);
		// 同じ元場の縦断を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("適応元場の河川縦断を準備"), Network.has_value())) return false;
		Network = Network->WithBalancedTerrainProfiles(Height);
		// 共有合流点を保った調整を準備出来ない場合の返戻
		if(!Test.TestTrue(TEXT("適応網の共有縦断を調整"), Network.has_value())) return false;
		const std::span<const PlanetCore::DrainageNode> Nodes = Network->Nodes();
		double MaximumRaise = 0, MaximumCut = 0, SquaredAtOriginal = 0;
		std::uint32_t OriginalLand = 0, BrokenRoutes = 0;
		for(std::uint32_t Index = 0; Index < Nodes.size(); ++Index) {
			const PlanetCore::DrainageNode &Node = Nodes[Index];
			if(Node.Ocean) continue;
			const std::uint32_t Parent = Node.Downstream;
			if(Parent >= Nodes.size() || !(Node.WaterMeters > Nodes[Parent].WaterMeters)) ++BrokenRoutes;
			if(Node.Outlet >= Nodes.size() || !Nodes[Node.Outlet].Ocean) ++BrokenRoutes;
			const double Head = Node.WaterMeters - Sea, Freeboard = DrainageSettings.BankFreeboardMeters;
			const double Delta = Node.WaterMeters + Freeboard * Head / (Head + Freeboard) - Node.OriginalElevationMeters;
			MaximumRaise = std::max(MaximumRaise, Delta);
			MaximumCut = std::max(MaximumCut, -Delta);
			if(Index < Base.Directions.size()) {
				SquaredAtOriginal += Delta * Delta;
				++OriginalLand;
			}
		}
		Test.TestTrue(TEXT("全陸上節点から大洋へ下降する接続"), BrokenRoutes == 0 && OriginalLand > 0);
		const double Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count();
		Test.AddInfo(
			FString::Printf(
				TEXT("Adaptive hydrology solve: radius=%.0f budget=%u nodes=%llu mouths=%u originalRms=%.12g")
				TEXT(" raise=%.12g cut=%.12g broken=%u milliseconds=%.6g"),
				Radius,
				Budget,
				static_cast<unsigned long long>(Nodes.size()),
				Network->Statistics().CoastalOutletNodes,
				std::sqrt(SquaredAtOriginal / std::max(1U, OriginalLand)),
				MaximumRaise,
				MaximumCut,
				BrokenRoutes,
				Milliseconds
			)
		);
		// 景観の改善とは別に，この準備経路が成立する事の返戻
		return !Test.HasAnyErrors();
	}
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FPlanetAdaptiveHydrologyComparison, "PlanetGenerator.SphericalRelief.AdaptiveHydrology",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 比較する物理尺度を列挙する
 * @param Names 表示名
 * @param Commands 実行引数
 */
void FPlanetAdaptiveHydrologyComparison::GetTests(TArray<FString> &Names, TArray<FString> &Commands) const {
	Names.Add(TEXT("MinimumRadius"));
	Commands.Add(TEXT("MinimumRadius"));
	Names.Add(TEXT("ReferenceRadius"));
	Commands.Add(TEXT("ReferenceRadius"));
	// 終了
	return;
}

/**
 * 同じ元地形で，排水前に起伏を拾う順序と粗い網での補正量を比較する
 * @param Parameters 半径を選択する検査名
 * @return 二つの準備経路が成立する場合に真，最終景観の合格は別
 */
bool FPlanetAdaptiveHydrologyComparison::RunTest(const FString &Parameters) {
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = Parameters == TEXT("MinimumRadius") ? 1000 : 1000000;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Definition);
	// 元地質を準備出来ない場合の返戻
	if(!TestTrue(TEXT("適応排水比較の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(*PreparedGeology);
	const std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 元山稜を準備出来ない場合の返戻
	if(!TestTrue(TEXT("適応排水比較の山稜を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*PreparedRidges);
	const std::optional<OceanConnectivityField> Ocean = OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		Ridges,
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	const std::optional<PlanetCore::SphereMesh> Base = PlanetCore::BuildUniformSphere(6);
	// 比較条件が無い場合の返戻
	if(!TestTrue(TEXT("比較元の海域と球面を準備"), Ocean.has_value() && Base.has_value())) return false;
	const bool HasCoarseSolution = CompareAdaptiveDrainage(*this, *Ocean, *Base, 0);
	const bool HasAdaptiveSolution = CompareAdaptiveDrainage(*this, *Ocean, *Base, 16384);
	// 両方の結果を隠さず報告する返戻
	return HasCoarseSolution && HasAdaptiveSolution && !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetAdaptiveHydrologySizeBound,
	"PlanetGenerator.SphericalRelief.AdaptiveHydrology.SizeBound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 公開排水解法の最大規模と，上限付近でも独立した海域検査を維持する事を確認する
 * @param Parameters 未使用の自動検査引数
 * @return 上限内の接続網が通り，過大入力と切断された水域が拒否される場合に真
 */
bool FPlanetAdaptiveHydrologySizeBound::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	// 最大規模の独立した円周状の接続網
	constexpr std::uint32_t Count = 65536;
	std::vector<PlanetCore::DrainageSite> Sites(Count);
	std::vector<std::array<std::uint32_t, 2>> Edges;
	for(std::uint32_t Index = 0; Index < Count; ++Index) {
		const double Angle = 2 * std::acos(-1.0) * Index / Count;
		Sites[Index].Direction = { std::cos(Angle), std::sin(Angle), 0 };
		Sites[Index].ElevationMeters = -1;
		Sites[Index].AreaSquareMeters = 1;
		Sites[Index].Ocean = true;
		if(Index) Edges.push_back({ Index - 1, Index });
	}
	PlanetCore::DrainageSettings Settings;
	Settings.RadiusMeters = 1000;
	const std::optional<DrainageNetwork> Solved = DrainageNetwork::Solve(Sites, Edges, Settings);
	// 正規の最大規模が呼出せない場合の返戻
	if(!TestTrue(TEXT("公開上限の接続網を解決"), Solved.has_value())) return false;
	TestTrue(TEXT("全ての水没点を接続対象に含める"), Solved->Statistics().OceanNodes == Count);
	// 最後の頂点を切離し，上限拡張で接続検査を省いていない事の確認
	Edges.pop_back();
	TestFalse(TEXT("上限内でも切断した海域を拒否"), DrainageNetwork::Solve(Sites, Edges, Settings).has_value());
	Edges.push_back({ Count - 2, Count - 1 });
	const PlanetCore::DrainageSite Extra = Sites.front();
	Sites.push_back(Extra);
	TestFalse(TEXT("公開上限を超える地点数を拒否"), DrainageNetwork::Solve(Sites, Edges, Settings).has_value());
	// 正規経路と拒否経路を同時に確認した返戻
	return !HasAnyErrors();
}

#endif
