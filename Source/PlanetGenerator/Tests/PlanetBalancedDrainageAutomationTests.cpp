#include "Core/PlanetDrainage.h"
#include "Core/PlanetRiverTerrain.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 単調補間の全区間と共有端点を，微分二次式の極値も含めて検査する
	 * 計算量：O(n)，n は河道節点数
	 * @param Test 実行中の検査
	 * @param Network 再構成した縦断
	 * @return 全区間で水位と河床が下降し，共有端点が一致する場合に真
	 */
	bool CheckBalancedProfiles(FAutomationTestBase &Test, const PlanetCore::DrainageNetwork &Network) {
		const std::span<const PlanetCore::DrainageNode> Nodes = Network.Nodes();
		for(const PlanetCore::DrainageNode &Node : Nodes) {
			if(Node.Ocean) continue;
			// 全支流の共有端点で下降しない場合の返戻
			if(!Test.TestTrue(TEXT("非選択の支流も含めて合流水位が下降"), Node.WaterMeters > Nodes[Node.Downstream].WaterMeters)) return false;
		}
		for(std::size_t Index = 0; Index < Network.Reaches().size(); ++Index) {
			const PlanetCore::RiverReach &Reach = Network.Reaches()[Index];
			const PlanetCore::RiverLongitudinalProfile &P = Network.Profiles()[Index];
			// 合流点を河道毎に別の未知数として解いていない事を確認する返戻
			if(
				!Test.TestTrue(
					TEXT("全支流が一つの合流水位を共有"),
					P.Evaluate(0).WaterMeters == Nodes[Reach.Upstream].WaterMeters &&
					P.Evaluate(1).WaterMeters == Nodes[Reach.Downstream].WaterMeters
				)
			) return false;
			if(Nodes[Reach.Downstream].Ocean) {
				// 海口の固定境界が変わった場合の返戻
				if(!Test.TestEqual(TEXT("河口は一定海面に固定"), P.EndWater, Network.Settings().SeaLevelMeters)) return false;
			}
			for(std::uint32_t Knot = 0; Knot < P.Segments; ++Knot) {
				const double A = P.Derivative[Knot] / P.Segments, B = P.Derivative[Knot + 1] / P.Segments;
				const double Delta = P.Headroom[Knot + 1] - P.Headroom[Knot], C = 3 * Delta - 2 * A - B, D = A + B - 2 * Delta;
				double Maximum = std::max(A, B);
				if(D) {
					const double Extremum = -C / (3 * D);
					if(Extremum > 0 && Extremum < 1) Maximum = std::max(Maximum, A + Extremum * (2 * C + 3 * D * Extremum));
				}
				// 二次微分式の端点と内点極値から，区間全体の逆流を判定する返戻
				if(!Test.TestTrue(TEXT("三次補間の全区間に逆勾配なし"), Delta <= 0 && Maximum <= 1E-12 && P.GradeStart > P.GradeEnd)) return false;
				const double T = (Knot + 0.37) / P.Segments, Epsilon = 1E-7;
				const PlanetCore::RiverHeights H = P.Evaluate(T), Before = P.Evaluate(T - Epsilon), After = P.Evaluate(T + Epsilon);
				const double Difference = (After.WaterMeters - Before.WaterMeters) / (2 * Epsilon * P.LengthMeters);
				// 解析微分と独立差分が一致しない場合の返戻
				if(
					!Test.TestTrue(
						TEXT("水位の解析微分と独立差分が一致"),
						H.WaterSlope < 0 && H.BedSlope < 0 && std::abs(Difference - H.WaterSlope) < std::max(1E-6, std::abs(H.WaterSlope) * 1E-6)
					)
				) return false;
			}
		}
		// 共有端点と全補間区間の検査成功の返戻
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBalancedDrainageContracts, "PlanetGenerator.Drainage.BalancedProfiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 開削・盛土の共有縦断を，内部窪地，合流，海口，実惑星で検査する
 * @param Parameters 未使用の UE 検査引数
 * @return 正規入力と拒否条件の全検査が成功する場合に真
 */
bool FPlanetBalancedDrainageContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	PlanetCore::DrainageSettings Settings;
	Settings.RadiusMeters = 1000;
	Settings.ChannelAreaFraction = 1E-6;
	const std::array<std::array<std::uint32_t, 2>, 3> Edges { { { 0, 1 }, { 1, 2 }, { 1, 3 } } };
	std::vector<PlanetCore::DrainageSite> Sites;
	for(std::uint32_t Index = 0; Index < 4; ++Index) {
		const double Angle = 0.01 * Index;
		Sites.push_back({ { std::cos(Angle), std::sin(Angle), 0 }, Index ? 10 + Index : -1.0, 1000, !Index });
	}
	const std::optional<PlanetCore::DrainageNetwork> Source = PlanetCore::DrainageNetwork::Solve(Sites, Edges, Settings);
	if(!TestTrue(TEXT("合流を持つ元排水を準備"), Source.has_value())) return false;
	// 河道端の間に深い窪地を置き，粗網だけの解では見落とす拘束を与える
	const auto Height = [](const PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 V = Direction.Vector();
		const double Angle = std::atan2(V.Y, V.X), Q = (Angle - 0.024) / 0.0015;
		// 解析的な河道内部の窪地を持つ地面の返戻
		return 10 + 100 * Angle - 9 * std::exp(-Q * Q);
	};
	const std::optional<PlanetCore::DrainageNetwork> Balanced = Source->WithBalancedTerrainProfiles(Height);
	if(!TestTrue(TEXT("河道内部を含む合流共有の縦断を準備"), Balanced.has_value())) return false;
	if(!CheckBalancedProfiles(*this, *Balanced)) return false;
	TestTrue(TEXT("元の水位を変更せず再構成候補として識別"), Balanced->RequiresBalancedTerrain() && !Source->RequiresBalancedTerrain());
	TestTrue(TEXT("内部の窪地で必要な盛土を隠さず記録"), Balanced->Statistics().MaximumSampledWaterAboveTerrainMeters > 1);
	TestTrue(TEXT("内部の障壁の開削も記録"), Balanced->Statistics().MaximumRequiredWaterCutMeters > 0);
	const std::optional<PlanetCore::DrainageNetwork> Repeat = Source->WithBalancedTerrainProfiles(Height);
	if(!TestTrue(TEXT("同じ地形の再準備が成功"), Repeat.has_value())) return false;
	for(std::size_t Index = 0; Index < Balanced->Profiles().size(); ++Index) {
		TestTrue(
			TEXT("再準備の節点・微分が一致"),
			Balanced->Profiles()[Index].Headroom == Repeat->Profiles()[Index].Headroom &&
			Balanced->Profiles()[Index].Derivative == Repeat->Profiles()[Index].Derivative
		);
	}
	const std::optional<PlanetCore::DrainageNetwork> CutOnly = Balanced->WithTerrainProfiles(Height);
	if(!TestTrue(TEXT("開削側への明示的な再準備は正規に通る"), CutOnly.has_value())) return false;
	TestFalse(TEXT("開削側に再構成した縦断は盛土要求を持たない"), CutOnly->RequiresBalancedTerrain());
	TestFalse(TEXT("空の標高関数を拒否"), Source->WithBalancedTerrainProfiles({}).has_value());
	TestFalse(
		TEXT("非有限な標高を拒否"),
		Source->WithBalancedTerrainProfiles(
			[](PlanetCore::UnitDirection) {
				return std::numeric_limits<double>::quiet_NaN();
			}
		).has_value()
	);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("中断済みの準備を拒否"), Source->WithBalancedTerrainProfiles(Height, Cancel.get_token()).has_value());
	std::stop_source During;
	std::uint32_t Calls = 0;
	TestFalse(
		TEXT("河道内部の準備中も中断"),
		Source->WithBalancedTerrainProfiles(
			[&](const PlanetCore::UnitDirection Direction) {
				if(++Calls == 25) During.request_stop();
				// 中断が標高評価中に届く正常な関数の返戻
				return Height(Direction);
			},
			During.get_token()
		).has_value()
	);
	Settings.BankFreeboardMeters = 0;
	const std::optional<PlanetCore::DrainageNetwork> ZeroFreeboard = PlanetCore::DrainageNetwork::Solve(Sites, Edges, Settings);
	if(!TestTrue(TEXT("余裕高０の正規設定を準備"), ZeroFreeboard.has_value())) return false;
	const std::optional<PlanetCore::DrainageNetwork> Flat = ZeroFreeboard->WithBalancedTerrainProfiles(
		[](PlanetCore::UnitDirection) {
			return 0.0;
		}
	);
	if(!TestTrue(TEXT("海面と余裕高０で非数を作らない"), Flat.has_value())) return false;
	if(!CheckBalancedProfiles(*this, *Flat)) return false;
	Settings.ChannelAreaFraction = 1;
	const std::optional<PlanetCore::DrainageNetwork> Unselected = PlanetCore::DrainageNetwork::Solve(Sites, Edges, Settings);
	if(!TestTrue(TEXT("描画河道を選択しない正規入力を準備"), Unselected.has_value())) return false;
	const std::optional<PlanetCore::DrainageNetwork> OnlyNodes = Unselected->WithBalancedTerrainProfiles(Height);
	if(!TestTrue(TEXT("非選択支流だけの共有縦断も準備"), OnlyNodes.has_value())) return false;
	TestTrue(TEXT("非選択支流から描画河道を勝手に追加しない"), OnlyNodes->Reaches().empty() && OnlyNodes->Profiles().empty());
	if(!CheckBalancedProfiles(*this, *OnlyNodes)) return false;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		PlanetCore::GeologySettings Definition;
		Definition.Seed = 42;
		Definition.RadiusMeters = Radius;
		const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Definition);
		if(!TestTrue(TEXT("実惑星の地質を準備"), Geology.has_value())) return false;
		const std::optional<PlanetCore::OceanConnectivityField> Ocean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geology));
		if(!TestTrue(TEXT("実惑星の海域を準備"), Ocean.has_value())) return false;
		const std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::Create(*Ocean);
		if(!TestTrue(TEXT("実惑星の元排水を準備"), Network.has_value())) return false;
		const double Started = FPlatformTime::Seconds();
		const std::optional<PlanetCore::DrainageNetwork> Candidate = Network->WithBalancedTerrainProfiles(
			[&Ocean](const PlanetCore::UnitDirection Direction) {
				return Ocean->EvaluateElevation(Direction);
			}
		);
		const double Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
		if(!TestTrue(TEXT("実河道の全内部標本を同時に再構成"), Candidate.has_value())) return false;
		if(!CheckBalancedProfiles(*this, *Candidate)) return false;
		TestEqual(TEXT("同じ海域の識別値を維持"), Candidate->SourceDefinitionHash(), Network->SourceDefinitionHash());
		for(std::size_t Index = 0; Index < Network->Nodes().size(); ++Index) {
			TestTrue(
				TEXT("元の大洋出口と集水面積を維持"),
				Candidate->Nodes()[Index].Outlet == Network->Nodes()[Index].Outlet &&
				Candidate->Nodes()[Index].ContributingAreaSquareMeters == Network->Nodes()[Index].ContributingAreaSquareMeters
			);
		}
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> SharedOcean =
		std::make_shared<const PlanetCore::OceanConnectivityField>(*Ocean);
		TestFalse(
			TEXT("盛土未統合の縦断を開削専用描画へ渡さない"),
			PlanetCore::RiverTerrainField::Create(SharedOcean, std::make_shared<const PlanetCore::DrainageNetwork>(*Candidate)).has_value()
		);
		TestFalse(
			TEXT("侵食経由でも盛土未統合の縦断を公開しない"),
			PlanetCore::RiverTerrainField::CreateWithErosion(SharedOcean, std::make_shared<const PlanetCore::DrainageNetwork>(*Candidate))
			.has_value()
		);
		TestTrue(
			TEXT("既存の正規の地形生成経路は引続き通る"),
			PlanetCore::RiverTerrainField::Create(SharedOcean, std::make_shared<const PlanetCore::DrainageNetwork>(*Network)).has_value()
		);
		AddInfo(
			FString::Printf(
				TEXT("Balanced profiles: radius=%.0f nodes=%llu reaches=%llu knots=%u evaluations=%llu ")
				TEXT("time=%.6fms maxNodeCut=%.6fm maxNodeFill=%.6fm maxKnotBedFill=%.6fm."),
				Radius,
				static_cast<unsigned long long>(Candidate->Nodes().size()),
				static_cast<unsigned long long>(Candidate->Reaches().size()),
				Candidate->Statistics().ProfileKnots,
				static_cast<unsigned long long>(Candidate->Statistics().ProfileTerrainEvaluations),
				Milliseconds,
				Candidate->Statistics().MaximumRequiredWaterCutMeters,
				Candidate->Statistics().MaximumRequiredBankRaiseMeters,
				Candidate->Statistics().MaximumUnresolvedKnotBedGapMeters
			)
		);
	}
	// 実河道と解析的な内部窪地，入力拒否の検査結果の返戻
	return !HasAnyErrors();
}

#endif
