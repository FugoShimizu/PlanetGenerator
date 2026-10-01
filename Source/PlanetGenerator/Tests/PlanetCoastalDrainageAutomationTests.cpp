#include "Core/PlanetDrainage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetCoastalDrainageContracts, "PlanetGenerator.Drainage.CoastalOutlets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 陸同士の辺に海が挟まる解析地形で，最初の海岸への終端と集水量の付替えを検査する
 * @param Parameters UE 自動検査の引数
 * @return 検査を実行出来た場合に真，個別の不一致は UE に記録
 */
bool FPlanetCoastalDrainageContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const auto Direction = [](const double Angle) {
		// 同じ大円上の地点の返戻
		return *PlanetCore::UnitDirection::FromVector({std::cos(Angle), std::sin(Angle), 0});
	};
	const auto Height = [](const PlanetCore::UnitDirection Point) {
		const double Angle = std::atan2(Point.Vector().Y, Point.Vector().X);
		// 角度 0.005 の海岸と，0.018～0.022 の海峡を持つ連続標高の返戻
		return std::min(200 * (Angle - 0.005), 10000 * (Angle - 0.018) * (Angle - 0.022));
	};
	std::array<PlanetCore::DrainageSite, 4> Sites;
	const std::array<double, 4> Angles{0, 0.01, 0.03, 0.05};
	for(std::size_t i = 0; i < Sites.size(); ++i)
		Sites[i] = {Direction(Angles[i]).Vector(), Height(Direction(Angles[i])), 10000, i == 0};
	const std::array<std::array<std::uint32_t, 2>, 3> Edges{{{0, 1}, {1, 2}, {2, 3}}};
	PlanetCore::DrainageSettings Settings;
	Settings.RadiusMeters = 1000;
	Settings.PreferBreaching = true;
	const std::optional<PlanetCore::DrainageNetwork> Original = PlanetCore::DrainageNetwork::Solve(Sites, Edges, Settings);
	// 元の経路を構築出来なかった場合の返戻
	if(!TestTrue(TEXT("解析用の排水木を準備"), Original.has_value())) return false;
	TestEqual(TEXT("修正前は海峡の先の陸へ排水する"), Original->Nodes()[2].Downstream, 1U);
	const std::optional<PlanetCore::ShoreCrossing> Shore =
		PlanetCore::FindShoreCrossing(Direction(0.03), Direction(0.01), 1000, 0, Height);
	// 陸同士の辺の途中に有る海を見付けられなかった場合の返戻
	if(!TestTrue(TEXT("両端が陸でも途中の海岸を検出"), Shore.has_value())) return false;
	TestTrue(TEXT("下流へ進んで最初の海岸を選択"),
		std::abs(std::atan2(Shore->Direction.Y, Shore->Direction.X) - 0.022) * 1000 <= 1E-5);
	const std::optional<PlanetCore::DrainageNetwork> Coastal = Original->WithCoastalOutlets(Height);
	// 経路の付替えを完了出来なかった場合の返戻
	if(!TestTrue(TEXT("海岸の流出口と集水量を再構成"), Coastal.has_value())) return false;
	TestEqual(TEXT("元の四地点へ二つの海岸出口だけを追加"), Coastal->Statistics().CoastalOutletNodes, 2U);
	TestEqual(TEXT("元の地点番号を保持した六地点"), Coastal->Nodes().size(), std::size_t{6});
	TestEqual(TEXT("海峡の先へ元の流量を流さない"), Coastal->Nodes()[1].ContributingAreaSquareMeters, 10000.0);
	TestEqual(TEXT("上流の集水量は新しい河口へ到達"),
		Coastal->Nodes()[Coastal->Nodes()[2].Downstream].ContributingAreaSquareMeters, 20000.0);
	TestEqual(TEXT("全陸域の面積が大洋出口へ保存"), Coastal->Statistics().OutletAreaSquareMeters, 30000.0);
	TestTrue(TEXT("海を越えて再び陸へ出る経路を切断"), Coastal->Nodes()[Coastal->Nodes()[2].Downstream].Ocean);
	TestEqual(TEXT("残る上流の接続を保持"), Coastal->Nodes()[3].Downstream, 2U);
	for(std::size_t i = 0; i < Sites.size(); ++i)
		TestTrue(
			TEXT("地形補間に使う元標本の位置は不変"), (Coastal->Nodes()[i].Direction - Sites[i].Direction).Length() < 1E-15);
	const std::optional<PlanetCore::DrainageNetwork> Profiled = Coastal->WithTerrainProfiles(Height);
	// 新しい出口へ下降する縦断を準備出来なかった場合の返戻
	if(!TestTrue(TEXT("短縮した経路に下降する水位を準備"), Profiled.has_value())) return false;
	for(std::size_t i = 0; i < Profiled->Reaches().size(); ++i) {
		double Previous = std::numeric_limits<double>::infinity();
		for(std::uint32_t j = 0; j <= 257; ++j) {
			const PlanetCore::RiverProfileSample Sample = *Profiled->SampleReach(i, j / 257.0);
			TestTrue(TEXT("新しい河道の全試料で水位が下流へ下降"), Sample.WaterMeters < Previous);
			TestTrue(TEXT("河口より手前で海を横断しない"),
				Height(*PlanetCore::UnitDirection::FromVector(Sample.Direction)) >= -1E-5);
			Previous = Sample.WaterMeters;
		}
	}
	// 短い河道では単位座標同士の射影差で桁落ちする為，非軸方向の極短辺を独立に検査する
	const PlanetCore::Vec3 Start = PlanetCore::UnitDirection::FromVector({0.3, 0.4, std::sqrt(0.75)})->Vector();
	const PlanetCore::Vec3 Tangent{0.8, -0.6, 0};
	for(const double Angle : {1E-4, 1E-6, 1E-8}) {
		const PlanetCore::Vec3 End =
			PlanetCore::UnitDirection::FromVector(Start * std::cos(Angle) + Tangent * std::sin(Angle))->Vector();
		const std::array<PlanetCore::DrainageSite, 2> ShortSites{{{End, -1, 10000, true}, {Start, 10, 10000, false}}};
		const std::array<std::array<std::uint32_t, 2>, 1> ShortEdges{{{0, 1}}};
		const std::optional<PlanetCore::DrainageNetwork> Short =
			PlanetCore::DrainageNetwork::Solve(ShortSites, ShortEdges, Settings);
		// 極短辺の解析用排水を準備出来なかった場合の返戻
		if(!TestTrue(TEXT("極短の排水辺を準備"), Short.has_value() && Short->Reaches().size() == 1)) return false;
		for(const double Fraction : {0.0, 0.0625, 0.5, 1.0}) {
			const PlanetCore::RiverProfileSample Sample = *Short->SampleReach(0, Fraction);
			TestTrue(
				TEXT("極短河道の接線が球面外向きと直交"), std::abs(Sample.Direction.Dot(Sample.DownstreamTangent)) < 1E-12);
			TestTrue(TEXT("極短河道の接線が単位長を保持"), std::abs(Sample.DownstreamTangent.Length() - 1) < 1E-12);
		}
	}
	std::stop_source Stop;
	Stop.request_stop();
	TestFalse(TEXT("中断した出口再構成を公開しない"), Original->WithCoastalOutlets(Height, Stop.get_token()).has_value());
	TestFalse(TEXT("流出口の二重追加を拒否"), Coastal->WithCoastalOutlets(Height).has_value());
	TestFalse(TEXT("非有限標高を海岸の無い辺と誤認しない"),
		Original
			->WithCoastalOutlets([](const PlanetCore::UnitDirection) {
				// 不正入力検査の非有限値の返戻
				return std::numeric_limits<double>::quiet_NaN();
			})
			.has_value());
	AddInfo(
		TEXT("Coastal routing: 4 fixed sites, 2 new outlets, 30000 square meters conserved; 771 independent river intervals."));
	// 個別の結果を検査器へ記録した上での返戻
	return true;
}
#endif
