#include "Core/PlanetRiverRouting.h"
#include "Core/PlanetRiverRoutingTerrain.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetRiverRoutingContracts,
	"PlanetGenerator.RiverRouting.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 有限な水位窓と両岸上限から，下降する経路だけを選ぶ契約を検査する
 * @param Parameters 未使用の実行引数
 * @return 最小水位・拒否・中断の契約が成立した場合に真
 */
bool FPlanetRiverRoutingContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	constexpr double Slope = 0.01;
	const std::optional<PlanetCore::RiverRoutingWindow> PhysicalWindow = PlanetCore::MakeRiverRoutingWindow(10, 14, 0.3, 5, 2, 0.5);
	TestTrue(TEXT("地面・左右岸から水位窓を準備"), PhysicalWindow.has_value());
	if(PhysicalWindow) {
		TestTrue(TEXT("中心の最大開削と最低水深を下界へ変換"), std::abs(PhysicalWindow->MinimumWaterMeters - 8.3) < 1E-12);
		TestTrue(TEXT("低い方の河岸と余裕高を上界へ変換"), std::abs(PhysicalWindow->MaximumWaterMeters - 13.5) < 1E-12);
	}
	TestFalse(TEXT("無盛土で河岸が水没する水位窓を拒否"), PlanetCore::MakeRiverRoutingWindow(10, 8, 0.3, 5, 2, 0.5).has_value());
	const std::array<PlanetCore::RiverRoutingWindow, 4> Sites {
		PlanetCore::RiverRoutingWindow{ 2, 5 },
		PlanetCore::RiverRoutingWindow{ 1, 3 },
		PlanetCore::RiverRoutingWindow{ 1, 4 },
		PlanetCore::RiverRoutingWindow{ 0, 0 }
	};
	const std::array<PlanetCore::RiverRoutingEdge, 4> Edges {
		PlanetCore::RiverRoutingEdge{ 0, 1, 100 },
		PlanetCore::RiverRoutingEdge{ 1, 3, 100 },
		PlanetCore::RiverRoutingEdge{ 0, 2, 200 },
		PlanetCore::RiverRoutingEdge{ 2, 3, 100 }
	};
	PlanetCore::RiverRoutingError Error = PlanetCore::RiverRoutingError::None;
	const std::optional<PlanetCore::RiverRoutingResult> Short =
	PlanetCore::FindRiverRoutingPath(Sites, Edges, 0, 3, 0, Slope, Error);
	if(!Short) {
		TestTrue(TEXT("水位窓の短い下降経路を発見"), false);
		// 必須経路を得られない場合の返戻
		return false;
	}
	TestTrue(TEXT("水源から大洋接続済み受入点へ到達"), Short->Nodes == std::vector<std::uint32_t>({ 0, 1, 3 }));
	TestTrue(TEXT("各節点の最小水位"), Short->WaterMeters == std::vector<double>({ 2, 1, 0 }));
	TestTrue(TEXT("経路の実長"), std::abs(Short->LengthMeters - 200) < 1E-12);
	std::array<PlanetCore::RiverRoutingWindow, 4> LowBank = Sites;
	LowBank[1] = { 0, 0.5 };
	const std::optional<PlanetCore::RiverRoutingResult> Detour =
	PlanetCore::FindRiverRoutingPath(LowBank, Edges, 0, 3, 0, Slope, Error);
	if(!Detour) {
		TestTrue(TEXT("河岸上限を守る別経路へ切替"), false);
		// 必須経路を得られない場合の返戻
		return false;
	}
	TestTrue(TEXT("河岸で水没する節点を避ける"), Detour->Nodes == std::vector<std::uint32_t>({ 0, 2, 3 }));
	TestTrue(TEXT("長い迂回でも厳密下降"), Detour->WaterMeters == std::vector<double>({ 3, 1, 0 }));
	LowBank[2] = { 0, 0.5 };
	PlanetCore::RiverRoutingDiagnostics Unreachable;
	TestFalse(
		TEXT("両通路の河岸が低ければ解なし"),
		PlanetCore::FindRiverRoutingPath(LowBank, Edges, 0, 3, 0, Slope, Error, {}, &Unreachable).has_value()
	);
	TestTrue(
		TEXT("受入点だけへ到達できる理由を残す"),
		Unreachable.MinimumRequiredWaterMeters.size() == Sites.size() && Unreachable.SettledNodes == 1 &&
		std::isfinite(Unreachable.MinimumRequiredWaterMeters[3]) && !std::isfinite(Unreachable.MinimumRequiredWaterMeters[0])
	);
	TestTrue(TEXT("解なしの理由"), Error == PlanetCore::RiverRoutingError::NoRoute);
	const std::array<PlanetCore::RiverRoutingReceiver, 2> Alternatives {
		PlanetCore::RiverRoutingReceiver{ 2, 0.5 },
		PlanetCore::RiverRoutingReceiver{ 3, 0 }
	};
	const std::optional<PlanetCore::RiverRoutingResult> AlternateReceiver =
	PlanetCore::FindRiverRoutingPath(LowBank, Edges, 0, Alternatives, Slope, Error);
	TestTrue(
		TEXT("別の大洋接続済み受入点へ下降経路を選ぶ"),
		AlternateReceiver && AlternateReceiver->Nodes == std::vector<std::uint32_t>({ 0, 2 }) &&
		AlternateReceiver->SelectedReceiver == 2 && AlternateReceiver->WaterMeters == std::vector<double>({ 2.5, 0.5 })
	);
	const std::array<PlanetCore::RiverRoutingWindow, 3> RaisedJunctionSites {
		PlanetCore::RiverRoutingWindow{ 3, 5 },
		PlanetCore::RiverRoutingWindow{ 1, 3 },
		PlanetCore::RiverRoutingWindow{ 0, 0 }
	};
	std::array<PlanetCore::RiverRoutingEdge, 2> RaisedJunctionEdges {
		PlanetCore::RiverRoutingEdge{ 0, 1, 100 },
		PlanetCore::RiverRoutingEdge{ 1, 2, 100 }
	};
	RaisedJunctionEdges[0].MinimumDownstreamWaterMeters = 2;
	const std::optional<PlanetCore::RiverRoutingResult> RaisedJunctionRoute =
	PlanetCore::FindRiverRoutingPath(RaisedJunctionSites, RaisedJunctionEdges, 0, 2, 0, Slope, Error);
	TestTrue(
		TEXT("合流点の最低水位だけでは不足する場合に許容区間内で水位を上げる"),
		RaisedJunctionRoute && RaisedJunctionRoute->Nodes == std::vector<std::uint32_t>({ 0, 1, 2 }) &&
		RaisedJunctionRoute->WaterMeters == std::vector<double>({ 3, 2, 0 })
	);
	const std::array<PlanetCore::RiverRoutingWindow, 4> SplitJunctionSites {
		PlanetCore::RiverRoutingWindow{ 3, 5 },
		PlanetCore::RiverRoutingWindow{ 1, 3 },
		PlanetCore::RiverRoutingWindow{ 0, 0 },
		PlanetCore::RiverRoutingWindow{ 1, 1 }
	};
	std::array<PlanetCore::RiverRoutingEdge, 3> SplitJunctionEdges {
		PlanetCore::RiverRoutingEdge{ 0, 1, 100 },
		PlanetCore::RiverRoutingEdge{ 1, 2, 100 },
		PlanetCore::RiverRoutingEdge{ 1, 3, 100 }
	};
	SplitJunctionEdges[0].MinimumDownstreamWaterMeters = 2;
	SplitJunctionEdges[1].MaximumUpstreamWaterMeters = 1.5;
	SplitJunctionEdges[2].MinimumUpstreamWaterMeters = 2;
	SplitJunctionEdges[2].MaximumUpstreamWaterMeters = 2.5;
	const std::array<PlanetCore::RiverRoutingReceiver, 2> SplitReceivers {
		PlanetCore::RiverRoutingReceiver{ 2, 0 },
		PlanetCore::RiverRoutingReceiver{ 3, 1 }
	};
	const std::optional<PlanetCore::RiverRoutingResult> SplitRoute =
	PlanetCore::FindRiverRoutingPath(SplitJunctionSites, SplitJunctionEdges, 0, SplitReceivers, Slope, Error);
	TestTrue(
		TEXT("同じ合流点の分離した許容水位区間を保持する"),
		SplitRoute && SplitRoute->Nodes == std::vector<std::uint32_t>({ 0, 1, 3 }) &&
		SplitRoute->WaterMeters == std::vector<double>({ 3, 2, 1 })
	);
	const std::array<PlanetCore::RiverRoutingReceiver, 2> DuplicateReceivers {
		PlanetCore::RiverRoutingReceiver{ 3, 0 },
		PlanetCore::RiverRoutingReceiver{ 3, 0 }
	};
	TestFalse(
		TEXT("同じ受入節点の重複を拒否"),
		PlanetCore::FindRiverRoutingPath(Sites, Edges, 0, DuplicateReceivers, Slope, Error).has_value()
	);
	TestTrue(TEXT("重複受入点の入力不正を区別"), Error == PlanetCore::RiverRoutingError::InvalidInput);
	const std::array<PlanetCore::RiverRoutingInteriorSample, 2> Interior {
		PlanetCore::RiverRoutingInteriorSample{ 25, { 2, 2 } },
		PlanetCore::RiverRoutingInteriorSample{ 75, { 3, 3 } }
	};
	PlanetCore::RiverRoutingEdgeError EdgeError = PlanetCore::RiverRoutingEdgeError::None;
	TestFalse(TEXT("内部標本の下降不能を辺準備で拒否"), PlanetCore::PrepareRiverRoutingEdge(0, 1, 100, Interior, Slope, &EdgeError).has_value());
	TestTrue(TEXT("内部水位窓の矛盾を区別"), EdgeError == PlanetCore::RiverRoutingEdgeError::IncompatibleWindows);
	const std::array<PlanetCore::RiverRoutingInteriorSample, 1> Passable {
		PlanetCore::RiverRoutingInteriorSample{ 50, { 1.5, 1.7 } }
	};
	const std::optional<PlanetCore::RiverRoutingEdge> Bounded = PlanetCore::PrepareRiverRoutingEdge(0, 1, 100, Passable, Slope);
	if(!Bounded) {
		TestTrue(TEXT("内部標本の上下限を辺へ縮約"), false);
		// 必須経路を得られない場合の返戻
		return false;
	}
	TestTrue(TEXT("内部標本から上流最低水位"), std::abs(Bounded->MinimumUpstreamWaterMeters - 2) < 1E-12);
	TestTrue(TEXT("内部標本から下流最高水位"), std::abs(Bounded->MaximumDownstreamWaterMeters - 1.2) < 1E-12);
	std::array<PlanetCore::RiverRoutingEdge, 4> Restricted = Edges;
	Restricted[0] = *Bounded;
	const std::optional<PlanetCore::RiverRoutingResult> InteriorRoute =
	PlanetCore::FindRiverRoutingPath(Sites, Restricted, 0, 3, 0, Slope, Error);
	TestTrue(TEXT("内部標本を守る経路も成立"), InteriorRoute.has_value());
	std::array<PlanetCore::RiverRoutingWindow, 4> RaisedJunction = Sites;
	RaisedJunction[1].MinimumWaterMeters = 1.5;
	const std::optional<PlanetCore::RiverRoutingResult> InteriorDetour =
	PlanetCore::FindRiverRoutingPath(RaisedJunction, Restricted, 0, 3, 0, Slope, Error);
	TestTrue(TEXT("辺途中の河岸上限を超える本線を避ける"), InteriorDetour && InteriorDetour->Nodes == std::vector<std::uint32_t>({ 0, 2, 3 }));
	std::stop_source Cancel;
	Cancel.request_stop();
	PlanetCore::RiverRoutingEdgeError CancelledEdge = PlanetCore::RiverRoutingEdgeError::None;
	TestFalse(
		TEXT("内部辺準備の中断は部分境界を返さない"),
		PlanetCore::PrepareRiverRoutingEdge(0, 1, 100, Passable, Slope, &CancelledEdge, Cancel.get_token()).has_value()
	);
	TestTrue(TEXT("辺の中断理由を区別"), CancelledEdge == PlanetCore::RiverRoutingEdgeError::Cancelled);
	TestFalse(
		TEXT("中断時に部分経路を返さない"),
		PlanetCore::FindRiverRoutingPath(Sites, Edges, 0, 3, 0, Slope, Error, Cancel.get_token()).has_value()
	);
	TestTrue(TEXT("中断理由を区別"), Error == PlanetCore::RiverRoutingError::Cancelled);
	constexpr double Radius = 1000000;
	const double Angle = 100 / Radius;
	const std::array<PlanetCore::UnitDirection, 2> Directions {
		*PlanetCore::UnitDirection::FromVector({ 1, 0, 0 }),
		*PlanetCore::UnitDirection::FromVector({ std::cos(Angle), std::sin(Angle), 0 })
	};
	const std::array<std::array<std::uint32_t, 2>, 1> PhysicalEdges { std::array<std::uint32_t, 2>{ 0, 1 } };
	PlanetCore::RiverRoutingTerrainSettings TerrainSettings;
	TerrainSettings.RadiusMeters = Radius;
	TerrainSettings.MaximumCutMeters = 2;
	TerrainSettings.MinimumDownstreamSlope = 0.001;
	PlanetCore::RiverRoutingTerrainError TerrainError = PlanetCore::RiverRoutingTerrainError::None;
	const std::optional<PlanetCore::RiverRoutingTerrainGraph> TerrainGraph = PlanetCore::PrepareRiverRoutingTerrainGraph(
		Directions,
		PhysicalEdges,
		[](const PlanetCore::UnitDirection) {
			return 10.0;
		},
		TerrainSettings,
		TerrainError
	);
	TestTrue(TEXT("同じ球面地形から両岸と内部標本付きの有向網を準備"), TerrainGraph.has_value());
	if(TerrainGraph) {
		TestTrue(TEXT("一辺の両方向を地形標本から準備"), TerrainGraph->Edges.size() == 2);
		TestTrue(
			TEXT("岸の余裕高が節点中心だけの上限より低い"),
			TerrainGraph->Sites[0].MaximumWaterMeters == 20 && TerrainGraph->Edges[0].MaximumUpstreamWaterMeters < 9.701
		);
		const std::optional<PlanetCore::RiverRoutingResult> PhysicalRoute =
		PlanetCore::FindRiverRoutingPath(TerrainGraph->Sites, TerrainGraph->Edges, 0, 1, 9, 0.001, Error);
		TestTrue(TEXT("岸を越えない水位で下流へ接続"), PhysicalRoute && PhysicalRoute->Nodes == std::vector<std::uint32_t>({ 0, 1 }));
		TestTrue(
			TEXT("岸端点の上限を超える勾配要求を拒否"),
			!PlanetCore::FindRiverRoutingPath(TerrainGraph->Sites, TerrainGraph->Edges, 0, 1, 9, 0.01, Error)
		);
		TestTrue(TEXT("岸端点の制約違反は経路なし"), Error == PlanetCore::RiverRoutingError::NoRoute);
	}
	const auto NarrowReceivingBank = [Radius](const PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 P = Direction.Vector();
		const double AlongMeters = Radius * std::atan2(P.Y, P.X), AcrossMeters = Radius * std::asin(P.Z);
		const double Squared = std::pow(AlongMeters - 100, 2) + std::pow(AcrossMeters - 20, 2);
		// 受入点の片側岸だけが低く，辺の内部は元の乾いた岸を持つ
		return 10 - 2 * std::exp(-0.25 * Squared);
	};
	const std::optional<PlanetCore::RiverRoutingTerrainGraph> BlockedReceiver =
	PlanetCore::PrepareRiverRoutingTerrainGraph(Directions, PhysicalEdges, NarrowReceivingBank, TerrainSettings, TerrainError);
	TestTrue(TEXT("低い受入端の岸を通常の河道では拒否"), BlockedReceiver && BlockedReceiver->Edges.empty());
	const std::array<std::uint32_t, 1> OwnedReceiver { 1 };
	const std::optional<PlanetCore::RiverRoutingTerrainGraph> SharedWaterBody = PlanetCore::PrepareRiverRoutingTerrainGraph(
		Directions,
		PhysicalEdges,
		NarrowReceivingBank,
		TerrainSettings,
		TerrainError,
		{},
		OwnedReceiver
	);
	TestTrue(TEXT("既存水域が所有する端点では新河道の岸だけを除外"), SharedWaterBody && SharedWaterBody->Edges.size() == 2);
	if(SharedWaterBody) {
		const std::optional<PlanetCore::RiverRoutingResult> Joined = PlanetCore::FindRiverRoutingPath(
			SharedWaterBody->Sites,
			SharedWaterBody->Edges,
			0,
			1,
			9,
			TerrainSettings.MinimumDownstreamSlope,
			Error
		);
		TestTrue(TEXT("乾いた辺の岸を保ち既存水域へ下降"), Joined && Joined->SelectedReceiver == 1);
	}
	const auto LowInteriorBank = [Radius](const PlanetCore::UnitDirection Direction) {
		const PlanetCore::Vec3 P = Direction.Vector();
		const double AlongMeters = Radius * std::atan2(P.Y, P.X), AcrossMeters = Radius * std::asin(P.Z);
		const double Squared = std::pow(AlongMeters - 50, 2) + std::pow(AcrossMeters - 20, 2);
		// 水域端点を共有しても途中の外岸が低ければ通路を許さない
		return 10 - 2 * std::exp(-0.25 * Squared);
	};
	const std::optional<PlanetCore::RiverRoutingTerrainGraph> InteriorBlocked = PlanetCore::PrepareRiverRoutingTerrainGraph(
		Directions,
		PhysicalEdges,
		LowInteriorBank,
		TerrainSettings,
		TerrainError,
		{},
		OwnedReceiver
	);
	TestTrue(TEXT("所有端点の指定は内部の低い岸を免除しない"), InteriorBlocked && InteriorBlocked->Edges.empty());
	const std::array<PlanetCore::UnitDirection, 3> WidthDirections {
		Directions[0],
		Directions[1],
		*PlanetCore::UnitDirection::FromVector({ std::cos(2 * Angle), std::sin(2 * Angle), 0 })
	};
	const std::array<std::array<std::uint32_t, 2>, 2> WidthEdges { std::array<std::uint32_t, 2>{ 0, 1 }, { 1, 2 } };
	const std::array<double, 2> Widths { 160, 20 };
	const auto WideningBank = [Radius](const PlanetCore::UnitDirection Direction) {
		const double AcrossMeters = Radius * std::asin(Direction.Vector().Z);
		// 湖口の外岸だけが高くなる解析地形の返戻
		return 10 + 2 * (1 - std::exp(-std::pow(AcrossMeters / 100, 2)));
	};
	const std::optional<PlanetCore::RiverRoutingTerrainGraph> VariableWidth = PlanetCore::PrepareRiverRoutingTerrainGraph(
		WidthDirections,
		WidthEdges,
		WideningBank,
		TerrainSettings,
		TerrainError,
		{},
		{},
		Widths
	);
	TestTrue(TEXT("候補辺ごとの河岸幅を同じ地形で評価"), VariableWidth && VariableWidth->Edges.size() == 4);
	if(VariableWidth) {
		double WideCeiling = -std::numeric_limits<double>::infinity(), NarrowCeiling = -std::numeric_limits<double>::infinity();
		for(const PlanetCore::RiverRoutingEdge &Edge : VariableWidth->Edges) {
			if(!Edge.Upstream && Edge.Downstream == 1) WideCeiling = Edge.MaximumDownstreamWaterMeters;
			if(Edge.Upstream == 1 && Edge.Downstream == 2) NarrowCeiling = Edge.MaximumDownstreamWaterMeters;
		}
		TestTrue(TEXT("広い湖口の岸と細い下流岸を別々の水位上限へ反映"), WideCeiling > NarrowCeiling + 1);
	}
	const std::array<double, 1> MissingWidth { 160 };
	TestFalse(
		TEXT("候補辺数と異なる岸幅を拒否"),
		PlanetCore::PrepareRiverRoutingTerrainGraph(
			WidthDirections,
			WidthEdges,
			WideningBank,
			TerrainSettings,
			TerrainError,
			{},
			{},
			MissingWidth
		).has_value()
	);
	TestTrue(TEXT("岸幅の個数不一致は入力違反"), TerrainError == PlanetCore::RiverRoutingTerrainError::InvalidInput);
	const std::array<double, 2> InvalidWidths { 160, -1 };
	TestFalse(
		TEXT("負の岸幅を拒否"),
		PlanetCore::PrepareRiverRoutingTerrainGraph(
			WidthDirections,
			WidthEdges,
			WideningBank,
			TerrainSettings,
			TerrainError,
			{},
			{},
			InvalidWidths
		).has_value()
	);
	TestTrue(TEXT("負の岸幅は入力違反"), TerrainError == PlanetCore::RiverRoutingTerrainError::InvalidInput);
	TerrainSettings.MaximumEvaluations = 1;
	TestFalse(
		TEXT("地形評価予算切れで部分網を返さない"),
		PlanetCore::PrepareRiverRoutingTerrainGraph(
			Directions,
			PhysicalEdges,
			[](const PlanetCore::UnitDirection) {
				return 10.0;
			},
			TerrainSettings,
			TerrainError
		).has_value()
	);
	TestTrue(TEXT("地形評価予算切れを区別"), TerrainError == PlanetCore::RiverRoutingTerrainError::EvaluationBudget);
	TerrainSettings.MaximumEvaluations = 1000000;
	TestFalse(
		TEXT("地形場の中断で部分網を返さない"),
		PlanetCore::PrepareRiverRoutingTerrainGraph(
			Directions,
			PhysicalEdges,
			[](const PlanetCore::UnitDirection) {
				return 10.0;
			},
			TerrainSettings,
			TerrainError,
			Cancel.get_token()
		).has_value()
	);
	TestTrue(TEXT("地形場の中断を区別"), TerrainError == PlanetCore::RiverRoutingTerrainError::Cancelled);
	// 全ての有限網の制約を確認した返戻
	return true;
}

#endif
