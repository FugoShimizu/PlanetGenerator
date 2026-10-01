#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillPassage.h"
#include "Core/PlanetSpillPath.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 費用の診断を除き，通路の全物理量と全方向を詰物無で比較する
	 * @param Passage 同じ元場・窓で準備した実測通路
	 * @return 数値の全ビットを保持した独立比較用の列
	 */
	std::vector<std::uint64_t> CollectPassageValues(const PlanetCore::SpillPassage &Passage) {
		std::vector<std::uint64_t> Values;
		const auto Number = [&](const double Value) -> void {
			Values.push_back(std::bit_cast<std::uint64_t>(Value));
			// 終了
			return;
		};
		const auto Direction = [&](const PlanetCore::UnitDirection D) -> void {
			Number(D.Vector().X);
			Number(D.Vector().Y);
			Number(D.Vector().Z);
			// 終了
			return;
		};
		Direction(Passage.Peak);
		for(
			const double Value :
			{
				Passage.CrestMeters,
				Passage.PathLengthMeters,
				Passage.MaximumIndependentExcessMeters,
				Passage.BoundaryEscapeMeters[0],
				Passage.BoundaryEscapeMeters[1],
				Passage.OutsideLowerBoundMeters
			}
		) Number(Value);
		Values.push_back(Passage.IsGraphInteriorMinimum);
		Values.push_back(Passage.TouchesBoundary);
		Values.push_back(Passage.Nodes);
		Values.push_back(Passage.PathNodes);
		Values.push_back(Passage.TotalEdges);
		Values.push_back(Passage.Path.size());
		for(const PlanetCore::UnitDirection D : Passage.Path) Direction(D);
		Values.push_back(Passage.PassCandidates.size());
		for(const PlanetCore::UnitDirection D : Passage.PassCandidates) Direction(D);
		// 同じ物理結果の全成分の返戻
		return Values;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetPathEdgeSelectionContracts,
	"PlanetGenerator.SpillPassage.PathEdgeSelection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 全辺の独立な排水階層と，候補経路だけを実測する最小最大路を比較する
 * @param Parameters 未使用の実行引数
 * @return 最低通過高度・全経路の実測・有限費用と拒否経路を保持した場合に真
 */
bool FPlanetPathEdgeSelectionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	std::uint64_t SelectedTotal = 0, FullTotal = 0;
	for(std::uint32_t Seed = 0; Seed < 64; ++Seed) {
		std::vector<PlanetCore::SpillGraphSite> Sites;
		for(std::uint32_t I = 0; I < 16; ++I) Sites.push_back({ I ? (Seed + 11 * I) % 7 * 0.125 : 0, !I });
		std::vector<PlanetCore::SpillGraphEdge> Bounds, Actual;
		for(std::uint32_t A = 0; A < Sites.size(); ++A) for(std::uint32_t B = A + 1; B < Sites.size(); ++B) {
			if(B != A + 1 && (A * 13 + B * 19 + Seed) % 3) continue;
			const double Lower = std::max(Sites[A].ElevationMeters, Sites[B].ElevationMeters);
			Bounds.push_back({ { A, B }, Lower });
			Actual.push_back({ { A, B }, Lower + (A * 17 + B * 23 + Seed * 31) % 19 * 0.125 });
		}
		const std::optional<PlanetCore::SpillGraph> Reference = PlanetCore::BuildSpillGraph(Sites, Actual, 0);
		// 独立な全辺参照を構築出来ない場合の返戻
		if(!TestTrue(TEXT("全実辺の独立な参照網"), Reference.has_value())) return false;
		for(std::uint32_t Start = 1; Start < Sites.size(); ++Start) {
			std::vector<unsigned> Calls(Actual.size(), 0);
			const auto Measure = [&](const std::uint32_t Edge) -> std::optional<double> {
				++Calls[Edge];
				// 安価な初期下限とは異なる，独立に準備した実辺の返戻
				return Actual[Edge].CrestMeters;
			};
			const std::optional<PlanetCore::MeasuredSpillPath> Path =
			PlanetCore::FindPathMeasuredSpillPath(Sites, Bounds, Start, 0, Measure);
			// 未実測経路又は失われた接続への返戻
			if(!TestTrue(TEXT("経路の全実辺を確認"), Path.has_value())) return false;
			TestEqual(TEXT("全辺版と最低通過高度が一致"), Path->CrestMeters, Reference->Nodes[Start].EscapeMeters, 0);
			TestEqual(TEXT("始点を保持"), Path->Nodes.front(), Start);
			TestEqual(TEXT("実根を保持"), Path->Nodes.back(), 0U);
			TestEqual(TEXT("節点と辺の数が対応"), Path->Nodes.size(), Path->Edges.size() + 1);
			double Maximum = std::max(Sites[Start].ElevationMeters, Sites[0].ElevationMeters);
			for(std::size_t I = 0; I < Path->Edges.size(); ++I) {
				const std::uint32_t Edge = Path->Edges[I], A = Path->Nodes[I], B = Path->Nodes[I + 1];
				TestEqual(TEXT("返す辺は一回だけ実測"), Calls[Edge], 1U);
				const std::array<std::uint32_t, 2> Ends = Actual[Edge].Nodes;
				TestTrue(TEXT("実辺の両端が連続"), Ends[0] == A && Ends[1] == B || Ends[0] == B && Ends[1] == A);
				Maximum = std::max(Maximum, Actual[Edge].CrestMeters);
			}
			TestEqual(TEXT("全実辺の最大値が解と一致"), Maximum, Path->CrestMeters, 0);
			unsigned Count = 0;
			for(const unsigned Value : Calls) {
				TestTrue(TEXT("経路切替後も各実辺は高々一回"), Value < 2);
				Count += Value;
			}
			TestEqual(TEXT("外側の実測数と一致"), Path->EvaluatedEdges, Count);
			SelectedTotal += Count;
			FullTotal += Actual.size();
		}
	}
	AddInfo(FString::Printf(TEXT("Path-selected minimax graphs=64 paths=960 measured=%llu full=%llu"), SelectedTotal, FullTotal));
	// 平坦な大きい格子で，経路から離れた辺を実測する費用を独立比較
	const std::vector<PlanetCore::SpillGraphSite> FlatSites(1024, { 0, false });
	std::vector<PlanetCore::SpillGraphEdge> Grid;
	for(std::uint32_t Y = 0; Y < 32; ++Y) for(std::uint32_t X = 0; X < 32; ++X) {
		const std::uint32_t I = 32 * Y + X;
		if(X < 31) Grid.push_back({ { I, I + 1 }, 0 });
		if(Y < 31) Grid.push_back({ { I, I + 32 }, 0 });
	}
	std::uint32_t OldCalls = 0, NewCalls = 0;
	const auto OldMeasure = [&](const std::uint32_t) -> std::optional<double> {
		++OldCalls;
		// 同じ解析格子の実通過値の返戻
		return 0;
	};
	const auto NewMeasure = [&](const std::uint32_t) -> std::optional<double> {
		++NewCalls;
		// 同じ解析格子の実通過値の返戻
		return 0;
	};
	const std::optional<PlanetCore::MeasuredSpillPath> Old =
	PlanetCore::FindMeasuredSpillPath(FlatSites, Grid, 1023, 0, OldMeasure);
	const std::optional<PlanetCore::MeasuredSpillPath> Selected =
	PlanetCore::FindPathMeasuredSpillPath(FlatSites, Grid, 1023, 0, NewMeasure);
	// 独立な費用比較の正規経路の返戻
	if(!TestTrue(TEXT("両方式の平坦格子の接続"), Old && Selected)) return false;
	TestEqual(TEXT("平坦格子の通過高度を保持"), Selected->CrestMeters, Old->CrestMeters, 0);
	TestTrue(TEXT("経路外の高価な実測を削減"), NewCalls < OldCalls);
	AddInfo(
		FString::Printf(
			TEXT("Flat path edge evaluations frontier=%u path=%u total=%llu"),
			OldCalls,
			NewCalls,
			static_cast<unsigned long long>(Grid.size())
		)
	);
	// ３２候補が続けて上昇する網は，同じ値を保持して既存の辺確定へ移る
	std::vector<PlanetCore::SpillGraphSite> FanSites(43, { 0, false });
	std::vector<PlanetCore::SpillGraphEdge> Fan;
	for(std::uint32_t Node = 2; Node < 43; ++Node) {
		Fan.push_back({ { 0, Node }, 0 });
		Fan.push_back({ { Node, 1 }, 0 });
	}
	std::vector<unsigned> FanCalls(Fan.size(), 0);
	const auto FanMeasure = [&](const std::uint32_t Edge) -> std::optional<double> {
		++FanCalls[Edge];
		// 全候補の下限を上げる，独立な扇状網の実辺の返戻
		return Edge % 2 ? 0 : Edge * 0.5 + 1;
	};
	const std::optional<PlanetCore::MeasuredSpillPath> Fallback =
	PlanetCore::FindPathMeasuredSpillPath(FanSites, Fan, 1, 0, FanMeasure);
	// 繰返し後に正規経路を失った場合の返戻
	if(!TestTrue(TEXT("繰返し後も全辺実測の最低経路"), Fallback.has_value())) return false;
	TestEqual(TEXT("切替後の独立な最低通過高度"), Fallback->CrestMeters, 1.0, 0);
	for(const unsigned Count : FanCalls) TestTrue(TEXT("切替でも再実測しない"), Count < 2);
	const std::vector<PlanetCore::SpillGraphSite> Sites { { 0, true }, { 0, false }, { 0, false } };
	const std::vector<PlanetCore::SpillGraphEdge> Bounds { { { 0, 1 }, 0 }, { { 1, 2 }, 0 } };
	const auto Missing = [](const std::uint32_t Edge) -> std::optional<double> {
		// 部分的な元場しか無い照会の返戻
		return Edge ? std::optional<double>{} : 1;
	};
	const auto Invalid = [](const std::uint32_t) -> std::optional<double> {
		// 成立する下限を破る検査値の返戻
		return -1;
	};
	const auto NonFinite = [](const std::uint32_t) -> std::optional<double> {
		// 非有限な実測値の返戻
		return std::numeric_limits<double>::quiet_NaN();
	};
	TestFalse(TEXT("未計測の途中辺を拒否"), PlanetCore::FindPathMeasuredSpillPath(Sites, Bounds, 2, 0, Missing).has_value());
	TestFalse(TEXT("下限違反を拒否"), PlanetCore::FindPathMeasuredSpillPath(Sites, Bounds, 2, 0, Invalid).has_value());
	TestFalse(TEXT("非有限値を拒否"), PlanetCore::FindPathMeasuredSpillPath(Sites, Bounds, 2, 0, NonFinite).has_value());
	std::stop_source Stop;
	unsigned CancelCalls = 0;
	const auto Cancel = [&](const std::uint32_t) -> std::optional<double> {
		++CancelCalls;
		Stop.request_stop();
		// 途中の中断直前に測った値の返戻
		return 1;
	};
	TestFalse(
		TEXT("途中中断の部分経路を拒否"),
		PlanetCore::FindPathMeasuredSpillPath(Sites, Bounds, 2, 0, Cancel, Stop.get_token()).has_value()
	);
	TestEqual(TEXT("途中中断後の追加照会は零"), CancelCalls, 1U);
	TestFalse(TEXT("開始前中断を拒否"), PlanetCore::FindPathMeasuredSpillPath(Sites, Bounds, 2, 0, Cancel, Stop.get_token()).has_value());
	TestEqual(TEXT("開始前中断では照会しない"), CancelCalls, 1U);
	const std::span<const PlanetCore::SpillGraphEdge> Disconnected(Bounds.data(), 1);
	TestFalse(TEXT("未接続の節点を拒否"), PlanetCore::FindPathMeasuredSpillPath(Sites, Disconnected, 2, 0, Missing).has_value());
	const std::vector<PlanetCore::SpillGraphEdge> BadNode { { { 0, 3 }, 0 } };
	TestFalse(TEXT("不正な辺番号を拒否"), PlanetCore::FindPathMeasuredSpillPath(Sites, BadNode, 2, 0, Missing).has_value());
	// 独立な最小値と正規・拒否の経路の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetPathSelectedPassageContracts,
	"PlanetGenerator.SpillPassage.PathSelectedPassage",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 両端通路の経路選択を，従来順と全辺版の全数値成分で独立比較する
 * @param Parameters 未使用の実行引数
 * @return 三半径の通過高度・外周条件・物理経路・候補が全ビット一致する場合に真
 */
bool FPlanetPathSelectedPassageContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::vector<std::uint64_t> Expected;
		std::uint64_t Baseline = 0;
		for(unsigned Mode = 0; Mode < 3; ++Mode) {
			std::uint64_t Calls = 0;
			const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
				++Calls;
				const PlanetCore::Vec3 D = Direction.Vector();
				const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
				// 空間刻みや勾配を変えない，同じ解析的な稜線場の返戻
				return {
					4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400,
					{ Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) }
				};
			};
			const PlanetCore::UnitDirection From = *PlanetCore::UnitDirection::FromVector({ -90 / Radius, 0, 1 });
			const PlanetCore::UnitDirection To = *PlanetCore::UnitDirection::FromVector({ 90 / Radius, 0, 1 });
			PlanetCore::SpillPassageSettings Settings;
			Settings.RadiusMeters = Radius;
			Settings.HalfWidthMeters = std::min(256.0, Radius * 0.125);
			Settings.SpacingMeters = 16;
			Settings.MaximumSampleStepMeters = 4;
			Settings.UsePathEdgeSelection = Mode == 1;
			Settings.MeasureAllPassageEdges = Mode == 2;
			const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(From, To, Source, Settings);
			// 比較する方式の正規通路を準備出来ない場合の返戻
			if(!TestTrue(TEXT("各方式の同じ実通路"), Passage.has_value())) return false;
			const std::vector<std::uint64_t> Values = CollectPassageValues(*Passage);
			if(!Mode) {
				Expected = Values;
				Baseline = Calls;
			} else TestTrue(TEXT("最高点・外周値・全経路・候補がビット一致"), Values == Expected);
			AddInfo(
				FString::Printf(
					TEXT("Path-selected passage R=%.0f mode=%u calls=%llu baseline=%llu edges=%u/%u components=%llu"),
					Radius,
					Mode,
					Calls,
					Baseline,
					Passage->EvaluatedEdges,
					Passage->TotalEdges,
					static_cast<unsigned long long>(Values.size())
				)
			);
		}
	}
	// 同じ元場の数値定義の検査結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetPathSelectedActualPassage,
	"PlanetGenerator.SpillPassage.PathSelectedActual",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 広域検査の失敗地点の同じ実峠と両極小で，元の辺精度と物理通路の一致を検査する
 * @param Parameters 未使用の実行引数
 * @return 同じ地形上の従来順と経路選択で，全通路と外周値がビット一致する場合に真
 */
bool FPlanetPathSelectedActualPassage::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::array<std::uint64_t, 3> Seeds { 0, 7, 90210 };
	const std::array<PlanetCore::Vec3, 3> Points {
		PlanetCore::Vec3{ 0.45055058299527795, -0.032545329714121066, 0.89215748255362071 },
		PlanetCore::Vec3{ -0.21971169009904129, 0.50510266972800977, 0.83462450615079598 },
		PlanetCore::Vec3{ -0.047324433481050726, 0.12584921208650446, 0.9909199633738861 }
	};
	for(unsigned Index = 0; Index < Seeds.size(); ++Index) {
		PlanetCore::GeologySettings Definition;
		Definition.RadiusMeters = 1E6;
		Definition.Seed = Seeds[Index];
		std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
		// 記録した Seed の地質を欠く場合の返戻
		if(!TestTrue(TEXT("同じ元地質の準備"), Base.has_value())) return false;
		const std::shared_ptr<const PlanetCore::GeologyField> Geology =
		std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
		std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
		// 元の造山場を欠く場合の返戻
		if(!TestTrue(TEXT("同じ造山場の準備"), Ridges.has_value())) return false;
		const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
			Geology,
			std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
			{},
			PlanetCore::PredrainageReliefMode::MesoAndLand,
			PlanetCore::OceanSamplingMode::SharedDrainage
		);
		// 元の海域接続を欠く場合の返戻
		if(!TestTrue(TEXT("同じ海域と起伏の準備"), Ocean.has_value())) return false;
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			// 編集していない元地形の標高と全勾配の返戻
			return Ocean->Evaluate(D);
		};
		const PlanetCore::UnitDirection Point = *PlanetCore::UnitDirection::FromVector(Points[Index]);
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.MaximumBranchLengthMeters = 100000;
		const std::optional<PlanetCore::SpillConnection> Connection =
		PlanetCore::PrepareSpillConnection(Point, Source, ConnectionSettings);
		// 実峠と両枝を元場で再確認出来ない場合の返戻
		if(!TestTrue(TEXT("同じ実峠と両下降枝"), Connection.has_value())) return false;
		const bool HasMinima = Connection->Branches[0].Minimum.has_value() && Connection->Branches[1].Minimum.has_value();
		// 未確認の両端を別の極小へ置換えない返戻
		if(!TestTrue(TEXT("記録した流出口の両極小"), HasMinima)) return false;
		std::vector<std::uint64_t> Expected;
		for(unsigned Mode = 0; Mode < 2; ++Mode) {
			Calls = 0;
			PlanetCore::SpillPassageSettings Settings;
			Settings.RadiusMeters = Definition.RadiusMeters;
			Settings.HalfWidthMeters = 256;
			Settings.SpacingMeters = 16;
			Settings.MaximumSampleStepMeters = 4;
			Settings.UsePathEdgeSelection = Mode == 1;
			const double Started = FPlatformTime::Seconds();
			const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(
				Connection->Branches[0].Minimum->Direction,
				Connection->Branches[1].Minimum->Direction,
				Source,
				Settings
			);
			// 同じ窓で正規の通路を作れない場合の返戻
			if(!TestTrue(TEXT("同じ実地形の通路"), Passage.has_value())) return false;
			const std::vector<std::uint64_t> Values = CollectPassageValues(*Passage);
			if(!Mode) Expected = Values;
			else TestTrue(TEXT("実地形の全通路・外周値がビット一致"), Values == Expected);
			AddInfo(
				FString::Printf(
					TEXT("Actual path selection seed=%llu mode=%u pass=%.17g crest=%.17g calls=%llu edges=%u/%u ms=%.9g components=%llu"),
					Seeds[Index],
					Mode,
					Connection->Pass.ElevationMeters,
					Passage->CrestMeters,
					Calls,
					Passage->EvaluatedEdges,
					Passage->TotalEdges,
					(FPlatformTime::Seconds() - Started) * 1000,
					static_cast<unsigned long long>(Values.size())
				)
			);
		}
	}
	// 実地形の独立比較の検査結果の返戻
	return !HasAnyErrors();
}

#endif
