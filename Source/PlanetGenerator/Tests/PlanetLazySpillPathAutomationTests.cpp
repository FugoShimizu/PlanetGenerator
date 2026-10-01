#include "Core/PlanetSpillPath.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLazySpillPathContracts,
	"PlanetGenerator.SpillPassage.MeasuredPath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 全辺実測の既存算法と，候補の下限が変わる経路・同水位の順序を独立比較する
 * @param Parameters 未使用の実行引数
 * @return 同じ実経路と有限費用を保持し，未実測又は下限違反を拒否する場合に真
 */
bool FPlanetLazySpillPathContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	std::uint64_t MeasuredTotal = 0, FullTotal = 0;
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
		// 独立な全辺参照の失敗への返戻
		if(!TestTrue(TEXT("全辺の参照網を準備"), Reference.has_value())) return false;
		for(std::uint32_t Start = 1; Start < Sites.size(); ++Start) {
			std::vector<unsigned> Calls(Actual.size(), 0);
			const auto Measure = [&](const std::uint32_t Edge) -> std::optional<double> {
				++Calls[Edge];
				// 初期下限とは異なる実際の辺高の返戻
				return Actual[Edge].CrestMeters;
			};
			const std::optional<PlanetCore::MeasuredSpillPath> Path = PlanetCore::FindMeasuredSpillPath(Sites, Bounds, Start, 0, Measure);
			// 一経路でも欠けた場合の返戻
			if(!TestTrue(TEXT("実測済の最小最大路を準備"), Path.has_value())) return false;
			TestEqual(TEXT("全辺版と通過高度が完全一致"), Path->CrestMeters, Reference->Nodes[Start].EscapeMeters, 0);
			std::uint32_t Node = Start;
			for(std::size_t I = 0; I < Path->Edges.size(); ++I) {
				TestEqual(TEXT("同水位も節点順を保持"), Path->Nodes[I], Node);
				TestEqual(TEXT("同水位も既存の辺を保持"), Path->Edges[I], Reference->Nodes[Node].DownstreamEdge);
				TestEqual(TEXT("返す辺は実際に一回測定"), Calls[Path->Edges[I]], 1U);
				Node = Reference->Nodes[Node].Downstream;
			}
			TestEqual(TEXT("根迄の全経路を復元"), Node, 0U);
			TestEqual(TEXT("末尾の実根を保持"), Path->Nodes.back(), 0U);
			TestEqual(TEXT("節点と辺の個数が対応"), Path->Nodes.size(), Path->Edges.size() + 1);
			unsigned Count = 0;
			for(const unsigned Value : Calls) {
				TestTrue(TEXT("各辺の実測は高々一回"), Value < 2);
				Count += Value;
			}
			TestEqual(TEXT("実測件数を外側から照合"), Path->EvaluatedEdges, Count);
			MeasuredTotal += Count;
			FullTotal += Actual.size();
		}
	}
	TestTrue(TEXT("全辺実測より実測回数が少ない"), MeasuredTotal < FullTotal);
	AddInfo(FString::Printf(TEXT("Measured minimax paths: graphs=64 paths=960 measured=%llu full=%llu"), MeasuredTotal, FullTotal));
	// 既知の一経路で，計測不要な遠方辺と拒否経路を検証する
	const std::vector<PlanetCore::SpillGraphSite> Sites { { 0, true }, { 0, false }, { 0, false } };
	const std::vector<PlanetCore::SpillGraphEdge> Bounds { { { 0, 1 }, 0 }, { { 1, 2 }, 0 } };
	unsigned Calls = 0;
	const auto Unit = [&](const std::uint32_t) -> std::optional<double> {
		++Calls;
		// 計測した有効な通過高度の返戻
		return 1;
	};
	const std::optional<PlanetCore::MeasuredSpillPath> Near = PlanetCore::FindMeasuredSpillPath(Sites, Bounds, 1, 0, Unit);
	TestTrue(TEXT("近い経路を受理"), Near.has_value());
	TestEqual(TEXT("先の不要な辺を計測しない"), Calls, 1U);
	const auto Broken = [](const std::uint32_t Edge) -> std::optional<double> {
		// 二辺目の未計測を伝える返戻
		return Edge ? std::optional<double>{} : 1;
	};
	TestFalse(TEXT("途中で計測出来ない経路を拒否"), PlanetCore::FindMeasuredSpillPath(Sites, Bounds, 2, 0, Broken).has_value());
	const auto InvalidBound = [](const std::uint32_t) -> std::optional<double> {
		return -1;
	};
	TestFalse(TEXT("下限の成立しない評価器を拒否"), PlanetCore::FindMeasuredSpillPath(Sites, Bounds, 2, 0, InvalidBound).has_value());
	const auto NonFinite = [](const std::uint32_t) -> std::optional<double> {
		return std::numeric_limits<double>::quiet_NaN();
	};
	TestFalse(TEXT("非有限な通過高度を拒否"), PlanetCore::FindMeasuredSpillPath(Sites, Bounds, 2, 0, NonFinite).has_value());
	std::stop_source Stop;
	Calls = 0;
	const auto Cancel = [&](const std::uint32_t) -> std::optional<double> {
		++Calls;
		Stop.request_stop();
		// 中断直前に測定した有限値の返戻
		return 1;
	};
	TestFalse(
		TEXT("計測途中の中断で部分経路を返さない"),
		PlanetCore::FindMeasuredSpillPath(Sites, Bounds, 2, 0, Cancel, Stop.get_token()).has_value()
	);
	TestEqual(TEXT("中断後の追加実測は零"), Calls, 1U);
	Calls = 0;
	TestFalse(TEXT("開始前の中断を拒否"), PlanetCore::FindMeasuredSpillPath(Sites, Bounds, 2, 0, Unit, Stop.get_token()).has_value());
	TestEqual(TEXT("開始前中断の実測は零"), Calls, 0U);
	const std::span<const PlanetCore::SpillGraphEdge> Disconnected(Bounds.data(), 1);
	TestFalse(TEXT("根から離れた節点の経路を拒否"), PlanetCore::FindMeasuredSpillPath(Sites, Disconnected, 2, 0, Unit).has_value());
	// 全条件の検査結果の返戻
	return !HasAnyErrors();
}

#endif
