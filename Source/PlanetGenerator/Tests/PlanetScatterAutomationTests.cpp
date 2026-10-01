#include "Core/PlanetScatter.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <limits>
#include <numbers>
#include <set>
#include <tuple>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetScatterContracts, "PlanetGenerator.Scatter.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

namespace {
	using FCellKey = std::tuple<std::uint8_t, std::uint64_t, std::uint64_t>;

	/**
	 * 比較用の順序を持つ識別子へ変換する
	 * @param Cell 面とセル番号
	 * @return 同一性を表す三成分
	 */
	FCellKey CellKey(const PlanetCore::ScatterCell Cell) {
		// 識別子の返戻
		return { Cell.Face, Cell.U, Cell.V };
	}
}

/**
 * 独立した全球列挙と近傍検索を比較し，面境界・大半径・再生成・予算を検査する
 * @param Parameters UE 検査の引数
 * @return 検査を実行出来た場合に真
 */
bool FPlanetScatterContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Start = FPlatformTime::Seconds();
	PlanetCore::ScatterSettings Settings;
	Settings.RadiusMeters = 1000;
	Settings.SpacingMeters = 62.5;
	const std::optional<PlanetCore::PlanetScatter> Scatter = PlanetCore::PlanetScatter::Create(Settings);
	// 有効な配置器を準備出来ない場合の返戻
	if(!TestTrue(TEXT("有効な配置定義を準備"), Scatter.has_value())) return false;
	std::vector<PlanetCore::ScatterCandidate> All;
	for(std::uint8_t Face = 0; Face < 6; ++Face) {
		for(std::uint64_t x = 0; x < 32; ++x) {
			for(std::uint64_t y = 0; y < 32; ++y) {
				if(const std::optional<PlanetCore::ScatterCandidate> Candidate = Scatter->Candidate({ x, y, Face })) {
					All.push_back(*Candidate);
				}
			}
		}
	}
	std::uint64_t Compared = 0;
	for(const PlanetCore::Vec3 Axis : std::array<PlanetCore::Vec3, 8>{
		{ { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 1, 1, 0 }, { -1, 1, 1 }, { 1, 1, 1 }, { 0.1, -0.3, 1 } }
	}) {
		const PlanetCore::UnitDirection Center = *PlanetCore::UnitDirection::FromVector(Axis);
		for(const double Radius : { 0.0, 40.0, 350.0, 1800.0, 4000.0 }) {
			const std::optional<PlanetCore::ScatterQueryResult> Query = Scatter->Query(Center, Radius, 6144);
			// 正規の範囲検索を実行出来ない場合の返戻
			if(!TestTrue(TEXT("予算内の範囲検索が完了"), Query.has_value())) return false;
			std::set<FCellKey> Expected, Actual;
			for(const PlanetCore::ScatterCandidate &Candidate : All) {
				const PlanetCore::Vec3 Point = Candidate.Direction.Vector(), C = Center.Vector();
				// 検索実装の弦距離式を使わず，外積と内積から球面距離を求める
				const double Distance = Settings.RadiusMeters * std::atan2(C.Cross(Point).Length(), C.Dot(Point));
				if(Distance <= Radius) Expected.insert(CellKey(Candidate.Cell));
				++Compared;
			}
			for(const PlanetCore::ScatterCandidate &Candidate : Query->Candidates) Actual.insert(CellKey(Candidate.Cell));
			TestTrue(TEXT("面境界を含め全球列挙と検索が一致"), Expected == Actual);
			TestEqual(TEXT("重複した候補が無い"), Actual.size(), Query->Candidates.size());
		}
	}
	const PlanetCore::UnitDirection Pole = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	TestFalse(TEXT("セル数の上限を超えた部分結果を公開しない"), Scatter->Query(Pole, 4000, 1).has_value());
	TestFalse(TEXT("負の範囲を拒否"), Scatter->Query(Pole, -1, 10000).has_value());
	TestFalse(TEXT("非有限の範囲を拒否"), Scatter->Query(Pole, std::numeric_limits<double>::infinity(), 10000).has_value());
	std::stop_source Cancellation;
	Cancellation.request_stop();
	TestFalse(TEXT("中断要求を反映"), Scatter->Query(Pole, 100, 10000, Cancellation.get_token()).has_value());
	std::uint64_t MaximumExamined = 0, Repeated = 0;
	for(const double Radius : { 1000.0, 1000000.0, 1000000000.0 }) {
		Settings.RadiusMeters = Radius;
		Settings.SpacingMeters = 0.75;
		const PlanetCore::PlanetScatter Fine = *PlanetCore::PlanetScatter::Create(Settings);
		for(const PlanetCore::Vec3 Axis : std::array<PlanetCore::Vec3, 3>{ { { 0, 0, 1 }, { 1, 1, 0 }, { 1, 1, 1 } } }) {
			const PlanetCore::UnitDirection Center = *PlanetCore::UnitDirection::FromVector(Axis);
			const PlanetCore::UnitDirection Moved = *PlanetCore::UnitDirection::FromVector(Axis + PlanetCore::Vec3{ 5 / Radius, 0, 0 });
			const std::optional<PlanetCore::ScatterQueryResult> First = Fine.Query(Center, 18, 50000);
			const std::optional<PlanetCore::ScatterQueryResult> Second = Fine.Query(Moved, 26, 50000);
			// 物理寸法が同じ近傍検索が完了しない場合の返戻
			if(!TestTrue(TEXT("大半径でも同じ近傍予算で検索"), First && Second)) return false;
			MaximumExamined = std::max({ MaximumExamined, First->ExaminedCells, Second->ExaminedCells });
			TestTrue(TEXT("近傍に候補が有る"), !First->Candidates.empty());
			std::set<FCellKey> Larger;
			for(const PlanetCore::ScatterCandidate &Candidate : Second->Candidates) Larger.insert(CellKey(Candidate.Cell));
			for(const PlanetCore::ScatterCandidate &Candidate : First->Candidates) {
				TestTrue(TEXT("包含する移動後の範囲に同じ候補が残る"), Larger.contains(CellKey(Candidate.Cell)));
				const std::optional<PlanetCore::ScatterCandidate> Again = Fine.Candidate(Candidate.Cell);
				// 同じセルを再現出来ない場合の返戻
				if(!TestTrue(TEXT("同じセルを再生成"), Again.has_value())) return false;
				const PlanetCore::Vec3 A = Candidate.Direction.Vector(), B = Again->Direction.Vector();
				TestTrue(TEXT("再生成で位置・回転・大きさ・選択値を保持"), A.X == B.X && A.Y == B.Y && A.Z == B.Z &&
					Candidate.RotationTurns == Again->RotationTurns && Candidate.Size == Again->Size &&
					Candidate.Selection == Again->Selection);
				++Repeated;
			}
		}
	}
	Settings.RadiusMeters = 999;
	TestFalse(TEXT("最小半径未満を拒否"), PlanetCore::PlanetScatter::Create(Settings).has_value());
	Settings.RadiusMeters = 1000;
	Settings.SpacingMeters = 0;
	TestFalse(TEXT("間隔０を拒否"), PlanetCore::PlanetScatter::Create(Settings).has_value());
	Settings.SpacingMeters = 1.0e-20;
	TestFalse(TEXT("精度範囲を越える分割数を拒否"), PlanetCore::PlanetScatter::Create(Settings).has_value());
	AddInfo(FString::Printf(TEXT("Scatter contracts: global comparisons=%llu repeated=%llu max cells=%llu seconds=%.6f"),
		Compared, Repeated, MaximumExamined, FPlatformTime::Seconds() - Start));
	// 検査の実行完了の返戻
	return true;
}
#endif
