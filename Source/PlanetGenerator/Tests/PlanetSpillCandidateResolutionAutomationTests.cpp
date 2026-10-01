#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillSeed.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_COMPLEX_AUTOMATION_TEST(
	FPlanetSpillCandidateResolution,
	"PlanetGenerator.ContinuousSpill.CandidateResolution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 異なる元場の候補探索を個別に測る
 * @param Names 表示名
 * @param Commands 元の失敗条件と同じ Seed
 */
void FPlanetSpillCandidateResolution::GetTests(TArray<FString> &Names, TArray<FString> &Commands) const {
	for(const FString &Seed : { FString(TEXT("0")), FString(TEXT("7")), FString(TEXT("90210")) }) {
		Names.Add(TEXT("Seed") + Seed);
		Commands.Add(Seed);
	}
	// 登録の終了
	return;
}

/**
 * 最初の陸上地点から準備した同じ実極小で，候補格子の密度と実接続の費用を比較する
 * @param Parameters 登録済の生成 Seed
 * @return 固定３密度と可変密度で元場の実鞍点・実終端を確認した接続を準備出来た場合に真
 */
bool FPlanetSpillCandidateResolution::RunTest(const FString &Parameters) {
	// 未登録の入力を別条件へ置換えない
	if(!TestTrue(TEXT("登録済の Seed"), Parameters == TEXT("0") || Parameters == TEXT("7") || Parameters == TEXT("90210"))) {
		// 無効な指定への返戻
		return false;
	}
	PlanetCore::GeologySettings Definition;
	Definition.Seed = Parameters == TEXT("0") ? 0 : Parameters == TEXT("7") ? 7 : 90210;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 比較の元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較の地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山場の欠落への返戻
	if(!TestTrue(TEXT("比較の造山場"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 未準備の元場への返戻
	if(!TestTrue(TEXT("比較の大洋場"), Ocean.has_value())) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		++Calls;
		// 同じ標高と解析勾配の返戻
		return Ocean->Evaluate(D);
	};
	std::optional<PlanetCore::UnitDirection> Location;
	for(unsigned Candidate = 0; Candidate < 256; ++Candidate) {
		const double Z = 1 - 2 * (Candidate + 0.5) / 256, R = std::sqrt(1 - Z * Z), Angle = Candidate * 2.39996322972865332;
		const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector({ R * std::cos(Angle), R * std::sin(Angle), Z });
		if(Ocean->EvaluateElevation(D) > 20) {
			Location = D;
			break;
		}
	}
	// 都合の良い場所へ選び直さない返戻
	if(!TestTrue(TEXT("元の最初の陸上地点"), Location.has_value())) return false;
	const std::optional<PlanetCore::SpillSeed> Seed = PlanetCore::FindSpillSeed(*Location, Source, {});
	// 同じ実極小を準備出来ない場合の返戻
	if(!TestTrue(TEXT("地点からの実極小と鞍点"), Seed.has_value())) return false;
	std::vector<PlanetCore::TerrainCriticalPoint> Minima;
	for(const PlanetCore::SpillConnectionBranch &Branch : Seed->Connection.Branches) {
		if(Branch.Minimum) Minima.push_back(*Branch.Minimum);
	}
	// 内陸の比較条件を失った場合の返戻
	if(!TestTrue(TEXT("比較する実極小"), !Minima.empty())) return false;
	std::optional<PlanetCore::UnitDirection> Target;
	double Nearest = std::numeric_limits<double>::infinity();
	for(const PlanetCore::UnitDirection D : Ocean->ConnectedOceanAnchors()) {
		if(!(Ocean->EvaluateElevation(D) < -1)) continue;
		const PlanetCore::Vec3 A = Seed->Connection.Pass.Direction.Vector(), B = D.Vector();
		const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
		if(Distance < Nearest) {
			Target = D;
			Nearest = Distance;
		}
	}
	// 目標方向の欠落への返戻
	if(!TestTrue(TEXT("元の水没した大洋標本"), Target.has_value())) return false;
	const std::array<std::array<std::uint32_t, 2>, 4> Grids {
		std::array<std::uint32_t, 2>{ 16, 0 },
		{ 32, 0 },
		{ 64, 0 },
		{ 64, 16 }
	};
	for(const std::array<std::uint32_t, 2> Grid : Grids) {
		const std::uint32_t HalfCells = Grid[0];
		Calls = 0;
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.HalfCells = HalfCells;
		Settings.CandidateSpacingMeters = Grid[1];
		Settings.MaximumHalfWidthMeters = 65536;
		Settings.MaximumEvaluations = 4000000;
		Settings.OutletTarget = Target;
		PlanetCore::SpillDiscoveryDiagnostics Report;
		const double Started = FPlatformTime::Seconds();
		const std::optional<PlanetCore::SpillDiscoveryBatch> Batch = PlanetCore::FindSpillComponentExits(
			Minima,
			Source,
			Settings,
			{},
			&Report,
			std::span<const PlanetCore::SpillConnection>(&Seed->Connection, 1)
		);
		AddInfo(
			FString::Printf(
				TEXT("Candidate resolution seed=%llu halfCells=%u targetSpacing=%.0f actualHalfCells=%u accepted=%d ")
				TEXT("evaluations=%llu windows=%u width=%.9g ")
				TEXT("connections=%llu newMinima=%llu criticalFailures=%u branchFailures=%u relaxed=%u bridges=%u ms=%.9g"),
				Definition.Seed,
				HalfCells,
				Settings.CandidateSpacingMeters,
				Report.LastHalfCells,
				static_cast<int>(Batch.has_value()),
				Calls,
				Report.Windows,
				Report.LastHalfWidthMeters,
				static_cast<unsigned long long>(Batch ? Batch->Connections.size() : 0),
				static_cast<unsigned long long>(Batch ? Batch->NewMinima.size() : 0),
				Report.UnresolvedCriticalPoints,
				Report.UnresolvedConnections,
				Report.RelaxedWindows,
				Report.RelaxedBridges,
				(FPlatformTime::Seconds() - Started) * 1000
			)
		);
		TestEqual(TEXT("元場を呼んだ実費用"), Calls, Report.Evaluations);
		TestTrue(
			TEXT("同じ入力の外へ繋がる実接続"),
			Batch && !Batch->Connections.empty() && (!Batch->NewMinima.empty() || Batch->HasSeaHeightTerminal)
		);
		if(!Batch) continue;
		for(const PlanetCore::SpillConnection &Connection : Batch->Connections) {
			TestTrue(TEXT("格子の極大で代用しない実鞍点"), Connection.Pass.Kind == PlanetCore::TerrainCriticalKind::Saddle);
			TestEqual(TEXT("同じ元標高を保持"), Connection.Pass.ElevationMeters, Ocean->EvaluateElevation(Connection.Pass.Direction));
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				TestTrue(TEXT("両下降枝の実終端を保持"), Branch.Kind == PlanetCore::SpillTerminalKind::SeaLevel || Branch.Minimum.has_value());
			}
		}
	}
	// 節点数の計算前に，非有限・負・過小な物理間隔を拒否
	for(
		const double InvalidSpacing :
		{ -1.0, 0.001, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() }
	) {
		Calls = 0;
		PlanetCore::SpillDiscoverySettings Invalid;
		Invalid.CandidateSpacingMeters = InvalidSpacing;
		PlanetCore::SpillDiscoveryDiagnostics Report;
		TestFalse(TEXT("不正な可変密度の設定を拒否"), PlanetCore::FindSpillComponentExits(Minima, Source, Invalid, {}, &Report).has_value());
		TestTrue(TEXT("入力不正と探索未発見を区別"), Report.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
		TestEqual(TEXT("不正な間隔で元場を呼出さない"), Calls, UINT64_C(0));
	}
	// 候補密度を変えても実接続の確認を省略しない検査結果の返戻
	return !HasAnyErrors();
}

#endif
