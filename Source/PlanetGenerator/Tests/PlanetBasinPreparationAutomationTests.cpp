#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetBasinPreparation.h"
#include "Core/PlanetOrogenicRidges.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinPreparationContracts, "PlanetGenerator.BasinReconstruction.Preparation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 再利用有無の同じ生成を照合し，互いに重ならない準備段階の時間を記録する
 * @param Parameters 未使用の UE 検査引数
 * @return 時間内訳と地形値の不変性を満たす場合に真，速度の合格は別に判定する
 */
bool FPlanetBasinPreparationContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::BasinPreparationPhase;
	using PlanetCore::BasinPreparationStatistics;
	using PlanetCore::RiverTerrainField;
	using PlanetCore::UnitDirection;
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Settings);
	// 元の地質を準備出来ない場合の返戻
	if(!TestTrue(TEXT("計測用の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(*PreparedGeology);
	const std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 共通の先行山地を準備出来ない場合の返戻
	if(!TestTrue(TEXT("計測用の先行山地を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*PreparedRidges);
	const std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges);
	// 海域を準備出来ない場合の返戻
	if(!TestTrue(TEXT("計測用の接続済み大洋を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedOcean);
	const std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 同じ排水網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("計測用の共有排水を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedDrainage);
	BasinPreparationStatistics Statistics;
	const std::optional<RiverTerrainField> Profiled = RiverTerrainField::CreateWithBasins(Ocean, Drainage, {}, &Statistics);
	// 計測有りの正規経路が生成出来ない場合の返戻
	if(!TestTrue(TEXT("計測付きでも通常の地形を準備"), Profiled.has_value() && Statistics.Completed)) return false;
	const double Started = FPlatformTime::Seconds();
	BasinPreparationStatistics Uncached;
	const std::optional<RiverTerrainField> Plain = RiverTerrainField::CreateWithBasins(Ocean, Drainage, {}, &Uncached, 0);
	const double PlainMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	// 既存の計測無しの経路が生成出来ない場合の返戻
	if(!TestTrue(TEXT("保持数０の比較経路も地形を準備"), Plain.has_value())) return false;
	for(const double Milliseconds : Statistics.PhaseMilliseconds) {
		TestTrue(TEXT("各段階の時間が有限且つ非負"), std::isfinite(Milliseconds) && Milliseconds >= 0);
	}
	const double Sum = std::accumulate(Statistics.PhaseMilliseconds.begin(), Statistics.PhaseMilliseconds.end(), 0.0);
	TestTrue(
		TEXT("相互に重ならない段階時間の和が準備全体と一致"),
		std::abs(Sum - Statistics.TotalMilliseconds) <= 1E-9 * std::max(1.0, Statistics.TotalMilliseconds)
	);
	TestTrue(TEXT("元地形の実評価を計数"), Statistics.OriginalHeightEvaluations > Drainage->Nodes().size());
	TestEqual(TEXT("再利用有無で公開河川数が一致"), Profiled->Drainage().Reaches().size(), Plain->Drainage().Reaches().size());
	// 誤差許容付きの数値比較を使わず，符号付き０も含む格納ビットを照合する
	const auto SameBits = [this](const TCHAR *Description, const double A, const double B) -> bool {
		// 同じ入力から同じ出力を保持した場合だけ成功する返戻
		return TestEqual(Description, std::bit_cast<std::uint64_t>(A), std::bit_cast<std::uint64_t>(B));
	};
	SameBits(TEXT("再利用有無で標高範囲が一致"), Profiled->AbsoluteReliefBoundMeters(), Plain->AbsoluteReliefBoundMeters());
	for(unsigned Index = 0; Index < 3072; ++Index) {
		const double Z = 1 - 2 * (Index + 0.5) / 3072, Radius = std::sqrt(std::max(0.0, 1 - Z * Z));
		const double Angle = Index * 2.39996322972865332;
		const UnitDirection Point = *UnitDirection::FromVector({ Radius * std::cos(Angle), Radius * std::sin(Angle), Z });
		const PlanetCore::RiverSurfaceSample A = Profiled->EvaluateSurface(Point), B = Plain->EvaluateSurface(Point);
		SameBits(TEXT("再利用有無で独立地点の標高が完全一致"), A.Terrain.ElevationMeters, B.Terrain.ElevationMeters);
		SameBits(TEXT("再利用有無で独立地点の微分Ｘが完全一致"), A.Terrain.TangentialGradientPerRadian.X, B.Terrain.TangentialGradientPerRadian.X);
		SameBits(TEXT("再利用有無で独立地点の微分Ｙが完全一致"), A.Terrain.TangentialGradientPerRadian.Y, B.Terrain.TangentialGradientPerRadian.Y);
		SameBits(TEXT("再利用有無で独立地点の微分Ｚが完全一致"), A.Terrain.TangentialGradientPerRadian.Z, B.Terrain.TangentialGradientPerRadian.Z);
		SameBits(TEXT("再利用有無で独立地点の水位が完全一致"), A.WaterElevationMeters, B.WaterElevationMeters);
		SameBits(TEXT("再利用有無で独立地点の水面支持が完全一致"), A.WaterSupport, B.WaterSupport);
	}
	for(std::size_t Reach = 0; Reach < Profiled->Drainage().Reaches().size(); ++Reach) {
		for(const double Fraction : { 0.0, 0.173, 0.507, 1.0 }) {
			const std::optional<PlanetCore::RiverHeights> A = Profiled->Drainage().SampleHeights(Reach, Fraction);
			const std::optional<PlanetCore::RiverHeights> B = Plain->Drainage().SampleHeights(Reach, Fraction);
			// 不正な公開河川を比較しない返戻
			if(!TestTrue(TEXT("両経路の同じ河川内部を照会"), A.has_value() && B.has_value())) return false;
			SameBits(TEXT("再利用有無で河川内部の水位が完全一致"), A->WaterMeters, B->WaterMeters);
			SameBits(TEXT("再利用有無で河川内部の河床が完全一致"), A->BedMeters, B->BedMeters);
			SameBits(TEXT("再利用有無で河川内部の水面勾配が完全一致"), A->WaterSlope, B->WaterSlope);
			SameBits(TEXT("再利用有無で河川内部の河床勾配が完全一致"), A->BedSlope, B->BedSlope);
		}
	}
	const auto PhaseTime = [&](const BasinPreparationPhase Phase) -> double {
		// 計測した同じ段階の時間の返戻
		return Statistics.PhaseMilliseconds[static_cast<std::size_t>(Phase)];
	};
	AddInfo(
		FString::Printf(
			TEXT("Basin preparation: total=%.6fms plain=%.6fms balanced=%.6fms coastal=%.6fms floorSamples=%.6fms ")
			TEXT("initialFill=%.6fms constraintAssembly=%.6fms constraintApplication=%.6fms finalProfiles=%.6fms ")
			TEXT("fieldAssembly=%.6fms originalEvaluations=%llu reaches=%llu independentSites=3072. ")
			TEXT("Same definition and exact cached/uncached terrain/profile equality; ")
			TEXT("this is a preparation breakdown, not performance acceptance."),
			Statistics.TotalMilliseconds,
			PlainMilliseconds,
			PhaseTime(BasinPreparationPhase::BalancedProfiles),
			PhaseTime(BasinPreparationPhase::CoastalProfiles),
			PhaseTime(BasinPreparationPhase::FloorSamples),
			PhaseTime(BasinPreparationPhase::InitialFill),
			PhaseTime(BasinPreparationPhase::ConstraintAssembly),
			PhaseTime(BasinPreparationPhase::ConstraintApplication),
			PhaseTime(BasinPreparationPhase::FinalProfiles),
			PhaseTime(BasinPreparationPhase::FieldAssembly),
			static_cast<unsigned long long>(Statistics.OriginalHeightEvaluations),
			static_cast<unsigned long long>(Profiled->Drainage().Reaches().size())
		)
	);
	TestTrue(TEXT("元地形の評価を実際に再利用"), Statistics.OriginalHeightHits > 0);
	TestEqual(
		TEXT("再利用前後で全ての照会を保持"),
		Statistics.OriginalHeightEvaluations + Statistics.OriginalHeightHits,
		Uncached.OriginalHeightEvaluations
	);
	TestEqual(TEXT("保持数０は再利用を行わない"), Uncached.OriginalHeightHits, std::uint64_t{ 0 });
	TestEqual(TEXT("元標高の保持を64MiB以内へ制限"), Statistics.HeightCacheBytes, std::size_t{ 67108864 });
	std::array<double, 3> CachedTimes { Statistics.TotalMilliseconds, 0, 0 }, UncachedTimes { Uncached.TotalMilliseconds, 0, 0 };
	for(unsigned Trial = 1; Trial < 3; ++Trial) for(unsigned Order = 0; Order < 2; ++Order) {
		const bool Cached = (Trial + Order) % 2 == 0;
		BasinPreparationStatistics Repeated;
		const std::optional<RiverTerrainField> Fresh =
		RiverTerrainField::CreateWithBasins(Ocean, Drainage, {}, &Repeated, Cached ? 2097152 : 0);
		// 同じ作業量を再生成出来ない計測を成功としない返戻
		if(!TestTrue(TEXT("毎回空の保持枠から同じ地形を再生成"), Fresh.has_value() && Repeated.Completed)) return false;
		(Cached ? CachedTimes : UncachedTimes)[Trial] = Repeated.TotalMilliseconds;
		TestEqual(
			TEXT("実行順を変えても照会数が一致"),
			Repeated.OriginalHeightEvaluations + Repeated.OriginalHeightHits,
			Uncached.OriginalHeightEvaluations
		);
		for(unsigned Index = 0; Index < 256; ++Index) {
			const double Z = 1 - 2 * (Index + 0.5) / 256, Radius = std::sqrt(std::max(0.0, 1 - Z * Z)), Angle = Index * 2.39996322972865332;
			const UnitDirection Point = *UnitDirection::FromVector({ Radius * std::cos(Angle), Radius * std::sin(Angle), Z });
			SameBits(TEXT("繰返しの順序で標高を変えない"), Fresh->EvaluateElevation(Point), Plain->EvaluateElevation(Point));
		}
	}
	BasinPreparationStatistics TinyStatistics;
	const std::optional<RiverTerrainField> Tiny = RiverTerrainField::CreateWithBasins(Ocean, Drainage, {}, &TinyStatistics, 1);
	// 追出しが多い小容量も正規の入力として保持する返戻
	if(!TestTrue(TEXT("保持枠１でも同じ地形を準備"), Tiny.has_value() && TinyStatistics.Completed)) return false;
	for(std::size_t Reach = 0; Reach < Plain->Drainage().Reaches().size(); ++Reach) {
		const std::optional<PlanetCore::RiverProfileSample> Point = Plain->Drainage().SampleReach(Reach, 0.317);
		// 比較する実河床の位置を求められない場合の返戻
		if(!TestTrue(TEXT("追出し検査の実河道を照会"), Point.has_value())) return false;
		const UnitDirection Direction = *UnitDirection::FromVector(Point->Direction);
		SameBits(TEXT("追出しや衝突で実河床の標高を変えない"), Tiny->EvaluateElevation(Direction), Plain->EvaluateElevation(Direction));
	}
	AddInfo(
		FString::Printf(
			TEXT("Basin height reuse: cached=[%.6f,%.6f,%.6f]ms uncached=[%.6f,%.6f,%.6f]ms ")
			TEXT("misses=%llu hits=%llu bytes=%llu tinyMisses=%llu tinyHits=%llu. ")
			TEXT("Cold bounded RAM per preparation; alternating execution order; exact independent/profile/eviction comparisons."),
			CachedTimes[0],
			CachedTimes[1],
			CachedTimes[2],
			UncachedTimes[0],
			UncachedTimes[1],
			UncachedTimes[2],
			static_cast<unsigned long long>(Statistics.OriginalHeightEvaluations),
			static_cast<unsigned long long>(Statistics.OriginalHeightHits),
			static_cast<unsigned long long>(Statistics.HeightCacheBytes),
			static_cast<unsigned long long>(TinyStatistics.OriginalHeightEvaluations),
			static_cast<unsigned long long>(TinyStatistics.OriginalHeightHits)
		)
	);
	TestFalse(TEXT("２の冪でない保持数を拒否"), RiverTerrainField::CreateWithBasins(Ocean, Drainage, {}, &Statistics, 3).has_value());
	TestFalse(TEXT("保持数の上限超過を拒否"), RiverTerrainField::CreateWithBasins(Ocean, Drainage, {}, &Statistics, 4194304).has_value());
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("中断を計測成功としない"),
		RiverTerrainField::CreateWithBasins(Ocean, Drainage, Cancel.get_token(), &Statistics).has_value()
	);
	TestFalse(TEXT("中断で以前の完了状態を残さない"), Statistics.Completed);
	TestEqual(TEXT("中断で以前の時間を残さない"), Statistics.TotalMilliseconds, 0.0);
	Statistics.Completed = true;
	TestFalse(TEXT("基盤なしを計測成功としない"), RiverTerrainField::CreateWithBasins({}, Drainage, {}, &Statistics).has_value());
	TestFalse(TEXT("不正入力で以前の完了状態を残さない"), Statistics.Completed);
	// 地形の同値性と計測の有効性を確認した結果の返戻
	return !HasAnyErrors();
}

#endif
