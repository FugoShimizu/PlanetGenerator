#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillDiscovery.h"
#include "Core/PlanetSpillString.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillClimbContracts,
	"PlanetGenerator.SpillString.Climb.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillClimbSeedSeven,
	"PlanetGenerator.SpillString.Climb.Seed7",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillClimbFrontier,
	"PlanetGenerator.SpillString.Climb.Frontier",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 曲がった解析谷で，谷底を固定した登坂候補と独立な実鞍点・両終端を検査する
 * @param Parameters 未使用の実行引数
 * @return ３半径の解析解，範囲・評価上限・無効入力・中断が成立した場合に真
 */
bool FPlanetSpillClimbContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Direction = [Radius](const double X, const double Y) -> PlanetCore::UnitDirection {
			const double Longitude = X / Radius, Latitude = Y / Radius;
			// 解析的な地表座標の単位方向の返戻
			return *PlanetCore::UnitDirection::FromVector(
				{ std::cos(Latitude) * std::cos(Longitude), std::cos(Latitude) * std::sin(Longitude), std::sin(Latitude) }
			);
		};
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 P = D.Vector();
			const double X = Radius * std::atan2(P.Y, P.X), Y = Radius * std::asin(P.Z), Horizontal = P.X * P.X + P.Y * P.Y;
			const double Q = X * X / 400 - 1, Across = Y - 0.01 * (X * X - 400), DY = Across / 200, DX = Q * X / 100 - DY * 0.02 * X;
			const PlanetCore::Vec3 G {
				-Radius * DX * P.Y / Horizontal,
				Radius * DX * P.X / Horizontal,
				Radius * DY / std::sqrt(1 - P.Z * P.Z)
			};
			// 峠が（０，－４），極小が（±２０，０）の同じ元標高と解析勾配の返戻
			return { Q * Q + Across * Across / 400, G - P * G.Dot(P) };
		};
		PlanetCore::TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Radius;
		Critical.MaximumDisplacementMeters = std::min(2000.0, Radius * 0.125);
		const std::optional<PlanetCore::TerrainCriticalPoint> Minimum =
		PlanetCore::RefineTerrainCriticalPoint(Direction(-20, 0), Source, Critical);
		// 解析解の入力極小を確認出来ない場合の返戻
		if(!TestTrue(TEXT("登坂の解析極小"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
		PlanetCore::SpillClimbSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.MaximumLengthMeters = 100;
		for(const std::array<double, 2> Tip : { std::array<double, 2>{ -16, 4 }, { -4, -2 }, { 6, -4 } }) {
			Calls = 0;
			const std::optional<PlanetCore::SpillClimb> Climb =
			PlanetCore::ClimbSpillString(*Minimum, Direction(Tip[0], Tip[1]), Source, Settings);
			// 未準備の候補を実鞍点へ読み替えない返戻
			if(!TestTrue(TEXT("曲がった解析谷で登坂候補を準備"), Climb.has_value())) return false;
			TestEqual(TEXT("元場の実照会数"), Calls, Climb->Evaluations);
			const PlanetCore::Vec3 Fixed = Climb->Path.front().Vector(), Original = Minimum->Direction.Vector();
			TestTrue(TEXT("谷底の３成分を厳密に固定"), Fixed.X == Original.X && Fixed.Y == Original.Y && Fixed.Z == Original.Z);
			TestTrue(TEXT("長さの設定上限を維持"), Climb->LengthMeters <= Settings.MaximumLengthMeters);
			TestTrue(
				TEXT("物理的な停止勾配を両条件で確認"),
				Climb->End == PlanetCore::SpillClimbEnd::Converged && Climb->TipGradientNorm <= Settings.GradientTolerance &&
				Climb->MaximumNormalSlope <= Settings.GradientTolerance
			);
			double Previous = -std::numeric_limits<double>::infinity();
			for(const PlanetCore::UnitDirection D : Climb->Path) {
				const double Height = Source(D).ElevationMeters;
				TestTrue(TEXT("元標高の標本を単調に保つ"), Height >= Previous);
				Previous = Height;
			}
			PlanetCore::SpillConnectionSettings ConnectionSettings;
			ConnectionSettings.RadiusMeters = Radius;
			ConnectionSettings.SeaLevelMeters = -100;
			const std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::PrepareSpillConnection(Climb->Path.back(), Source, ConnectionSettings);
			// 候補の停留点分類又は実下降終端を確認出来ない場合の返戻
			if(!TestTrue(TEXT("候補を元場の実鞍点と両枝で検査"), Connection.has_value())) return false;
			const PlanetCore::Vec3 P = Connection->Pass.Direction.Vector();
			const double X = Radius * std::atan2(P.Y, P.X), Y = Radius * std::asin(P.Z);
			TestTrue(TEXT("独立な解析鞍点の位置"), std::hypot(X, Y + 4) <= 1E-4);
			TestTrue(TEXT("独立な解析通過標高"), std::abs(Connection->Pass.ElevationMeters - 1) <= 1E-8);
			unsigned InputBranches = 0;
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection->Branches) {
				TestTrue(TEXT("両終端は同じ元場の実極小"), Branch.Minimum.has_value());
				if(!Branch.Minimum) continue;
				const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector(), B = Minimum->Direction.Vector();
				if(Radius * std::atan2(A.Cross(B).Length(), A.Dot(B)) <= 0.001) ++InputBranches;
			}
			TestEqual(TEXT("固定した谷底へ実際に隣接"), InputBranches, 1U);
			if(Tip[0] > 0) TestTrue(TEXT("別盆地まで延びた初期曲線を短縮"), Climb->Truncations > 0);
			AddInfo(
				FString::Printf(
					TEXT("Climbing string R=%.0f initial=(%.0f,%.0f) end=%u iterations=%u evaluations=%llu truncations=%u ")
					TEXT("length=%.12g normal=%.12g tipGradient=%.12g saddle=(%.12g,%.12g)"),
					Radius,
					Tip[0],
					Tip[1],
					static_cast<unsigned>(Climb->End),
					Climb->Iterations,
					Climb->Evaluations,
					Climb->Truncations,
					Climb->LengthMeters,
					Climb->MaximumNormalSlope,
					Climb->TipGradientNorm,
					X,
					Y
				)
			);
		}
		// 有限評価，中断，距離上限及び不正設定の正規経路と拒否経路
		Settings.MaximumEvaluations = 100;
		Calls = 0;
		TestFalse(TEXT("評価途中の上限を拒否"), PlanetCore::ClimbSpillString(*Minimum, Direction(-16, 4), Source, Settings).has_value());
		TestEqual(TEXT("上限以上に元場を呼出さない"), Calls, UINT64_C(100));
		Settings.MaximumEvaluations = 100000;
		std::stop_source Stop;
		Stop.request_stop();
		Calls = 0;
		TestFalse(
			TEXT("開始前の中断を拒否"),
			PlanetCore::ClimbSpillString(*Minimum, Direction(-16, 4), Source, Settings, Stop.get_token()).has_value()
		);
		TestEqual(TEXT("中断時に元場を呼出さない"), Calls, UINT64_C(0));
		for(
			const double InvalidStep :
			{ 0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() }
		) {
			Settings.TimeStepMeters = InvalidStep;
			TestFalse(TEXT("不正な時間刻みを拒否"), PlanetCore::ClimbSpillString(*Minimum, Direction(-16, 4), Source, Settings).has_value());
			TestEqual(TEXT("無効入力時に元場を呼出さない"), Calls, UINT64_C(0));
		}
		Settings.TimeStepMeters = 5000;
		// 最初の配列評価の途中，最後の要素，反復の途中の中断
		for(const std::uint64_t CancelAt : { UINT64_C(5), UINT64_C(17), UINT64_C(43) }) {
			Calls = 0;
			std::stop_source During;
			const auto CancelSource = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
				const PlanetCore::TerrainSample Value = Source(D);
				if(Calls == CancelAt) During.request_stop();
				// 中断を要求した同じ元場の返戻
				return Value;
			};
			TestFalse(
				TEXT("元場の照会からの中断を返戻前にも検査"),
				PlanetCore::ClimbSpillString(*Minimum, Direction(-16, 4), CancelSource, Settings, During.get_token()).has_value()
			);
			TestEqual(TEXT("中断を要求した照会を最後に停止"), Calls, CancelAt);
		}
		Calls = 0;
		PlanetCore::TerrainCriticalPoint InvalidMinimum = *Minimum;
		InvalidMinimum.GradientNorm = -1;
		TestFalse(TEXT("負の入力勾配を拒否"), PlanetCore::ClimbSpillString(InvalidMinimum, Direction(-16, 4), Source, Settings).has_value());
		TestEqual(TEXT("入力極小の不正時に元場を呼出さない"), Calls, UINT64_C(0));
		const auto InvalidSource = [](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			static_cast<void>(D);
			// 意図した非有限な元場の返戻
			return { std::numeric_limits<double>::quiet_NaN(), {} };
		};
		TestFalse(TEXT("非有限な元場を拒否"), PlanetCore::ClimbSpillString(*Minimum, Direction(-16, 4), InvalidSource, Settings).has_value());
		Settings.MaximumLengthMeters = 8;
		const std::optional<PlanetCore::SpillClimb> Limited =
		PlanetCore::ClimbSpillString(*Minimum, Direction(-16, 4), Source, Settings);
		TestTrue(
			TEXT("長さ上限を収束から区別"),
			Limited && Limited->End == PlanetCore::SpillClimbEnd::LengthLimit && Limited->LengthMeters <= Settings.MaximumLengthMeters
		);
	}
	// 元場の独立な解析解と有限予算の検査結果の返戻
	return !HasAnyErrors();
}

/**
 * Seed 7 の未到達地点で，同じ実極小から登坂し，入力への実下降終端を独立に確認する
 * @param Parameters 未使用の実行引数
 * @return 同じ元場の実鞍点と入力に隣接する両下降枝を準備出来た場合に真
 */
bool FPlanetSpillClimbSeedSeven::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 7;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 再現する地質場の欠落への返戻
	if(!TestTrue(TEXT("登坂の元地質"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山基盤の欠落への返戻
	if(!TestTrue(TEXT("登坂の元造山場"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 同じ大洋場を再現出来ない場合の返戻
	if(!TestTrue(TEXT("登坂の元大洋場"), Ocean.has_value())) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		++Calls;
		// 同じ不変な標高と解析勾配の返戻
		return Ocean->Evaluate(D);
	};
	const std::array<PlanetCore::UnitDirection, 2> Starts {
		*PlanetCore::UnitDirection::FromVector({ -0.26325341885685799, 0.53324576257311995, 0.80395683600426437 }),
		*PlanetCore::UnitDirection::FromVector({ -0.26516096830143476, 0.53667877432912447, 0.80104029491283046 })
	};
	for(const PlanetCore::UnitDirection Start : Starts) {
		const std::optional<PlanetCore::TerrainCriticalPoint> Minimum = PlanetCore::RefineTerrainCriticalPoint(Start, Source, {});
		// 失敗に記録した入力を都合の良い地点へ置換えない返戻
		if(!TestTrue(TEXT("未到達記録の実極小"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
		std::optional<PlanetCore::UnitDirection> Target;
		double Best = -2;
		for(const PlanetCore::UnitDirection D : Ocean->ConnectedOceanAnchors()) {
			if(!(Ocean->EvaluateElevation(D) < -1)) continue;
			const double Near = Start.Vector().Dot(D.Vector());
			if(Near > Best) {
				Best = Near;
				Target = D;
			}
		}
		// 実際の水没大洋の方向を取得出来ない場合の返戻
		if(!TestTrue(TEXT("登坂の大洋目標"), Target.has_value())) return false;
		const PlanetCore::Vec3 P = Minimum->Direction.Vector(), Q = Target->Vector();
		const PlanetCore::Vec3 Tangent = PlanetCore::UnitDirection::FromVector(Q - P * Q.Dot(P))->Vector();
		const PlanetCore::UnitDirection Tip = *PlanetCore::UnitDirection::FromVector(P * std::cos(16E-6) + Tangent * std::sin(16E-6));
		Calls = 0;
		const double Began = FPlatformTime::Seconds();
		const std::optional<PlanetCore::SpillClimb> Climb = PlanetCore::ClimbSpillString(*Minimum, Tip, Source, {});
		// 候補準備の失敗を記録した結果の返戻
		if(!TestTrue(TEXT("未到達の同じ谷底から登坂"), Climb.has_value())) return false;
		TestEqual(TEXT("登坂の実照会数"), Calls, Climb->Evaluations);
		const std::optional<PlanetCore::SpillConnection> Connection =
		PlanetCore::PrepareSpillConnection(Climb->Path.back(), Source, {});
		unsigned Incident = 0;
		if(Connection) for(const PlanetCore::SpillConnectionBranch &Branch : Connection->Branches) {
			if(!Branch.Minimum) continue;
			const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector(), B = Minimum->Direction.Vector();
			if(
				1E6 * std::atan2(A.Cross(B).Length(), A.Dot(B)) <= 0.001 &&
				std::abs(Branch.Minimum->ElevationMeters - Minimum->ElevationMeters) <= 1E-6
			) ++Incident;
		}
		AddInfo(
			FString::Printf(
				TEXT("Actual climbing string start=(%.17g,%.17g,%.17g) end=%u iterations=%u evaluations=%llu truncations=%u ")
				TEXT("length=%.17g normal=%.17g tipGradient=%.17g connected=%d incident=%u totalCalls=%llu ms=%.9g"),
				P.X,
				P.Y,
				P.Z,
				static_cast<unsigned>(Climb->End),
				Climb->Iterations,
				Climb->Evaluations,
				Climb->Truncations,
				Climb->LengthMeters,
				Climb->MaximumNormalSlope,
				Climb->TipGradientNorm,
				static_cast<int>(Connection.has_value()),
				Incident,
				Calls,
				(FPlatformTime::Seconds() - Began) * 1000
			)
		);
		TestTrue(TEXT("登坂終点を元場の実鞍点で確認"), Connection.has_value());
		TestEqual(TEXT("元の入力極小に隣接する実下降枝"), Incident, 1U);
		// 通常の窓探索へ統合した時も，実接続及び全窓の費用・拒否条件を確認
		const std::array<PlanetCore::TerrainCriticalPoint, 1> Input { *Minimum };
		PlanetCore::SpillDiscoverySettings Discovery;
		Discovery.CandidateSpacingMeters = 16;
		Discovery.MaximumClimbingAttempts = 8;
		Discovery.OutletTarget = Target;
		PlanetCore::SpillDiscoveryDiagnostics Report;
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Batch =
		PlanetCore::FindSpillComponentExits(Input, Source, Discovery, {}, &Report);
		TestTrue(TEXT("統合後も入力の外へ繋がる実接続"), Batch && (!Batch->NewMinima.empty() || Batch->HasSeaHeightTerminal));
		TestEqual(TEXT("登坂を含む全窓の元場照会数"), Calls, Report.Evaluations);
		AddInfo(
			FString::Printf(
				TEXT("Climbing discovery accepted=%d evaluations=%llu windows=%u attempts=%u candidates=%u failed=%u"),
				static_cast<int>(Batch.has_value()),
				Calls,
				Report.Windows,
				Report.ClimbingAttempts,
				Report.ClimbingCandidates,
				Report.ClimbingFailures
			)
		);
		Calls = 0;
		Discovery.MaximumClimbingAttempts = 9;
		TestFalse(TEXT("有限な登坂方向の範囲外を拒否"), PlanetCore::FindSpillComponentExits(Input, Source, Discovery, {}, &Report).has_value());
		TestTrue(TEXT("不正な登坂回数の理由"), Report.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
		TestEqual(TEXT("不正回数で元場を呼出さない"), Calls, UINT64_C(0));
		Discovery.MaximumClimbingAttempts = 8;
		Discovery.MaximumEvaluations = 100;
		TestFalse(TEXT("登坂指定時も共通の評価上限を拒否"), PlanetCore::FindSpillComponentExits(Input, Source, Discovery, {}, &Report).has_value());
		TestTrue(TEXT("共通予算の不足を区別"), Report.Failure == PlanetCore::SpillDiscoveryFailure::BudgetLimit);
		TestEqual(TEXT("登坂指定で評価予算を増やさない"), Calls, UINT64_C(100));
	}
	// 誤った盆地を受理しない実接続の検査結果の返戻
	return !HasAnyErrors();
}

/**
 * 目標へ最も近い極小が成分内部に在る場合，別の確認済の中心から未知の盆地へ接続する
 * @param Parameters 未使用の実行引数
 * @return ３半径で内部中心の未発見と，前線中心の実接続及び不正指定の拒否が成立した場合に真
 */
bool FPlanetSpillClimbFrontier::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Direction = [Radius](const double X) -> PlanetCore::UnitDirection {
			// 同じ解析座標上の単位方向の返戻
			return *PlanetCore::UnitDirection::FromVector({ X / Radius, 0, 1 });
		};
		const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 P = D.Vector();
			const double X = Radius * P.X / P.Z, Y = Radius * P.Y / P.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::Vec3 G { Radius * DX / P.Z, Radius * DY / P.Z, -Radius * (DX * P.X + DY * P.Y) / (P.Z * P.Z) };
			// 同じ元場の周期的な極小・鞍点と，直交方向の閉じた谷の返戻
			return { 4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400, G };
		};
		const double Pi = std::acos(-1.0), Phase = std::asin(0.2);
		PlanetCore::TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Radius;
		Critical.MaximumDisplacementMeters = std::min(2000.0, Radius * 0.125);
		std::vector<PlanetCore::TerrainCriticalPoint> Minima;
		for(unsigned I = 0; I < 3; ++I) {
			const std::optional<PlanetCore::TerrainCriticalPoint> Minimum =
			PlanetCore::RefineTerrainCriticalPoint(Direction(40 * (Pi - Phase + 2 * Pi * I)), Source, Critical);
			// 同じ成分の解析極小を確認出来ない場合の返戻
			if(!TestTrue(TEXT("前線の解析極小"), Minimum && Minimum->Kind == PlanetCore::TerrainCriticalKind::Minimum)) return false;
			Minima.push_back(*Minimum);
		}
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Radius;
		ConnectionSettings.SeaLevelMeters = -100;
		std::vector<PlanetCore::SpillConnection> Completed;
		for(unsigned I = 1; I < 3; ++I) {
			std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::PrepareSpillConnection(Direction(40 * (Phase + 2 * Pi * I)), Source, ConnectionSettings);
			// 入力の内部接続を元場で確認出来ない場合の返戻
			if(!TestTrue(TEXT("入力成分の確認済の内部接続"), Connection.has_value())) return false;
			Completed.push_back(std::move(*Connection));
		}
		const PlanetCore::Vec3 Middle = Minima[1].Direction.Vector();
		const PlanetCore::UnitDirection Target =
		*PlanetCore::UnitDirection::FromVector(Middle * std::cos(0.5) + PlanetCore::Vec3{ 0, std::sin(0.5), 0 });
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -100;
		Settings.InitialHalfWidthMeters = std::min(256.0, Radius * 0.0625);
		Settings.MaximumHalfWidthMeters = std::min(65536.0, Radius * 0.25);
		Settings.OutletTarget = Target;
		Settings.MaximumClimbingAttempts = 8;
		Settings.ClimbingOnly = true;
		PlanetCore::SpillDiscoveryDiagnostics Report;
		Settings.MaximumKnownMinima = 2;
		Calls = 0;
		TestFalse(
			TEXT("既知の極小数の予算を入力段階で拒否"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed).has_value()
		);
		TestTrue(TEXT("小さ過ぎる保持予算の入力理由"), Report.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
		TestEqual(TEXT("不正な保持量では元場を呼出さない"), Calls, UINT64_C(0));
		Settings.MaximumKnownMinima = 4;
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Internal =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed);
		TestFalse(TEXT("内部中心の既知の接続を新出口にしない"), Internal.has_value());
		TestTrue(TEXT("有限方向の未発見を区別"), Report.Failure == PlanetCore::SpillDiscoveryFailure::NoIncidentExit);
		TestEqual(TEXT("登坂だけの照会では窓を作らない"), Report.Windows, 0U);
		TestEqual(TEXT("全方向の有限な照会"), Report.ClimbingAttempts, 8U);
		TestEqual(TEXT("内部中心の全照会数"), Calls, Report.Evaluations);
		const std::uint64_t InternalCalls = Calls;
		// 両終端が既知でも，未登録の別峠は新しい接続として残す
		Settings.IncludeInternalConnections = true;
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Cycle = PlanetCore::FindSpillComponentExits(
			Minima,
			Source,
			Settings,
			{},
			&Report,
			std::span<const PlanetCore::SpillConnection>(Completed).first(1)
		);
		TestTrue(TEXT("既知の極小間の未登録通路を保持"), Cycle && !Cycle->Connections.empty());
		TestEqual(TEXT("別通路の全照会数"), Calls, Report.Evaluations);
		if(Cycle) {
			TestTrue(TEXT("新極小無しの接続を捨てない"), Cycle->NewMinima.empty());
			TestEqual(TEXT("独立に除外した一つの実鞍点"), Cycle->Connections.size(), static_cast<std::size_t>(1));
			const PlanetCore::Vec3 A = Completed[1].Pass.Direction.Vector();
			for(const PlanetCore::SpillConnection &Connection : Cycle->Connections) {
				const PlanetCore::Vec3 B = Connection.Pass.Direction.Vector();
				TestTrue(TEXT("除外した解析通路の位置"), Radius * std::atan2(A.Cross(B).Length(), A.Dot(B)) <= 0.001);
				TestTrue(TEXT("除外した解析通路の標高"), std::abs(Connection.Pass.ElevationMeters - Completed[1].Pass.ElevationMeters) <= 1E-6);
			}
		}
		Calls = 0;
		TestFalse(
			TEXT("単一の集合外出口の照会は内部通路だけで成功しない"),
			PlanetCore::FindSpillComponentExit(
				Minima,
				Source,
				Settings,
				{},
				&Report,
				std::span<const PlanetCore::SpillConnection>(Completed).first(1)
			).has_value()
		);
		TestTrue(TEXT("単一照会の外部出口未発見"), Report.Failure == PlanetCore::SpillDiscoveryFailure::NoIncidentExit);
		TestEqual(TEXT("単一照会の全照会数"), Calls, Report.Evaluations);
		Settings.IncludeInternalConnections = false;
		Settings.PreferredCenter = Minima.front().Direction;
		Calls = 0;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Frontier =
		PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed);
		TestTrue(TEXT("別の入力中心から成分外の実極小へ接続"), Frontier && !Frontier->NewMinima.empty());
		TestEqual(TEXT("前線の全照会数"), Calls, Report.Evaluations);
		if(Frontier) for(const PlanetCore::TerrainCriticalPoint &Minimum : Frontier->NewMinima) {
			const PlanetCore::Vec3 P = Minimum.Direction.Vector();
			const double X = Radius * P.X / P.Z, Expected = 40 * (-Pi - Phase);
			TestTrue(TEXT("独立な未知の解析極小へ到達"), std::abs(X - Expected) <= 0.001);
		}
		AddInfo(
			FString::Printf(
				TEXT("Climbing frontier R=%.0f internalCalls=%llu frontierCalls=%llu accepted=%d attempts=%u newMinima=%llu"),
				Radius,
				InternalCalls,
				Calls,
				static_cast<int>(Frontier.has_value()),
				Report.ClimbingAttempts,
				static_cast<unsigned long long>(Frontier ? Frontier->NewMinima.size() : 0)
			)
		);
		// 小窓の有限な未発見を，登坂又は広窓の探索失敗へ混ぜない
		Settings.ClimbingOnly = false;
		Settings.InitialWindowOnly = true;
		Settings.InitialHalfWidthMeters = std::min(32.0, Radius * 0.03125);
		Calls = 0;
		TestFalse(
			TEXT("峠を含まない小窓の未発見"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed).has_value()
		);
		TestTrue(TEXT("小窓のみの未発見を区別"), Report.Failure == PlanetCore::SpillDiscoveryFailure::NoIncidentExit);
		TestEqual(TEXT("小窓だけの照会は一窓"), Report.Windows, 1U);
		TestEqual(TEXT("小窓から登坂へ進めない"), Report.ClimbingAttempts, 0U);
		TestEqual(TEXT("小窓から経路緩和へ進めない"), Report.RelaxedWindows, 0U);
		TestEqual(TEXT("小窓から橋渡しへ進めない"), Report.BridgeCorridors, 0U);
		TestEqual(TEXT("小窓の全照会数"), Calls, Report.Evaluations);
		Settings.ClimbingOnly = true;
		Calls = 0;
		TestFalse(
			TEXT("小窓のみと登坂のみの重複指定を拒否"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed).has_value()
		);
		TestTrue(TEXT("重複した探索指定の理由"), Report.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
		TestEqual(TEXT("重複指定で元場を呼出さない"), Calls, UINT64_C(0));
		Settings.InitialWindowOnly = false;
		// 入力に無い中心の指定と，登坂だけの無試行指定は照会前に拒否
		Settings.PreferredCenter = Direction(-200);
		Calls = 0;
		TestFalse(
			TEXT("成分外の未確認中心を拒否"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed).has_value()
		);
		TestTrue(TEXT("不正な中心指定の理由"), Report.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
		TestEqual(TEXT("不正な中心で元場を呼出さない"), Calls, UINT64_C(0));
		Settings.PreferredCenter = Minima.front().Direction;
		Settings.MaximumClimbingAttempts = 0;
		TestFalse(
			TEXT("登坂のみの無試行指定を拒否"),
			PlanetCore::FindSpillComponentExits(Minima, Source, Settings, {}, &Report, Completed).has_value()
		);
		TestEqual(TEXT("無試行の不正指定で元場を呼出さない"), Calls, UINT64_C(0));
	}
	// 成分を捨てずに前線を変更した実接続の独立検査結果の返戻
	return !HasAnyErrors();
}

#endif
