#include "Misc/AutomationTest.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stop_token>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillConnectionContracts,
	"PlanetGenerator.SpillConnection.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 両側の行先が既知の窪地と海面出口で，連続な鞍点接続を検査する
 * @param Parameters 未使用の実行引数
 * @return 元場上の下降と終端の分類を確認した場合に真
 */
bool FPlanetSpillConnectionContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const PlanetCore::UnitDirection Origin = *PlanetCore::UnitDirection::FromVector({ 0, 0, 1 });
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto Field = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = X / 40, V = Y / 20, DX = (4 * U * U * U - 4 * U) / 40;
			const double DY = 2 * V / 20;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 高さ１の鞍点と高さ０の２極小を持つ解析場の返戻
			return { U * U * U * U - 2 * U * U + V * V + 1, Gradient };
		};
		PlanetCore::SpillConnectionSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		PlanetCore::SpillConnectionDiagnostics Diagnostic;
		const std::optional<PlanetCore::SpillConnection> Result =
		PlanetCore::PrepareSpillConnection(Origin, Field, Settings, {}, &Diagnostic);
		AddInfo(
			FString::Printf(
				TEXT("Analytic connection R=%.0f accepted=%d failure=%u side=%u end=%u move=%.12g"),
				Radius,
				static_cast<int>(Result.has_value()),
				static_cast<unsigned>(Diagnostic.Failure),
				Diagnostic.Branch,
				static_cast<unsigned>(Diagnostic.TraceEnd),
				Diagnostic.TerminalMoveMeters
			)
		);
		// 両側の極小を確認出来ない場合の返戻
		if(!TestTrue(TEXT("解析場の両側の極小へ接続"), Result.has_value())) return false;
		for(const PlanetCore::SpillConnectionBranch &Branch : Result->Branches) {
			TestTrue(TEXT("極小への到達を分類"), Branch.Kind == PlanetCore::SpillTerminalKind::Minimum && Branch.Minimum.has_value());
			TestTrue(TEXT("終点は解析解の極小"), std::abs(Branch.Path.Samples.back().ElevationMeters) < 1E-10);
			const PlanetCore::Vec3 End = Branch.Path.Samples.back().Direction.Vector();
			TestTrue(TEXT("既知の極小位置に一致"), std::abs(std::abs(Radius * End.X / End.Z) - 40) < 1E-4);
			TestTrue(TEXT("鞍点を省略しない"), Branch.Path.Samples.front().Direction.Vector().Dot(Origin.Vector()) == 1);
			for(std::size_t i = 1; i < Branch.Path.Samples.size(); ++i) {
				const PlanetCore::TerrainDescentSample &A = Branch.Path.Samples[i - 1], &B = Branch.Path.Samples[i];
				TestTrue(TEXT("元場上の下降"), B.ElevationMeters <= A.ElevationMeters);
				TestTrue(TEXT("累積距離が増加"), B.DistanceMeters > A.DistanceMeters);
				TestEqual(TEXT("元場の標高を保持"), B.ElevationMeters, Field(B.Direction).ElevationMeters);
			}
		}
		TestTrue(
			TEXT("異なる側の極小へ接続"),
			Result->Branches[0].Minimum->Direction.Vector().X * Result->Branches[1].Minimum->Direction.Vector().X < 0
		);
		Settings.SeaLevelMeters = 0.5;
		const std::optional<PlanetCore::SpillConnection> Sea = PlanetCore::PrepareSpillConnection(Origin, Field, Settings);
		// 海面高度の終端を確認出来ない場合の返戻
		if(!TestTrue(TEXT("途中で海面高度へ到達"), Sea.has_value())) return false;
		for(const PlanetCore::SpillConnectionBranch &Branch : Sea->Branches) {
			TestTrue(TEXT("海面高度への到達を極小と混同しない"), Branch.Kind == PlanetCore::SpillTerminalKind::SeaLevel && !Branch.Minimum);
			TestTrue(TEXT("海面高度を跨ぐ終端"), Branch.Path.Samples.back().ElevationMeters <= 0.5);
		}
		Settings.SeaLevelMeters = -1;
		Settings.MaximumBranchVertices = 2;
		TestFalse(TEXT("頂点予算で切れた経路を受理しない"), PlanetCore::PrepareSpillConnection(Origin, Field, Settings, {}, &Diagnostic).has_value());
		TestTrue(TEXT("未確定の終端を通知"), Diagnostic.Failure == PlanetCore::SpillConnectionFailure::UnresolvedTerminal);
		Settings.MaximumBranchVertices = 8192;
		Settings.MaximumBranchLengthMeters = 6;
		TestFalse(TEXT("短い距離予算で終端を偽装しない"), PlanetCore::PrepareSpillConnection(Origin, Field, Settings, {}, &Diagnostic).has_value());
		TestTrue(TEXT("距離上限での未達を通知"), Diagnostic.Failure == PlanetCore::SpillConnectionFailure::UnresolvedTerminal);
		Settings.MaximumBranchLengthMeters = 20000;
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(
			TEXT("中断を接続の完成としない"),
			PlanetCore::PrepareSpillConnection(Origin, Field, Settings, Cancel.get_token(), &Diagnostic).has_value()
		);
		TestTrue(TEXT("中断理由を保持"), Diagnostic.Failure == PlanetCore::SpillConnectionFailure::Cancelled);
		Settings.TerminalPositionToleranceMeters = 0;
		TestFalse(TEXT("無効な終端精度を拒否"), PlanetCore::PrepareSpillConnection(Origin, Field, Settings, {}, &Diagnostic).has_value());
		TestTrue(TEXT("入力不正を通知"), Diagnostic.Failure == PlanetCore::SpillConnectionFailure::InvalidInput);
	}
	PlanetCore::GeologySettings Definition;
	Definition.RadiusMeters = 1E6;
	Definition.Seed = 42;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元地形を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の造山を準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 海域を含む元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("変更しない元場を準備"), Ocean.has_value())) return false;
	const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
		// 同じ元場の返戻
		return Ocean->Evaluate(Direction);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
	PlanetCore::SpillConnectionDiagnostics Diagnostic;
	const std::optional<PlanetCore::SpillConnection> Connection =
	PlanetCore::PrepareSpillConnection(Candidate, Source, {}, {}, &Diagnostic);
	AddInfo(
		FString::Printf(
			TEXT("Actual connection accepted=%d failure=%u side=%u end=%u move=%.12g"),
			static_cast<int>(Connection.has_value()),
			static_cast<unsigned>(Diagnostic.Failure),
			Diagnostic.Branch,
			static_cast<unsigned>(Diagnostic.TraceEnd),
			Diagnostic.TerminalMoveMeters
		)
	);
	// 実地形の両終端が未確定の場合の返戻
	if(!TestTrue(TEXT("元地形の両側の窪地へ接続"), Connection.has_value())) return false;
	double Shallow = -1E9, Deep = 1E9;
	for(const PlanetCore::SpillConnectionBranch &Branch : Connection->Branches) {
		// 極小に至らない経路を窪地接続としない返戻
		if(!TestTrue(TEXT("実地形の極小を確認"), Branch.Minimum.has_value())) return false;
		Shallow = std::max(Shallow, Branch.Minimum->ElevationMeters);
		Deep = std::min(Deep, Branch.Minimum->ElevationMeters);
		for(std::size_t i = 1; i < Branch.Path.Samples.size(); ++i) {
			const PlanetCore::TerrainDescentSample &A = Branch.Path.Samples[i - 1], &B = Branch.Path.Samples[i];
			TestTrue(TEXT("実地形上の下降"), B.ElevationMeters <= A.ElevationMeters);
			TestTrue(TEXT("実経路の累積距離が増加"), B.DistanceMeters > A.DistanceMeters);
			TestEqual(TEXT("元地形の標高を保持"), B.ElevationMeters, Source(B.Direction).ElevationMeters);
		}
		AddInfo(
			FString::Printf(
				TEXT("Actual branch points=%llu length=%.12g minimum=%.12g gradient=%.12g"),
				static_cast<unsigned long long>(Branch.Path.Samples.size()),
				Branch.Path.Samples.back().DistanceMeters,
				Branch.Minimum->ElevationMeters,
				Branch.Minimum->GradientNorm
			)
		);
	}
	TestTrue(TEXT("浅い窪地の回帰"), std::abs(Shallow - 845.964425405) < 1E-6);
	TestTrue(TEXT("深い窪地の回帰"), std::abs(Deep - 527.56567) < 0.001);
	AddInfo(FString::Printf(TEXT("Connection evaluations=%llu"), static_cast<unsigned long long>(Connection->Evaluations)));
	// 同じ浅い湖に接する別の峠は，低い峠と同じ２窪地へ到達する
	const PlanetCore::UnitDirection Alternate =
	*PlanetCore::UnitDirection::FromVector({ 0.54180310267017084, 0.81200372324380465, -0.21702384978424616 });
	const std::optional<PlanetCore::SpillConnection> Parallel =
	PlanetCore::PrepareSpillConnection(Alternate, Source, {}, {}, &Diagnostic);
	AddInfo(
		FString::Printf(
			TEXT("Alternate connection accepted=%d failure=%u side=%u end=%u move=%.12g"),
			static_cast<int>(Parallel.has_value()),
			static_cast<unsigned>(Diagnostic.Failure),
			Diagnostic.Branch,
			static_cast<unsigned>(Diagnostic.TraceEnd),
			Diagnostic.TerminalMoveMeters
		)
	);
	// 同じ湖の別出口を確認出来ない場合の返戻
	if(!TestTrue(TEXT("別の鞍点も同じ元場で解決"), Parallel.has_value())) return false;
	TestTrue(TEXT("先の峠の方が低い"), Connection->Pass.ElevationMeters < Parallel->Pass.ElevationMeters);
	for(const PlanetCore::SpillConnectionBranch &Branch : Parallel->Branches) {
		// 極小を持たない枝を対応比較に使わない返戻
		if(!TestTrue(TEXT("別出口の終端も極小"), Branch.Minimum.has_value())) return false;
		double Nearest = 1E9;
		for(const PlanetCore::SpillConnectionBranch &Known : Connection->Branches) {
			const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector(), B = Known.Minimum->Direction.Vector();
			Nearest = std::min(Nearest, Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B)));
		}
		TestTrue(TEXT("別出口の終端は同じ２窪地に対応"), Nearest < 0.1);
		AddInfo(FString::Printf(TEXT("Parallel connection terminal distance=%.12g"), Nearest));
	}
	for(
		const PlanetCore::Vec3 Direction :
		{
			PlanetCore::Vec3{ -0.159802600625922, -0.15059270024919291, -0.97559467375793352 },
			PlanetCore::Vec3{ 0.7973056191635417, 0.51339418771102174, -0.31738014694492389 }
		}
	) {
		const std::optional<PlanetCore::SpillConnection> Other =
		PlanetCore::PrepareSpillConnection(*PlanetCore::UnitDirection::FromVector(Direction), Source, {}, {}, &Diagnostic);
		AddInfo(
			FString::Printf(
				TEXT("Other connection accepted=%d failure=%u side=%u end=%u move=%.12g"),
				static_cast<int>(Other.has_value()),
				static_cast<unsigned>(Diagnostic.Failure),
				Diagnostic.Branch,
				static_cast<unsigned>(Diagnostic.TraceEnd),
				Diagnostic.TerminalMoveMeters
			)
		);
		TestTrue(TEXT("他地域の鞍点も元地形の終端へ接続"), Other.has_value());
	}
	// 有限の検証条件に於ける接続結果の返戻
	return !HasAnyErrors();
}

#endif
