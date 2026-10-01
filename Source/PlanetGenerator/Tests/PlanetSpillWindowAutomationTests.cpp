#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillWindowContracts,
	"PlanetGenerator.SpillPassage.WindowExit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 解析的な窪地と実地形で，外周の到達と実鞍点を区別して確認する
 * @param Parameters 未使用の実行引数
 * @return 予算を守った候補探索と元場での実接続を確認した場合に真
 */
bool FPlanetSpillWindowContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		// 二重窪地と独立な解析勾配
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 V = Direction.Vector();
			const double Horizontal = V.X * V.X + V.Y * V.Y, X = Radius * std::atan2(V.Y, V.X), Y = Radius * std::asin(V.Z);
			const double Q = X * X / 400 - 1, DX = Q * X / 100, DY = Y / 200;
			const PlanetCore::Vec3 Gradient {
				-Radius * DX * V.Y / Horizontal,
				Radius * DX * V.X / Horizontal,
				Radius * DY / std::sqrt(1 - V.Z * V.Z)
			};
			// 元場の高さと球面接線微分の返戻
			return { Q * Q + Y * Y / 400, Gradient - V * Gradient.Dot(V) };
		};
		const PlanetCore::UnitDirection Center =
		*PlanetCore::UnitDirection::FromVector({ std::cos(-20 / Radius), std::sin(-20 / Radius), 0 });
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.HalfWidthMeters = 10;
		Settings.SpacingMeters = 2;
		const std::optional<PlanetCore::SpillWindowExit> Small = PlanetCore::FindSpillWindowExit(Center, Source, Settings);
		// 小さな窓の出口を計測出来ない場合の返戻
		if(!TestTrue(TEXT("小さな探索窓の外周へ到達"), Small.has_value())) return false;
		TestTrue(TEXT("鞍点に届かない外周を拡張対象に分類"), Small->PeakOnBoundary && Small->CrestMeters < 1);
		Settings.HalfWidthMeters = 45;
		const std::optional<PlanetCore::SpillWindowExit> Exit = PlanetCore::FindSpillWindowExit(Center, Source, Settings);
		// 真の峠を含む窓を準備出来ない場合の返戻
		if(!TestTrue(TEXT("鞍点を含む探索窓を準備"), Exit.has_value())) return false;
		// 同じ標本と辺高を全て先に測る参照版との独立比較
		Settings.MeasureAllWindowEdges = true;
		Calls = 0;
		const std::optional<PlanetCore::SpillWindowExit> Full = PlanetCore::FindSpillWindowExit(Center, Source, Settings);
		Settings.MeasureAllWindowEdges = false;
		// 全辺参照を失った比較への返戻
		if(!TestTrue(TEXT("全辺実測の窓を準備"), Full.has_value())) return false;
		TestEqual(TEXT("全辺実測の評価回数を照合"), Full->Evaluations, Calls);
		TestEqual(TEXT("窓の通過高度が完全一致"), Exit->CrestMeters, Full->CrestMeters, 0);
		TestEqual(TEXT("同じ候補経路の長さ"), Exit->Path.size(), Full->Path.size());
		for(std::size_t I = 0; I < std::min(Exit->Path.size(), Full->Path.size()); ++I) {
			TestEqual(TEXT("窓内の経路方向が完全一致"), (Exit->Path[I].Vector() - Full->Path[I].Vector()).Length(), 0.0, 0);
		}
		TestEqual(TEXT("峠候補の数を保持"), Exit->PassCandidates.size(), Full->PassCandidates.size());
		for(std::size_t I = 0; I < std::min(Exit->PassCandidates.size(), Full->PassCandidates.size()); ++I) {
			TestEqual(TEXT("峠候補の位置を保持"), (Exit->PassCandidates[I].Vector() - Full->PassCandidates[I].Vector()).Length(), 0.0, 0);
		}
		TestTrue(TEXT("窓の実測する辺を削減"), Exit->EvaluatedEdges < Full->EvaluatedEdges);
		TestTrue(TEXT("窓の元場評価を削減"), Exit->Evaluations < Full->Evaluations);
		// 候補経路の先行実測を，同じ全辺参照と元場で独立に比較する
		Settings.UsePathEdgeSelection = true;
		Calls = 0;
		const std::optional<PlanetCore::SpillWindowExit> Selected = PlanetCore::FindSpillWindowExit(Center, Source, Settings);
		Settings.UsePathEdgeSelection = false;
		// 未実測の通路を完成した窓へ読み替えない返戻
		if(!TestTrue(TEXT("窓の候補経路の実測を完了"), Selected.has_value())) return false;
		TestEqual(TEXT("候補経路版の元場照会を集計"), Selected->Evaluations, Calls);
		TestEqual(TEXT("全辺参照と同じ最低通過高度"), Selected->CrestMeters, Full->CrestMeters, 0);
		TestTrue(TEXT("全辺参照より実測を削減"), Selected->Evaluations < Full->Evaluations);
		TestTrue(TEXT("選択した窓は実外周へ到達"), Selected->Path.size() > 2);
		for(std::size_t I = 1; I < Selected->Path.size(); ++I) {
			const PlanetCore::Vec3 A = Selected->Path[I - 1].Vector(), B = Selected->Path[I].Vector();
			for(unsigned Probe = 0; Probe < 11; ++Probe) {
				const double Fraction = (Probe + 0.5) / 11;
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
				TestTrue(TEXT("独立な辺内標本は実測通過高以下"), Source(D).ElevationMeters <= Selected->CrestMeters + 1E-6);
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("Selected window R=%.0f source=%llu frontier=%llu all=%llu edges=%u/%u"),
				Radius,
				Selected->Evaluations,
				Exit->Evaluations,
				Full->Evaluations,
				Selected->EvaluatedEdges,
				Full->EvaluatedEdges
			)
		);
		AddInfo(
			FString::Printf(
				TEXT("Measured window R=%.0f evaluations=%llu/%llu edges=%u/%u"),
				Radius,
				Exit->Evaluations,
				Full->Evaluations,
				Exit->EvaluatedEdges,
				Full->EvaluatedEdges
			)
		);
		TestFalse(TEXT("既知の峠は窓の内部"), Exit->PeakOnBoundary);
		TestTrue(TEXT("解析解の通過水位に一致"), std::abs(Exit->CrestMeters - 1) < 1E-6);
		TestTrue(TEXT("窓外周に峠より低い受入側が存在"), Exit->MinimumBoundaryMeters < 1);
		TestTrue(TEXT("実方向だけの経路が存在"), Exit->Path.size() > 2 && Exit->Nodes <= Settings.MaximumNodes);
		PlanetCore::SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Radius;
		ConnectionSettings.SeaLevelMeters = -100;
		const std::optional<PlanetCore::SpillConnection> Connection =
		PlanetCore::PrepareSpillConnection(Exit->Peak, Source, ConnectionSettings);
		// 単なる網の最大点を鞍点と同一視しない返戻
		if(!TestTrue(TEXT("元場で実鞍点と両側の極小を確認"), Connection.has_value())) return false;
		TestTrue(TEXT("連続場でも峠の水位に一致"), std::abs(Connection->Pass.ElevationMeters - 1) < 1E-6);
		AddInfo(
			FString::Printf(
				TEXT("Window exit R=%.0f nodes=%u path=%llu crest=%.12g boundary=%.12g evaluations=%llu"),
				Radius,
				Exit->Nodes,
				static_cast<unsigned long long>(Exit->Path.size()),
				Exit->CrestMeters,
				Exit->MinimumBoundaryMeters,
				static_cast<unsigned long long>(Exit->Evaluations)
			)
		);
		// 同じ地形でも指定した前方だけから出て，後方の安い通路へ戻らない検査
		for(const double Sign : { -1.0, 1.0 }) {
			Settings.WindowExitTarget =
			PlanetCore::UnitDirection::FromVector({ std::cos(-20 / Radius + Sign * 0.1), std::sin(-20 / Radius + Sign * 0.1), 0 });
			const std::optional<PlanetCore::SpillWindowExit> Directed = PlanetCore::FindSpillWindowExit(Center, Source, Settings);
			// 指定方向への有限な経路が無い場合の返戻
			if(!TestTrue(TEXT("指定した方向の外周へ到達"), Directed.has_value())) return false;
			const PlanetCore::Vec3 Up = Center.Vector(), Target = Settings.WindowExitTarget->Vector();
			const PlanetCore::Vec3 Forward = Target - Up * Target.Dot(Up);
			const PlanetCore::Vec3 End = Directed->Path.back().Vector();
			TestTrue(TEXT("候補終端は指定した前方の半面内"), (End - Up * End.Dot(Up)).Dot(Forward) > 0);
			if(Sign > 0) TestTrue(TEXT("指定した峠側の通過高度を維持"), std::abs(Directed->CrestMeters - 1) < 1E-6);
			else TestTrue(TEXT("反対側の高い外周を低い峠へ置換えない"), Directed->CrestMeters > 1);
		}
		Settings.WindowExitTarget = Center;
		Calls = 0;
		TestFalse(TEXT("前方を定義出来ない同じ中心を拒否"), PlanetCore::FindSpillWindowExit(Center, Source, Settings).has_value());
		TestEqual(TEXT("退化した方向では元場を評価しない"), Calls, UINT64_C(0));
		Settings.WindowExitTarget = PlanetCore::UnitDirection::FromVector(Center.Vector() * -1);
		TestFalse(TEXT("前方を定義出来ない対蹠点を拒否"), PlanetCore::FindSpillWindowExit(Center, Source, Settings).has_value());
		TestEqual(TEXT("対蹠点の拒否でも元場を評価しない"), Calls, UINT64_C(0));
		Settings.WindowExitTarget.reset();
		// 評価予算・節点予算・中断の拒否と呼出回数
		Settings.MaximumEvaluations = 100;
		Calls = 0;
		TestFalse(TEXT("評価予算で不完全な経路を拒否"), PlanetCore::FindSpillWindowExit(Center, Source, Settings).has_value());
		TestEqual(TEXT("予算を超えて元場を評価しない"), Calls, UINT64_C(100));
		Settings.MaximumEvaluations = 4000000;
		Settings.MaximumNodes = 100;
		Calls = 0;
		TestFalse(TEXT("節点予算の超過を拒否"), PlanetCore::FindSpillWindowExit(Center, Source, Settings).has_value());
		TestEqual(TEXT("節点予算の拒否では評価しない"), Calls, UINT64_C(0));
		Settings.MaximumNodes = 65536;
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(TEXT("開始前の中断を拒否"), PlanetCore::FindSpillWindowExit(Center, Source, Settings, Stop.get_token()).has_value());
		TestEqual(TEXT("中断後に評価しない"), Calls, UINT64_C(0));
		Settings.SpacingMeters = 1E-300;
		TestFalse(TEXT("解像出来ない刻みを拒否"), PlanetCore::FindSpillWindowExit(Center, Source, Settings).has_value());
	}
	// 実際の湖から流れ下った先の窪地
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 元地質が無い場合の返戻
	if(!TestTrue(TEXT("実地形の地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 元の造山場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の造山を準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 海域を含む元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実地形の海域を準備"), Ocean.has_value())) return false;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection KnownPass =
	*PlanetCore::UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
	PlanetCore::SpillConnectionSettings ConnectionSettings;
	const std::optional<PlanetCore::SpillConnection> Original =
	PlanetCore::PrepareSpillConnection(KnownPass, Source, ConnectionSettings);
	// 既知の下降先を準備出来ない場合の返戻
	if(!TestTrue(TEXT("既存の峠から下流側へ到達"), Original.has_value())) return false;
	// 両終端の極小が無い場合の返戻
	if(!TestTrue(TEXT("既存の峠の両側に極小が存在"), Original->Branches[0].Minimum.has_value() && Original->Branches[1].Minimum.has_value())) {
		// 実接続の終端を失った場合の返戻
		return false;
	}
	const std::optional<PlanetCore::TerrainCriticalPoint> Minimum =
	Original->Branches[0].Minimum->ElevationMeters < Original->Branches[1].Minimum->ElevationMeters ?
	Original->Branches[0].Minimum :
	Original->Branches[1].Minimum;
	bool HasConnectedExit = false;
	for(const double HalfWidth : { 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0 }) {
		PlanetCore::SpillPassageSettings Settings;
		Settings.HalfWidthMeters = HalfWidth;
		Settings.SpacingMeters = HalfWidth / 64;
		const std::optional<PlanetCore::SpillWindowExit> Exit = PlanetCore::FindSpillWindowExit(Minimum->Direction, Source, Settings);
		// 計算を完了出来なかった窓を成功としない返戻
		if(!TestTrue(TEXT("実窪地の有限な外周経路を計算"), Exit.has_value())) return false;
		const PlanetCore::Vec3 Peak = Exit->Peak.Vector();
		AddInfo(
			FString::Printf(
				TEXT("Actual window half=%.0f crest=%.12g boundary=%u peak=(%.17g,%.17g,%.17g) evaluations=%llu"),
				HalfWidth,
				Exit->CrestMeters,
				static_cast<unsigned>(Exit->PeakOnBoundary),
				Peak.X,
				Peak.Y,
				Peak.Z,
				static_cast<unsigned long long>(Exit->Evaluations)
			)
		);
		if(Exit->PeakOnBoundary) continue;
		PlanetCore::SpillConnectionDiagnostics Diagnostics;
		const std::optional<PlanetCore::SpillConnection> Connection =
		PlanetCore::PrepareSpillConnection(Exit->Peak, Source, ConnectionSettings, {}, &Diagnostics);
		AddInfo(
			FString::Printf(
				TEXT("Actual window connection accepted=%u failure=%u"),
				static_cast<unsigned>(Connection.has_value()),
				static_cast<unsigned>(Diagnostics.Failure)
			)
		);
		if(!Connection) continue;
		unsigned Matching = 0;
		for(const PlanetCore::SpillConnectionBranch &Branch : Connection->Branches) {
			if(!Branch.Minimum) continue;
			const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector(), B = Minimum->Direction.Vector();
			const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
			if(Distance < 0.001) ++Matching;
			AddInfo(FString::Printf(TEXT("Window terminal H=%.12g targetDistance=%.12g"), Branch.Minimum->ElevationMeters, Distance));
		}
		if(Matching == 1) {
			HasConnectedExit = true;
			break;
		}
	}
	TestTrue(TEXT("下流側の実窪地に接する次の鞍点を確認"), HasConnectedExit);
	// 大洋到達とは区別した局所接続検査の返戻
	return !HasAnyErrors();
}

#endif
