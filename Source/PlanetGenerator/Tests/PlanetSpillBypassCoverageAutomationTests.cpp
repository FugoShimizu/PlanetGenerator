#include "Core/PlanetFlowOutlet.h"
#include "Core/PlanetFlowPreparation.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillDiscovery.h"
#include "Core/PlanetSpillLake.h"
#include "Core/PlanetSpillPassage.h"
#include "Core/PlanetSpillSampleCache.h"
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
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	/**
	 * 記録した失敗条件と同じ元場だけを準備する
	 * 計算量：O(G+R+W)，G・R・W は地質・造山・海域の各準備費用で，追加空間も各保持量に従う
	 * @param Seed 不変な生成定義
	 * @return 地質・造山・海域接続を共有所有する元場，準備不能時は空
	 */
	std::shared_ptr<const PlanetCore::OceanConnectivityField> MakeFailureTopologyOcean(const std::uint64_t Seed) {
		PlanetCore::GeologySettings Definition;
		Definition.Seed = Seed;
		Definition.RadiusMeters = 1E6;
		std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
		// 元の地質を欠く場合の返戻
		if(!Base) return {};
		const std::shared_ptr<const PlanetCore::GeologyField> Geology =
		std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
		std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
		// 元の造山場を欠く場合の返戻
		if(!Ridges) return {};
		std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
			Geology,
			std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
			{},
			PlanetCore::PredrainageReliefMode::MesoAndLand,
			PlanetCore::OceanSamplingMode::SharedDrainage
		);
		// 孤立した大洋又は元場の欠落への返戻
		if(!Ocean || Ocean->Statistics().ComponentsAfter != 1) return {};
		// 同じ元場を独立な検査へ渡す返戻
		return std::make_shared<const PlanetCore::OceanConnectivityField>(std::move(*Ocean));
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillBypassSeed0Coverage,
	"PlanetGenerator.SpillLake.Seed0BypassCoverage",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 負の面積で拒否した同じ実峠の迂回を，固定密度と有界な広域候補で切分ける
 * @param Parameters 未使用の実行引数
 * @return 同じ元場に旧峠より低い実測通路を確認した場合に真；未確認は失敗として保持
 */
bool FPlanetSpillBypassSeed0Coverage::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(0);
	// 別の元場で失敗を置換えない返戻
	if(!TestTrue(TEXT("Seed０の同じ元場"), Ocean != nullptr)) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		if(Calls > 3999999) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 有界な検査費用を超える場合の返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++Calls;
		// 補正を加えない同じ元地形の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ 0.44380729954063752, -0.029805510885292881, 0.89562643573942902 });
	const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(Candidate, Source, {});
	// 元の拒否した実峠を確認出来ない場合の返戻
	if(!TestTrue(TEXT("元の実峠と両下降枝"), Connection.has_value())) return false;
	TestTrue(TEXT("記録した元水位の一致"), std::abs(Connection->Pass.ElevationMeters - 115.26953414855741) <= 1E-6);
	PlanetCore::TerrainContourSettings ShoreSettings;
	ShoreSettings.MaximumLengthMeters = 1E6;
	PlanetCore::SpillLakeDiagnostics Shore;
	TestFalse(
		TEXT("元の負の湖岸の拒否を保持"),
		PlanetCore::PrepareSpillLakeShore(Connection->Pass, false, Source, ShoreSettings, {}, &Shore).has_value()
	);
	TestTrue(TEXT("島を湖へ反転しない"), Shore.Failure == PlanetCore::SpillLakeFailure::Boundary && Shore.AreaMetersSquared < 0);
	const bool HasMinima =
	TestTrue(TEXT("同じ実峠の両終端は実極小"), Connection->Branches[0].Minimum.has_value() && Connection->Branches[1].Minimum.has_value());
	// 未確認の終端へ置換えない返戻
	if(!HasMinima) return false;
	const PlanetCore::UnitDirection From = Connection->Branches[0].Minimum->Direction;
	const PlanetCore::UnitDirection To = Connection->Branches[1].Minimum->Direction;
	const PlanetCore::Vec3 A = From.Vector(), B = To.Vector();
	AddInfo(
		FString::Printf(
			TEXT("Fixed bypass terminals: distance=%.17g H=(%.17g,%.17g)"),
			1E6 * std::atan2(A.Cross(B).Length(), A.Dot(B)),
			Source(From).ElevationMeters,
			Source(To).ElevationMeters
		)
	);
	bool HasLower = false;
	const std::array<std::array<double, 2>, 7> Windows {
		std::array<double, 2>{ 256, 16 },
		{ 512, 16 },
		{ 1024, 16 },
		{ 2048, 16 },
		{ 4096, 32 },
		{ 8192, 64 },
		{ 16384, 128 }
	};
	for(const std::array<double, 2> Window : Windows) {
		const std::uint64_t Before = Calls;
		const double Started = FPlatformTime::Seconds();
		PlanetCore::SpillPassageSettings Settings;
		Settings.HalfWidthMeters = Window[0];
		Settings.SpacingMeters = Window[1];
		Settings.MaximumSampleStepMeters = 4;
		const std::optional<PlanetCore::SpillPassage> Bypass = PlanetCore::FindSpillPassage(From, To, Source, Settings);
		AddInfo(
			FString::Printf(
				TEXT("Fixed bypass window: width=%.0f spacing=%.0f accepted=%u calls=%llu ms=%.9g ")
				TEXT("crest=%.17g excess=%.17g pass=%.17g nodes=%u candidates=%u"),
				Window[0],
				Window[1],
				static_cast<unsigned>(Bypass.has_value()),
				Calls - Before,
				(FPlatformTime::Seconds() - Started) * 1000,
				Bypass ? Bypass->CrestMeters : 0,
				Bypass ? Bypass->MaximumIndependentExcessMeters : 0,
				Connection->Pass.ElevationMeters,
				Bypass ? Bypass->Nodes : 0,
				static_cast<unsigned>(Bypass ? Bypass->PassCandidates.size() : 0)
			)
		);
		if(!Bypass || !(Bypass->CrestMeters + Bypass->MaximumIndependentExcessMeters < Connection->Pass.ElevationMeters - 1E-6)) {
			continue;
		}
		HasLower = true;
		unsigned Checked = 0;
		for(const PlanetCore::UnitDirection PassCandidate : Bypass->PassCandidates) {
			if(Checked++ > 31) break;
			const std::optional<PlanetCore::SpillConnection> Lower = PlanetCore::PrepareSpillConnection(PassCandidate, Source, {});
			AddInfo(
				FString::Printf(
					TEXT("Fixed bypass real candidate: accepted=%u height=%.17g"),
					static_cast<unsigned>(Lower.has_value()),
					Lower ? Lower->Pass.ElevationMeters : 0
				)
			);
		}
		break;
	}
	TestTrue(TEXT("旧峠より低い実通路を元場で確認"), HasLower);
	TestTrue(TEXT("有限な全検査予算"), Calls < 4000000);
	AddInfo(FString::Printf(TEXT("Fixed bypass total calls=%llu"), Calls));
	// 拒否を隠さず実測した結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowOutletCurveSeed90210,
	"PlanetGenerator.FlowChannel.OutletCurveSeed90210",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 正しい受入水位へ降った後に曲線を拒否した同じ河道を独立に再現する
 * @param Parameters 未使用の実行引数
 * @return 元の位置・向き・受入水位で河道曲線を構成出来た場合に真；未解決は失敗を保持
 */
bool FPlanetFlowOutletCurveSeed90210::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(90210);
	// 元の拒否と別の元場へ進まない返戻
	if(!TestTrue(TEXT("Seed９０２１０の同じ元場"), Ocean != nullptr)) return false;
	std::uint64_t SourceCalls = 0;
	const auto Source = [Ocean, &SourceCalls](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		++SourceCalls;
		// 拒否した河道と同じ標高・解析勾配の返戻
		return Ocean->Evaluate(D);
	};
	PlanetCore::FlowOutletSettings Settings;
	Settings.DownstreamWaterMeters = 219.80397126278055;
	Settings.IsPositiveBranch = true;
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ -0.025485052050744098, 0.11884002292861584, 0.99258629905529971 });
	PlanetCore::FlowOutletDiagnostics OldReport;
	Settings.LaunchDistanceMeters = 5;
	const std::optional<PlanetCore::FlowOutletRoute> Old =
	PlanetCore::PrepareFlowOutletRoute(Candidate, Source, Settings, {}, &OldReport);
	TestFalse(TEXT("元の５ｍ区間の曲線拒否を保持"), Old.has_value());
	TestTrue(
		TEXT("元の区間の曲率予算超過"),
		OldReport.Failure == PlanetCore::FlowOutletFailure::Curve && OldReport.Curve.Failure == PlanetCore::FlowRouteFailure::Segment &&
		OldReport.Curve.CurvatureProducts[1] > 1
	);
	AddInfo(
		FString::Printf(
			TEXT("Old fixed launch: interval=%u length=%.17g curvature=%.17g"),
			OldReport.Curve.Index,
			OldReport.Curve.ChordMeters,
			OldReport.Curve.CurvatureProducts[1]
		)
	);
	Settings.LaunchDistanceMeters = 1;
	PlanetCore::FlowOutletDiagnostics Report;
	const std::optional<PlanetCore::FlowOutletRoute> Outlet =
	PlanetCore::PrepareFlowOutletRoute(Candidate, Source, Settings, {}, &Report);
	const PlanetCore::FlowRouteDiagnostics &Curve = Report.Curve;
	const PlanetCore::Vec3 First = Curve.Directions[0], Last = Curve.Directions[1];
	AddInfo(
		FString::Printf(
			TEXT("Fixed outlet curve: accepted=%u failure=%u upstream=%u downstream=%u curve=%u index=%u ")
			TEXT("distance=%.17g chord=%.17g forward=(%.17g,%.17g) curvature=(%.17g,%.17g)"),
			static_cast<unsigned>(Outlet.has_value()),
			static_cast<unsigned>(Report.Failure),
			Report.UpstreamVertices,
			Report.DownstreamVertices,
			static_cast<unsigned>(Curve.Failure),
			Curve.Index,
			Curve.DistanceDifferenceMeters,
			Curve.ChordMeters,
			Curve.ForwardProducts[0],
			Curve.ForwardProducts[1],
			Curve.CurvatureProducts[0],
			Curve.CurvatureProducts[1]
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fixed outlet rejected interval: from=(%.17g,%.17g,%.17g) to=(%.17g,%.17g,%.17g)"),
			First.X,
			First.Y,
			First.Z,
			Last.X,
			Last.Y,
			Last.Z
		)
	);
	TestTrue(TEXT("同じ向きと受入水位の河道曲線"), Outlet.has_value());
	if(Outlet) {
		TestTrue(TEXT("同じ元鞍点の標高"), std::abs(Outlet->Pass.ElevationMeters - 253.49454772259435) <= 1E-6);
		double Distance = 0, MaximumDerivative = -std::numeric_limits<double>::infinity();
		for(const PlanetCore::TerrainFlowSegment &Segment : Outlet->Route.GetSegments()) {
			for(const double Fraction : { 0.0, 0.25, 0.5, 0.75, 1.0 }) {
				if(Distance + Fraction * Segment.GetParameterLengthMeters() < Outlet->PassDistanceMeters) continue;
				const PlanetCore::TerrainFlowCurveSample Point = *Segment.Evaluate(Fraction);
				const double Derivative = Source(Point.Direction).TangentialGradientPerRadian.Dot(Point.DirectionDerivativePerMeter);
				MaximumDerivative = std::max(MaximumDerivative, Derivative);
				TestTrue(TEXT("曲線内部も同じ元地形の下降方向"), std::isfinite(Derivative) && Derivative <= 1E-8);
			}
			Distance += Segment.GetParameterLengthMeters();
		}
		AddInfo(FString::Printf(TEXT("Fixed smaller launch maximum source derivative=%.17g"), MaximumDerivative));
	}
	for(
		const double InvalidLaunch :
		{ 0.0, 0.01, std::nextafter(5.0, 6.0), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() }
	) {
		Settings.LaunchDistanceMeters = InvalidLaunch;
		SourceCalls = 0;
		PlanetCore::FlowOutletDiagnostics InvalidReport;
		TestFalse(
			TEXT("無効な開始距離を評価前に拒否"),
			PlanetCore::PrepareFlowOutletRoute(Candidate, Source, Settings, {}, &InvalidReport).has_value()
		);
		TestTrue(TEXT("開始距離の入力拒否を分類"), InvalidReport.Failure == PlanetCore::FlowOutletFailure::InvalidInput);
		TestEqual(TEXT("無効入力で元場を照会しない"), SourceCalls, std::uint64_t{ 0 });
	}
	// 受入水位到達だけを曲線の成功としない返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetDescentReceivingDomain,
	"PlanetGenerator.TerrainDescent.ReceivingDomain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ高度へ達しても所属が未達なら下降を継続し，実領域に入って停止する
 * @param Parameters 未使用の実行引数
 * @return 高度のみ・両条件・未達・照会失敗・中断を区別出来た場合に真
 */
bool FPlanetDescentReceivingDomain::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto X = [Radius](const PlanetCore::UnitDirection Direction) -> double {
			const PlanetCore::Vec3 D = Direction.Vector();
			// 解析場の物理座標の返戻
			return Radius * D.X / D.Z;
		};
		const auto Source = [Radius, X](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			// 平滑で停留点の無い下降場の返戻
			return { X(Direction), { Radius / D.Z, 0, -Radius * D.X / (D.Z * D.Z) } };
		};
		const auto Domain = [X](const PlanetCore::UnitDirection Direction) -> double {
			// 高度到達点の４ｍｍ内側に在る実領域の返戻
			return 1.0089 - X(Direction);
		};
		PlanetCore::TerrainDescentSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.StopElevationMeters = 2;
		Settings.MaximumStepMeters = 1;
		Settings.MaximumLengthMeters = 10;
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({ 2.013 / Radius, 0, 1 });
		const std::optional<PlanetCore::TerrainDescentPath> HeightOnly = PlanetCore::TraceTerrainDescent(Start, Source, Settings);
		const std::optional<PlanetCore::TerrainDescentPath> Both = PlanetCore::TraceTerrainDescent(Start, Source, Settings, {}, Domain);
		const bool HasPaths = TestTrue(TEXT("同じ元場の２経路"), HeightOnly.has_value() && Both.has_value());
		// 経路の欠落への返戻
		if(!HasPaths) return false;
		TestTrue(TEXT("旧高度終端は所属の外側"), Domain(HeightOnly->Samples.back().Direction) < 0);
		TestTrue(TEXT("実領域へ入る迄下降を継続"), Both->Samples.size() > HeightOnly->Samples.size());
		TestTrue(
			TEXT("同じ受入高度と実領域の両方"),
			Both->End == PlanetCore::TerrainDescentEnd::HeightLimit && !(2 < Both->Samples.back().ElevationMeters) &&
			Domain(Both->Samples.back().Direction) >= 0
		);
		for(std::size_t I = 1; I < Both->Samples.size(); ++I) {
			TestTrue(TEXT("元地形の下降を維持"), Both->Samples[I].ElevationMeters <= Both->Samples[I - 1].ElevationMeters);
		}
		const auto Unreachable = [](const PlanetCore::UnitDirection Direction) -> double {
			static_cast<void>(Direction);
			// 範囲外の受入領域を表す返戻
			return -100;
		};
		const std::optional<PlanetCore::TerrainDescentPath> Missing =
		PlanetCore::TraceTerrainDescent(Start, Source, Settings, {}, Unreachable);
		TestTrue(TEXT("未達を高度到達へ置換えない"), Missing && Missing->End == PlanetCore::TerrainDescentEnd::LengthLimit);
		const auto Invalid = [](const PlanetCore::UnitDirection Direction) -> double {
			static_cast<void>(Direction);
			// 照会不能な受入域の返戻
			return std::numeric_limits<double>::quiet_NaN();
		};
		TestFalse(TEXT("非有限な受入域を拒否"), PlanetCore::TraceTerrainDescent(Start, Source, Settings, {}, Invalid).has_value());
		std::stop_source Cancel;
		const auto Cancelling = [&Cancel](const PlanetCore::UnitDirection Direction) -> double {
			static_cast<void>(Direction);
			Cancel.request_stop();
			// 中断前に高度到達を公開させない返戻
			return 0;
		};
		TestFalse(
			TEXT("所属照会中の中断"),
			PlanetCore::TraceTerrainDescent(Start, Source, Settings, Cancel.get_token(), Cancelling).has_value()
		);
		AddInfo(
			FString::Printf(
				TEXT("Receiving domain R=%.0f vertices=%u lastH=%.17g distance=%.17g"),
				Radius,
				static_cast<unsigned>(Both->Samples.size()),
				Both->Samples.back().ElevationMeters,
				Domain(Both->Samples.back().Direction)
			)
		);
	}
	// 同じ精度と元場で行った両条件の結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetMeasuredPassageComparison,
	"PlanetGenerator.SpillPassage.MeasuredComparison",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ解析場の全辺実測と必要辺実測で，両端・外周・最短経路・候補峠を独立比較する
 * @param Parameters 未使用の実行引数
 * @return 同じ出力と精度を保ち，実際の元場照会と辺実測を削減した場合に真
 */
bool FPlanetMeasuredPassageComparison::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = X / 40, V = Y / 20, DX = (4 * U * U * U - 4 * U) / 40;
			const double DY = 2 * V / 20;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 高さ１の解析峠を持つ同じ元場の返戻
			return { U * U * U * U - 2 * U * U + V * V + 1, Gradient };
		};
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({ -40 / Radius, 0, 1 });
		const PlanetCore::UnitDirection End = *PlanetCore::UnitDirection::FromVector({ 40 / Radius, 0, 1 });
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SpacingMeters = 8;
		Settings.HalfWidthMeters = 64;
		Settings.MaximumSampleStepMeters = 1;
		const std::optional<PlanetCore::SpillPassage> Lazy = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
		const std::uint64_t LazyCalls = Calls;
		Calls = 0;
		Settings.MeasureAllPassageEdges = true;
		const std::optional<PlanetCore::SpillPassage> Full = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
		const bool Prepared = TestTrue(TEXT("同じ両端通路を二方式で実測"), Lazy.has_value() && Full.has_value());
		// 一方の欠落を比較成功としない返戻
		if(!Prepared) return false;
		TestEqual(TEXT("必要辺版の実呼出数"), Lazy->Evaluations, LazyCalls);
		TestEqual(TEXT("全辺版の実呼出数"), Full->Evaluations, Calls);
		TestEqual(TEXT("全辺版は全ての実辺を測る"), Full->EvaluatedEdges, Full->TotalEdges);
		TestTrue(TEXT("同じ格子の辺実測を削減"), Lazy->EvaluatedEdges < Full->EvaluatedEdges);
		TestTrue(TEXT("同じ場の実呼出を削減"), LazyCalls < Calls);
		TestEqual(TEXT("最低通過高度の完全一致"), Lazy->CrestMeters, Full->CrestMeters, 0);
		TestEqual(TEXT("外周の下限の完全一致"), Lazy->OutsideLowerBoundMeters, Full->OutsideLowerBoundMeters, 0);
		for(unsigned Side = 0; Side < 2; ++Side) {
			TestEqual(TEXT("両側の外周への実測高度"), Lazy->BoundaryEscapeMeters[Side], Full->BoundaryEscapeMeters[Side], 0);
		}
		TestEqual(TEXT("独立試料残差の完全一致"), Lazy->MaximumIndependentExcessMeters, Full->MaximumIndependentExcessMeters, 0);
		TestEqual(TEXT("実経路長の完全一致"), Lazy->PathLengthMeters, Full->PathLengthMeters, 0);
		TestEqual(TEXT("経路頂点数を保持"), Lazy->Path.size(), Full->Path.size());
		TestEqual(TEXT("峠候補数を保持"), Lazy->PassCandidates.size(), Full->PassCandidates.size());
		for(std::size_t I = 0; I < std::min(Lazy->Path.size(), Full->Path.size()); ++I) {
			TestEqual(TEXT("経路方向の完全一致"), (Lazy->Path[I].Vector() - Full->Path[I].Vector()).Length(), 0.0, 0);
		}
		for(std::size_t I = 0; I < std::min(Lazy->PassCandidates.size(), Full->PassCandidates.size()); ++I) {
			TestEqual(TEXT("実峠候補の完全一致"), (Lazy->PassCandidates[I].Vector() - Full->PassCandidates[I].Vector()).Length(), 0.0, 0);
		}
		TestEqual(TEXT("最高点方向の完全一致"), (Lazy->Peak.Vector() - Full->Peak.Vector()).Length(), 0.0, 0);
		TestEqual(TEXT("外周接触を保持"), Lazy->TouchesBoundary, Full->TouchesBoundary);
		TestEqual(TEXT("窓内最低解の判定を保持"), Lazy->IsGraphInteriorMinimum, Full->IsGraphInteriorMinimum);
		AddInfo(
			FString::Printf(
				TEXT("Measured passage R=%.0f calls=%llu/%llu edges=%u/%u crest=%.17g boundary=(%.17g,%.17g)"),
				Radius,
				LazyCalls,
				Calls,
				Lazy->EvaluatedEdges,
				Full->EvaluatedEdges,
				Lazy->CrestMeters,
				Lazy->BoundaryEscapeMeters[0],
				Lazy->BoundaryEscapeMeters[1]
			)
		);
	}
	// 出力・拒否条件・実費用を照合した結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillShoreSeed7Budget,
	"PlanetGenerator.SpillLake.Seed7ShoreBudget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 未帰還の同じ岸を再現し，倍の有限保持量での帰還と繰返し位置を切分ける
 * @param Parameters 未使用の実行引数
 * @return 同じ位置・水位・精度で帰還し，正の単純な岸を確認した場合に真
 */
bool FPlanetSpillShoreSeed7Budget::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(7);
	// 同じ拒否地点の元場を欠く場合の返戻
	if(!TestTrue(TEXT("Seed７の同じ元場"), Ocean != nullptr)) return false;
	const auto Source = [Ocean](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		// 同じ生成版の元地形の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ -0.18181765104874983, 0.48988486504501727, 0.85261665522492602 });
	const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(Candidate, Source, {});
	// 記録した峠を別の終端へ読み替えない返戻
	if(!TestTrue(TEXT("固定した実峠"), Connection.has_value())) return false;
	const PlanetCore::TerrainCriticalPoint &Pass = Connection->Pass;
	TestTrue(TEXT("記録した水位の一致"), std::abs(Pass.ElevationMeters - 855.30577761299583) <= 1E-6);
	PlanetCore::TerrainContourSettings Settings;
	Settings.MaximumLengthMeters = 1E6;
	PlanetCore::SpillLakeDiagnostics OriginalReport;
	const std::optional<PlanetCore::SpillLakeShore> Original =
	PlanetCore::PrepareSpillLakeShore(Pass, false, Source, Settings, {}, &OriginalReport);
	TestTrue(TEXT("旧保持量での未帰還を再現"), !Original && OriginalReport.End == PlanetCore::TerrainContourEnd::VertexLimit);
	// 保持量の影響だけを比較し，元場・弦・高さ・位置精度は同じ
	Settings.MaximumVertices = 65536;
	PlanetCore::SpillLakeDiagnostics ExtendedReport;
	const std::optional<PlanetCore::SpillLakeShore> Extended =
	PlanetCore::PrepareSpillLakeShore(Pass, false, Source, Settings, {}, &ExtendedReport);
	// 部分経路は診断だけに使い，公開する閉じた水域へ変換しない
	PlanetCore::TerrainContourSettings TraceSettings = Settings;
	TraceSettings.LevelMeters = Pass.ElevationMeters;
	TraceSettings.Terminals = { Pass.Direction };
	TraceSettings.MaximumVertices = Settings.MaximumVertices - 1;
	const PlanetCore::Vec3 Up = Pass.Direction.Vector();
	const PlanetCore::Vec3 Along =
	PlanetCore::UnitDirection::FromVector(Pass.MinimumCurvatureDirection - Up * Pass.MinimumCurvatureDirection.Dot(Up))->Vector() *
	-1;
	const PlanetCore::Vec3 Across = Up.Cross(Along);
	const double Ratio = std::sqrt(-Pass.MinimumCurvaturePerMeter / Pass.MaximumCurvaturePerMeter);
	const PlanetCore::Vec3 Arm = PlanetCore::UnitDirection::FromVector(Along - Across * Ratio)->Vector();
	const double Angle = Settings.ChordToleranceMeters * 0.5 / Settings.RadiusMeters;
	const PlanetCore::UnitDirection Initial = *PlanetCore::UnitDirection::FromVector(Up * std::cos(Angle) + Arm * std::sin(Angle));
	const std::optional<PlanetCore::TerrainContour> Partial = PlanetCore::TraceTerrainContour(Initial, Source, TraceSettings);
	double NearestOld = std::numeric_limits<double>::infinity(), NearestOldArc = 0;
	if(Partial && !Partial->Samples.empty()) {
		const PlanetCore::TerrainContourSample &Last = Partial->Samples.back();
		const PlanetCore::Vec3 P = Last.Direction.Vector();
		for(const PlanetCore::TerrainContourSample &Point : Partial->Samples) {
			if(Last.DistanceMeters - Point.DistanceMeters < 100) continue;
			const PlanetCore::Vec3 Q = Point.Direction.Vector();
			const double Distance = Settings.RadiusMeters * std::atan2(P.Cross(Q).Length(), P.Dot(Q));
			if(Distance < NearestOld) {
				NearestOld = Distance;
				NearestOldArc = Last.DistanceMeters - Point.DistanceMeters;
			}
		}
	}
	const PlanetCore::Vec3 Last = ExtendedReport.LastDirection;
	const double ToPass = Settings.RadiusMeters * std::atan2(Last.Cross(Up).Length(), Last.Dot(Up));
	AddInfo(
		FString::Printf(
			TEXT("Fixed Seed7 shore: original=(%u,%u,%.17g) extended=(%u,%u,%u,%.17g,%.17g) ")
			TEXT("lastToPass=%.17g nearestOld=%.17g oldArc=%.17g"),
			static_cast<unsigned>(OriginalReport.Failure),
			OriginalReport.Vertices,
			OriginalReport.LengthMeters,
			static_cast<unsigned>(ExtendedReport.Failure),
			static_cast<unsigned>(ExtendedReport.End),
			ExtendedReport.Vertices,
			ExtendedReport.LengthMeters,
			ExtendedReport.AreaMetersSquared,
			ToPass,
			NearestOld,
			NearestOldArc
		)
	);
	TestTrue(TEXT("有限な保持量で同じ岸が正しく帰還"), Extended.has_value());
	const bool HasMinima = Connection->Branches[0].Minimum.has_value() && Connection->Branches[1].Minimum.has_value();
	// 極小の欠落による所属検査の省略を，成功にしない返戻
	if(!TestTrue(TEXT("所属を検査する両実極小"), HasMinima)) return false;
	if(Extended) {
		const double UpstreamDistance = Extended->SignedPlanarDistanceMeters(Connection->Branches[0].Minimum->Direction);
		const double DownstreamDistance = Extended->SignedPlanarDistanceMeters(Connection->Branches[1].Minimum->Direction);
		AddInfo(
			FString::Printf(TEXT("Fixed Seed7 branch membership: upstream=%.17g downstream=%.17g"), UpstreamDistance, DownstreamDistance)
		);
		TestTrue(TEXT("外岸が上流の実極小を含む"), UpstreamDistance >= 0);
		TestTrue(TEXT("外岸が下流の実極小を含まない"), DownstreamDistance < 0);
	}
	// 未確認を成功へ置換えない診断結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillPathCrestContracts,
	"PlanetGenerator.SpillPassage.PathCrest",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 曲線候補の頂点間の極大を，解析解・予算超過・中断と独立に検査する
 * @param Parameters 未使用の実行引数
 * @return 同じ測定精度と実照会予算を満たす場合に真
 */
bool FPlanetSpillPathCrestContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, U = X - 1, Height = 100 - U * U, DX = -2 * U;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, 0, -Radius * DX * D.X / (D.Z * D.Z) };
			// 頂点間に解析極大を持つ同じ放物線の返戻
			return { Height, Gradient };
		};
		const std::array<PlanetCore::UnitDirection, 2> Path {
			*PlanetCore::UnitDirection::FromVector({ -5 / Radius, 0, 1 }),
			*PlanetCore::UnitDirection::FromVector({ 5 / Radius, 0, 1 })
		};
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.MaximumSampleStepMeters = 4;
		const std::optional<PlanetCore::SpillPathCrest> Measured = PlanetCore::MeasureSpillPathCrest(Path, Source, Settings);
		// 実際の測定が無い場合の返戻
		if(!TestTrue(TEXT("経路全辺を同じ元場で測定"), Measured.has_value())) return false;
		TestEqual(TEXT("解析極大の一致"), Measured->CrestMeters, 100.0, 1E-7);
		TestEqual(TEXT("実測回数の一致"), Measured->Evaluations, Calls);
		TestTrue(
			TEXT("頂点だけでは見落とす極大"),
			Measured->CrestMeters > std::max(Source(Path[0]).ElevationMeters, Source(Path[1]).ElevationMeters)
		);
		TestTrue(TEXT("鞍点照会へ渡せる実測候補"), !Measured->PassCandidates.empty());
		const PlanetCore::Vec3 Peak = Measured->Peak.Vector();
		TestTrue(TEXT("解析最高点の位置"), std::abs(Radius * Peak.X / Peak.Z - 1) < 1E-4);
		Settings.MaximumEvaluations = 1;
		Calls = 0;
		TestFalse(TEXT("全経路の予算不足を拒否"), PlanetCore::MeasureSpillPathCrest(Path, Source, Settings).has_value());
		TestEqual(TEXT("予算を超えて元場を呼ばない"), Calls, UINT64_C(1));
		Settings.MaximumEvaluations = 4000000;
		std::stop_source Cancel;
		Cancel.request_stop();
		Calls = 0;
		TestFalse(TEXT("中断済の経路を拒否"), PlanetCore::MeasureSpillPathCrest(Path, Source, Settings, Cancel.get_token()).has_value());
		TestEqual(TEXT("中断後に元場を呼ばない"), Calls, UINT64_C(0));
		const std::array<PlanetCore::UnitDirection, 2> Degenerate { Path.front(), Path.front() };
		TestFalse(TEXT("退化辺を測定済にしない"), PlanetCore::MeasureSpillPathCrest(Degenerate, Source, Settings).has_value());
	}
	// 独立な測定条件の結果を返す
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillBypassContinuousSeed0,
	"PlanetGenerator.SpillLake.Seed0ContinuousBypass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 広域再検査で拒否した同じ低い出口を，格子候補の連続な絞込みで再調査する
 * @param Parameters 未使用の実行引数
 * @return 元の地形に同じ両終端を結ぶ低い実測通路を確認した場合に真
 */
bool FPlanetSpillBypassContinuousSeed0::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(0);
	// 別 Seed 又は別地形へ置換えない返戻
	if(!TestTrue(TEXT("同じ Seed０の元場"), Ocean != nullptr)) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		if(Calls > 3999999) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 全検査予算を超える場合の返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++Calls;
		// 同じ不変な元標高・勾配の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ 0.45907779761942397, -0.042729408115227421, 0.88736789068290123 });
	const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(Candidate, Source, {});
	// 元の鞍点を欠く場合の返戻
	if(!TestTrue(TEXT("元の両枝を持つ実鞍点"), Connection.has_value())) return false;
	TestTrue(TEXT("元の７．４４３９ｍの実水位"), std::abs(Connection->Pass.ElevationMeters - 7.4439186228890115) <= 1E-6);
	const bool HasMinima =
	TestTrue(TEXT("同じ両終端は実極小"), Connection->Branches[0].Minimum.has_value() && Connection->Branches[1].Minimum.has_value());
	// 未確認の両端を照会しない返戻
	if(!HasMinima) return false;
	PlanetCore::TerrainContourSettings ShoreSettings;
	ShoreSettings.MaximumVertices = 65536;
	ShoreSettings.MaximumLengthMeters = 1E6;
	PlanetCore::SpillLakeDiagnostics Shore;
	TestFalse(
		TEXT("元の島の境界を拒否"),
		PlanetCore::PrepareSpillLakeShore(Connection->Pass, false, Source, ShoreSettings, {}, &Shore).has_value()
	);
	TestTrue(TEXT("負の面積を湖へ反転しない"), Shore.Failure == PlanetCore::SpillLakeFailure::Boundary && Shore.AreaMetersSquared < 0);
	std::optional<PlanetCore::SpillPassage> Best;
	const std::array<std::array<double, 2>, 7> Windows {
		std::array<double, 2>{ 256, 16 },
		{ 512, 16 },
		{ 1024, 16 },
		{ 2048, 16 },
		{ 4096, 32 },
		{ 8192, 64 },
		{ 16384, 128 }
	};
	for(const std::array<double, 2> Window : Windows) {
		PlanetCore::SpillPassageSettings Settings;
		Settings.HalfWidthMeters = Window[0];
		Settings.SpacingMeters = Window[1];
		Settings.MaximumSampleStepMeters = 4;
		std::optional<PlanetCore::SpillPassage> Grid = PlanetCore::FindSpillPassage(
			Connection->Branches[0].Minimum->Direction,
			Connection->Branches[1].Minimum->Direction,
			Source,
			Settings
		);
		if(Grid && (!Best || Grid->CrestMeters < Best->CrestMeters)) Best = std::move(Grid);
	}
	// 元の格子候補が無い場合の返戻
	if(!TestTrue(TEXT("同じ両端の有界格子候補"), Best.has_value())) return false;
	PlanetCore::SpillStringSettings RelaxSettings;
	RelaxSettings.SpacingMeters = 16;
	RelaxSettings.TimeStepMeters = 5000;
	RelaxSettings.MaximumMoveMeters = 8;
	RelaxSettings.MaximumIterations = 512;
	RelaxSettings.MaximumEvaluations = 1000000;
	const std::optional<PlanetCore::SpillString> Relaxed = PlanetCore::RelaxSpillString(Best->Path, Source, RelaxSettings);
	// 絞込み失敗を実測通路へ変換しない返戻
	if(!TestTrue(TEXT("同じ元場で経路を絞込み"), Relaxed.has_value())) return false;
	PlanetCore::SpillPassageSettings MeasureSettings;
	MeasureSettings.MaximumSampleStepMeters = 4;
	const std::optional<PlanetCore::SpillPathCrest> Measured =
	PlanetCore::MeasureSpillPathCrest(Relaxed->Path, Source, MeasureSettings);
	// 頂点の高さだけで低い出口としない返戻
	if(!TestTrue(TEXT("絞込み後の全辺を再測定"), Measured.has_value())) return false;
	AddInfo(
		FString::Printf(
			TEXT("Continuous bypass seed=0 old=%.17g grid=%.17g measured=%.17g calls=%llu ")
			TEXT("iterations=%u motion=%.17g slope=%.17g candidates=%u"),
			Connection->Pass.ElevationMeters,
			Best->CrestMeters,
			Measured->CrestMeters,
			Calls,
			Relaxed->Iterations,
			Relaxed->MaximumMotionMeters,
			Relaxed->MaximumNormalSlope,
			static_cast<unsigned>(Measured->PassCandidates.size())
		)
	);
	unsigned Checked = 0, Accepted = 0;
	for(const PlanetCore::UnitDirection Pass : Measured->PassCandidates) {
		if(Checked++ > 31) break;
		const std::optional<PlanetCore::SpillConnection> Lower = PlanetCore::PrepareSpillConnection(Pass, Source, {});
		if(Lower && Lower->Pass.ElevationMeters < Connection->Pass.ElevationMeters - 1E-6) ++Accepted;
		AddInfo(
			FString::Printf(
				TEXT("Continuous bypass actual pass: accepted=%u H=%.17g"),
				static_cast<unsigned>(Lower.has_value()),
				Lower ? Lower->Pass.ElevationMeters : 0
			)
		);
	}
	TestTrue(
		TEXT("同じ終端間の実通過高度が旧出口より低い"),
		Measured->CrestMeters + Measured->MaximumIndependentExcessMeters < Connection->Pass.ElevationMeters - 1E-6
	);
	TestTrue(TEXT("低い実鞍点を元場で確認"), Accepted > 0);
	TestTrue(TEXT("全検査予算を維持"), Calls < 4000000);
	// 元の失敗と同じ条件で確認した結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLowHeadFlowFieldSeed7,
	"PlanetGenerator.FlowChannel.Seed7LowHeadField",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 全体検査で場の構成を拒否した同じ出口を，受入湖の所属とは独立に調べる
 * @param Parameters 未使用の実行引数
 * @return 同じ源場・水位・向きの河道場が構成出来た場合に真；未解決は失敗として保持
 */
bool FPlanetLowHeadFlowFieldSeed7::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(7);
	// 同じ元場が無い場合の返戻
	if(!TestTrue(TEXT("同じ Seed７の元地形"), Ocean != nullptr)) return false;
	const auto Source = [Ocean](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		// 水位又は地形を変更しない返戻
		return Ocean->Evaluate(D);
	};
	const auto Height = [Ocean](const PlanetCore::UnitDirection D) -> double {
		// 同じ元標高専用の返戻
		return Ocean->EvaluateElevation(D);
	};
	PlanetCore::FlowPreparationSettings Settings;
	Settings.Seed = 7;
	Settings.Outlet.DownstreamWaterMeters = 872.92013311102505;
	Settings.Outlet.IsPositiveBranch = true;
	const PlanetCore::UnitDirection Pass =
	*PlanetCore::UnitDirection::FromVector({ -0.20671847613317093, 0.50660861157759407, 0.83702758992795567 });
	PlanetCore::FlowPreparationDiagnostics Report;
	const std::optional<PlanetCore::PreparedFlowChannel> Channel =
	PlanetCore::PrepareFlowChannel(Pass, Source, Height, Settings, {}, {}, &Report);
	AddInfo(
		FString::Printf(
			TEXT("Low head seed=7 accepted=%u failure=%u shore=%u water=%u knots=%u passes=%u ")
			TEXT("evaluations=%u Herror=%.17g Gerror=%.17g"),
			static_cast<unsigned>(Channel.has_value()),
			static_cast<unsigned>(Report.Failure),
			static_cast<unsigned>(Report.Shore.Failure),
			static_cast<unsigned>(Report.Water.Failure),
			Report.Water.Knots,
			Report.Water.Passes,
			Report.Water.Evaluations,
			Report.Water.HeightErrorMeters,
			Report.Water.SlopeError
		)
	);
	TestTrue(TEXT("同じ微小水位差の場を構成"), Channel.has_value());
	// 元の失敗を独立に切分けた結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLowHeadWaterJoinContracts,
	"PlanetGenerator.FlowChannel.LowHeadWaterJoin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 峠の水位・微分を解析縦断へ保ち，実水位差の大小に依らず下流へ接続する事を検査する
 * @param Parameters 未使用の実行引数
 * @return 連続な上流接続と非増加な縦断が，同じ既存精度内で確認出来た場合に真
 */
bool FPlanetLowHeadWaterJoinContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	std::uint64_t Checked = 0;
	for(const double Upstream : { 0.1, 100.0, 872.96413096560207 }) {
		for(const double Head : { 0.0, 1E-6, 1E-4, 0.01, 0.04399785457702432, 0.05, 0.1, 1.0, 100.0 }) {
			PlanetCore::FlowChannelWaterSettings Settings;
			Settings.DownstreamWaterMeters = Upstream - Head;
			const auto Source = [Upstream](const double Distance) -> PlanetCore::FlowChannelWaterLevel {
				// 解析鞍点を零距離に持つ，元の下降する放物線の返戻
				return { Upstream - 0.0005 * Distance * Distance, -0.001 * Distance };
			};
			const auto Water = [&](const double Distance) -> std::optional<PlanetCore::FlowChannelWaterLevel> {
				// 補正対象の同じ水位接続だけを呼ぶ返戻
				return PlanetCore::SampleFlowOutletWaterLevel(Source(Distance), Distance, Upstream, Settings);
			};
			const std::optional<PlanetCore::FlowChannelWaterLevel> At = Water(0), Near = Water(0.001);
			// 未構成の水位を解析解へ置換えない返戻
			if(!TestTrue(TEXT("峠と直後を同じ式で接続"), At.has_value() && Near.has_value())) return false;
			TestEqual(TEXT("峠の上流水位を保持"), At->ElevationMeters, Upstream, 0);
			TestEqual(TEXT("峠の静水面の微分"), At->DerivativePerMeter, 0.0, 0);
			TestTrue(TEXT("微小水位差でも峠直後が連続"), std::abs(Near->ElevationMeters - Upstream) < 1E-8);
			TestTrue(TEXT("峠直後の微分が零へ接続"), std::abs(Near->DerivativePerMeter) < 1E-8);
			double Previous = Upstream;
			for(unsigned I = 0; I < 1001; ++I) {
				const double Distance = I * 0.5;
				const std::optional<PlanetCore::FlowChannelWaterLevel> Value = Water(Distance);
				// 部分的な縦断を成功としない返戻
				if(!TestTrue(TEXT("解析縦断の全独立点"), Value.has_value())) return false;
				TestTrue(TEXT("下流向きの水位の非増加"), Value->ElevationMeters <= Previous);
				TestTrue(TEXT("下流向きの微分の非正"), !(Value->DerivativePerMeter > 0));
				TestTrue(TEXT("受入静水位を下回らない"), Value->ElevationMeters >= Settings.DownstreamWaterMeters);
				if(I && I < 1000) {
					const std::optional<PlanetCore::FlowChannelWaterLevel> Before = Water(Distance - 0.001), After = Water(Distance + 0.001);
					// 差分の欠落を飛越さない返戻
					if(!TestTrue(TEXT("独立差分の両側"), Before.has_value() && After.has_value())) return false;
					const std::optional<PlanetCore::FlowChannelWaterLevel> HalfBefore = Water(Distance - 0.0005);
					const std::optional<PlanetCore::FlowChannelWaterLevel> HalfAfter = Water(Distance + 0.0005);
					const std::optional<PlanetCore::FlowChannelWaterLevel> QuarterBefore = Water(Distance - 0.00025);
					const std::optional<PlanetCore::FlowChannelWaterLevel> QuarterAfter = Water(Distance + 0.00025);
					// 別の刻み幅でも差分の欠落を成功としない返戻
					if(!TestTrue(TEXT("差分の刻み幅縮小"), HalfBefore && HalfAfter && QuarterBefore && QuarterAfter)) return false;
					const double Difference = (After->ElevationMeters - Before->ElevationMeters) / 0.002;
					const double Half = (HalfAfter->ElevationMeters - HalfBefore->ElevationMeters) / 0.001;
					const double Quarter = (QuarterAfter->ElevationMeters - QuarterBefore->ElevationMeters) / 0.0005;
					// 狭い遷移部の中央差分の打切り誤差を，独立な二段階で除く
					const double Richardson = (4 * Half - Difference) / 3, Refined = (4 * Quarter - Half) / 3;
					TestTrue(TEXT("接続式の微分を独立差分で照合"), std::abs(Refined - Value->DerivativePerMeter) < 1E-6);
					TestTrue(TEXT("独立な二段階の差分収束"), std::abs(Refined - Richardson) < 1E-6);
				}
				Previous = Value->ElevationMeters;
				++Checked;
			}
			TestEqual(TEXT("下流静水面へ実際に到達"), Previous, Settings.DownstreamWaterMeters, 0);
		}
	}
	AddInfo(FString::Printf(TEXT("Low head water joins: cases=27 samples=%llu"), Checked));
	// 独立な値と微分・終端条件の結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetLowHeadFlowFieldSeed0,
	"PlanetGenerator.FlowChannel.Seed0LowHeadField",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 第１７版で初めて到達した同じ出口を，水位差２．１２㎜の実条件で再検査する
 * @param Parameters 未使用の実行引数
 * @return 同じ元場と上下流水位で準備した場合に真；不成功は失敗として残す
 */
bool FPlanetLowHeadFlowFieldSeed0::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(0);
	// 同じ元場が無い場合の返戻
	if(!TestTrue(TEXT("同じ Seed０の元地形"), Ocean != nullptr)) return false;
	const auto Source = [Ocean](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		// 水位又は地形を変更しない返戻
		return Ocean->Evaluate(D);
	};
	const auto Height = [Ocean](const PlanetCore::UnitDirection D) -> double {
		// 同じ元標高専用の返戻
		return Ocean->EvaluateElevation(D);
	};
	PlanetCore::FlowPreparationSettings Settings;
	Settings.Seed = 0;
	Settings.Outlet.DownstreamWaterMeters = 4.2384648373983076;
	Settings.Outlet.IsPositiveBranch = false;
	const PlanetCore::UnitDirection Pass =
	*PlanetCore::UnitDirection::FromVector({ 0.46044431963701743, -0.040743385562438166, 0.88675306881167015 });
	PlanetCore::FlowPreparationDiagnostics Report;
	const std::optional<PlanetCore::PreparedFlowChannel> Channel =
	PlanetCore::PrepareFlowChannel(Pass, Source, Height, Settings, {}, {}, &Report);
	AddInfo(
		FString::Printf(
			TEXT("Low head seed=0 accepted=%u failure=%u shore=%u water=%u knots=%u passes=%u ")
			TEXT("evaluations=%u Herror=%.17g Gerror=%.17g"),
			static_cast<unsigned>(Channel.has_value()),
			static_cast<unsigned>(Report.Failure),
			static_cast<unsigned>(Report.Shore.Failure),
			static_cast<unsigned>(Report.Water.Failure),
			Report.Water.Knots,
			Report.Water.Passes,
			Report.Water.Evaluations,
			Report.Water.HeightErrorMeters,
			Report.Water.SlopeError
		)
	);
	TestTrue(TEXT("同じ微小水位差の場を構成"), Channel.has_value());
	// 元の失敗を独立に切分けた結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillWindowPathDiversity,
	"PlanetGenerator.SpillLake.Seed0WindowPathDiversity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 同じ失敗峠の各有限窓を独立に緩和し，最低格子候補の選択と連続な実通路を区別する
 * @param Parameters 未使用の実行引数
 * @return 二つの記録した入力で，同じ両終端を結ぶ低い実測通路と実鞍点を有界予算で確認した場合に真
 */
bool FPlanetSpillWindowPathDiversity::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(0);
	// 別の元地形で同じ峠を代用しない返戻
	if(!TestTrue(TEXT("記録した Seed０の同じ元場"), Ocean != nullptr)) return false;
	std::optional<PlanetCore::SpillSampleCache> Cache = PlanetCore::SpillSampleCache::Create(262144);
	// 厳密な再利用枠を作成出来ない場合の返戻
	if(!TestTrue(TEXT("同じ元場だけを保持する厳密な再利用"), Cache.has_value())) return false;
	std::uint64_t Calls = 0, Hits = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		if(const std::optional<PlanetCore::TerrainSample> Stored = Cache->Find(D)) {
			++Hits;
			// 同じ方向の全ビットに対応した元場の返戻
			return *Stored;
		}
		if(Calls > 3999999) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 全入力・全窓を合算した評価不足の返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++Calls;
		const PlanetCore::TerrainSample Value = Ocean->Evaluate(D);
		Cache->Store(D, Value);
		// 同じ不変な元標高と勾配の返戻
		return Value;
	};
	const std::array<PlanetCore::Vec3, 2> Inputs {
		PlanetCore::Vec3{ 0.45907779761942397, -0.042729408115227421, 0.88736789068290123 },
		{ 0.45907779757841888, -0.042729408122133397, 0.88736789070378264 }
	};
	const std::array<std::array<double, 2>, 7> Windows {
		std::array<double, 2>{ 256, 16 },
		{ 512, 16 },
		{ 1024, 16 },
		{ 2048, 16 },
		{ 4096, 32 },
		{ 8192, 64 },
		{ 16384, 128 }
	};
	unsigned Case = 0;
	for(const PlanetCore::Vec3 Input : Inputs) {
		const PlanetCore::UnitDirection Candidate = *PlanetCore::UnitDirection::FromVector(Input);
		const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(Candidate, Source, {});
		const bool HasBoth = Connection && Connection->Branches[0].Minimum && Connection->Branches[1].Minimum;
		// 記録した峠の両終端を作成出来ない場合の返戻
		if(!TestTrue(TEXT("同じ失敗峠と両実極小を準備"), HasBoth)) return false;
		TestTrue(TEXT("同じ実峠の物理標高"), std::abs(Connection->Pass.ElevationMeters - 7.4439186228890115) <= 1E-6);
		for(unsigned Side = 0; Side < 2; ++Side) {
			const PlanetCore::Vec3 D = Connection->Branches[Side].Minimum->Direction.Vector();
			AddInfo(FString::Printf(TEXT("Window diversity case=%u terminal=%u D=(%.17g,%.17g,%.17g)"), Case, Side, D.X, D.Y, D.Z));
		}
		double BestGrid = std::numeric_limits<double>::infinity(), BestRelaxed = 0;
		unsigned BestWindow = 0, Checked = 0, LowerPaths = 0, LowerPasses = 0;
		for(const std::array<double, 2> Window : Windows) {
			PlanetCore::SpillPassageSettings Settings;
			Settings.HalfWidthMeters = Window[0];
			Settings.SpacingMeters = Window[1];
			Settings.MaximumSampleStepMeters = 4;
			Settings.UsePathEdgeSelection = true;
			const std::optional<PlanetCore::SpillPassage> Grid = PlanetCore::FindSpillPassage(
				Connection->Branches[0].Minimum->Direction,
				Connection->Branches[1].Minimum->Direction,
				Source,
				Settings
			);
			if(!Grid) {
				AddInfo(FString::Printf(TEXT("Window diversity case=%u width=%.0f grid=absent calls=%llu"), Case, Window[0], Calls));
				continue;
			}
			++Checked;
			PlanetCore::SpillStringSettings RelaxSettings;
			RelaxSettings.SpacingMeters = 16;
			RelaxSettings.TimeStepMeters = 5000;
			RelaxSettings.MaximumMoveMeters = 8;
			RelaxSettings.MaximumIterations = 512;
			RelaxSettings.MaximumEvaluations = 1000000;
			const std::optional<PlanetCore::SpillString> Relaxed = PlanetCore::RelaxSpillString(Grid->Path, Source, RelaxSettings);
			std::optional<PlanetCore::SpillPathCrest> Measured;
			if(Relaxed) Measured = PlanetCore::MeasureSpillPathCrest(Relaxed->Path, Source, Settings);
			const double Raw = Grid->CrestMeters + Grid->MaximumIndependentExcessMeters;
			const double Final =
			Measured ? Measured->CrestMeters + Measured->MaximumIndependentExcessMeters : std::numeric_limits<double>::infinity();
			if(Raw < BestGrid) {
				BestGrid = Raw;
				BestRelaxed = Final;
				BestWindow = static_cast<unsigned>(Window[0]);
			}
			const bool Lower = Measured && Final < Connection->Pass.ElevationMeters - 1E-6;
			if(Lower) {
				++LowerPaths;
				for(const PlanetCore::UnitDirection Pass : Measured->PassCandidates) {
					const std::optional<PlanetCore::SpillConnection> Actual = PlanetCore::PrepareSpillConnection(Pass, Source, {});
					if(Actual && Actual->Pass.ElevationMeters < Connection->Pass.ElevationMeters - 1E-6) ++LowerPasses;
				}
			}
			AddInfo(
				FString::Printf(
					TEXT("Window diversity case=%u width=%.0f grid=%.17g relaxed=%.17g lower=%d iterations=%u calls=%llu hits=%llu"),
					Case,
					Window[0],
					Raw,
					Final,
					Lower,
					Relaxed ? Relaxed->Iterations : 0,
					Calls,
					Hits
				)
			);
		}
		AddInfo(
			FString::Printf(
				TEXT(
					"Window diversity result case=%u checked=%u bestWidth=%u bestGrid=%.17g bestRelaxed=%.17g old=%.17g lowerPaths=%u passes=%u"
				),
				Case,
				Checked,
				BestWindow,
				BestGrid,
				BestRelaxed,
				Connection->Pass.ElevationMeters,
				LowerPaths,
				LowerPasses
			)
		);
		TestTrue(TEXT("同じ両端の有限窓に低い連続通路が存在"), LowerPaths > 0);
		TestTrue(TEXT("低い実鞍点の両枝も同じ元場で確認"), LowerPasses > 0);
		++Case;
	}
	TestTrue(TEXT("二入力の全窓と実峠を合算して予算を維持"), Calls < 4000000);
	// 同じ入力条件で各窓を測定した結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillContinuousFallbackCandidates,
	"PlanetGenerator.SpillLake.Seed0ContinuousFallbackCandidates",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 格子の通路が未確認でも，同じ実極小からの連続候補が低い実鞍点へ至るか調べる
 * @param Parameters 未使用の実行引数
 * @return 記録した元場で両枝を確認した低い候補を得た場合に真；未確認は失敗として保持
 */
bool FPlanetSpillContinuousFallbackCandidates::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean = MakeFailureTopologyOcean(0);
	// 元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("記録した Seed０ の元場"), Ocean != nullptr)) return false;
	std::optional<PlanetCore::SpillSampleCache> Cache = PlanetCore::SpillSampleCache::Create(262144);
	// 厳密な保持枠が無い場合の返戻
	if(!TestTrue(TEXT("限定した厳密な標本保持枠"), Cache.has_value())) return false;
	std::uint64_t Calls = 0, Hits = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		if(const std::optional<PlanetCore::TerrainSample> Stored = Cache->Find(D)) {
			++Hits;
			// 同じ元場の同じ方向の実測済成分の返戻
			return *Stored;
		}
		if(Calls > 1999999) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 共通の限定予算後に元場を照会しない返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++Calls;
		const PlanetCore::TerrainSample Value = Ocean->Evaluate(D);
		Cache->Store(D, Value);
		// 値を変えない実標高と勾配の返戻
		return Value;
	};
	const PlanetCore::UnitDirection Candidate =
	*PlanetCore::UnitDirection::FromVector({ 0.41536108372054598, -0.0086633486137249593, 0.90961536735110771 });
	const std::optional<PlanetCore::SpillConnection> Old = PlanetCore::PrepareSpillConnection(Candidate, Source, {});
	// 元の峠と両枝を準備出来ない場合の返戻
	if(!TestTrue(TEXT("同じ峠の実接続"), Old.has_value())) return false;
	TestTrue(TEXT("旧水位を精度内で再現"), std::abs(Old->Pass.ElevationMeters - 131.25787932872947) <= 1E-6);
	const bool HasMinima = Old->Branches[0].Minimum && Old->Branches[1].Minimum;
	// 未確認の極小を候補探索へ渡さない返戻
	if(!TestTrue(TEXT("両終端は実極小"), HasMinima)) return false;
	const std::array<PlanetCore::TerrainCriticalPoint, 2> Known { *Old->Branches[0].Minimum, *Old->Branches[1].Minimum };
	const std::array<PlanetCore::SpillConnection, 1> Completed { *Old };
	AddInfo(
		FString::Printf(
			TEXT("Fallback old=%.17g curvature=(%.17g,%.17g) minima=(%.17g,%.17g)"),
			Old->Pass.ElevationMeters,
			Old->Pass.MinimumCurvaturePerMeter,
			Old->Pass.MaximumCurvaturePerMeter,
			Known[0].ElevationMeters,
			Known[1].ElevationMeters
		)
	);
	bool HasLower = false;
	for(unsigned Side = 0; Side < 2; ++Side) {
		PlanetCore::TerrainContourSettings ShoreSettings;
		ShoreSettings.MaximumLengthMeters = 1E6;
		PlanetCore::SpillLakeDiagnostics ShoreReport;
		const std::optional<PlanetCore::SpillLakeShore> Shore =
		PlanetCore::PrepareSpillLakeShore(Old->Pass, Side != 0, Source, ShoreSettings, {}, &ShoreReport);
		AddInfo(
			FString::Printf(
				TEXT("Fallback shore side=%u accepted=%d failure=%u area=%.17g length=%.17g"),
				Side,
				Shore.has_value(),
				static_cast<unsigned>(ShoreReport.Failure),
				ShoreReport.AreaMetersSquared,
				ShoreReport.LengthMeters
			)
		);
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.RadiusMeters = 1E6;
		Settings.PreferredCenter = Known[Side].Direction;
		Settings.OutletTarget = Known[1 - Side].Direction;
		Settings.InitialHalfWidthMeters = 256;
		Settings.MaximumHalfWidthMeters = 256;
		Settings.CandidateSpacingMeters = 16;
		Settings.IncludeInternalConnections = true;
		Settings.MaximumClimbingAttempts = 8;
		Settings.MaximumEvaluations = 2000000;
		PlanetCore::SpillDiscoveryDiagnostics Report;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Batch =
		PlanetCore::FindSpillComponentExits(Known, Source, Settings, {}, &Report, Completed);
		AddInfo(
			FString::Printf(
				TEXT("Fallback candidates side=%u accepted=%d failure=%u windows=%u climbs=%u calls=%llu hits=%llu"),
				Side,
				Batch.has_value(),
				static_cast<unsigned>(Report.Failure),
				Report.Windows,
				Report.ClimbingAttempts,
				Calls,
				Hits
			)
		);
		if(Batch) for(const PlanetCore::SpillConnection &Connection : Batch->Connections) {
			AddInfo(
				FString::Printf(
					TEXT("Fallback actual pass side=%u H=%.17g D=(%.17g,%.17g,%.17g) terminals=(%u,%u)"),
					Side,
					Connection.Pass.ElevationMeters,
					Connection.Pass.Direction.Vector().X,
					Connection.Pass.Direction.Vector().Y,
					Connection.Pass.Direction.Vector().Z,
					static_cast<unsigned>(Connection.Branches[0].Kind),
					static_cast<unsigned>(Connection.Branches[1].Kind)
				)
			);
			HasLower = HasLower || Connection.Pass.ElevationMeters < Old->Pass.ElevationMeters - 1E-6;
		}
	}
	TestTrue(TEXT("格子通路とは別に両枝を確認した低い実峠"), HasLower);
	TestTrue(TEXT("全段階共通の元場予算を維持"), Calls < 2000001);
	// 有限な直接探索の結果の返戻
	return !HasAnyErrors();
}

#endif
