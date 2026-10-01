#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillConnection.h"
#include "Core/PlanetSpillPassage.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillPassageContracts,
	"PlanetGenerator.SpillPassage.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 解析解と実際の低い峠で，元場の通路・評価予算・端点の保持を検査する
 * @param Parameters 未使用の実行引数
 * @return 有限な探索範囲での通過高度と拒否条件の検査結果
 */
bool FPlanetSpillPassageContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const auto Field = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, U = X / 40, V = Y / 20, DX = (4 * U * U * U - 4 * U) / 40;
			const double DY = 2 * V / 20;
			const PlanetCore::Vec3 Gradient { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 高さ１の鞍点と高さ０の２極小を持つ解析場の返戻
			return { U * U * U * U - 2 * U * U + V * V + 1, Gradient };
		};
		const PlanetCore::UnitDirection Start = *PlanetCore::UnitDirection::FromVector({ -40 / Radius, 0, 1 });
		const PlanetCore::UnitDirection End = *PlanetCore::UnitDirection::FromVector({ 40 / Radius, 0, 1 });
		PlanetCore::SpillPassageSettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SpacingMeters = 5;
		Settings.HalfWidthMeters = 40;
		const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(Start, End, Field, Settings);
		// 通路を準備出来ない場合の返戻
		if(!TestTrue(TEXT("解析場の最低通過高度を探索"), Passage.has_value())) return false;
		TestTrue(TEXT("解析的な峠の高度１に一致"), std::abs(Passage->CrestMeters - 1) < 1E-8);
		TestTrue(TEXT("標本網の外へ出る経路がより低い解を持たない"), Passage->IsGraphInteriorMinimum);
		TestEqual(TEXT("始点を格子へ丸めない"), (Passage->Path.front().Vector() - Start.Vector()).Length(), 0.0);
		TestEqual(TEXT("終点を格子へ丸めない"), (Passage->Path.back().Vector() - End.Vector()).Length(), 0.0);
		TestTrue(TEXT("独立試料も同じ高度以下"), Passage->MaximumIndependentExcessMeters < 1E-8);
		for(std::size_t i = 1; i < Passage->Path.size(); ++i) {
			const PlanetCore::Vec3 A = Passage->Path[i - 1].Vector(), B = Passage->Path[i].Vector();
			for(unsigned j = 0; j < 7; ++j) {
				const double Fraction = (j + 0.5) / 7;
				const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
				TestTrue(TEXT("別の通路内部標本で上限を保持"), Field(D).ElevationMeters <= 1 + 1E-8);
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("Analytic passage R=%.0f nodes=%u path=%llu crest=%.12g excess=%.12g evaluations=%llu"),
				Radius,
				Passage->Nodes,
				static_cast<unsigned long long>(Passage->Path.size()),
				Passage->CrestMeters,
				Passage->MaximumIndependentExcessMeters,
				static_cast<unsigned long long>(Passage->Evaluations)
			)
		);
		TestFalse(TEXT("同一の端点を拒否"), PlanetCore::FindSpillPassage(Start, Start, Field, Settings).has_value());
		std::stop_source Cancel;
		Cancel.request_stop();
		TestFalse(TEXT("中断時に通路を公開しない"), PlanetCore::FindSpillPassage(Start, End, Field, Settings, Cancel.get_token()).has_value());
		Settings.MaximumEvaluations = 1000;
		std::uint64_t Evaluations = 0;
		const auto Counted = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Evaluations;
			// 実際の呼出数を別に記録した元場の返戻
			return Field(Direction);
		};
		TestFalse(TEXT("評価予算で未完成の通路を拒否"), PlanetCore::FindSpillPassage(Start, End, Counted, Settings).has_value());
		TestEqual(TEXT("外部の場の呼出を予算内に保持"), Evaluations, std::uint64_t{ 1000 });
		Settings.MaximumEvaluations = 4000000;
		Settings.MaximumNodes = 100;
		TestFalse(TEXT("頂点予算を越える格子を拒否"), PlanetCore::FindSpillPassage(Start, End, Field, Settings).has_value());
		Settings.MaximumNodes = 65536;
		const auto Invalid = [](const PlanetCore::UnitDirection) -> PlanetCore::TerrainSample {
			// 元場の欠落を表す検査入力の返戻
			return { std::numeric_limits<double>::quiet_NaN(), {} };
		};
		TestFalse(TEXT("非有限な場を上限計測から捨てない"), PlanetCore::FindSpillPassage(Start, End, Invalid, Settings).has_value());
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
	const PlanetCore::UnitDirection Start =
	*PlanetCore::UnitDirection::FromVector({ 0.54187601419366105, 0.81191502638215118, -0.2171736060769405 });
	const PlanetCore::UnitDirection End =
	*PlanetCore::UnitDirection::FromVector({ 0.54165180349932751, 0.81202943116404547, -0.21730514648604751 });
	PlanetCore::SpillPassageSettings Settings;
	Settings.SpacingMeters = 4;
	Settings.HalfWidthMeters = 512;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
	const double Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count();
	AddInfo(FString::Printf(TEXT("Actual passage preparation milliseconds=%.12g"), Milliseconds));
	// 実地形の有限幅探索が失敗した場合の返戻
	if(!TestTrue(TEXT("実地形から低い峠の通路を発見"), Passage.has_value())) return false;
	TestTrue(TEXT("元の通路測定と一致"), std::abs(Passage->CrestMeters - 846.538494422) < 1E-6);
	TestTrue(TEXT("実経路の独立標本も上限を保持"), Passage->MaximumIndependentExcessMeters < 1E-6);
	TestTrue(TEXT("広い範囲でも標本網の内部で最小"), Passage->IsGraphInteriorMinimum);
	const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(Passage->Peak, Source, {});
	// 候補の最高点を其の儘流出口へ置換えない返戻
	if(!TestTrue(TEXT("発見した候補から元場の鞍点と両側を確認"), Connection.has_value())) return false;
	TestTrue(TEXT("確認した元鞍点の高度"), std::abs(Connection->Pass.ElevationMeters - 846.538491980) < 1E-6);
	AddInfo(
		FString::Printf(
			TEXT("Actual passage nodes=%u path=%llu crest=%.12g outside=%.12g excess=%.12g evaluations=%llu saddle=%.12g"),
			Passage->Nodes,
			static_cast<unsigned long long>(Passage->Path.size()),
			Passage->CrestMeters,
			Passage->OutsideLowerBoundMeters,
			Passage->MaximumIndependentExcessMeters,
			static_cast<unsigned long long>(Passage->Evaluations),
			Connection->Pass.ElevationMeters
		)
	);
	// 有限の探索と元場の接続確認の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillGapBridgeContracts,
	"PlanetGenerator.SpillPassage.GapBridge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 粗い経路で途切れた二つの実極小の間を，全ての局所通過候補で再検査する
 * @param Parameters 未使用の実行引数
 * @return 元の停留点と下降枝だけで両極小の接続を確認した場合に真
 */
bool FPlanetSpillGapBridgeContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Definition;
	Definition.RadiusMeters = 1E6;
	Definition.Seed = 42;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 準備出来ない元地形を別の場へ置換えない返戻
	if(!TestTrue(TEXT("欠落接続の元地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山の欠落への返戻
	if(!TestTrue(TEXT("欠落接続の元造山を準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 実地形を失った場合の返戻
	if(!TestTrue(TEXT("欠落接続の同じ元場を準備"), Ocean.has_value())) return false;
	std::uint64_t Calls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		if(Calls > 1999999) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 診断の有限予算を使切った場合の返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++Calls;
		// 同じ不変な元場の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection Start =
	*PlanetCore::UnitDirection::FromVector({ 0.53807264594770188, 0.81528922314557617, -0.21396567553119897 });
	const PlanetCore::UnitDirection End =
	*PlanetCore::UnitDirection::FromVector({ 0.53274614553869648, 0.81643261992931515, -0.222753948402753 });
	const std::optional<PlanetCore::TerrainCriticalPoint> First = PlanetCore::RefineTerrainCriticalPoint(Start, Source, {});
	const std::optional<PlanetCore::TerrainCriticalPoint> Last = PlanetCore::RefineTerrainCriticalPoint(End, Source, {});
	const bool HasMinima = First && Last && First->Kind == PlanetCore::TerrainCriticalKind::Minimum &&
	Last->Kind == PlanetCore::TerrainCriticalKind::Minimum;
	// 再現入力を独立に確認出来ない場合の返戻
	if(!TestTrue(TEXT("実窪地の両始終点を再確認"), HasMinima)) return false;
	PlanetCore::SpillPassageSettings Settings;
	Settings.SpacingMeters = 25;
	Settings.HalfWidthMeters = 512;
	const std::optional<PlanetCore::SpillPassage> Passage = PlanetCore::FindSpillPassage(Start, End, Source, Settings);
	// 通路の未準備を接続としない返戻
	if(!TestTrue(TEXT("途切れた区間を有界な通路で探索"), Passage.has_value())) return false;
	std::vector<PlanetCore::SpillConnection> Connections;
	for(const PlanetCore::UnitDirection D : Passage->PassCandidates) {
		const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(D, Source, {});
		if(Connection) Connections.push_back(*Connection);
	}
	std::vector<PlanetCore::TerrainCriticalPoint> Known { *First };
	const auto IsKnown = [&](const PlanetCore::TerrainCriticalPoint &P) -> bool {
		const PlanetCore::Vec3 A = P.Direction.Vector();
		for(const PlanetCore::TerrainCriticalPoint &Q : Known) {
			const PlanetCore::Vec3 B = Q.Direction.Vector();
			const bool Matches = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B)) <= 0.001 &&
			std::abs(P.ElevationMeters - Q.ElevationMeters) <= 1E-6;
			// 距離と元標高の独立照合の返戻
			if(Matches) return true;
		}
		// 別の極小を共有点としない返戻
		return false;
	};
	std::vector<bool> Selected(Connections.size(), false);
	const auto ExpandKnown = [&]() -> void {
		bool Changed = true;
		while(Changed) {
			Changed = false;
			for(std::size_t I = 0; I < Connections.size(); ++I) {
				if(Selected[I]) continue;
				const PlanetCore::SpillConnection &Connection = Connections[I];
				const bool HasFirst = Connection.Branches[0].Minimum && IsKnown(*Connection.Branches[0].Minimum);
				const bool HasSecond = Connection.Branches[1].Minimum && IsKnown(*Connection.Branches[1].Minimum);
				if(!HasFirst && !HasSecond) continue;
				for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
					if(Branch.Minimum && !IsKnown(*Branch.Minimum)) Known.push_back(*Branch.Minimum);
				}
				Selected[I] = true;
				Changed = true;
			}
		}
	};
	ExpandKnown();
	for(unsigned Step = 0; Step < 16 && !IsKnown(*Last); ++Step) {
		// 分離した実極小の間だけを再検査し，長い通路全体を一様に細分化しない
		std::vector<PlanetCore::TerrainCriticalPoint> Others { *Last };
		for(const PlanetCore::SpillConnection &Connection : Connections) {
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				if(Branch.Minimum && !IsKnown(*Branch.Minimum)) Others.push_back(*Branch.Minimum);
			}
		}
		double Nearest = std::numeric_limits<double>::infinity();
		PlanetCore::UnitDirection From = Known.front().Direction, To = Last->Direction;
		for(const PlanetCore::TerrainCriticalPoint &A : Known) for(const PlanetCore::TerrainCriticalPoint &B : Others) {
			const PlanetCore::Vec3 P = A.Direction.Vector(), Q = B.Direction.Vector();
			const double Distance = Definition.RadiusMeters * std::atan2(P.Cross(Q).Length(), P.Dot(Q));
			if(Distance < Nearest) {
				Nearest = Distance;
				From = A.Direction;
				To = B.Direction;
			}
		}
		PlanetCore::SpillPassageSettings LocalSettings;
		LocalSettings.SpacingMeters = std::min(25.0, Nearest / 64);
		LocalSettings.HalfWidthMeters = std::min(512.0, std::max(16.0, Nearest * 0.5));
		const std::optional<PlanetCore::SpillPassage> Local = PlanetCore::FindSpillPassage(From, To, Source, LocalSettings);
		AddInfo(
			FString::Printf(
				TEXT("Gap refinement step=%u distance=%.12g spacing=%.12g accepted=%u evaluations=%llu"),
				Step,
				Nearest,
				LocalSettings.SpacingMeters,
				static_cast<unsigned>(Local.has_value()),
				static_cast<unsigned long long>(Calls)
			)
		);
		if(!Local) break;
		const std::size_t Before = Connections.size();
		for(const PlanetCore::UnitDirection D : Local->PassCandidates) {
			const std::optional<PlanetCore::SpillConnection> Connection = PlanetCore::PrepareSpillConnection(D, Source, {});
			if(!Connection) continue;
			const PlanetCore::Vec3 P = Connection->Pass.Direction.Vector();
			const bool Duplicate = std::any_of(
				Connections.begin(),
				Connections.end(),
				[&](const PlanetCore::SpillConnection &Other) -> bool {
					const PlanetCore::Vec3 Q = Other.Pass.Direction.Vector();
					// 同じ峠の再発見を探索の進捗と数えない返戻
					return Definition.RadiusMeters * std::atan2(P.Cross(Q).Length(), P.Dot(Q)) <= 0.001 &&
					std::abs(Connection->Pass.ElevationMeters - Other.Pass.ElevationMeters) <= 1E-6;
				}
			);
			if(!Duplicate) Connections.push_back(*Connection);
		}
		Selected.resize(Connections.size(), false);
		ExpandKnown();
		if(Connections.size() == Before) break;
	}
	AddInfo(
		FString::Printf(
			TEXT("Gap bridge candidates=%llu resolved=%llu connectedMinima=%llu reachesTarget=%u ")
			TEXT("evaluations=%llu crest=%.12g independentExcess=%.12g"),
			static_cast<unsigned long long>(Passage->PassCandidates.size()),
			static_cast<unsigned long long>(Connections.size()),
			static_cast<unsigned long long>(Known.size()),
			static_cast<unsigned>(IsKnown(*Last)),
			static_cast<unsigned long long>(Calls),
			Passage->CrestMeters,
			Passage->MaximumIndependentExcessMeters
		)
	);
	for(const PlanetCore::SpillConnection &Connection : Connections) {
		AddInfo(FString::Printf(TEXT("Gap bridge pass H=%.12g"), Connection.Pass.ElevationMeters));
		for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
			if(!Branch.Minimum) continue;
			const PlanetCore::Vec3 D = Branch.Minimum->Direction.Vector();
			AddInfo(
				FString::Printf(
					TEXT("Gap bridge terminal H=%.12g known=%u direction=(%.17g,%.17g,%.17g)"),
					Branch.Minimum->ElevationMeters,
					static_cast<unsigned>(IsKnown(*Branch.Minimum)),
					D.X,
					D.Y,
					D.Z
				)
			);
		}
	}
	TestTrue(TEXT("最高点以外の実接続も保持して対象の極小へ到達"), IsKnown(*Last));
	TestTrue(TEXT("高密度通路でも独立標本の高度を維持"), Passage->MaximumIndependentExcessMeters < 1E-6);
	// 海面又は大洋の全域接続と区別した局所検査の返戻
	return !HasAnyErrors();
}

#endif
