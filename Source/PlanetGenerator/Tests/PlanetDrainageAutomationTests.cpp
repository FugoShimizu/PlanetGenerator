#include "Core/PlanetDrainage.h"
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetDrainageContracts,"PlanetGenerator.Drainage.Contracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 排水木・下降水位・解析海岸・曲線上の河口微分を独立に検査する
 * @param Parameters 未使用の Automation 引数
 * @return 全ての排水条件が合格した場合に真
 */
bool FPlanetDrainageContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	const double Start = FPlatformTime::Seconds();
	const PlanetCore::UnitDirection ShoreStart = *PlanetCore::UnitDirection::FromVector({ 1, 0, 0 });
	const PlanetCore::UnitDirection ShoreEnd = *PlanetCore::UnitDirection::FromVector({ std::cos(0.02), std::sin(0.02), 0 });
	constexpr double KnownAngle = 0.01234567, Sea = 87;
	const auto LinearShore = [](PlanetCore::UnitDirection D) {
		const PlanetCore::Vec3 V = D.Vector();
		// 既知の海岸角度に対する解析標高の返戻
		return Sea + 1000 * (KnownAngle - std::atan2(V.Y, V.X));
	};
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		const std::optional<PlanetCore::ShoreCrossing> Shore =
		PlanetCore::FindShoreCrossing(ShoreStart, ShoreEnd, Radius, Sea, LinearShore);
		const bool Accepted1 = TestTrue(TEXT("Analytical coast found across radii"), Shore.has_value());
		// 検査の不合格を保持する返戻
		if(!Accepted1) return false;
		const double KnownFraction = KnownAngle / 0.02;
		const bool Accepted2 = TestTrue(
			TEXT("Independent exact crossing lies inside reported bracket"),
			Shore->LandFraction <= KnownFraction && Shore->WetFraction >= KnownFraction &&
			Shore->BracketWidthMeters <= std::max(1E-5, Radius * 1E-12) &&
			std::abs(std::atan2(Shore->Direction.Y, Shore->Direction.X) - KnownAngle) * Radius <= std::max(1E-5, Radius * 1E-12)
		);
		// 検査の不合格を保持する返戻
		if(!Accepted2) return false;
	}
	const auto MultipleShore = [](PlanetCore::UnitDirection D) {
		const PlanetCore::Vec3 V = D.Vector();
		const double A = std::atan2(V.Y, V.X);
		// 三つの交差を持つ解析海岸の返戻
		return -1000 * (A - 0.003) * (A - 0.007) * (A - 0.012);
	};
	const std::optional<PlanetCore::ShoreCrossing> Multiple =
	PlanetCore::FindShoreCrossing(ShoreStart, ShoreEnd, 1E6, 0, MultipleShore);
	TestTrue(
		TEXT("Scan resolves first of separated coastline crossings"),
		Multiple && std::abs(std::atan2(Multiple->Direction.Y, Multiple->Direction.X) - 0.003) < 1E-10
	);
	TestFalse(
		TEXT("All-land edge is not a river mouth"),
		PlanetCore::FindShoreCrossing(
			ShoreStart,
			ShoreEnd,
			1E6,
			0,
			[](PlanetCore::UnitDirection) {
				return 1.0;
			}
		).has_value()
	);
	TestFalse(
		TEXT("Reversed land-sea bracket rejected"),
		PlanetCore::FindShoreCrossing(ShoreEnd, ShoreStart, 1E6, Sea, LinearShore).has_value()
	);
	TestFalse(
		TEXT("Nonfinite shoreline rejected"),
		PlanetCore::FindShoreCrossing(
			ShoreStart,
			ShoreEnd,
			1E6,
			0,
			[](PlanetCore::UnitDirection) {
				return std::numeric_limits<double>::quiet_NaN();
			}
		).has_value()
	);
	PlanetCore::DrainageSettings S;
	S.RadiusMeters = 1000;
	std::vector<PlanetCore::DrainageSite> Sites;
	const std::array<double, 6> Heights { -1, 8, 1, 3, 2, 9 };
	for(unsigned I = 0; I < 6; ++I) Sites.push_back({ { std::cos(0.01 * I), std::sin(0.01 * I), 0 }, Heights[I], 1E6, !I });
	std::vector<std::array<std::uint32_t, 2>> Edges { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 1 }, { 0, 5 }, { 5, 3 } };
	const std::optional<PlanetCore::DrainageNetwork> A = PlanetCore::DrainageNetwork::Solve(Sites, Edges, S);
	const bool Accepted3 = TestTrue(TEXT("Pit and branching graph solved"), A.has_value());
	// 検査の不合格を保持する返戻
	if(!Accepted3) return false;
	// 全ての単純経路を独立に列挙し，大洋から上流へ水位上限を算出する
	for(std::uint32_t StartNode = 1; StartNode < 6; ++StartNode) {
		double Best = std::numeric_limits<double>::infinity();
		std::vector<std::uint32_t> Path { StartNode };
		const auto Enumerate = [&](auto &&Self, std::uint32_t At) -> void {
			if(!At) {
				double Water = 0;
				for(std::size_t K = Path.size() - 1; K > 0; --K) {
					const std::uint32_t U = Path[K - 1], V = Path[K];
					const double L =
					std::atan2(Sites[U].Direction.Cross(Sites[V].Direction).Length(), Sites[U].Direction.Dot(Sites[V].Direction)) * S.RadiusMeters;
					Water = std::max(
						Sites[U].ElevationMeters - S.BankFreeboardMeters,
						std::nextafter(Water + S.MinimumSlope * L, std::numeric_limits<double>::infinity())
					);
				}
				Best = std::min(Best, Water);
				// 大洋へ達した経路の列挙を終了する
				return;
			}
			for(const std::array<std::uint32_t, 2> E : Edges) {
				const std::uint32_t Next = E[0] == At ? E[1] : E[1] == At ? E[0] : 99U;
				if(Next == 99 || std::find(Path.begin(), Path.end(), Next) != Path.end()) continue;
				Path.push_back(Next);
				Self(Self, Next);
				Path.pop_back();
			}
			// 全ての隣接辺を列挙した終了
			return;
		};
		Enumerate(Enumerate, StartNode);
		const bool Accepted4 =
		TestTrue(TEXT("Independent exhaustive minimum profile agrees"), std::abs(A->Nodes()[StartNode].WaterMeters - Best) < 1E-12);
		// 検査の不合格を保持する返戻
		if(!Accepted4) return false;
	}
	std::reverse(Edges.begin(), Edges.end());
	Edges.push_back(Edges[0]);
	const std::optional<PlanetCore::DrainageNetwork> B = PlanetCore::DrainageNetwork::Solve(Sites, Edges, S);
	const bool Accepted5 = TestTrue(TEXT("Reordered duplicate edges accepted"), B.has_value());
	// 検査の不合格を保持する返戻
	if(!Accepted5) return false;
	for(unsigned I = 0; I < 6; ++I) {
		const bool Accepted6 = TestTrue(
			TEXT("Input edge order does not change water or receiver"),
			A->Nodes()[I].WaterMeters == B->Nodes()[I].WaterMeters && A->Nodes()[I].Downstream == B->Nodes()[I].Downstream
		);
		// 検査の不合格を保持する返戻
		if(!Accepted6) return false;
	}
	TestTrue(
		TEXT("All land area arrives at ocean"),
		A->Statistics().LandAreaSquareMeters == 5E6 && A->Statistics().OutletAreaSquareMeters == 5E6
	);
	TestTrue(
		TEXT("Depression conditioning is exposed, not silently rendered"),
		A->Statistics().MaximumRequiredBankRaiseMeters > 6.9
	);
	PlanetCore::DrainageSettings BreachSettings = S;
	BreachSettings.PreferBreaching = true;
	const std::optional<PlanetCore::DrainageNetwork> Breached = PlanetCore::DrainageNetwork::Solve(Sites, Edges, BreachSettings);
	const bool Accepted7 = TestTrue(TEXT("Breach-side profile solved"), Breached.has_value());
	// 検査の不合格を保持する返戻
	if(!Accepted7) return false;
	for(unsigned I = 1; I < 6; ++I) {
		const PlanetCore::DrainageNode &Node = Breached->Nodes()[I];
		const bool Accepted8 = TestTrue(
			TEXT("Breaching preserves outlets and avoids raising this pit"),
			Node.Downstream == A->Nodes()[I].Downstream && !Node.Outlet && Node.WaterMeters <= Heights[I] - S.BankFreeboardMeters + 1E-10 &&
			Node.WaterMeters > Breached->Nodes()[Node.Downstream].WaterMeters
		);
		// 検査の不合格を保持する返戻
		if(!Accepted8) return false;
	}
	TestTrue(
		TEXT("Barrier cut is exposed separately from bank fill"),
		Breached->Statistics().MaximumRequiredBankRaiseMeters < 1E-10 && Breached->Statistics().MaximumRequiredWaterCutMeters > 6.9 &&
		Breached->Nodes()[1].WaterMeters < 1
	);
	std::vector<PlanetCore::DrainageSite> Invalid = Sites;
	Invalid[0].Ocean = false;
	TestFalse(TEXT("No ocean seed rejected"), PlanetCore::DrainageNetwork::Solve(Invalid, Edges, S).has_value());
	Invalid = Sites;
	Invalid[3].Ocean = true;
	Invalid[3].ElevationMeters = -1;
	TestFalse(
		TEXT("Disconnected wet pit is not an ocean outlet"),
		PlanetCore::DrainageNetwork::Solve(Invalid, Edges, S).has_value()
	);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("Cancelled shoreline rejected"),
		PlanetCore::FindShoreCrossing(ShoreStart, ShoreEnd, 1E6, Sea, LinearShore, Cancel.get_token()).has_value()
	);
	TestFalse(
		TEXT("Cancelled solver rejected"),
		PlanetCore::DrainageNetwork::Solve(Sites, Edges, S, Cancel.get_token()).has_value()
	);
	TestFalse(TEXT("Invalid profile index rejected"), A->SampleReach(A->Reaches().size(), 0.5).has_value());
	TestFalse(TEXT("Out of range profile parameter rejected"), A->SampleReach(0, 1.1).has_value());
	std::vector<PlanetCore::DrainageSite> Chain;
	for(unsigned I = 0; I < 4; ++I) {
		Chain.push_back({ { std::cos(0.01 * I), std::sin(0.01 * I), 0 }, !I ? -1.0 : 7.0 + I, 1E6, !I });
	}
	const std::array<std::array<std::uint32_t, 2>, 3> ChainEdges { { { 0, 1 }, { 1, 2 }, { 2, 3 } } };
	const std::optional<PlanetCore::DrainageNetwork> Coarse = PlanetCore::DrainageNetwork::Solve(Chain, ChainEdges, BreachSettings);
	const bool Accepted9 = TestTrue(TEXT("Synthetic valley drainage solved"), Coarse.has_value());
	// 検査の不合格を保持する返戻
	if(!Accepted9) return false;
	const auto InteriorValley = [](PlanetCore::UnitDirection D) {
		const PlanetCore::Vec3 V = D.Vector();
		const double A = std::atan2(V.Y, V.X), Q = (A - 0.015) / 0.0015;
		// 河道途中の窪地を持つ解析標高の返戻
		return 7 + 100 * A - 8 * std::exp(-Q * Q);
	};
	const std::optional<PlanetCore::DrainageNetwork> Refined = Coarse->WithTerrainProfiles(InteriorValley);
	const bool Accepted10 = TestTrue(TEXT("Interior terrain constraints prepared"), Refined.has_value());
	// 検査の不合格を保持する返戻
	if(!Accepted10) return false;
	TestFalse(
		TEXT("Cancelled longitudinal conditioning rejected"),
		Coarse->WithTerrainProfiles(InteriorValley, Cancel.get_token()).has_value()
	);
	TestFalse(
		TEXT("Nonfinite longitudinal terrain rejected"),
		Coarse->WithTerrainProfiles(
			[](PlanetCore::UnitDirection) {
				return std::numeric_limits<double>::quiet_NaN();
			}
		).has_value()
	);
	bool ValleyChecked = false;
	for(std::size_t I = 0; I < Refined->Reaches().size(); ++I) {
		if(Refined->Reaches()[I].Upstream != 2) continue;
		const std::optional<PlanetCore::RiverProfileSample> Before = Coarse->SampleReach(I, 0.5), After = Refined->SampleReach(I, 0.5);
		const double H = InteriorValley(*PlanetCore::UnitDirection::FromVector(After->Direction));
		const bool Accepted11 = TestTrue(
			TEXT("Known intermediate hollow no longer lies metres below the intended riverbed"),
			Before->BedMeters - H > 5 && After->BedMeters <= H && Before->WaterMeters - After->WaterMeters > 5
		);
		// 検査の不合格を保持する返戻
		if(!Accepted11) return false;
		ValleyChecked = true;
	}
	TestTrue(TEXT("Interior hollow regression was exercised"), ValleyChecked);
	const auto ValidatePolynomials = [&](const PlanetCore::DrainageNetwork &Network) -> bool {
		for(const PlanetCore::RiverLongitudinalProfile &P : Network.Profiles()) {
			for(unsigned K = 0; K < PlanetCore::RiverLongitudinalProfile::Segments; ++K) {
				const double M = P.Derivative[K] / P.Segments, N = P.Derivative[K + 1] / P.Segments, Delta = P.Headroom[K + 1] - P.Headroom[K];
				const double C = 3 * Delta - 2 * M - N, D = M + N - 2 * Delta;
				// 微分二次式の内部極値を含めて，区間全体の非増加を検査する
				double Max = std::max(M, N);
				if(D) {
					const double U = -C / (3 * D);
					if(U > 0 && U < 1) Max = std::max(Max, M + U * (2 * C + 3 * D * U));
				}
				const bool Accepted12 = TestTrue(
					TEXT("Complete Hermite interval has nonincreasing headroom and strict grade"),
					Delta <= 0 && Max <= 1E-12 && P.GradeStart > P.GradeEnd
				);
				// 検査の不合格を保持する返戻
				if(!Accepted12) return false;
				const double T = (K + 0.37) / P.Segments;
				constexpr double E = 1E-7;
				const PlanetCore::RiverHeights H = P.Evaluate(T), Lo = P.Evaluate(T - E), Hi = P.Evaluate(T + E);
				const double FD = (Hi.WaterMeters - Lo.WaterMeters) / (2 * E * P.LengthMeters);
				const bool Accepted13 = TestTrue(
					TEXT("Cubic water derivative matches independent finite difference"),
					H.WaterSlope < 0 && std::abs(FD - H.WaterSlope) < std::max(1E-6, std::abs(H.WaterSlope) * 1E-6)
				);
				// 検査の不合格を保持する返戻
				if(!Accepted13) return false;
			}
		}
		// 全区間の解析的な非増加を確認した返戻
		return true;
	};
	// 解析縦断の不合格を保持する返戻
	if(!ValidatePolynomials(*Refined)) return false;
	std::uint64_t CheckedNodes = 0, ProfileSamples = 0;
	for(
		const std::pair<std::uint64_t, double> Definition :
		{ std::pair<std::uint64_t, double>{ 0, 1000 }, { 42, 1000000 }, { 7, 6371000 }, { 42, 1E9 } }
	) {
		PlanetCore::GeologySettings Settings;
		Settings.Seed = Definition.first;
		Settings.RadiusMeters = Definition.second;
		const std::optional<PlanetCore::GeologyField> Geo = PlanetCore::GeologyField::Create(Settings);
		const bool Accepted14 = TestTrue(TEXT("Actual geology prepared"), Geo.has_value());
		// 検査の不合格を保持する返戻
		if(!Accepted14) return false;
		const std::optional<PlanetCore::OceanConnectivityField> Ocean =
		PlanetCore::OceanConnectivityField::Create(std::make_shared<const PlanetCore::GeologyField>(*Geo));
		const bool Accepted15 = TestTrue(TEXT("Actual connected ocean prepared"), Ocean.has_value());
		// 検査の不合格を保持する返戻
		if(!Accepted15) return false;
		const std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::Create(*Ocean);
		const bool Accepted16 = TestTrue(TEXT("Actual drainage network prepared"), Network.has_value());
		// 検査の不合格を保持する返戻
		if(!Accepted16) return false;
		// 実地形の縦断区間に逆勾配が有る場合の返戻
		if(!ValidatePolynomials(*Network)) return false;
		const std::span<const PlanetCore::DrainageNode> Nodes = Network->Nodes();
		for(std::uint32_t I = 0; I < Nodes.size(); ++I) {
			std::uint32_t Current = I;
			for(std::size_t Steps = 0; !Nodes[Current].Ocean; ++Steps) {
				const bool Accepted17 = TestTrue(
					TEXT("Every land path reaches ocean without cycles"),
					Steps < Nodes.size() && Nodes[Current].Downstream < Nodes.size()
				);
				// 検査の不合格を保持する返戻
				if(!Accepted17) return false;
				const std::uint32_t Down = Nodes[Current].Downstream;
				const bool Accepted18 = TestTrue(
					TEXT("Water and bed descend; accumulation grows"),
					Nodes[Current].WaterMeters > Nodes[Down].WaterMeters &&
					Nodes[Current].WaterMeters - Nodes[Current].DepthMeters > Nodes[Down].WaterMeters - Nodes[Down].DepthMeters &&
					Nodes[Current].ContributingAreaSquareMeters <= Nodes[Down].ContributingAreaSquareMeters
				);
				// 検査の不合格を保持する返戻
				if(!Accepted18) return false;
				Current = Down;
			}
			const bool Accepted19 = TestTrue(TEXT("Stored outlet matches independent path"), Current == Nodes[I].Outlet);
			// 検査の不合格を保持する返戻
			if(!Accepted19) return false;
			++CheckedNodes;
		}
		std::vector<bool> Selected(Nodes.size(), false);
		for(const PlanetCore::RiverReach &Reach : Network->Reaches()) Selected[Reach.Upstream] = true;
		for(std::size_t I = 0; I < Network->Reaches().size(); ++I) {
			const PlanetCore::RiverReach &Reach = Network->Reaches()[I];
			const bool Curved = TestTrue(TEXT("Production river uses the shared curved planform"), Reach.Planform.has_value());
			// 本番河道が直線へ置換えられた場合の返戻
			if(!Curved) return false;
			const bool Accepted20 =
			TestTrue(TEXT("Selected channel never terminates on land"), Nodes[Reach.Downstream].Ocean || Selected[Reach.Downstream]);
			// 検査の不合格を保持する返戻
			if(!Accepted20) return false;
			double Previous = std::numeric_limits<double>::infinity();
			for(unsigned K = 0; K < 17; ++K) {
				const std::optional<PlanetCore::RiverProfileSample> Sample = Network->SampleReach(I, K / 16.0);
				const bool Accepted21 = TestTrue(
					TEXT("Continuous profile descends and covers bed"),
					Sample && Sample->WaterMeters < Previous && Sample->BedMeters < Sample->WaterMeters && Sample->DownstreamWaterSlope < 0 &&
					std::abs(Sample->Direction.Length() - 1) < 1E-12 && std::abs(Sample->Direction.Dot(Sample->DownstreamTangent)) < 1E-12
				);
				// 検査の不合格を保持する返戻
				if(!Accepted21) return false;
				Previous = Sample->WaterMeters;
				++ProfileSamples;
			}
			const std::optional<PlanetCore::RiverProfileSample> Begin = Network->SampleReach(I, 0), End = Network->SampleReach(I, 1);
			const bool Accepted22 = TestTrue(
				TEXT("Junction values shared exactly"),
				Begin->WaterMeters == Nodes[Reach.Upstream].WaterMeters && End->WaterMeters == Nodes[Reach.Downstream].WaterMeters &&
				End->BedMeters == Nodes[Reach.Downstream].WaterMeters - Nodes[Reach.Downstream].DepthMeters
			);
			// 検査の不合格を保持する返戻
			if(!Accepted22) return false;
		}
		std::size_t ExpectedMouths = 0;
		for(const PlanetCore::RiverReach &Reach : Network->Reaches()) if(Nodes[Reach.Downstream].Ocean) ++ExpectedMouths;
		const bool Accepted23 = TestTrue(
			TEXT("Every ocean-bound selected reach has a coastal mouth"),
			ExpectedMouths > 0 && Network->Mouths().size() == ExpectedMouths
		);
		// 検査の不合格を保持する返戻
		if(!Accepted23) return false;
		for(const PlanetCore::RiverMouth &Mouth : Network->Mouths()) {
			const PlanetCore::RiverReach &Reach = Network->Reaches()[Mouth.Reach];
			const std::optional<PlanetCore::RiverProfileSample> End = Network->SampleReach(Mouth.Reach, 1);
			const PlanetCore::UnitDirection D = *PlanetCore::UnitDirection::FromVector(End->Direction);
			const double H = Ocean->Evaluate(D).ElevationMeters, Level = Settings.SeaLevelMeters;
			const PlanetCore::DrainageNode &From = Nodes[Reach.Upstream];
			const PlanetCore::RiverProfileSample Begin = *Network->SampleReach(Mouth.Reach, 0);
			const double EndFraction = Reach.EndFraction;
			const double NominalLength =
			std::atan2(End->Direction.Dot(Begin.DownstreamTangent), End->Direction.Dot(From.Direction)) * Settings.RadiusMeters;
			const bool Accepted24 = TestTrue(
				TEXT("Mouth lies on analytical terrain shore and constant spherical sea"),
				Nodes[Reach.Downstream].Ocean && End->WaterMeters == Level && End->BedMeters < Level && EndFraction > 0 && EndFraction <= 1 &&
				std::abs(H - Level) < 0.001 && End->Direction.X == Mouth.Shore.Direction.X && End->Direction.Y == Mouth.Shore.Direction.Y &&
				End->Direction.Z == Mouth.Shore.Direction.Z &&
				std::abs(NominalLength - Reach.ProfileLengthMeters()) < std::max(1E-6, Settings.RadiusMeters * 1E-14)
			);
			// 検査の不合格を保持する返戻
			if(!Accepted24) return false;
			constexpr double Epsilon = 1E-6;
			const std::optional<PlanetCore::RiverProfileSample> Near = Network->SampleReach(Mouth.Reach, 1 - Epsilon);
			const std::optional<PlanetCore::RiverProfileSample> Nearer = Network->SampleReach(Mouth.Reach, 1 - Epsilon / 2);
			const double CoarseDistance =
			std::atan2(Near->Direction.Cross(End->Direction).Length(), Near->Direction.Dot(End->Direction)) * Settings.RadiusMeters;
			const double FineDistance =
			std::atan2(Nearer->Direction.Cross(End->Direction).Length(), Nearer->Direction.Dot(End->Direction)) * Settings.RadiusMeters;
			const double CoarseSlope = (Level - Near->WaterMeters) / CoarseDistance;
			const double FineSlope = (Level - Nearer->WaterMeters) / FineDistance;
			const bool Accepted25 = TestTrue(
				TEXT("Cubic river approaches sea with its endpoint derivative"),
				Near && Nearer && Near->WaterMeters > Nearer->WaterMeters && Nearer->WaterMeters > Level &&
				std::abs(2 * FineSlope - CoarseSlope - End->DownstreamWaterSlope) < 1E-9
			);
			// 検査の不合格を保持する返戻
			if(!Accepted25) return false;
		}
		const PlanetCore::DrainageStatistics &Stats = Network->Statistics();
		double OffKnotGap = 0;
		for(std::size_t I = 0; I < Network->Reaches().size(); ++I) for(unsigned K = 0; K < 128; ++K) {
			const std::optional<PlanetCore::RiverProfileSample> P = Network->SampleReach(I, (K + 0.5) / 128.0);
			OffKnotGap =
			std::max(OffKnotGap, P->BedMeters - Ocean->Evaluate(*PlanetCore::UnitDirection::FromVector(P->Direction)).ElevationMeters);
		}
		AddInfo(
			FString::Printf(
				TEXT("Longitudinal seed=%llu R=%.0f knots=%u max_knot_gap=%.9g m max_off_knot_gap=%.9g m max_water_lowering=%.9g m"),
				static_cast<unsigned long long>(Definition.first),
				Definition.second,
				Stats.ProfileKnots,
				Stats.MaximumUnresolvedKnotBedGapMeters,
				OffKnotGap,
				Stats.MaximumProfileWaterLoweringMeters
			)
		);
		const bool Accepted26 = TestTrue(
			TEXT("Integrated catchment area conserved"),
			std::abs(Stats.OutletAreaSquareMeters - Stats.LandAreaSquareMeters) <= Stats.LandAreaSquareMeters * 1E-12
		);
		// 検査の不合格を保持する返戻
		if(!Accepted26) return false;
		AddInfo(
			FString::Printf(
				TEXT("Mouths seed=%llu R=%.0f count=%llu max_bracket=%.9g m max_height_residual=%.9g m"),
				static_cast<unsigned long long>(Definition.first),
				Definition.second,
				static_cast<unsigned long long>(ExpectedMouths),
				Stats.MaximumMouthBracketWidthMeters,
				Stats.MaximumMouthHeightResidualMeters
			)
		);
		AddInfo(
			FString::Printf(
				TEXT("Drainage seed=%llu R=%.0f nodes=%u land=%u reaches=%u max_bank_raise=%.6g m ")
				TEXT("mean_raise=%.6g m max_water_cut=%.6g m longest_route=%.6g m"),
				static_cast<unsigned long long>(Definition.first),
				Definition.second,
				Stats.Nodes,
				Stats.LandNodes,
				Stats.Reaches,
				Stats.MaximumRequiredBankRaiseMeters,
				Stats.AreaWeightedBankRaiseMeters,
				Stats.MaximumRequiredWaterCutMeters,
				Stats.LongestRouteMeters
			)
		);
	}
	AddInfo(
		FString::Printf(
			TEXT("Drainage contracts: %llu nodes, %llu continuous profile samples, 4 actual planets, ")
			TEXT("%.3f s. Channel geometry and final terrain conditioning are not validated."),
			static_cast<unsigned long long>(CheckedNodes),
			static_cast<unsigned long long>(ProfileSamples),
			FPlatformTime::Seconds() - Start
		)
	);
	// 排水経路・水位・河口が全条件を満たした返戻
	return true;
}

#endif
