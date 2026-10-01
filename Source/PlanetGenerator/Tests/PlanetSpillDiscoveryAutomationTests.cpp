#include "Core/PlanetContinuousSpillGraph.h"
#include "Core/PlanetGeology.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetSpillDiscovery.h"
#include "Core/PlanetSpillLake.h"
#include "Core/PlanetSpillNetwork.h"
#include "Core/PlanetSpillPassage.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetSpillDiscoveryContracts,
	"PlanetGenerator.ContinuousSpill.Discovery",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 未指定の峠を元場から発見し，解析解と実地形の下降終端で検証する
 * @param Parameters 未使用の実行引数
 * @return 探索予算と正規の連続接続を確認した場合に真
 */
bool FPlanetSpillDiscoveryContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	for(const double Radius : { 1000.0, 1E6, 1E9 }) {
		std::uint64_t Calls = 0;
		const auto Source = [&](const PlanetCore::UnitDirection Direction) -> PlanetCore::TerrainSample {
			++Calls;
			const PlanetCore::Vec3 D = Direction.Vector();
			const double X = Radius * D.X / D.Z, Y = Radius * D.Y / D.Z, DX = -0.1 * std::sin(X / 40) + 0.02, DY = Y / 200;
			const PlanetCore::Vec3 G { Radius * DX / D.Z, Radius * DY / D.Z, -Radius * (DX * D.X + DY * D.Y) / (D.Z * D.Z) };
			// 峠と極小の座標が既知の傾いた周期場の返戻
			return { 4 * std::cos(X / 40) + 0.02 * X + 2 + Y * Y / 400, G };
		};
		const double X = 40 * (5 * std::acos(-1.0) - std::asin(0.2));
		PlanetCore::TerrainCriticalSettings CriticalSettings;
		CriticalSettings.RadiusMeters = Radius;
		CriticalSettings.MaximumDisplacementMeters = std::min(CriticalSettings.MaximumDisplacementMeters, Radius * 0.125);
		const std::optional<PlanetCore::TerrainCriticalPoint> Initial =
		PlanetCore::RefineTerrainCriticalPoint(*PlanetCore::UnitDirection::FromVector({ X / Radius, 0, 1 }), Source, CriticalSettings);
		// 解析的な初期極小が無い場合の返戻
		if(!TestTrue(TEXT("解析場の初期極小を確認"), Initial.has_value())) return false;
		std::vector<PlanetCore::TerrainCriticalPoint> Minima { *Initial };
		std::vector<PlanetCore::SpillConnection> Connections;
		PlanetCore::SpillDiscoverySettings Settings;
		Settings.RadiusMeters = Radius;
		Settings.SeaLevelMeters = -1;
		Settings.InitialHalfWidthMeters = 32;
		Settings.MaximumHalfWidthMeters = 400;
		// 一つの広い窓から得た連続する峠を，一件ずつ再準備しない検査
		PlanetCore::SpillDiscoverySettings BatchSettings = Settings;
		BatchSettings.InitialHalfWidthMeters = 400;
		const std::optional<PlanetCore::SpillDiscoveryBatch> InitialBatch =
		PlanetCore::FindSpillComponentExits(Minima, Source, BatchSettings);
		// 接続群を準備出来ない場合の返戻
		if(!TestTrue(TEXT("一窓の接続群を準備"), InitialBatch.has_value())) return false;
		TestTrue(TEXT("同じ窓の複数の新極小を保持"), InitialBatch->NewMinima.size() > 1);
		TestTrue(TEXT("複数の実接続を一括して保持"), InitialBatch->Connections.size() > 1);
		BatchSettings.MaximumBranchSamples = 1;
		PlanetCore::SpillDiscoveryDiagnostics LimitedBatch;
		TestFalse(
			TEXT("下降経路の保持予算で部分的な接続群を返さない"),
			PlanetCore::FindSpillComponentExits(Minima, Source, BatchSettings, {}, &LimitedBatch).has_value()
		);
		TestTrue(TEXT("記憶予算の不足を通知"), LimitedBatch.Failure == PlanetCore::SpillDiscoveryFailure::BudgetLimit);
		for(unsigned Step = 0; Step < 3; ++Step) {
			PlanetCore::SpillDiscoveryDiagnostics Report;
			const std::optional<PlanetCore::SpillConnection> Connection =
			PlanetCore::FindSpillComponentExit(Minima, Source, Settings, {}, &Report);
			AddInfo(
				FString::Printf(
					TEXT("Analytic discovery R=%.0f step=%u accepted=%u failure=%u windows=%u half=%.9g evaluations=%llu"),
					Radius,
					Step,
					static_cast<unsigned>(Connection.has_value()),
					static_cast<unsigned>(Report.Failure),
					Report.Windows,
					Report.LastHalfWidthMeters,
					static_cast<unsigned long long>(Report.Evaluations)
				)
			);
			// 部分的な発見を連続した水系としない返戻
			if(!TestTrue(TEXT("峠の位置を指定せず次の接続を発見"), Connection.has_value())) return false;
			const double ExpectedPassX = 40 * std::asin(0.2) + (2 - Step) * 80 * std::acos(-1.0);
			const PlanetCore::Vec3 Pass = Connection->Pass.Direction.Vector();
			TestTrue(TEXT("正しい順で解析解の峠を発見"), std::abs(Radius * Pass.X / Pass.Z - ExpectedPassX) < 0.001);
			Connections.push_back(*Connection);
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection->Branches) {
				if(!Branch.Minimum) continue;
				const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector();
				const bool Known = std::any_of(
					Minima.begin(),
					Minima.end(),
					[&](const PlanetCore::TerrainCriticalPoint &P) -> bool {
						const PlanetCore::Vec3 B = P.Direction.Vector();
						// 独立な距離照合の返戻
						return Radius * std::atan2(A.Cross(B).Length(), A.Dot(B)) <= 0.001;
					}
				);
				if(!Known) Minima.push_back(*Branch.Minimum);
			}
		}
		PlanetCore::ContinuousSpillSettings GraphSettings;
		GraphSettings.RadiusMeters = Radius;
		GraphSettings.SeaLevelMeters = -1;
		const auto IsOcean = [&](const PlanetCore::UnitDirection D) -> bool {
			// 此の解析場で指定した下流の水域への所属の返戻
			return D.Vector().Z > 0 && D.Vector().X < 0 && !(Source(D).ElevationMeters > -1);
		};
		const std::optional<PlanetCore::ContinuousSpillGraph> Graph =
		PlanetCore::BuildContinuousSpillGraph(Connections, IsOcean, GraphSettings);
		TestTrue(TEXT("発見した実接続だけで一水域へ達する階層を構成"), Graph.has_value());
		// 有限の共通予算と開始前の中断
		Settings.MaximumEvaluations = 100;
		Calls = 0;
		PlanetCore::SpillDiscoveryDiagnostics Report;
		TestFalse(TEXT("準備未完の共通評価予算を拒否"), PlanetCore::FindSpillComponentExit(Minima, Source, Settings, {}, &Report).has_value());
		TestEqual(TEXT("全窓共通で元場の評価は予算以内"), Calls, UINT64_C(100));
		TestTrue(TEXT("共通評価予算の理由を保持"), Report.Failure == PlanetCore::SpillDiscoveryFailure::BudgetLimit);
		Calls = 0;
		std::stop_source Stop;
		Stop.request_stop();
		TestFalse(
			TEXT("開始前の中断で元場を呼出さない"),
			PlanetCore::FindSpillComponentExit(Minima, Source, Settings, Stop.get_token(), &Report).has_value()
		);
		TestEqual(TEXT("中断後の評価は零"), Calls, UINT64_C(0));
		TestTrue(TEXT("中断理由を保持"), Report.Failure == PlanetCore::SpillDiscoveryFailure::Cancelled);
		Settings.MaximumEvaluations = 0;
		TestFalse(TEXT("評価予算零は不正入力"), PlanetCore::FindSpillComponentExit(Minima, Source, Settings, {}, &Report).has_value());
		TestTrue(TEXT("不正入力と予算消費を区別"), Report.Failure == PlanetCore::SpillDiscoveryFailure::InvalidInput);
	}
	// 同じ実地形の窪地から次の未登録水域を順次発見
	PlanetCore::GeologySettings Definition;
	Definition.Seed = 42;
	Definition.RadiusMeters = 1E6;
	std::optional<PlanetCore::GeologyField> Base = PlanetCore::GeologyField::Create(Definition);
	// 実地形の準備に失敗した場合の返戻
	if(!TestTrue(TEXT("実地質を準備"), Base.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(std::move(*Base));
	std::optional<PlanetCore::PlanetOrogenicRidges> Ridges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 造山場が無い場合の返戻
	if(!TestTrue(TEXT("実造山場を準備"), Ridges.has_value())) return false;
	const std::optional<PlanetCore::OceanConnectivityField> Ocean = PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(
		Geology,
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(std::move(*Ridges)),
		{},
		PlanetCore::PredrainageReliefMode::MesoAndLand,
		PlanetCore::OceanSamplingMode::SharedDrainage
	);
	// 大洋接続の元場を準備出来ない場合の返戻
	if(!TestTrue(TEXT("実海域を準備"), Ocean.has_value())) return false;
	// 診断全体も有限の評価予算内で実行
	constexpr std::uint64_t MaximumActualCalls = 32000000;
	std::uint64_t ActualCalls = 0;
	const auto Source = [&](const PlanetCore::UnitDirection D) -> PlanetCore::TerrainSample {
		if(ActualCalls >= MaximumActualCalls) {
			const double Invalid = std::numeric_limits<double>::quiet_NaN();
			// 診断全体の予算超過を未計測として伝える返戻
			return { Invalid, { Invalid, Invalid, Invalid } };
		}
		++ActualCalls;
		// 同じ元場の評価の返戻
		return Ocean->Evaluate(D);
	};
	const PlanetCore::UnitDirection KnownPass =
	*PlanetCore::UnitDirection::FromVector({ 0.54180127453542282, 0.81192128360203597, -0.21733662403243018 });
	const std::optional<PlanetCore::SpillConnection> Original = PlanetCore::PrepareSpillConnection(KnownPass, Source, {});
	// 始点を含む実接続が無い場合の返戻
	if(!TestTrue(TEXT("既知の実峠を準備"), Original.has_value())) return false;
	std::vector<PlanetCore::TerrainCriticalPoint> Minima;
	for(const PlanetCore::SpillConnectionBranch &Branch : Original->Branches) {
		// 既知の出発点の極小が欠けた場合の返戻
		if(!TestTrue(TEXT("元の両終端に極小が存在"), Branch.Minimum.has_value())) return false;
		Minima.push_back(*Branch.Minimum);
	}
	std::vector<PlanetCore::SpillConnection> Connections { *Original };
	bool HasSeaTerminal = false;
	const double DiscoveryAt = FPlatformTime::Seconds();
	// 既知の水系の広がり約３２㎞を囲む有限の探索範囲
	PlanetCore::SpillDiscoverySettings ActualSettings;
	ActualSettings.MaximumHalfWidthMeters = 65536;
	ActualSettings.MaximumEvaluations = 4000000;
	// 独立な組立でも，同じ版の準備関数と候補探索の定義を揃える
	ActualSettings.CandidateSpacingMeters = 16;
	ActualSettings.MaximumClimbingAttempts = 8;
	// 接続済の全球標本上の海域を，探索方向を決める目標として選択
	const std::optional<PlanetCore::SphereMesh> OceanMesh = PlanetCore::BuildUniformSphere(6);
	// 同じ全球標本が無い場合の返戻
	if(!TestTrue(TEXT("探索方向の海域標本を準備"), OceanMesh.has_value())) return false;
	double NearestOcean = std::numeric_limits<double>::infinity();
	std::vector<PlanetCore::UnitDirection> OceanAnchors;
	for(const PlanetCore::UnitDirection D : OceanMesh->Directions) {
		if(!(Source(D).ElevationMeters < -1)) continue;
		OceanAnchors.push_back(D);
		const PlanetCore::Vec3 A = KnownPass.Vector(), B = D.Vector();
		const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
		if(Distance < NearestOcean) {
			NearestOcean = Distance;
			ActualSettings.OutletTarget = D;
		}
	}
	// 海域を未確認の方向で置換えない返戻
	if(!TestTrue(TEXT("実際に水没した方向を発見"), ActualSettings.OutletTarget.has_value())) return false;
	AddInfo(
		FString::Printf(
			TEXT("Discovery ocean direction distance=%.12g H=%.12g"),
			NearestOcean,
			Source(*ActualSettings.OutletTarget).ElevationMeters
		)
	);
	for(unsigned Step = 0; Step < 512 && !HasSeaTerminal; ++Step) {
		PlanetCore::SpillDiscoveryDiagnostics Report;
		const std::optional<PlanetCore::SpillDiscoveryBatch> Batch =
		PlanetCore::FindSpillComponentExits(Minima, Source, ActualSettings, {}, &Report);
		AddInfo(
			FString::Printf(
				TEXT(
					"Actual discovery step=%u accepted=%u failure=%u windows=%u half=%.9g evaluations=%llu repeated=%u unrelated=%u unresolved=%u"
				),
				Step,
				static_cast<unsigned>(Batch.has_value()),
				static_cast<unsigned>(Report.Failure),
				Report.Windows,
				Report.LastHalfWidthMeters,
				static_cast<unsigned long long>(Report.Evaluations),
				Report.RepeatedConnections,
				Report.UnrelatedConnections,
				Report.UnresolvedPasses
			)
		);
		if(Report.BridgeCorridors) {
			AddInfo(FString::Printf(TEXT("Bridge discovery corridors=%u rejected=%u"), Report.BridgeCorridors, Report.RejectedBridges));
		}
		if(Report.RelaxedWindows || Report.RelaxationFailures) {
			AddInfo(
				FString::Printf(
					TEXT("Relaxed discovery windows=%u failures=%u normalSlope=%.12g motion=%.12g"),
					Report.RelaxedWindows,
					Report.RelaxationFailures,
					Report.LastRelaxedNormalSlope,
					Report.LastRelaxedMotionMeters
				)
			);
		}
		if(!Batch) {
			// 同じ元場・最終窓で，失った接続の原因を独立に記録
			const PlanetCore::TerrainCriticalPoint &Center = *std::min_element(
				Minima.begin(),
				Minima.end(),
				[&](const PlanetCore::TerrainCriticalPoint &A, const PlanetCore::TerrainCriticalPoint &B) -> bool {
					// 指定した海域に最も近い診断中心の比較の返戻
					return A.Direction.Vector().Dot(ActualSettings.OutletTarget->Vector()) >
					B.Direction.Vector().Dot(ActualSettings.OutletTarget->Vector());
				}
			);
			PlanetCore::SpillPassageSettings WindowSettings;
			WindowSettings.HalfWidthMeters = Report.LastHalfWidthMeters;
			WindowSettings.SpacingMeters = Report.LastHalfWidthMeters / 64;
			WindowSettings.MaximumEvaluations = 4000000;
			WindowSettings.WindowExitTarget = ActualSettings.OutletTarget;
			const std::optional<PlanetCore::SpillWindowExit> Exit =
			PlanetCore::FindSpillWindowExit(Center.Direction, Source, WindowSettings);
			if(Exit) for(const PlanetCore::UnitDirection Candidate : Exit->PassCandidates) {
				PlanetCore::SpillConnectionDiagnostics Detail;
				const std::optional<PlanetCore::SpillConnection> Local = PlanetCore::PrepareSpillConnection(Candidate, Source, {}, {}, &Detail);
				const PlanetCore::Vec3 P = Candidate.Vector();
				AddInfo(
					FString::Printf(
						TEXT("Missing candidate=(%.17g,%.17g,%.17g) accepted=%u failure=%u side=%u end=%u move=%.12g"),
						P.X,
						P.Y,
						P.Z,
						static_cast<unsigned>(Local.has_value()),
						static_cast<unsigned>(Detail.Failure),
						Detail.Branch,
						static_cast<unsigned>(Detail.TraceEnd),
						Detail.TerminalMoveMeters
					)
				);
				if(!Local) continue;
				for(const PlanetCore::SpillConnectionBranch &Branch : Local->Branches) {
					if(!Branch.Minimum) continue;
					double Nearest = std::numeric_limits<double>::infinity();
					const PlanetCore::TerrainCriticalPoint *NearestMinimum = nullptr;
					const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector();
					for(const PlanetCore::TerrainCriticalPoint &Known : Minima) {
						const PlanetCore::Vec3 B = Known.Direction.Vector();
						const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
						if(Distance < Nearest) {
							Nearest = Distance;
							NearestMinimum = &Known;
						}
					}
					AddInfo(FString::Printf(TEXT("Missing terminal H=%.12g nearestKnown=%.12g"), Branch.Minimum->ElevationMeters, Nearest));
					if(!NearestMinimum || Nearest <= 0.001) continue;
					const PlanetCore::Vec3 From = NearestMinimum->Direction.Vector();
					AddInfo(
						FString::Printf(TEXT("Bridge endpoints from=(%.17g,%.17g,%.17g) to=(%.17g,%.17g,%.17g)"), From.X, From.Y, From.Z, A.X, A.Y, A.Z)
					);
					// 不連続な候補列の両側に在る実極小を，狭い高密度の通路で再調査
					PlanetCore::SpillPassageSettings BridgeSettings;
					BridgeSettings.SpacingMeters = 25;
					BridgeSettings.HalfWidthMeters = 512;
					const std::optional<PlanetCore::SpillPassage> Bridge =
					PlanetCore::FindSpillPassage(NearestMinimum->Direction, Branch.Minimum->Direction, Source, BridgeSettings);
					if(!Bridge) {
						AddInfo(TEXT("Bridge corridor rejected"));
						continue;
					}
					AddInfo(
						FString::Printf(
							TEXT("Bridge corridor crest=%.12g evaluations=%llu nodes=%u independentExcess=%.12g"),
							Bridge->CrestMeters,
							static_cast<unsigned long long>(Bridge->Evaluations),
							Bridge->Nodes,
							Bridge->MaximumIndependentExcessMeters
						)
					);
					PlanetCore::SpillConnectionDiagnostics BridgeDetail;
					const std::optional<PlanetCore::SpillConnection> Crossing =
					PlanetCore::PrepareSpillConnection(Bridge->Peak, Source, {}, {}, &BridgeDetail);
					AddInfo(
						FString::Printf(
							TEXT("Bridge pass accepted=%u failure=%u"),
							static_cast<unsigned>(Crossing.has_value()),
							static_cast<unsigned>(BridgeDetail.Failure)
						)
					);
					if(!Crossing) continue;
					for(const PlanetCore::SpillConnectionBranch &CrossBranch : Crossing->Branches) {
						if(!CrossBranch.Minimum) continue;
						const PlanetCore::Vec3 Q = CrossBranch.Minimum->Direction.Vector();
						double NearKnown = std::numeric_limits<double>::infinity();
						for(const PlanetCore::TerrainCriticalPoint &Known : Minima) {
							const PlanetCore::Vec3 B = Known.Direction.Vector();
							NearKnown = std::min(NearKnown, Definition.RadiusMeters * std::atan2(Q.Cross(B).Length(), Q.Dot(B)));
						}
						AddInfo(FString::Printf(TEXT("Bridge terminal H=%.12g nearestKnown=%.12g"), CrossBranch.Minimum->ElevationMeters, NearKnown));
					}
				}
			}
			break;
		}
		const std::size_t PreviousCount = Minima.size();
		for(const PlanetCore::SpillConnection &Connection : Batch->Connections) {
			const PlanetCore::Vec3 PassDirection = Connection.Pass.Direction.Vector();
			const bool Duplicate = std::any_of(
				Connections.begin(),
				Connections.end(),
				[&](const PlanetCore::SpillConnection &Other) -> bool {
					const PlanetCore::Vec3 Q = Other.Pass.Direction.Vector();
					// 別窓で再確認した同じ実峠を重ねて保持しない返戻
					return Definition.RadiusMeters * std::atan2(PassDirection.Cross(Q).Length(), PassDirection.Dot(Q)) <= 0.001 &&
					std::abs(Connection.Pass.ElevationMeters - Other.Pass.ElevationMeters) <= 1E-6;
				}
			);
			if(Duplicate) continue;
			Connections.push_back(Connection);
			AddInfo(FString::Printf(TEXT("Discovered pass H=%.12g"), Connection.Pass.ElevationMeters));
			for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
				if(Branch.Kind == PlanetCore::SpillTerminalKind::SeaLevel) HasSeaTerminal = true;
				if(!Branch.Minimum) continue;
				const PlanetCore::Vec3 A = Branch.Minimum->Direction.Vector();
				const bool Known = std::any_of(
					Minima.begin(),
					Minima.end(),
					[&](const PlanetCore::TerrainCriticalPoint &P) -> bool {
						const PlanetCore::Vec3 B = P.Direction.Vector();
						// 実地形の終端の独立照合の返戻
						return Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B)) <= 0.001;
					}
				);
				if(!Known) {
					Minima.push_back(*Branch.Minimum);
					AddInfo(
						FString::Printf(
							TEXT("Discovered minimum H=%.12g direction=(%.17g,%.17g,%.17g)"),
							Branch.Minimum->ElevationMeters,
							A.X,
							A.Y,
							A.Z
						)
					);
				}
			}
		}
		TestEqual(TEXT("接続群の新極小数が独立な照合と一致"), Minima.size() - PreviousCount, Batch->NewMinima.size());
	}
	AddInfo(
		FString::Printf(
			TEXT("Actual discovered minima=%llu connections=%llu seaHeightTerminal=%u oceanMembershipVerified=0"),
			static_cast<unsigned long long>(Minima.size()),
			static_cast<unsigned long long>(Connections.size()),
			static_cast<unsigned>(HasSeaTerminal)
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Actual discovery evaluations=%llu milliseconds=%.9g"),
			static_cast<unsigned long long>(ActualCalls),
			(FPlatformTime::Seconds() - DiscoveryAt) * 1000
		)
	);
	TestTrue(TEXT("実窪地から海面高度迄の未指定経路を発見"), HasSeaTerminal);
	// 海面高度への到達だけでは，同じ大洋の階層へ登録しない
	std::vector<PlanetCore::UnitDirection> VerifiedSeaTerminals;
	for(const PlanetCore::SpillConnection &Connection : Connections) {
		for(const PlanetCore::SpillConnectionBranch &Branch : Connection.Branches) {
			if(Branch.Kind != PlanetCore::SpillTerminalKind::SeaLevel) continue;
			const PlanetCore::UnitDirection Terminal = Branch.Path.Samples.back().Direction;
			double Nearest = std::numeric_limits<double>::infinity();
			PlanetCore::UnitDirection Anchor = *ActualSettings.OutletTarget;
			for(const PlanetCore::UnitDirection D : OceanAnchors) {
				const PlanetCore::Vec3 A = Terminal.Vector(), B = D.Vector();
				const double Distance = Definition.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				if(Distance < Nearest) {
					Nearest = Distance;
					Anchor = D;
				}
			}
			bool Verified = false;
			for(const double HalfWidth : { 1000.0, 2000.0, 4000.0 }) {
				PlanetCore::SpillPassageSettings WetSettings;
				WetSettings.SpacingMeters = 50;
				WetSettings.HalfWidthMeters = HalfWidth;
				const std::optional<PlanetCore::SpillPassage> Wet = PlanetCore::FindSpillPassage(Terminal, Anchor, Source, WetSettings);
				if(!Wet) continue;
				const double CheckedMaximum = Wet->CrestMeters + Wet->MaximumIndependentExcessMeters;
				Verified = CheckedMaximum <= ActualSettings.SeaLevelMeters && Ocean->Statistics().ComponentsAfter == 1;
				AddInfo(
					FString::Printf(
						TEXT("Ocean terminal anchorDistance=%.12g halfWidth=%.12g maximumH=%.12g verified=%u evaluations=%llu"),
						Nearest,
						HalfWidth,
						CheckedMaximum,
						static_cast<unsigned>(Verified),
						static_cast<unsigned long long>(Wet->Evaluations)
					)
				);
				if(Verified) break;
			}
			TestTrue(TEXT("海面終端から接続済海域標本へ水没した通路が存在"), Verified);
			if(Verified) VerifiedSeaTerminals.push_back(Terminal);
		}
	}
	const auto IsVerifiedOcean = [&](const PlanetCore::UnitDirection D) -> bool {
		const PlanetCore::Vec3 A = D.Vector();
		for(const PlanetCore::UnitDirection Verified : VerifiedSeaTerminals) {
			const PlanetCore::Vec3 B = Verified.Vector();
			// 実際に通路を検査した同じ終端だけを受理する返戻
			if(A.X == B.X && A.Y == B.Y && A.Z == B.Z) return true;
		}
		// 水位だけ一致する未確認の水域を受理しない返戻
		return false;
	};
	PlanetCore::ContinuousSpillDiagnostics GraphReport;
	const std::optional<PlanetCore::ContinuousSpillGraph> Graph =
	PlanetCore::BuildContinuousSpillGraph(Connections, IsVerifiedOcean, {}, {}, &GraphReport);
	TestTrue(TEXT("実地形の全確認済極小を同じ大洋の排水階層へ統合"), Graph.has_value());
	AddInfo(
		FString::Printf(
			TEXT("Actual continuous graph accepted=%u failure=%u sites=%llu edges=%llu verifiedSeaTerminals=%llu totalEvaluations=%llu"),
			static_cast<unsigned>(Graph.has_value()),
			static_cast<unsigned>(GraphReport.Failure),
			static_cast<unsigned long long>(Graph ? Graph->Sites.size() : 0),
			static_cast<unsigned long long>(Graph ? Graph->Edges.size() : 0),
			static_cast<unsigned long long>(VerifiedSeaTerminals.size()),
			static_cast<unsigned long long>(ActualCalls)
		)
	);
	if(Graph) TestEqual(TEXT("発見した極小を捨てずに大洋と共有"), Graph->Sites.size(), Minima.size() + 1);
	// 再利用する準備関数を，試験内で独立に組立てた同じ水系へ照合
	std::set<std::tuple<double, double, double>> OceanKeys;
	for(const PlanetCore::UnitDirection D : OceanAnchors) {
		const PlanetCore::Vec3 P = D.Vector();
		OceanKeys.emplace(P.X, P.Y, P.Z);
	}
	const auto IsOceanAnchor = [&](const PlanetCore::UnitDirection D) -> bool {
		const PlanetCore::Vec3 P = D.Vector();
		// 同じ接続済の海域網で確認した標本だけを受理する返戻
		return Ocean->Statistics().ComponentsAfter == 1 && OceanKeys.contains({ P.X, P.Y, P.Z });
	};
	PlanetCore::SpillNetworkSettings NetworkSettings;
	NetworkSettings.UseClimbingFrontier = false;
	PlanetCore::SpillNetworkDiagnostics NetworkReport;
	const std::optional<PlanetCore::SpillNetwork> Network =
	PlanetCore::PrepareSpillNetwork(KnownPass, Source, OceanAnchors, IsOceanAnchor, NetworkSettings, {}, &NetworkReport);
	TestTrue(TEXT("再利用可能な準備関数が同じ実水系を生成"), Network.has_value());
	AddInfo(
		FString::Printf(
			TEXT("Prepared network accepted=%u failure=%u steps=%u evaluations=%llu"),
			static_cast<unsigned>(Network.has_value()),
			static_cast<unsigned>(NetworkReport.Failure),
			NetworkReport.Steps,
			static_cast<unsigned long long>(NetworkReport.Evaluations)
		)
	);
	if(Network && Graph) {
		bool Matches = Network->Connections.size() == Connections.size() && Network->Graph.Sites.size() == Graph->Sites.size() &&
		Network->Graph.Edges.size() == Graph->Edges.size() && Network->Graph.BranchSites == Graph->BranchSites &&
		Network->Graph.EdgeConnections == Graph->EdgeConnections;
		if(Matches) {
			for(std::size_t I = 0; I < Graph->Sites.size(); ++I) {
				Matches = Matches && Network->Graph.Sites[I].IsOcean == Graph->Sites[I].IsOcean &&
				std::bit_cast<std::uint64_t>(Network->Graph.Sites[I].ElevationMeters) ==
				std::bit_cast<std::uint64_t>(Graph->Sites[I].ElevationMeters);
			}
			for(std::size_t I = 0; I < Graph->Edges.size(); ++I) {
				Matches = Matches && Network->Graph.Edges[I].Nodes == Graph->Edges[I].Nodes &&
				std::bit_cast<std::uint64_t>(Network->Graph.Edges[I].CrestMeters) == std::bit_cast<std::uint64_t>(Graph->Edges[I].CrestMeters);
			}
		}
		TestTrue(TEXT("独立に組立てた地点・接続・標高とビット一致"), Matches);
	}
	if(Network) {
		// 包含された全極小から，実枝と非増加の水位を辿って大洋へ達する検査
		for(std::uint32_t Site = 1; Site < Network->Graph.Sites.size(); ++Site) {
			std::uint32_t Current = Site, Hops = 0;
			double PreviousWater = std::numeric_limits<double>::infinity();
			while(Current && Hops < Network->Graph.Sites.size()) {
				const std::optional<PlanetCore::SpillNetworkOutlet> Outlet = PlanetCore::GetSpillNetworkOutlet(*Network, Current);
				if(!TestTrue(TEXT("全ての実極小で流出口と受入水位を取得"), Outlet.has_value())) break;
				TestTrue(
					TEXT("水域の連鎖で水位が上昇しない"),
					Outlet->UpstreamWaterMeters <= PreviousWater && Outlet->DownstreamWaterMeters <= Outlet->UpstreamWaterMeters
				);
				const std::uint32_t Side = Outlet->IsPositiveDownstream ? 1 : 0;
				Current = Network->Graph.BranchSites[Outlet->Connection][Side];
				PreviousWater = Outlet->DownstreamWaterMeters;
				++Hops;
			}
			TestEqual(TEXT("受入水域の連鎖が大洋へ到達"), Current, UINT32_C(0));
		}
		const std::uint32_t InitialSite = Network->Graph.BranchSites[0][0];
		const std::optional<PlanetCore::SpillNetworkOutlet> Outlet = PlanetCore::GetSpillNetworkOutlet(*Network, InitialSite);
		if(Outlet) {
			AddInfo(
				FString::Printf(
					TEXT("Initial lake outlet connection=%u positive=%u water=%.12g receiver=%.12g basin=%u receivingBasin=%u"),
					Outlet->Connection,
					static_cast<unsigned>(Outlet->IsPositiveDownstream),
					Outlet->UpstreamWaterMeters,
					Outlet->DownstreamWaterMeters,
					Outlet->Basin,
					Outlet->ReceivingBasin
				)
			);
		}
		if(Outlet) {
			// 上流枝が到達する受入水域の実流出口と，同じ標高の岸
			const std::uint32_t ReceivingSite = Network->Graph.BranchSites[Outlet->Connection][Outlet->IsPositiveDownstream ? 1 : 0];
			const std::optional<PlanetCore::SpillNetworkOutlet> Receiving = PlanetCore::GetSpillNetworkOutlet(*Network, ReceivingSite);
			if(TestTrue(TEXT("受入湖も実流出口を持つ"), Receiving.has_value())) {
				const PlanetCore::SpillConnection &Exit = Network->Connections[Receiving->Connection];
				PlanetCore::TerrainContourSettings ShoreSettings;
				ShoreSettings.RadiusMeters = Definition.RadiusMeters;
				ShoreSettings.MaximumLengthMeters = std::min(1000000.0, Definition.RadiusMeters * 4);
				PlanetCore::SpillLakeDiagnostics ShoreReport;
				const double ShoreStarted = FPlatformTime::Seconds();
				const std::optional<PlanetCore::SpillLakeShore> Shore =
				PlanetCore::PrepareSpillLakeShore(Exit.Pass, !Receiving->IsPositiveDownstream, Source, ShoreSettings, {}, &ShoreReport);
				AddInfo(
					FString::Printf(
						TEXT("Receiving shore trace: pass=%u reason=%u end=%u vertices=%u length=%.12g residual=%.12g gap=%.12g area=%.12g"),
						Receiving->Connection,
						static_cast<unsigned>(ShoreReport.Failure),
						static_cast<unsigned>(ShoreReport.End),
						ShoreReport.Vertices,
						ShoreReport.LengthMeters,
						ShoreReport.HeightResidualMeters,
						ShoreReport.FirstGapMeters,
						ShoreReport.AreaMetersSquared
					)
				);
				const PlanetCore::Vec3 Last = ShoreReport.LastDirection, First = Exit.Pass.Direction.Vector();
				AddInfo(
					FString::Printf(
						TEXT("Receiving shore location: last=(%.17g,%.17g,%.17g) returnDistance=%.12g m"),
						Last.X,
						Last.Y,
						Last.Z,
						Definition.RadiusMeters * std::atan2(Last.Cross(First).Length(), Last.Dot(First))
					)
				);
				TestTrue(TEXT("受入湖の実際の岸を追跡して閉じる"), Shore.has_value());
				if(Shore) {
					const PlanetCore::SpillConnectionBranch &Incoming =
					Network->Connections[Outlet->Connection].Branches[Outlet->IsPositiveDownstream ? 1 : 0];
					TestTrue(TEXT("上流河川の下降先が受入湖内に在る"), Shore->Contains(Incoming.Path.Samples.back().Direction));
					// 既存の全辺走査を参照値とし，大きな実湖で索引の枝刈を独立比較
					const PlanetCore::Vec3 Origin = Exit.Pass.Direction.Vector();
					const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector(
						Exit.Pass.MinimumCurvatureDirection - Origin * Exit.Pass.MinimumCurvatureDirection.Dot(Origin)
					)->Vector() * (Receiving->IsPositiveDownstream ? -1.0 : 1.0);
					const PlanetCore::Vec3 North = Origin.Cross(East);
					const std::span<const std::array<double, 2>> Polygon = Shore->GetPlanarVertices();
					std::uint64_t IndexedEdges = 0;
					double MaximumIndexError = 0;
					for(std::uint32_t I = 0; I < 96; ++I) {
						const std::size_t Vertex = I * (Shore->GetContour().Samples.size() - 1) / 96;
						const PlanetCore::Vec3 Boundary = Shore->GetContour().Samples[Vertex].Direction.Vector();
						const double Shift = (static_cast<int>(I % 3) - 1) * 20.0 / Definition.RadiusMeters;
						const PlanetCore::UnitDirection Query = *PlanetCore::UnitDirection::FromVector(Boundary + East * Shift);
						const PlanetCore::Vec3 D = Query.Vector();
						const double Z = D.Dot(Origin), X = Definition.RadiusMeters * D.Dot(East) / Z, Y = Definition.RadiusMeters * D.Dot(North) / Z;
						double MinimumSquared = std::numeric_limits<double>::infinity();
						bool Inside = false, OnEdge = false;
						for(std::size_t J = 0; J < Polygon.size(); ++J) {
							const std::array<double, 2> &A = Polygon[J], &B = Polygon[(J + 1) % Polygon.size()];
							const double DX = B[0] - A[0], DY = B[1] - A[1], Cross = DX * (Y - A[1]) - DY * (X - A[0]);
							OnEdge = OnEdge ||
							!Cross && X >= std::min(A[0], B[0]) && X <= std::max(A[0], B[0]) && Y >= std::min(A[1], B[1]) && Y <= std::max(A[1], B[1]);
							const double T = std::clamp(((X - A[0]) * DX + (Y - A[1]) * DY) / (DX * DX + DY * DY), 0.0, 1.0), QX = X - A[0] - T * DX;
							const double QY = Y - A[1] - T * DY;
							MinimumSquared = std::min(MinimumSquared, QX * QX + QY * QY);
							if(A[1] > Y != B[1] > Y && X < A[0] + (Y - A[1]) * DX / DY) Inside ^= true;
						}
						const double Expected = OnEdge ? 0 : std::sqrt(MinimumSquared) * (Inside ? 1 : -1);
						std::uint32_t TestedEdges = 0;
						const double Actual = Shore->SignedPlanarDistanceMeters(Query, &TestedEdges);
						IndexedEdges += TestedEdges;
						MaximumIndexError = std::max(MaximumIndexError, std::abs(Actual - Expected));
						TestTrue(TEXT("岸の索引が全辺走査の符号と距離に一致"), std::abs(Actual - Expected) < 1E-9);
					}
					TestTrue(TEXT("大きな実湖で全辺の再走査を削減"), IndexedEdges < Polygon.size() * 96 / 4);
					AddInfo(
						FString::Printf(
							TEXT("Receiving shore index: samples=96 testedEdges=%llu allEdges=%llu maximumError=%.12g"),
							static_cast<unsigned long long>(IndexedEdges),
							static_cast<unsigned long long>(Polygon.size() * 96),
							MaximumIndexError
						)
					);
					AddInfo(
						FString::Printf(
							TEXT("Receiving lake shore: pass=%u water=%.12g vertices=%u length=%.12g residual=%.12g seconds=%.9g"),
							Receiving->Connection,
							Exit.Pass.ElevationMeters,
							static_cast<unsigned>(Shore->GetContour().Samples.size()),
							Shore->GetContour().Samples.back().DistanceMeters,
							Shore->GetContour().MaximumHeightResidualMeters,
							FPlatformTime::Seconds() - ShoreStarted
						)
					);
				}
			}
		}
		TestFalse(TEXT("大洋其の物に陸上の流出口を作らない"), PlanetCore::GetSpillNetworkOutlet(*Network, 0).has_value());
		TestFalse(TEXT("未登録の地点を流出口へ丸めない"), PlanetCore::GetSpillNetworkOutlet(*Network, UINT32_MAX).has_value());
	}
	// 部分的な準備を成功として返さない拒否経路
	NetworkSettings.MaximumEvaluations = 100;
	const std::uint64_t CallsBeforeLimit = ActualCalls;
	TestFalse(
		TEXT("全準備の共通評価予算を守る"),
		PlanetCore::PrepareSpillNetwork(KnownPass, Source, OceanAnchors, IsOceanAnchor, NetworkSettings, {}, &NetworkReport).has_value()
	);
	TestEqual(TEXT("準備関数の外側でも評価回数を確認"), ActualCalls - CallsBeforeLimit, UINT64_C(100));
	TestTrue(TEXT("共通予算の不足を通知"), NetworkReport.Failure == PlanetCore::SpillNetworkFailure::BudgetLimit);
	std::stop_source Cancel;
	Cancel.request_stop();
	const std::uint64_t CallsBeforeCancel = ActualCalls;
	TestFalse(
		TEXT("開始前の中断で水系を公開しない"),
		PlanetCore::PrepareSpillNetwork(
			KnownPass,
			Source,
			OceanAnchors,
			IsOceanAnchor,
			NetworkSettings,
			Cancel.get_token(),
			&NetworkReport
		).has_value()
	);
	TestEqual(TEXT("中断後の元場照会は零"), ActualCalls, CallsBeforeCancel);
	TestTrue(TEXT("中断の理由を区別"), NetworkReport.Failure == PlanetCore::SpillNetworkFailure::Cancelled);
	NetworkSettings.MaximumEvaluations = 32000000;
	const auto RefuseAnchor = [](const PlanetCore::UnitDirection) -> bool {
		return false;
	};
	TestFalse(
		TEXT("所属未確認の海域で水系を成立させない"),
		PlanetCore::PrepareSpillNetwork(KnownPass, Source, OceanAnchors, RefuseAnchor, NetworkSettings, {}, &NetworkReport).has_value()
	);
	TestTrue(TEXT("大洋標本の欠落を通知"), NetworkReport.Failure == PlanetCore::SpillNetworkFailure::NoOceanAnchor);
	NetworkSettings.MaximumBranchSamples = 1;
	TestFalse(
		TEXT("初期接続も保持量の上限へ含める"),
		PlanetCore::PrepareSpillNetwork(KnownPass, Source, OceanAnchors, IsOceanAnchor, NetworkSettings, {}, &NetworkReport).has_value()
	);
	TestTrue(TEXT("初期経路の保持予算を通知"), NetworkReport.Failure == PlanetCore::SpillNetworkFailure::BudgetLimit);
	// 有限標本の接続証拠を，全地点・全ての乱数種の保証と区別した返戻
	return !HasAnyErrors();
}

#endif
