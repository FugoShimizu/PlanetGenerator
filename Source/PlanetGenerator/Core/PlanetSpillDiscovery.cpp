#include "PlanetSpillDiscovery.h"
#include "PlanetSpillPassage.h"
#include "PlanetSpillString.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		/**
		 * 同じ元場の停留点を，種類・物理距離・標高で照合する
		 * @param A 一方の確認済の停留点
		 * @param B 他方の確認済の停留点
		 * @param Radius 惑星半径
		 * @return 既存の水系統合と同じ許容値内の場合に真
		 */
		bool SameSpillCriticalPoint(const TerrainCriticalPoint &A, const TerrainCriticalPoint &B, const double Radius) {
			const Vec3 P = A.Direction.Vector(), Q = B.Direction.Vector();
			// 近い位置だけで異なる種類又は標高を同一視しない返戻
			return A.Kind == B.Kind && Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q)) <= 0.001 &&
			std::abs(A.ElevationMeters - B.ElevationMeters) <= 1E-6;
		}
	}

	/**
	 * 既知の極小集合から窓を拡張し，集合外の極小又は海面高度へ繋がる実鞍点を探す
	 * 全出口の列挙・大洋所属・全球の最低通過高度の保証は行わない
	 * @param Minima 同じ不変な元場で接続を確認した極小の集合
	 * @param Source 元場の標高と解析接線微分
	 * @param Settings 物理範囲と全窓・両下降枝の共通評価予算
	 * @param Stop 中断要求
	 * @param Diagnostics 未発見の理由と実際の計算費用
	 * @param CompletedConnections 同じ元場・尺度・水位で準備済且つ両極小が入力集合内に在る接続
	 * @return 入力から実際に繋がる接続群と新たな極小；未発見・予算超過は空
	 */
	std::optional<SpillDiscoveryBatch> FindSpillComponentExits(
		const std::span<const TerrainCriticalPoint> Minima,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillDiscoverySettings &Settings,
		const std::stop_token Stop,
		SpillDiscoveryDiagnostics *const Diagnostics,
		const std::span<const SpillConnection> CompletedConnections
	) {
		// 出力先を省略した場合も同じ費用制限を使う
		SpillDiscoveryDiagnostics Local;
		SpillDiscoveryDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const SpillDiscoveryFailure Reason) -> std::optional<SpillDiscoveryBatch> {
			Report.Failure = Stop.stop_requested() ?
			SpillDiscoveryFailure::Cancelled :
			Reason != SpillDiscoveryFailure::InvalidInput && Report.Evaluations >= Settings.MaximumEvaluations ?
			SpillDiscoveryFailure::BudgetLimit :
			Reason;
			// 不完全な接続を公開しない返戻
			return {};
		};
		// 外部の入力と整数化前の有限な範囲
		const bool Valid = Source && !Minima.empty() && Settings.MaximumKnownMinima > 0 && Settings.MaximumKnownMinima < 4098 &&
		Minima.size() <= Settings.MaximumKnownMinima && CompletedConnections.size() < 4097 && std::isfinite(Settings.RadiusMeters) &&
		!(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 && std::isfinite(Settings.SeaLevelMeters) && (
			!Settings.MaximumPassMeters ||
			std::isfinite(*Settings.MaximumPassMeters) && *Settings.MaximumPassMeters > Settings.SeaLevelMeters
		) && std::isfinite(Settings.InitialHalfWidthMeters) && !(Settings.InitialHalfWidthMeters < 1) &&
		std::isfinite(Settings.MaximumHalfWidthMeters) && Settings.MaximumHalfWidthMeters >= Settings.InitialHalfWidthMeters &&
		Settings.MaximumHalfWidthMeters < Settings.RadiusMeters * 0.5 && Settings.HalfCells > 3 && Settings.HalfCells < 128 &&
		std::isfinite(Settings.CandidateSpacingMeters) && Settings.CandidateSpacingMeters >= 0 &&
		(!Settings.CandidateSpacingMeters || Settings.CandidateSpacingMeters >= 0.01) &&
		Settings.InitialHalfWidthMeters / Settings.HalfCells >= 0.01 && Settings.MaximumEvaluations > 0 &&
		Settings.MaximumEvaluations < 100000001 && Settings.MaximumConnections > 0 && Settings.MaximumConnections < 1025 &&
		Settings.MaximumBranchSamples > 0 && Settings.MaximumBranchSamples < 262145 && Settings.MaximumBridgeCorridors < 65 &&
		Settings.MaximumClimbingAttempts < 9 && (!Settings.ClimbingOnly || Settings.MaximumClimbingAttempts > 0) &&
		!(Settings.ClimbingOnly && Settings.InitialWindowOnly);
		// 無効な設定への返戻
		if(!Valid) return Reject(SpillDiscoveryFailure::InvalidInput);
		// 確認済の極小以外を出発点にしない返戻
		for(const TerrainCriticalPoint &Minimum : Minima) {
			if(Minimum.Kind != TerrainCriticalKind::Minimum || !std::isfinite(Minimum.ElevationMeters)) {
				// 連続場で未確認の始点への返戻
				return Reject(SpillDiscoveryFailure::InvalidInput);
			}
		}
		// 別の盆地への未確認接続を，再利用の名目で探索対象から外さない
		for(const SpillConnection &Connection : CompletedConnections) {
			if(Connection.Pass.Kind != TerrainCriticalKind::Saddle || !std::isfinite(Connection.Pass.ElevationMeters)) {
				// 非鞍点の省略指定への返戻
				return Reject(SpillDiscoveryFailure::InvalidInput);
			}
			for(const SpillConnectionBranch &Branch : Connection.Branches) {
				// 大洋到達前の内部接続だけを再利用する返戻
				if(Branch.Kind != SpillTerminalKind::Minimum || !Branch.Minimum) return Reject(SpillDiscoveryFailure::InvalidInput);
				const bool IsInput = std::any_of(
					Minima.begin(),
					Minima.end(),
					[&](const TerrainCriticalPoint &P) -> bool {
						// 同じ確認済の極小か否かの返戻
						return SameSpillCriticalPoint(P, *Branch.Minimum, Settings.RadiusMeters);
					}
				);
				// 集合外へ繋がる枝を省略しない返戻
				if(!IsInput) return Reject(SpillDiscoveryFailure::InvalidInput);
			}
		}
		// 開始前の中断への返戻
		if(Stop.stop_requested()) return Reject(SpillDiscoveryFailure::Cancelled);
		bool HasFiniteSource = true, HasCompleteEvaluations = true;
		const auto Sample = [&](const UnitDirection D) -> TerrainSample {
			if(Stop.stop_requested() || Report.Evaluations >= Settings.MaximumEvaluations) {
				HasCompleteEvaluations = false;
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 予算後に元場を呼出さない返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Report.Evaluations;
			const TerrainSample Value = Source(D);
			HasFiniteSource =
			HasFiniteSource && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			// 窓と枝に共通する元場の評価の返戻
			return Value;
		};
		// 出口の指定時は其の方向へ進んだ極小，省略時は最も低い極小を選択
		const auto Key = [&Settings](const TerrainCriticalPoint &P) -> std::tuple<double, double, double, double, double> {
			const Vec3 D = P.Direction.Vector();
			// 同標高の場合も決定的な中心選択のキーの返戻
			return { Settings.OutletTarget ? -D.Dot(Settings.OutletTarget->Vector()) : 0, P.ElevationMeters, D.X, D.Y, D.Z };
		};
		std::span<const TerrainCriticalPoint>::iterator CenterIt = std::min_element(
			Minima.begin(),
			Minima.end(),
			[&](const TerrainCriticalPoint &A, const TerrainCriticalPoint &B) -> bool {
				// 元場の同じ極小集合に対する比較の返戻
				return Key(A) < Key(B);
			}
		);
		if(Settings.PreferredCenter) {
			const Vec3 Expected = Settings.PreferredCenter->Vector();
			CenterIt = std::find_if(
				Minima.begin(),
				Minima.end(),
				[Expected](const TerrainCriticalPoint &Minimum) -> bool {
					const Vec3 Actual = Minimum.Direction.Vector();
					// 照会点を近くの別盆地へ丸めない，入力集合内の厳密な中心一致の返戻
					return Actual.X == Expected.X && Actual.Y == Expected.Y && Actual.Z == Expected.Z;
				}
			);
			// 入力に無い盆地を既知の成分に混ぜない返戻
			if(CenterIt == Minima.end()) return Reject(SpillDiscoveryFailure::InvalidInput);
		}
		const TerrainCriticalPoint &Center = *CenterIt;
		Report.CenterDirection = Center.Direction;
		// 各窓で確認した峠を保持し，同じ高価な経路から得た接続をまとめる
		SpillDiscoveryBatch Batch;
		std::vector<TerrainCriticalPoint> Known(Minima.begin(), Minima.end()), Passes;
		std::vector<SpillConnection> Pending;
		std::vector<bool> Selected;
		std::size_t BranchSamples = 0;
		const auto IsKnown = [&](const SpillConnectionBranch &Branch) -> bool {
			// 海面終端には陸上の極小を割当てない返戻
			if(!Branch.Minimum) return false;
			for(const TerrainCriticalPoint &Minimum : Known) {
				// 同じ実極小を確認した場合の返戻
				if(SameSpillCriticalPoint(*Branch.Minimum, Minimum, Settings.RadiusMeters)) return true;
			}
			// 対応する極小が無い場合の返戻
			return false;
		};
		const auto ReusePass = [&](const TerrainCriticalPoint &Pass) -> bool {
			for(const SpillConnection &Completed : CompletedConnections) {
				if(!SameSpillCriticalPoint(Completed.Pass, Pass, Settings.RadiusMeters)) continue;
				++Report.RepeatedConnections;
				++Report.ReusedConnections;
				// 呼出元の水系に既に存在する接続の返戻
				return true;
			}
			for(std::size_t I = 0; I < Passes.size(); ++I) {
				if(!SameSpillCriticalPoint(Passes[I], Pass, Settings.RadiusMeters)) continue;
				if(Selected[I] || IsKnown(Pending[I].Branches[0]) && IsKnown(Pending[I].Branches[1])) ++Report.RepeatedConnections;
				else if(!IsKnown(Pending[I].Branches[0]) && !IsKnown(Pending[I].Branches[1])) ++Report.UnrelatedConnections;
				++Report.ReusedConnections;
				// 此の探索で確認済の両枝を再追跡しない返戻
				return true;
			}
			// 同じ物理位置・種類・標高の接続を未確認の場合の返戻
			return false;
		};
		const auto PrepareCandidates = [&](const std::span<const UnitDirection> Candidates, const double SpacingMeters) -> bool {
			for(const UnitDirection Candidate : Candidates) {
				// 候補間の中断・場の欠落・予算超過への返戻
				if(!HasFiniteSource || Stop.stop_requested() || Report.Evaluations >= Settings.MaximumEvaluations) return false;
				// 高価な両枝追跡に先立ち，同じ鞍点の再発見を照合する
				TerrainCriticalSettings Critical;
				Critical.RadiusMeters = Settings.RadiusMeters;
				// 粗い候補の格子間隔に合わせる探索範囲；収束の位置精度と勾配精度は維持する
				Critical.MaximumDisplacementMeters =
				std::min(std::max(Critical.MaximumDisplacementMeters, SpacingMeters * 2), Settings.RadiusMeters * 0.125);
				Critical.MaximumStepMeters = std::min(std::max(Critical.MaximumStepMeters, SpacingMeters), Critical.MaximumDisplacementMeters);
				Report.MaximumCriticalStepMeters = std::max(Report.MaximumCriticalStepMeters, Critical.MaximumStepMeters);
				Report.MaximumCriticalSearchRadiusMeters =
				std::max(Report.MaximumCriticalSearchRadiusMeters, Critical.MaximumDisplacementMeters);
				TerrainCriticalDiagnostics CriticalReport;
				const std::optional<TerrainCriticalPoint> Pass = RefineTerrainCriticalPoint(Candidate, Sample, Critical, Stop, &CriticalReport);
				++Report.CriticalFailures[static_cast<std::size_t>(CriticalReport.Failure)];
				if(Pass) ++Report.CriticalKinds[static_cast<std::size_t>(Pass->Kind)];
				else if(!Report.FirstFailedCriticalCandidate) {
					Report.FirstFailedCriticalCandidate = Candidate;
					Report.FirstFailedCritical = CriticalReport;
				}
				if(!Pass || Pass->Kind != TerrainCriticalKind::Saddle || !(Pass->ElevationMeters > Settings.SeaLevelMeters)) {
					++Report.UnresolvedPasses;
					++Report.UnresolvedCriticalPoints;
					continue;
				}
				if(Settings.MaximumPassMeters && !(Pass->ElevationMeters < *Settings.MaximumPassMeters)) {
					++Report.ExcludedHighPasses;
					continue;
				}
				if(ReusePass(*Pass)) continue;
				SpillConnectionSettings ConnectionSettings;
				ConnectionSettings.RadiusMeters = Settings.RadiusMeters;
				ConnectionSettings.SeaLevelMeters = Settings.SeaLevelMeters;
				SpillConnectionDiagnostics ConnectionReport;
				std::optional<SpillConnection> Connection =
				PrepareSpillConnection(Pass->Direction, Sample, ConnectionSettings, Stop, &ConnectionReport);
				if(!Connection) {
					++Report.UnresolvedPasses;
					++Report.UnresolvedConnections;
					if(!Report.FirstFailedConnectionCandidate) {
						Report.FirstFailedConnectionCandidate = Pass->Direction;
						Report.FirstFailedConnection = ConnectionReport;
					}
					continue;
				}
				const bool First = IsKnown(Connection->Branches[0]), Second = IsKnown(Connection->Branches[1]);
				if(First && Second) ++Report.NewInternalConnections;
				else if(!First && !Second) ++Report.UnrelatedConnections;
				const bool Duplicate = std::any_of(
					Passes.begin(),
					Passes.end(),
					[&](const TerrainCriticalPoint &P) -> bool {
						// 初期候補の違いだけで同じ実鞍点を重複しない返戻
						return SameSpillCriticalPoint(P, Connection->Pass, Settings.RadiusMeters);
					}
				);
				if(Duplicate) continue;
				const std::size_t Samples = Connection->Branches[0].Path.Samples.size() + Connection->Branches[1].Path.Samples.size();
				// 下降経路を含む保持量の上限への返戻
				if(Pending.size() >= Settings.MaximumConnections || BranchSamples + Samples > Settings.MaximumBranchSamples) return false;
				BranchSamples += Samples;
				Passes.push_back(Connection->Pass);
				Pending.push_back(std::move(*Connection));
				Selected.push_back(false);
			}
			// 全候補を有限な元場で検査した結果の返戻
			return HasFiniteSource && HasCompleteEvaluations && !Stop.stop_requested();
		};
		const auto SelectConnected = [&]() -> bool {
			bool Changed = true;
			while(Changed) {
				Changed = false;
				for(std::size_t I = 0; I < Pending.size(); ++I) {
					if(Selected[I]) continue;
					SpillConnection &Connection = Pending[I];
					if(!IsKnown(Connection.Branches[0]) && !IsKnown(Connection.Branches[1])) continue;
					for(const SpillConnectionBranch &Branch : Connection.Branches) {
						if(Branch.Kind == SpillTerminalKind::SeaLevel) Batch.HasSeaHeightTerminal = true;
						if(!Branch.Minimum || IsKnown(Branch)) continue;
						// 次の正規呼出でも扱える極小の上限への返戻
						if(Known.size() >= Settings.MaximumKnownMinima) return false;
						Known.push_back(*Branch.Minimum);
						Batch.NewMinima.push_back(*Branch.Minimum);
					}
					Selected[I] = true;
					Changed = true;
					Batch.Connections.push_back(std::move(Connection));
				}
				// 集合の接続判定中の中断への返戻
				if(Stop.stop_requested()) return false;
			}
			// 入力の水系へ実接続した成分だけの選択成功の返戻
			return true;
		};
		const auto HasDiscoveredConnection = [&]() -> bool {
			// 同じ端点でも，未登録の実峠は水位を変える別の通路として扱う返戻
			return !Batch.NewMinima.empty() || Batch.HasSeaHeightTerminal ||
			Settings.IncludeInternalConnections && !Batch.Connections.empty();
		};
		// 未接続の短区間を優先し，元の実鞍点と両終端の確認へ候補だけを渡す
		const auto RelaxCandidates =
		[&](const std::span<const UnitDirection> Path, const bool IsBridge, const double SpacingMeters) -> bool {
			SpillStringSettings StringSettings;
			StringSettings.RadiusMeters = Settings.RadiusMeters;
			StringSettings.MaximumEvaluations = Settings.MaximumEvaluations;
			if(IsBridge) {
				StringSettings.TimeStepMeters = 5000;
				StringSettings.MaximumIterations = 512;
			}
			const std::optional<SpillString> Relaxed = RelaxSpillString(Path, Sample, StringSettings, Stop);
			if(Relaxed) {
				if(IsBridge) ++Report.RelaxedBridges;
				else ++Report.RelaxedWindows;
				Report.LastRelaxedNormalSlope = Relaxed->MaximumNormalSlope;
				Report.LastRelaxedMotionMeters = Relaxed->MaximumMotionMeters;
				// 緩和後も同じ保持予算と実下降終端を確認した結果の返戻
				return PrepareCandidates(Relaxed->PassCandidates, SpacingMeters) && SelectConnected();
			}
			++Report.RelaxationFailures;
			// 候補の点数不足だけは別経路へ進め，場・全体予算・中断の不足は隠さない返戻
			return HasFiniteSource && HasCompleteEvaluations && !Stop.stop_requested();
		};
		// 長い二次元窓を緩和する前に，入力の実極小に隣接する峠候補を少ない点で準備
		const auto ClimbCandidates = [&]() -> bool {
			const Vec3 P = Center.Direction.Vector();
			const Vec3 Aim = Settings.OutletTarget ? Settings.OutletTarget->Vector() : Center.MinimumCurvatureDirection;
			const std::optional<UnitDirection> Toward = UnitDirection::FromVector(Aim - P * Aim.Dot(P));
			// 接線を得られない時は元の窓探索へ委ねる返戻
			if(!Toward) return true;
			const Vec3 Forward = Toward->Vector(), Side = P.Cross(Forward);
			const double Quarter = std::acos(-1.0) * 0.25, Angle = std::min(16.0, Settings.RadiusMeters * 0.004) / Settings.RadiusMeters;
			const std::array<double, 8> Turns { 0, Quarter, -Quarter, 2 * Quarter, -2 * Quarter, 3 * Quarter, -3 * Quarter, 4 * Quarter };
			while(Report.ClimbingAttempts < Settings.MaximumClimbingAttempts) {
				// 同じ窓・元場の共通予算を使い，局所候補の予算を別枠へ逃さない
				const bool HasBudget =
				HasFiniteSource && HasCompleteEvaluations && !Stop.stop_requested() && Report.Evaluations < Settings.MaximumEvaluations;
				// 同じ元場の残り予算を失った場合の返戻
				if(!HasBudget) return false;
				const double Turn = Turns[Report.ClimbingAttempts++];
				const Vec3 Along = Forward * std::cos(Turn) + Side * std::sin(Turn);
				const UnitDirection Tip = *UnitDirection::FromVector(P * std::cos(Angle) + Along * std::sin(Angle));
				SpillClimbSettings ClimbSettings;
				ClimbSettings.RadiusMeters = Settings.RadiusMeters;
				ClimbSettings.MaximumLengthMeters = std::min(2000.0, Settings.RadiusMeters * 0.125);
				ClimbSettings.MaximumEvaluations =
				static_cast<std::uint32_t>(std::min<std::uint64_t>(100000, Settings.MaximumEvaluations - Report.Evaluations));
				const std::optional<SpillClimb> Climb = ClimbSpillString(Center, Tip, Sample, ClimbSettings, Stop);
				if(!Climb) {
					++Report.ClimbingFailures;
					// 元場又は共通予算を失った場合は別候補へ進めない返戻
					if(!HasFiniteSource || !HasCompleteEvaluations || Stop.stop_requested()) return false;
					continue;
				}
				++Report.ClimbingCandidates;
				Report.LastClimbEnd = static_cast<std::uint32_t>(Climb->End);
				Report.LastClimbNormalSlope = Climb->MaximumNormalSlope;
				Report.LastClimbTipGradientNorm = Climb->TipGradientNorm;
				// 反復上限の候補も其れ自体を接続とせず，既存の実鞍点・両下降枝へ委ねる
				if(!PrepareCandidates(std::span<const UnitDirection>(&Climb->Path.back(), 1), 0) || !SelectConnected()) return false;
				// 入力に隣接する実接続で新たな盆地へ到達した結果の返戻
				if(HasDiscoveredConnection()) return true;
			}
			// 有限方向で未発見の場合は，既存の窓拡張を継続する返戻
			return true;
		};
		if(Settings.ClimbingOnly) {
			const bool HasCandidates = ClimbCandidates();
			// 登坂の途中失敗又は共通予算の不足への返戻
			if(!HasCandidates) return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
			// 元場の両下降枝で入力成分から外へ繋がった結果の返戻
			if(HasDiscoveredConnection()) return Batch;
			// 有限方向での未発見を閉鎖水域又は二次元窓の完全探索へ昇格しない返戻
			return Reject(SpillDiscoveryFailure::NoIncidentExit);
		}
		double HalfWidth = Settings.InitialHalfWidthMeters;
		while(true) {
			// 有限な物理窓の準備
			SpillPassageSettings WindowSettings;
			WindowSettings.RadiusMeters = Settings.RadiusMeters;
			WindowSettings.HalfWidthMeters = HalfWidth;
			// 狭い探索窓の過剰な節点を省き，窓の拡大時には既存の上限迄密度を上げる
			const std::uint32_t HalfCells = Settings.CandidateSpacingMeters ?
			static_cast<std::uint32_t>(
				std::min(static_cast<double>(Settings.HalfCells), std::max(4.0, std::ceil(HalfWidth / Settings.CandidateSpacingMeters)))
			) :
			Settings.HalfCells;
			WindowSettings.SpacingMeters = HalfWidth / HalfCells;
			Report.LastHalfCells = HalfCells;
			WindowSettings.MaximumEvaluations = Settings.MaximumEvaluations;
			WindowSettings.WindowExitTarget = Settings.OutletTarget;
			// 全辺の下限から選ぶ候補経路を先に実測し，元場の精度と全窓の共通予算を維持する
			WindowSettings.UsePathEdgeSelection = true;
			WindowSettings.UseCellCriticalCandidates = true;
			Report.LastHalfWidthMeters = HalfWidth;
			++Report.Windows;
			const std::optional<SpillWindowExit> Exit = FindSpillWindowExit(Center.Direction, Sample, WindowSettings, Stop);
			// 予算・中断又は元場の欠落を窓拡張で隠さない返戻
			if(!Exit) return Reject(SpillDiscoveryFailure::WindowFailed);
			Report.CellCriticalCandidates += Exit->CellCriticalCandidates;
			// 候補集合を途中の予算超過で切取って公開しない返戻
			const std::span<const UnitDirection> Candidates = Exit->PassCandidates;
			const std::size_t HeightCandidates = Candidates.size() - Exit->CellCriticalCandidates;
			if(!PrepareCandidates(Candidates.first(HeightCandidates), WindowSettings.SpacingMeters) || !SelectConnected()) {
				// 元場の欠落と，評価又は保持量の不足を区別した返戻
				return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
			}
			// 一つの新接続に続く，同じ経路上の確認済接続も合わせた返戻
			if(HasDiscoveredConnection()) return Batch;
			// 従来の正規の出口を得た場合は余分な下降を追わず，未発見の窓だけ解析勾配で補う
			if(!PrepareCandidates(Candidates.last(Exit->CellCriticalCandidates), WindowSettings.SpacingMeters) || !SelectConnected()) {
				// 同じ総予算又は元場の欠落への返戻
				return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
			}
			// 補間候補でも実鞍点と両下降枝で確認した接続の返戻
			if(HasDiscoveredConnection()) return Batch;
			// 小窓の未発見を登坂や広い窓の探索済へ置換えない返戻
			if(Settings.InitialWindowOnly) return Reject(SpillDiscoveryFailure::NoIncidentExit);
			// 通常の小さな窓で新たな実接続が無い場合だけ，一端登坂を有限回試す
			const bool HasCandidates = ClimbCandidates();
			// 途中の元場又は共通予算の欠落への返戻
			if(!HasCandidates) return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
			// 登坂終点の候補でなく，元場の実鞍点と両下降終端を確認した接続群の返戻
			if(HasDiscoveredConnection()) return Batch;
			// 格子候補が停留点へ収束しない場合も，同じ経路の解析勾配から候補を探す
			if(Report.Evaluations < Settings.MaximumEvaluations && !Stop.stop_requested()) {
				// 粗い通路の既知側と未接続側の実極小を選び，欠けた接続だけを再探索
				while(Report.BridgeCorridors < Settings.MaximumBridgeCorridors) {
					double Nearest = std::numeric_limits<double>::infinity();
					UnitDirection From = Center.Direction, To = Center.Direction;
					for(std::size_t I = 0; I < Pending.size(); ++I) {
						if(Selected[I]) continue;
						for(const SpillConnectionBranch &Branch : Pending[I].Branches) {
							if(!Branch.Minimum || IsKnown(Branch)) continue;
							const Vec3 B = Branch.Minimum->Direction.Vector();
							for(const TerrainCriticalPoint &Minimum : Known) {
								const Vec3 A = Minimum.Direction.Vector();
								const double Distance = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
								if(Distance < Nearest) {
									Nearest = Distance;
									From = Minimum.Direction;
									To = Branch.Minimum->Direction;
								}
							}
						}
					}
					if(!std::isfinite(Nearest)) break;
					SpillPassageSettings BridgeSettings;
					BridgeSettings.RadiusMeters = Settings.RadiusMeters;
					BridgeSettings.SpacingMeters = std::min(25.0, Nearest / 64);
					BridgeSettings.HalfWidthMeters = std::min({ 512.0, Settings.RadiusMeters * 0.25, std::max(16.0, Nearest * 0.5) });
					BridgeSettings.MaximumEvaluations = Settings.MaximumEvaluations;
					++Report.BridgeCorridors;
					Report.BridgeEndpoints.push_back({ From, To });
					const std::optional<SpillPassage> Bridge = FindSpillPassage(From, To, Sample, BridgeSettings, Stop);
					if(!Bridge) {
						++Report.RejectedBridges;
						if(!Report.FirstRejectedBridgeStart) {
							Report.FirstRejectedBridgeStart = From;
							Report.FirstRejectedBridgeEnd = To;
							Report.FirstRejectedBridgeLengthMeters = Nearest;
						}
						// 元場の欠落と全体予算の不足を，別の候補探索で隠さない返戻
						if(!HasFiniteSource || !HasCompleteEvaluations || Stop.stop_requested()) return Reject(SpillDiscoveryFailure::WindowFailed);
						break;
					}
					const std::size_t Before = Pending.size();
					// 中間の低い峠も，元場の両下降終端を確認して保持
					if(!PrepareCandidates(Bridge->PassCandidates, BridgeSettings.SpacingMeters) || !SelectConnected()) {
						// 部分的な準備又は記憶量の不足への返戻
						return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
					}
					// 新たに繋がった実接続群の返戻
					if(HasDiscoveredConnection()) return Batch;
					// 外周へ出る長い経路より先に，欠けた実極小間の経路を緩和
					if(!RelaxCandidates(Bridge->Path, true, BridgeSettings.SpacingMeters)) {
						// 部分的な候補又は共有予算の不足への返戻
						return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
					}
					// 同じ元場で確認した新接続群の返戻
					if(HasDiscoveredConnection()) return Batch;
					if(Pending.size() == Before) break;
				}
				// 格子の候補に接続を確認出来なかった同じ経路を，元場の勾配で緩和
				if(!RelaxCandidates(Exit->Path, false, WindowSettings.SpacingMeters)) {
					// 部分的な候補集合を公開しない返戻
					return Reject(HasFiniteSource ? SpillDiscoveryFailure::BudgetLimit : SpillDiscoveryFailure::WindowFailed);
				}
				// 元場で繋がった接続群の返戻
				if(HasDiscoveredConnection()) return Batch;
			}
			// 中断又は共通予算を消費した場合の返戻
			if(Stop.stop_requested() || Report.Evaluations >= Settings.MaximumEvaluations) {
				// 次の窓で失敗を上書しない返戻
				return Reject(SpillDiscoveryFailure::BudgetLimit);
			}
			if(HalfWidth == Settings.MaximumHalfWidthMeters) break;
			HalfWidth = std::min(HalfWidth * 2, Settings.MaximumHalfWidthMeters);
		}
		// 有限範囲で確認出来なかった事を，閉鎖水域の確定にしない返戻
		return Reject(SpillDiscoveryFailure::NoIncidentExit);
	}

	/**
	 * 一つの出口だけを必要とする既存の照会を，接続群の準備へ委譲する
	 * @param Minima 入力の確認済水系
	 * @param Source 同じ不変な元場
	 * @param Settings 有限の探索・評価・保持予算
	 * @param Stop 中断要求
	 * @param Diagnostics 実際に行った接続群の準備費用
	 * @param CompletedConnections 同じ元場で準備済の入力集合内の接続
	 * @return 入力集合を跨ぐ最初の実接続；未発見時は空
	 */
	std::optional<SpillConnection> FindSpillComponentExit(
		const std::span<const TerrainCriticalPoint> Minima,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillDiscoverySettings &Settings,
		const std::stop_token Stop,
		SpillDiscoveryDiagnostics *const Diagnostics,
		const std::span<const SpillConnection> CompletedConnections
	) {
		// 単一照会は集合外の出口を探すので，内部の別通路だけで打切らない
		SpillDiscoverySettings ExternalSettings = Settings;
		ExternalSettings.IncludeInternalConnections = false;
		std::optional<SpillDiscoveryBatch> Batch =
		FindSpillComponentExits(Minima, Source, ExternalSettings, Stop, Diagnostics, CompletedConnections);
		// 準備失敗又は中断への返戻
		if(!Batch || Stop.stop_requested()) return {};
		for(SpillConnection &Connection : Batch->Connections) {
			const auto IsInput = [&](const SpillConnectionBranch &Branch) -> bool {
				// 海面終端の返戻
				if(!Branch.Minimum) return false;
				for(const TerrainCriticalPoint &Minimum : Minima) {
					// 入力の実極小との一致の返戻
					if(SameSpillCriticalPoint(*Branch.Minimum, Minimum, Settings.RadiusMeters)) return true;
				}
				// 入力集合外の返戻
				return false;
			};
			// 両終端で入力集合の境界を跨ぐ場合の返戻
			if(IsInput(Connection.Branches[0]) != IsInput(Connection.Branches[1])) return std::move(Connection);
		}
		// 新しい接続を持たない不整合な結果を公開しない返戻
		return {};
	}
}
