#include "PlanetSpillConnection.h"
#include "PlanetMinimumLink.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace PlanetCore {
	/**
	 * 元地形の鞍点の両側を辿り，実際の極小又は海面高度への接続を確定する
	 * @param Candidate 元場の鞍点付近の方向
	 * @param Source 変更しない標高と解析接線勾配
	 * @param Settings 物理尺度と各枝の計算予算
	 * @param Stop 中断要求
	 * @param Diagnostics 部分経路を受理出来ない理由の任意出力
	 * @return 両枝の終端を確認した接続；未確定の枝を含む候補は空
	 */
	std::optional<SpillConnection> PrepareSpillConnection(
		const UnitDirection Candidate,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillConnectionSettings &Settings,
		const std::stop_token Stop,
		SpillConnectionDiagnostics *const Diagnostics
	) {
		// 診断の初期化と失敗の分類
		if(Diagnostics) *Diagnostics = {};
		const auto Reject = [&](const SpillConnectionFailure Failure) -> std::optional<SpillConnection> {
			if(Diagnostics) Diagnostics->Failure = Stop.stop_requested() ? SpillConnectionFailure::Cancelled : Failure;
			// 不完全な接続を確定した水系として公開しない返戻
			return {};
		};
		// 入力の有限性と計算範囲
		const bool HasValidInput =
		Source && std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 &&
		std::isfinite(Settings.MaximumBranchLengthMeters) && Settings.MaximumBranchLengthMeters > 5.001 &&
		Settings.MaximumBranchVertices > 1 && Settings.MaximumBranchVertices < 65537 &&
		std::isfinite(Settings.TerminalPositionToleranceMeters) && Settings.TerminalPositionToleranceMeters >= 0.001 &&
		Settings.TerminalPositionToleranceMeters <= 0.1 && std::isfinite(Settings.SeaLevelMeters);
		// 無効な依存場又は尺度への返戻
		if(!HasValidInput) return Reject(SpillConnectionFailure::InvalidInput);
		// 開始前の中断を区別する返戻
		if(Stop.stop_requested()) return Reject(SpillConnectionFailure::Cancelled);
		// 元場の評価回数の集計
		std::uint64_t Evaluations = 0;
		const auto Sample = [&](const UnitDirection Direction) -> TerrainSample {
			++Evaluations;
			// 元場を変更せず，全準備費用を集計した返戻
			return Source(Direction);
		};
		// 元場の鞍点の精査
		TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Settings.RadiusMeters;
		Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Settings.RadiusMeters * 0.125);
		const std::optional<TerrainCriticalPoint> Pass = RefineTerrainCriticalPoint(Candidate, Sample, Critical, Stop);
		const bool HasLandSaddle = Pass && Pass->Kind == TerrainCriticalKind::Saddle && Pass->ElevationMeters > Settings.SeaLevelMeters;
		// 極小又は水面下の峠を陸上の流出口へ読み替えない返戻
		if(!HasLandSaddle) return Reject(SpillConnectionFailure::SaddleNotFound);
		// 両側の下降経路の準備
		SpillConnection Connection { *Pass };
		for(unsigned Side = 0; Side < 2; ++Side) {
			if(Diagnostics) {
				Diagnostics->Branch = Side;
				Diagnostics->TerminalMoveMeters = 0;
				Diagnostics->TerminalCritical = {};
			}
			const Vec3 Along = Pass->MinimumCurvatureDirection * (Side ? 1.0 : -1.0);
			const double InitialAngle = 5 / Settings.RadiusMeters;
			const UnitDirection Start =
			*UnitDirection::FromVector(Pass->Direction.Vector() * std::cos(InitialAngle) + Along * std::sin(InitialAngle));
			// 出発する短い弧も，同じ元場で下降する事を独立に検査する
			double PreviousHeight = Pass->ElevationMeters;
			for(unsigned Step = 1; Step < 5; ++Step) {
				const double Angle = InitialAngle * Step * 0.25;
				const UnitDirection D = *UnitDirection::FromVector(Pass->Direction.Vector() * std::cos(Angle) + Along * std::sin(Angle));
				const TerrainSample Value = Sample(D);
				const bool IsDescending = std::isfinite(Value.ElevationMeters) && Value.ElevationMeters <= PreviousHeight &&
				std::isfinite(Value.TangentialGradientPerRadian.Length()) && Value.TangentialGradientPerRadian.Dot(Along) < 0;
				// 誤った枝への初期移動を通常の下降としない返戻
				if(!IsDescending) return Reject(SpillConnectionFailure::InvalidDeparture);
				PreviousHeight = Value.ElevationMeters;
			}
			// 両枝の有界な下降追跡
			TerrainDescentSettings Flow;
			Flow.RadiusMeters = Settings.RadiusMeters;
			Flow.StopElevationMeters = Settings.SeaLevelMeters;
			Flow.MaximumLengthMeters = std::min(Settings.MaximumBranchLengthMeters, Settings.RadiusMeters * 0.5) - 5;
			Flow.MaximumStepMeters = std::min(Flow.MaximumStepMeters, Flow.MaximumLengthMeters);
			Flow.MaximumVertices = Settings.MaximumBranchVertices;
			Flow.PositionToleranceMeters = 1E-5;
			std::optional<TerrainDescentPath> Path = TraceTerrainDescent(Start, Sample, Flow, Stop);
			// 元場の欠落又は中断時の返戻
			if(!Path || Path->Samples.empty()) return Reject(SpillConnectionFailure::TraceFailed);
			// 勾配の小ささだけでは極小迄の距離を保証しない
			TerrainCriticalSettings EndSettings = Critical;
			EndSettings.DifferenceStepMeters = Settings.TerminalPositionToleranceMeters * 0.5;
			EndSettings.MaximumStepMeters = Settings.TerminalPositionToleranceMeters;
			EndSettings.MaximumDisplacementMeters = Settings.TerminalPositionToleranceMeters;
			std::optional<TerrainCriticalPoint> End;
			for(unsigned Attempt = 0; Attempt < 2; ++Attempt) {
				if(Diagnostics) Diagnostics->TraceEnd = Path->End;
				if(Path->End != TerrainDescentEnd::GradientLimit && Path->End != TerrainDescentEnd::StepLimit) break;
				// 終点近傍の極小を確認出来た時だけ下降を完了する
				const TerrainDescentSample Last = Path->Samples.back();
				TerrainCriticalDiagnostics EndReport;
				End = RefineTerrainCriticalPoint(Last.Direction, Sample, EndSettings, Stop, &EndReport);
				if(Diagnostics) Diagnostics->TerminalCritical = EndReport;
				if(End || Path->End != TerrainDescentEnd::GradientLimit || Attempt) break;
				// 未解決の緩い斜面では，残りの距離と頂点予算内で下降を続ける
				const double Remaining = Flow.MaximumLengthMeters - Last.DistanceMeters;
				if(Remaining < Flow.MinimumStepMeters || Path->Samples.size() >= Flow.MaximumVertices) break;
				TerrainDescentSettings Continuation = Flow;
				// 勾配の小ささだけで停留位置を確定せず，測定した正の曲率で位置精度に合わせる
				Continuation.GradientTolerance = EndReport.MinimumCurvaturePerMeter > 0 ?
				std::min(Critical.GradientTolerance, EndReport.MinimumCurvaturePerMeter * EndSettings.MaximumDisplacementMeters * 0.25) :
				Critical.GradientTolerance;
				Continuation.MaximumLengthMeters = Remaining;
				Continuation.MaximumStepMeters = std::min(Flow.MaximumStepMeters, Remaining);
				Continuation.MaximumVertices -= static_cast<std::uint32_t>(Path->Samples.size() - 1);
				std::optional<TerrainDescentPath> Tail = TraceTerrainDescent(Last.Direction, Sample, Continuation, Stop);
				// 元場又は中断に因る失敗を接続として受理しない返戻
				if(!Tail || Tail->Samples.empty()) return Reject(SpillConnectionFailure::TraceFailed);
				for(std::size_t i = 1; i < Tail->Samples.size(); ++i) {
					TerrainDescentSample Point = Tail->Samples[i];
					Point.DistanceMeters += Last.DistanceMeters;
					Path->Samples.push_back(Point);
				}
				Path->Evaluations += Tail->Evaluations;
				Path->RejectedSteps += Tail->RejectedSteps;
				Path->MaximumEstimatedErrorMeters = std::max(Path->MaximumEstimatedErrorMeters, Tail->MaximumEstimatedErrorMeters);
				Path->End = Tail->End;
			}
			// 追跡理由と終端の分類
			if(Diagnostics) {
				Diagnostics->TraceEnd = Path->End;
				Diagnostics->TerminalDirection = Path->Samples.back().Direction.Vector();
				Diagnostics->TerminalHeightMeters = Path->Samples.back().ElevationMeters;
				Diagnostics->TraceLengthMeters = Path->Samples.back().DistanceMeters;
			}
			SpillConnectionBranch &Branch = Connection.Branches[Side];
			if(Path->End == TerrainDescentEnd::HeightLimit) Branch.Kind = SpillTerminalKind::SeaLevel;
			else {
				// 停止点付近の極小の確認
				const bool CanResolveMinimum = Path->End == TerrainDescentEnd::GradientLimit || Path->End == TerrainDescentEnd::StepLimit;
				// 距離又は頂点予算で止まった枝を水域の終端として採用しない返戻
				if(!CanResolveMinimum) return Reject(SpillConnectionFailure::UnresolvedTerminal);
				// 停止点からの局所精査
				const TerrainDescentSample &Last = Path->Samples.back();
				// 別の停留点を極小として併合しない返戻
				if(!End || End->Kind != TerrainCriticalKind::Minimum) return Reject(SpillConnectionFailure::UnresolvedTerminal);
				// 補正前後の物理距離の照合
				const double Move = Settings.RadiusMeters * std::atan2(
					Last.Direction.Vector().Cross(End->Direction.Vector()).Length(),
					Last.Direction.Vector().Dot(End->Direction.Vector())
				);
				if(Diagnostics) Diagnostics->TerminalMoveMeters = Move;
				if(Move > Settings.TerminalPositionToleranceMeters || !(End->ElevationMeters < Pass->ElevationMeters)) {
					// 確認範囲外の停留点を受理しない返戻
					return Reject(SpillConnectionFailure::UnresolvedTerminal);
				}
				// 停留点直前の微小な標高差は，同じ元場の解析微分でも検査する
				const std::optional<TerrainMinimumLink> Link =
				PrepareTerrainMinimumLink(Path->Samples, *End, Settings.RadiusMeters, Settings.TerminalPositionToleranceMeters, Sample, Stop);
				// 実標高と解析微分の下降を確認出来ない末尾への返戻
				if(!Link) return Reject(SpillConnectionFailure::UnresolvedTerminal);
				Path->Samples.erase(Path->Samples.begin() + static_cast<std::ptrdiff_t>(Link->RetainedSamples), Path->Samples.end());
				Path->Samples.push_back(Link->Anchor);
				const double LinkLength = Settings.RadiusMeters * std::atan2(
					Link->Anchor.Direction.Vector().Cross(End->Direction.Vector()).Length(),
					Link->Anchor.Direction.Vector().Dot(End->Direction.Vector())
				);
				if(LinkLength > 0) Path->Samples.push_back({ End->Direction, End->ElevationMeters, Link->Anchor.DistanceMeters + LinkLength });
				Branch.Minimum = End;
			}
			// 鞍点を含む公開経路への変換
			for(TerrainDescentSample &Point : Path->Samples) Point.DistanceMeters += 5;
			Path->Samples.insert(Path->Samples.begin(), { Pass->Direction, Pass->ElevationMeters, 0 });
			// 初期弧と終端補正も含めた，公開する枝の予算
			const bool FitsBudget = Path->Samples.size() <= Settings.MaximumBranchVertices &&
			Path->Samples.back().DistanceMeters <= Settings.MaximumBranchLengthMeters;
			// 追跡本体だけの予算で公開全体を受理しない返戻
			if(!FitsBudget) return Reject(SpillConnectionFailure::UnresolvedTerminal);
			Branch.Path = std::move(*Path);
		}
		// 中断後の準備結果を受理しない返戻
		if(Stop.stop_requested()) return Reject(SpillConnectionFailure::Cancelled);
		Connection.Evaluations = Evaluations;
		// 包含又は下流順序をまだ付けない，両側の物理的な接続の返戻
		return Connection;
	}
}
