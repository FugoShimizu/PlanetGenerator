#include "PlanetSpillSeed.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace PlanetCore {
	/**
	 * 既知の峠を指定せず，陸上地点から下降した実盆地の流出口を探す
	 * @param Location 同じ元場の出発地点
	 * @param Source 変更しない連続標高と接線勾配
	 * @param Settings 物理尺度・海面・全段階の有限評価予算
	 * @param Stop 中断要求
	 * @param Diagnostics 到達段階と元場を実際に呼出した回数
	 * @return 入力地点の盆地に接続する実鞍点；大洋所属は別途水系準備で検証
	 */
	std::optional<SpillSeed> FindSpillSeed(
		const UnitDirection Location,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillSeedSettings &Settings,
		const std::stop_token Stop,
		SpillSeedDiagnostics *const Diagnostics
	) {
		SpillSeedDiagnostics Local;
		SpillSeedDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		bool IsFinite = true, IsWithinBudget = true;
		const auto Reject = [&](const SpillSeedFailure Reason) -> std::optional<SpillSeed> {
			Report.Failure = Stop.stop_requested() ?
			SpillSeedFailure::Cancelled :
			!IsWithinBudget || Reason != SpillSeedFailure::InvalidInput && Report.Evaluations >= Settings.MaximumEvaluations ?
			SpillSeedFailure::BudgetLimit :
			!IsFinite ? SpillSeedFailure::InvalidSource : Reason;
			// 部分的な下降路を確認済の流出口へ昇格しない返戻
			return {};
		};
		const bool Valid =
		Source && std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 &&
		std::isfinite(Settings.SeaLevelMeters) && Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 100000001;
		// 不正入力又は中断後に元場を呼出さない返戻
		if(!Valid || Stop.stop_requested()) return Reject(SpillSeedFailure::InvalidInput);
		const auto Sample = [&](const UnitDirection Direction) -> TerrainSample {
			if(Stop.stop_requested() || Report.Evaluations >= Settings.MaximumEvaluations) {
				IsWithinBudget = false;
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 共通予算外で元場を評価しない返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Report.Evaluations;
			const TerrainSample Value = Source(Direction);
			IsFinite = IsFinite && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			// 全準備段階で同じ不変な場を使用した返戻
			return Value;
		};
		TerrainDescentSettings Descent;
		Descent.RadiusMeters = Settings.RadiusMeters;
		Descent.StopElevationMeters = Settings.SeaLevelMeters;
		Descent.MaximumLengthMeters = std::min(64000.0, Settings.RadiusMeters * 0.5);
		Descent.MaximumVertices = 16384;
		Descent.PositionToleranceMeters = 1E-5;
		// 勾配停止は位置精度を保証しない為，未収束時は残りの予算で下降を継続
		Descent.GradientTolerance = 1E-8;
		std::optional<TerrainDescentPath> Path = TraceTerrainDescent(Location, Sample, Descent, Stop);
		// 場の欠落・中断又は未完成の経路への返戻
		if(!Path || Path->Samples.empty()) return Reject(SpillSeedFailure::Descent);
		TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Settings.RadiusMeters;
		Critical.DifferenceStepMeters = 0.005;
		Critical.MaximumStepMeters = 0.01;
		Critical.MaximumDisplacementMeters = 0.01;
		std::optional<TerrainCriticalPoint> Minimum;
		TerrainDescentSettings RemainingFlow = Descent;
		double TracedDistanceMeters = 0;
		std::uint32_t UsedVertices = 0;
		for(unsigned Attempt = 0; Attempt < 4; ++Attempt) {
			const TerrainDescentSample Last = Path->Samples.back();
			TracedDistanceMeters += Last.DistanceMeters;
			UsedVertices += static_cast<std::uint32_t>(Path->Samples.size() - 1);
			Report.DescentEnd = Path->End;
			Report.DescentEndpoint = Last;
			Report.DescentEndpoint->DistanceMeters = TracedDistanceMeters;
			// 直接海面へ降る地点へ架空の湖や峠を作らない返戻
			if(Path->End == TerrainDescentEnd::HeightLimit) return Reject(SpillSeedFailure::ReachesSeaLevel);
			const bool CanRefine = Path->End == TerrainDescentEnd::GradientLimit || Path->End == TerrainDescentEnd::StepLimit;
			// 長さ又は頂点予算で停止した地点を盆地として公開しない返戻
			if(!CanRefine) return Reject(SpillSeedFailure::Descent);
			Minimum = RefineTerrainCriticalPoint(Last.Direction, Sample, Critical, Stop, &Report.Critical);
			if(!Attempt) Report.InitialCritical = Report.Critical;
			if(Minimum && Minimum->Kind == TerrainCriticalKind::Minimum) break;
			const double Curvature = Report.Critical.MinimumCurvaturePerMeter;
			const bool CanContinue =
			!Minimum && Path->End == TerrainDescentEnd::GradientLimit && Report.Critical.Failure == TerrainCriticalFailure::Range &&
			Curvature > 0 && Attempt < 3;
			if(!CanContinue) break;
			// 緩い実極小では，物理曲率と同じ位置精度に整合した小さい勾配閾値で再追跡
			const double Tolerance = std::min(RemainingFlow.GradientTolerance, Curvature * Critical.MaximumDisplacementMeters * 0.25);
			const double Remaining = Descent.MaximumLengthMeters - TracedDistanceMeters;
			if(
				!(Tolerance > 0) || !(Tolerance < RemainingFlow.GradientTolerance) || Remaining < Descent.MinimumStepMeters ||
				UsedVertices >= Descent.MaximumVertices - 1
			) break;
			RemainingFlow.GradientTolerance = Tolerance;
			RemainingFlow.MaximumLengthMeters = Remaining;
			RemainingFlow.MaximumStepMeters = std::min(Descent.MaximumStepMeters, Remaining);
			RemainingFlow.MaximumVertices = Descent.MaximumVertices - UsedVertices;
			Path = TraceTerrainDescent(Last.Direction, Sample, RemainingFlow, Stop);
			++Report.Continuations;
			// 再追跡の場欠落・中断又は共通予算超過を成功にしない返戻
			if(!Path || Path->Samples.empty()) return Reject(SpillSeedFailure::Descent);
		}
		// 分水界の鞍点又は平坦な未確定領域を極小としない返戻
		if(!Minimum || Minimum->Kind != TerrainCriticalKind::Minimum) return Reject(SpillSeedFailure::Minimum);
		SpillDiscoverySettings Discovery;
		Discovery.RadiusMeters = Settings.RadiusMeters;
		Discovery.SeaLevelMeters = Settings.SeaLevelMeters;
		Discovery.InitialHalfWidthMeters = std::min(256.0, Settings.RadiusMeters * 0.0625);
		Discovery.MaximumHalfWidthMeters = std::min(262144.0, Settings.RadiusMeters * 0.25);
		Discovery.MaximumEvaluations = Settings.MaximumEvaluations;
		const std::array<TerrainCriticalPoint, 1> Minima { *Minimum };
		std::optional<SpillConnection> Connection = FindSpillComponentExit(Minima, Sample, Discovery, Stop, &Report.Discovery);
		// 同じ盆地への実接続を得られない場合の返戻
		if(!Connection || !IsFinite || !IsWithinBudget || Stop.stop_requested()) return Reject(SpillSeedFailure::Exit);
		// 大洋への所属は後続の水系準備へ委ねる実接続の返戻
		return SpillSeed{ *Minimum, std::move(*Connection) };
	}
}
