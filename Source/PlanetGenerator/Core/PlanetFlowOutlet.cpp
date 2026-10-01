#include "PlanetFlowOutlet.h"
#include "PlanetMinimumLink.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 鞍点の両側を同じ元場で辿り，隙間を残さず共有曲線へ繋ぐ
	 * @param Candidate 流出口候補の方向
	 * @param Source 変更しない元地形と解析勾配
	 * @param Settings 物理尺度と下流水位，下降する枝の向き
	 * @param Stop 中断要求
	 * @param Diagnostics 準備の拒否箇所と追跡終端，省略可
	 * @param StopDomainDistance 受入水域の符号付物理距離，省略時は比較用の高度終端
	 * @return 下流水位へ届いた経路と正確な鞍点距離，未達・不正入力は空
	 */
	std::optional<FlowOutletRoute> PrepareFlowOutletRoute(
		const UnitDirection Candidate,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const FlowOutletSettings &Settings,
		const std::stop_token Stop,
		FlowOutletDiagnostics *const Diagnostics,
		const std::function<double(UnitDirection)> &StopDomainDistance
	) {
		FlowOutletDiagnostics Local;
		FlowOutletDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowOutletFailure Failure) -> std::optional<FlowOutletRoute> {
			Report.Failure = Stop.stop_requested() ? FlowOutletFailure::Cancelled : Failure;
			// 不完全な経路を公開しない返戻
			return {};
		};
		const bool HasValidRadius =
		std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9;
		const bool HasValidLengths = std::isfinite(Settings.UpstreamLengthMeters) && Settings.UpstreamLengthMeters > 5 &&
		std::isfinite(Settings.MaximumDownstreamLengthMeters) && Settings.MaximumDownstreamLengthMeters > 5 &&
		std::isfinite(Settings.LaunchDistanceMeters) && Settings.LaunchDistanceMeters > 0.01 && !(5 < Settings.LaunchDistanceMeters);
		const bool CanPrepare =
		HasValidRadius && HasValidLengths && std::isfinite(Settings.DownstreamWaterMeters) && Source && !Stop.stop_requested();
		// 無効な尺度又は中断時の返戻
		if(!CanPrepare) return Reject(FlowOutletFailure::InvalidInput);
		TerrainCriticalSettings Critical;
		Critical.RadiusMeters = Settings.RadiusMeters;
		Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Settings.RadiusMeters * 0.125);
		std::optional<TerrainCriticalPoint> Pass = RefineTerrainCriticalPoint(Candidate, Source, Critical, Stop);
		if(!Pass || Pass->Kind != TerrainCriticalKind::Saddle || !(Pass->ElevationMeters > Settings.DownstreamWaterMeters)) {
			// 元の鞍点より高い下流水位を下降経路へ読み替えない返戻
			return Reject(FlowOutletFailure::Critical);
		}
		if(Settings.ReferenceMinimumDirection) {
			const Vec3 Up = Pass->Direction.Vector(), Reference = *Settings.ReferenceMinimumDirection;
			const std::optional<UnitDirection> Tangent = UnitDirection::FromVector(Reference - Up * Reference.Dot(Up));
			if(!Tangent || std::abs(Tangent->Vector().Dot(Pass->MinimumCurvatureDirection)) < 0.5) {
				// 同じ主軸に対応しない入力への返戻
				return Reject(FlowOutletFailure::InvalidInput);
			}
			if(Tangent->Vector().Dot(Pass->MinimumCurvatureDirection) < 0) {
				Pass->MinimumCurvatureDirection = Pass->MinimumCurvatureDirection * -1;
			}
		}
		// 鞍点直後の緩い斜面を，勾配だけで終端へ読み替えない
		const double TerminalGradientTolerance =
		std::min(Critical.GradientTolerance, std::abs(Pass->MinimumCurvaturePerMeter) * Settings.LaunchDistanceMeters * 0.25);
		const Vec3 Along = Pass->MinimumCurvatureDirection * (Settings.IsPositiveBranch ? 1.0 : -1.0);
		Report.DownstreamAxis = Along;
		const double Offset = Settings.LaunchDistanceMeters / Settings.RadiusMeters;
		const UnitDirection UpstreamStart =
		*UnitDirection::FromVector(Pass->Direction.Vector() * std::cos(Offset) - Along * std::sin(Offset));
		const UnitDirection DownstreamStart =
		*UnitDirection::FromVector(Pass->Direction.Vector() * std::cos(Offset) + Along * std::sin(Offset));
		TerrainDescentSettings UpstreamSettings;
		UpstreamSettings.RadiusMeters = Settings.RadiusMeters;
		UpstreamSettings.PositionToleranceMeters = 1E-5;
		UpstreamSettings.GradientTolerance = TerminalGradientTolerance;
		UpstreamSettings.StopElevationMeters = -std::numeric_limits<double>::max();
		UpstreamSettings.MaximumLengthMeters = std::min(Settings.UpstreamLengthMeters, Settings.RadiusMeters * 0.5);
		UpstreamSettings.MaximumStepMeters = std::min(UpstreamSettings.MaximumStepMeters, UpstreamSettings.MaximumLengthMeters);
		UpstreamSettings.MaximumVertices = 1024;
		std::optional<TerrainDescentPath> Upstream = TraceTerrainDescent(UpstreamStart, Source, UpstreamSettings, Stop);
		if(Upstream) {
			Report.UpstreamEnd = Upstream->End;
			Report.UpstreamVertices = static_cast<std::uint32_t>(Upstream->Samples.size());
			if(!Upstream->Samples.empty()) {
				const TerrainDescentSample &Last = Upstream->Samples.back();
				Report.UpstreamDirection = Last.Direction.Vector();
				Report.UpstreamDistanceMeters = Last.DistanceMeters;
				Report.UpstreamHeightMeters = Last.ElevationMeters;
			}
		}
		bool HasUpstream = Upstream && Upstream->Samples.size() > 1 &&
		(Upstream->End == TerrainDescentEnd::LengthLimit || Upstream->End == TerrainDescentEnd::GradientLimit);
		std::uint32_t TerminalEvaluations = 0;
		if(Upstream && Upstream->Samples.size() > 1 && Upstream->End == TerrainDescentEnd::StepLimit) {
			// 最小歩幅内の極小を実際に確認し，単なる積分失敗と区別する
			const TerrainDescentSample &Last = Upstream->Samples.back();
			TerrainCriticalSettings EndSettings = Critical;
			EndSettings.DifferenceStepMeters = UpstreamSettings.MinimumStepMeters * 0.5;
			EndSettings.MaximumStepMeters = UpstreamSettings.MinimumStepMeters;
			EndSettings.MaximumDisplacementMeters = UpstreamSettings.MinimumStepMeters;
			const std::optional<TerrainCriticalPoint> Minimum =
			RefineTerrainCriticalPoint(Last.Direction, Source, EndSettings, Stop, &Report.UpstreamCritical);
			TerminalEvaluations += Report.UpstreamCritical.Evaluations;
			if(Minimum) {
				const double Move = Settings.RadiusMeters * std::atan2(
					Last.Direction.Vector().Cross(Minimum->Direction.Vector()).Length(),
					Last.Direction.Vector().Dot(Minimum->Direction.Vector())
				);
				Report.UpstreamMinimumMoveMeters = Move;
				bool HasMinimum = Minimum->Kind == TerrainCriticalKind::Minimum && Move <= UpstreamSettings.MinimumStepMeters &&
				Minimum->ElevationMeters < Pass->ElevationMeters;
				const auto CountTerminal = [&](const UnitDirection D) -> TerrainSample {
					++TerminalEvaluations;
					// 上流の最終短区間も元の解析場で照合する返戻
					return Source(D);
				};
				if(HasMinimum) {
					const std::optional<TerrainMinimumLink> Link = PrepareTerrainMinimumLink(
						Upstream->Samples,
						*Minimum,
						Settings.RadiusMeters,
						UpstreamSettings.MinimumStepMeters,
						CountTerminal,
						Stop
					);
					HasMinimum = Link.has_value();
					if(Link) {
						Upstream->Samples.erase(
							Upstream->Samples.begin() + static_cast<std::ptrdiff_t>(Link->RetainedSamples),
							Upstream->Samples.end()
						);
						Upstream->Samples.push_back(Link->Anchor);
					}
				}
				// 湖側の表示用経路は確認した極小の直前迄を保持し，観測した停止理由を改変しない
				HasUpstream = Report.HasUpstreamMinimum = HasMinimum;
			}
		}
		// 不完全な湖側の経路を通常の接続としない返戻
		if(!HasUpstream) return Reject(FlowOutletFailure::Upstream);
		TerrainDescentSettings DownstreamSettings;
		DownstreamSettings.RadiusMeters = Settings.RadiusMeters;
		DownstreamSettings.PositionToleranceMeters = 1E-5;
		DownstreamSettings.GradientTolerance = TerminalGradientTolerance;
		DownstreamSettings.MaximumVertices = 8192;
		// 受入水域への侵入を要求し，浅い湖に存在しない固定１０㎝の深さを要求しない
		DownstreamSettings.StopElevationMeters =
		std::nextafter(Settings.DownstreamWaterMeters, -std::numeric_limits<double>::infinity());
		DownstreamSettings.MaximumLengthMeters = std::min(Settings.MaximumDownstreamLengthMeters, Settings.RadiusMeters * 0.5);
		DownstreamSettings.MaximumStepMeters = std::min(DownstreamSettings.MaximumStepMeters, DownstreamSettings.MaximumLengthMeters);
		const std::optional<TerrainDescentPath> Downstream =
		TraceTerrainDescent(DownstreamStart, Source, DownstreamSettings, Stop, StopDomainDistance);
		if(Downstream) {
			Report.DownstreamEnd = Downstream->End;
			Report.DownstreamVertices = static_cast<std::uint32_t>(Downstream->Samples.size());
		}
		if(!Downstream || Downstream->Samples.size() < 2 || Downstream->End != TerrainDescentEnd::HeightLimit) {
			// 受入側の水位まで届かない経路を途中で閉じない返戻
			return Reject(FlowOutletFailure::Downstream);
		}
		std::vector<FlowRouteKnot> Knots;
		Knots.reserve(Upstream->Samples.size() + Downstream->Samples.size() + 1);
		const double UpstreamLength = Upstream->Samples.back().DistanceMeters;
		for(std::size_t i = Upstream->Samples.size(); i-- > 0;) {
			const TerrainDescentSample &Point = Upstream->Samples[i];
			Knots.push_back(
				{ Point.Direction, Source(Point.Direction).TangentialGradientPerRadian, UpstreamLength - Point.DistanceMeters }
			);
		}
		const std::size_t PassIndex = Knots.size();
		const double PassDistance = UpstreamLength + Settings.LaunchDistanceMeters;
		Knots.push_back({ Pass->Direction, Along, PassDistance });
		for(const TerrainDescentSample &Point : Downstream->Samples) {
			Knots.push_back(
				{
					Point.Direction,
					Source(Point.Direction).TangentialGradientPerRadian * -1,
					PassDistance + Settings.LaunchDistanceMeters + Point.DistanceMeters
				}
			);
		}
		// 準備中に中断された場を公開しない返戻
		if(Stop.stop_requested()) return Reject(FlowOutletFailure::Cancelled);
		std::optional<TerrainFlowRoute> Route = TerrainFlowRoute::Create(Knots, Settings.RadiusMeters, &Report.Curve);
		// 鞍点前後を一つの曲線として作れない場合の返戻
		if(!Route) return Reject(FlowOutletFailure::Curve);
		double ParameterAtPass = 0;
		for(std::size_t i = 0; i < PassIndex; ++i) ParameterAtPass += Route->GetSegments()[i].GetParameterLengthMeters();
		const std::uint32_t Evaluations = Pass->Evaluations + Upstream->Evaluations + Downstream->Evaluations + TerminalEvaluations +
		static_cast<std::uint32_t>(Knots.size() - 1);
		// 元の追跡距離と公称距離を混同せず，鞍点の正確な区間境界を返す
		return FlowOutletRoute{ std::move(*Route), *Pass, ParameterAtPass, Upstream->End, Evaluations };
	}
}
