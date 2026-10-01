#include "PlanetCriticalPoint.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	namespace {
		/**
		 * 単位方向から直交する接線基底を選ぶ
		 * @param Direction 基準方向
		 * @return 右手系の２接線
		 */
		std::array<Vec3, 2> CriticalFrame(const Vec3 Direction) {
			// 方向とほぼ平行な基準軸を避ける処理
			const Vec3 Reference = std::abs(Direction.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
			const Vec3 East = UnitDirection::FromVector(Reference.Cross(Direction))->Vector();
			// 接線基底の返戻
			return { East, Direction.Cross(East) };
		}

		/**
		 * 接平面内の物理変位を球面へ戻す
		 * @param Center 基準方向
		 * @param Offset 接線上のメートル変位
		 * @param Radius 惑星半径
		 * @return 同じ球面上の正規方向
		 */
		UnitDirection CriticalOffset(const UnitDirection Center, const Vec3 Offset, const double Radius) {
			return *UnitDirection::FromVector(Center.Vector() + Offset * (1 / Radius));
		}
	}

	/**
	 * 局所探索の物理寸法，精度，反復予算を検査する
	 * @return 全て有限且つ対応範囲内の場合に真
	 */
	bool TerrainCriticalSettings::IsValid() const {
		// 過大な局所座標又は無制限な反復を許可しない返戻
		return std::isfinite(RadiusMeters) && !(RadiusMeters < 1000) && RadiusMeters <= 1E9 && std::isfinite(DifferenceStepMeters) &&
		DifferenceStepMeters > 0 && DifferenceStepMeters < RadiusMeters * 0.01 && std::isfinite(MaximumStepMeters) &&
		MaximumStepMeters >= DifferenceStepMeters && std::isfinite(MaximumDisplacementMeters) &&
		MaximumDisplacementMeters >= MaximumStepMeters && MaximumDisplacementMeters < RadiusMeters * 0.25 &&
		std::isfinite(GradientTolerance) && GradientTolerance > 0 && GradientTolerance < 0.01 &&
		std::isfinite(PositionToleranceMeters) && PositionToleranceMeters > 0 && PositionToleranceMeters < 0.01 &&
		MaximumIterations > 0 && MaximumIterations < 257;
	}

	/**
	 * 不変な連続場の勾配を零にする局所位置を求め，二段階差分で曲率の符号を確認する
	 * 計算量：O(I B) 回の場評価，I は反復上限，B は最大１２回の縮小探索を含む固定回数
	 * @param Initial 探索の初期方向
	 * @param Source 有限な標高と同じ場の解析接線勾配
	 * @param Settings 物理半径，差分寸法，局所範囲，精度と反復上限
	 * @param Stop 中断要求
	 * @param Diagnostics 元場照会を増やさず記録する収束位置・費用と拒否理由
	 * @return 局所の非退化な停留点，入力不正・範囲外・未収束・退化・中断時は空
	 */
	std::optional<TerrainCriticalPoint> RefineTerrainCriticalPoint(
		const UnitDirection Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainCriticalSettings &Settings,
		const std::stop_token Stop,
		TerrainCriticalDiagnostics *const Diagnostics
	) {
		TerrainCriticalDiagnostics Local;
		TerrainCriticalDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		Report.Direction = Initial.Vector();
		const auto Reject = [&](const TerrainCriticalFailure Failure) -> std::optional<TerrainCriticalPoint> {
			if(Stop.stop_requested()) Report.Failure = TerrainCriticalFailure::Cancelled;
			else if(Report.Failure == TerrainCriticalFailure::None) Report.Failure = Failure;
			// 未収束の位置や不安定な曲率を停留点として公開しない返戻
			return {};
		};
		// 不正な入口又は中断時の返戻
		if(!Source || !Settings.IsValid() || Stop.stop_requested()) return Reject(TerrainCriticalFailure::InvalidInput);
		// 探索中も変更しない元場の照会
		std::uint32_t &Evaluations = Report.Evaluations;
		const auto Sample = [&](const UnitDirection Direction) -> std::optional<TerrainSample> {
			// 中断時の返戻
			if(Stop.stop_requested()) {
				Report.Failure = TerrainCriticalFailure::Cancelled;
				// 中断後の追加照会を避ける返戻
				return {};
			}
			// 現在方向の元場の照会
			TerrainSample Value = Source(Direction);
			++Evaluations;
			// 元の解析勾配と有限性
			const Vec3 Gradient = Value.TangentialGradientPerRadian;
			// 非有限な場を追跡しない返戻
			if(Stop.stop_requested() || !std::isfinite(Value.ElevationMeters) || !std::isfinite(Gradient.Length())) {
				Report.Failure = Stop.stop_requested() ? TerrainCriticalFailure::Cancelled : TerrainCriticalFailure::InvalidSource;
				// 中断又は非有限な元場を収束成功にしない返戻
				return {};
			}
			Value.TangentialGradientPerRadian =
			(Gradient - Direction.Vector() * Gradient.Dot(Direction.Vector())) * (1 / Settings.RadiusMeters);
			// 物理勾配へ変換した値の返戻
			return Value;
		};
		// 探索中の局所位置
		UnitDirection Position = Initial;
		for(std::uint32_t Iteration = 0; Iteration < Settings.MaximumIterations; ++Iteration) {
			// 現在位置の物理勾配と接線基底
			const std::optional<TerrainSample> Center = Sample(Position);
			// 場を評価出来ない場合の返戻
			if(!Center) return Reject(TerrainCriticalFailure::InvalidSource);
			// 現在の接線基底と物理勾配
			const std::array<Vec3, 2> Frame = CriticalFrame(Position.Vector());
			const Vec3 Gradient = Center->TangentialGradientPerRadian;
			const double Norm = Gradient.Length(), X = Gradient.Dot(Frame[0]), Y = Gradient.Dot(Frame[1]);
			Report.Direction = Position.Vector();
			Report.ElevationMeters = Center->ElevationMeters;
			Report.GradientNorm = Norm;
			Report.Iterations = Iteration + 1;
			Report.DisplacementMeters =
			Settings.RadiusMeters * std::atan2(Initial.Vector().Cross(Position.Vector()).Length(), Initial.Vector().Dot(Position.Vector()));
			// 同じ中心基底へ射影した勾配の中央差分
			const auto Hessian = [&](const double Step) -> std::optional<std::array<double, 3>> {
				// 接線２方向の対称な差分試料
				const std::optional<TerrainSample> East = Sample(CriticalOffset(Position, Frame[0] * Step, Settings.RadiusMeters));
				const std::optional<TerrainSample> West = Sample(CriticalOffset(Position, Frame[0] * -Step, Settings.RadiusMeters));
				const std::optional<TerrainSample> North = Sample(CriticalOffset(Position, Frame[1] * Step, Settings.RadiusMeters));
				const std::optional<TerrainSample> South = Sample(CriticalOffset(Position, Frame[1] * -Step, Settings.RadiusMeters));
				// 差分の途中で評価出来ない場合の返戻
				if(!East || !West || !North || !South) return {};
				// 現在の基底へ揃えた勾配差
				const Vec3 Across = (East->TangentialGradientPerRadian - West->TangentialGradientPerRadian) * (0.5 / Step);
				const Vec3 Along = (North->TangentialGradientPerRadian - South->TangentialGradientPerRadian) * (0.5 / Step);
				// 対称化した球面上の二階微分の返戻
				return std::array<double, 3>{ Across.Dot(Frame[0]), (Across.Dot(Frame[1]) + Along.Dot(Frame[0])) * 0.5, Along.Dot(Frame[1]) };
			};
			const std::optional<std::array<double, 3>> Matrix = Hessian(Settings.DifferenceStepMeters);
			// 二階微分を取得出来ない場合の返戻
			if(!Matrix) return Reject(TerrainCriticalFailure::InvalidSource);
			// 曲率行列と可逆性の尺度
			const double XX = (*Matrix)[0], XY = (*Matrix)[1], YY = (*Matrix)[2];
			const double Scale = std::max({ std::abs(XX), std::abs(XY), std::abs(YY) }), Determinant = XX * YY - XY * XY;
			// 退化した停留線又は不安定な逆行列を採用しない返戻
			if(!std::isfinite(Scale) || !std::isfinite(Determinant) || !Scale || std::abs(Determinant) < 1E-12 * Scale * Scale) {
				// 退化した曲率を実停留点へ読み替えない返戻
				return Reject(TerrainCriticalFailure::Degenerate);
			}
			const double CoarseHalfTrace = (XX + YY) * 0.5, CoarseDelta = std::hypot((XX - YY) * 0.5, XY);
			Report.MinimumCurvaturePerMeter = CoarseHalfTrace - CoarseDelta;
			Report.MaximumCurvaturePerMeter = CoarseHalfTrace + CoarseDelta;
			// 緩い曲率では勾配だけで位置精度を保証出来ない為，未制限の Newton 変位も確認する
			Vec3 Offset = Frame[0] * ((XY * Y - YY * X) / Determinant) + Frame[1] * ((XY * X - XX * Y) / Determinant);
			const double Length = Offset.Length();
			Report.NewtonStepMeters = Length;
			if(!Iteration) Report.InitialNewtonStepMeters = Length;
			// 非有限な位置更新を収束として受理しない返戻
			if(!std::isfinite(Length)) return Reject(TerrainCriticalFailure::NonFiniteStep);
			if(Norm <= Settings.GradientTolerance && Length <= Settings.PositionToleranceMeters) {
				// 刻み幅を半分にした独立な曲率安定性
				const std::optional<std::array<double, 3>> Fine = Hessian(Settings.DifferenceStepMeters * 0.5);
				// 曲率を再評価出来ない場合の返戻
				if(!Fine) return Reject(TerrainCriticalFailure::InvalidSource);
				// 半幅の曲率と固有値の符号安定性
				const double FineXX = (*Fine)[0], FineXY = (*Fine)[1], FineYY = (*Fine)[2], HalfTrace = (FineXX + FineYY) * 0.5;
				const double Delta = std::hypot((FineXX - FineYY) * 0.5, FineXY), Minimum = HalfTrace - Delta, Maximum = HalfTrace + Delta;
				const double Difference = std::max({ std::abs(XX - FineXX), std::abs(XY - FineXY), std::abs(YY - FineYY) });
				const bool HasStableCurvature =
				std::isfinite(Minimum) && std::isfinite(Maximum) && 2 * Difference < 0.1 * std::min(std::abs(Minimum), std::abs(Maximum));
				Report.MinimumCurvaturePerMeter = Minimum;
				Report.MaximumCurvaturePerMeter = Maximum;
				Report.CurvatureDifferencePerMeter = Difference;
				// 差分の誤差で曲率の符号が変わり得る場合の返戻
				if(!HasStableCurvature) return Reject(TerrainCriticalFailure::UnstableCurvature);
				// 最小固有値に対応する接線方向
				double AxisX = FineXY, AxisY = Minimum - FineXX;
				if(std::hypot(AxisX, AxisY) < 1E-12 * Scale) {
					AxisX = Minimum - FineYY;
					AxisY = FineXY;
				}
				if(!AxisX && !AxisY) AxisX = 1;
				// 世界座標へ戻した主方向と停留点の分類
				const Vec3 Principal = UnitDirection::FromVector(Frame[0] * AxisX + Frame[1] * AxisY)->Vector();
				const TerrainCriticalKind Kind =
				Minimum > 0 ? TerrainCriticalKind::Minimum : Maximum < 0 ? TerrainCriticalKind::Maximum : TerrainCriticalKind::Saddle;
				Report.AcceptedKind = Kind;
				// 精査した元場上の停留点の返戻
				return TerrainCriticalPoint{
					Position,
					Center->ElevationMeters,
					Norm,
					Minimum,
					Maximum,
					Principal,
					Kind,
					Evaluations,
					Iteration + 1
				};
			}
			// 有効な更新方向が無い場合の返戻
			if(!Length) return Reject(TerrainCriticalFailure::NoProgress);
			// 収束判定後に，勾配の零点へ向かう変位の歩幅を制限
			if(Length > Settings.MaximumStepMeters) Offset = Offset * (Settings.MaximumStepMeters / Length);
			// 物理勾配の残差を減らす候補だけを受理
			bool HasAccepted = false, HasCandidateInside = false;
			for(unsigned Trial = 0; Trial < 12; ++Trial) {
				// 範囲制限に照合する候補位置
				const UnitDirection Candidate = CriticalOffset(Position, Offset, Settings.RadiusMeters);
				const double Displacement = Settings.RadiusMeters *
				std::atan2(Initial.Vector().Cross(Candidate.Vector()).Length(), Initial.Vector().Dot(Candidate.Vector()));
				if(Displacement <= Settings.MaximumDisplacementMeters) {
					HasCandidateInside = true;
					// 候補位置での独立な元場の照会
					const std::optional<TerrainSample> Next = Sample(Candidate);
					// 更新先で場を評価出来ない場合の返戻
					if(!Next) return Reject(TerrainCriticalFailure::InvalidSource);
					if(Next->TangentialGradientPerRadian.Length() < Norm) {
						Position = Candidate;
						HasAccepted = true;
						break;
					}
				}
				Offset = Offset * 0.5;
			}
			// 局所範囲で勾配残差を減らせない場合の返戻
			if(!HasAccepted) return Reject(HasCandidateInside ? TerrainCriticalFailure::NoProgress : TerrainCriticalFailure::Range);
		}
		// 反復上限迄に停留点へ収束しなかった場合の返戻
		return Reject(TerrainCriticalFailure::IterationLimit);
	}
}
