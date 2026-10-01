#include "PlanetFlowWaterProfile.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <utility>

namespace PlanetCore {
	/**
	 * 端点の値と共有微分から同じ三次式の水位・距離微分を返す
	 * @param A 上流節点
	 * @param B 下流節点
	 * @param DistanceMeters 両端の間の公称距離
	 * @return 節点では値を正確に保持した水位と同じ曲線の微分
	 */
	FlowChannelWaterLevel FlowWaterProfile::Interpolate(const Knot &A, const Knot &B, const double DistanceMeters) {
		// 接続点を別の演算順で丸めない返戻
		if(DistanceMeters == A.DistanceMeters) return { A.Source.ElevationMeters, A.Slope };
		// 下流節点の値も正確に保つ返戻
		if(DistanceMeters == B.DistanceMeters) return { B.Source.ElevationMeters, B.Slope };
		const double Length = B.DistanceMeters - A.DistanceMeters, T = (DistanceMeters - A.DistanceMeters) / Length, U = 1 - T;
		const double Difference = B.Source.ElevationMeters - A.Source.ElevationMeters;
		const double Middle = 3 * (Difference / Length) - (A.Slope + B.Slope);
		const double Height =
		A.Source.ElevationMeters + Difference * T * T * (3 - 2 * T) + Length * T * U * (A.Slope * U - B.Slope * T);
		const double Derivative = A.Slope * U * U + 2 * Middle * T * U + B.Slope * T * T;
		// Bernstein 基底の非正係数で下降する水位の返戻
		return { Height, Derivative };
	}

	/**
	 * 元水位の値・微分を標本化し，共有微分の単調条件と誤差を満たす迄局所分割する
	 * @param Source 準備中に不変な元水位と距離微分
	 * @param InitialDistances 零から始まる厳密増加列，最低２点
	 * @param Settings 正の誤差許容値と有限な準備予算
	 * @param Stop 中断要求
	 * @param Diagnostics 失敗段階と最後の検査誤差，省略可
	 * @return 元場を所有しない不変な近似；不正・逆勾配・予算超過では空
	 * @note 誤差は節点と各区間の３内部標本で確認する；任意の元関数の全点誤差の証明ではない
	 */
	std::optional<FlowWaterProfile> FlowWaterProfile::Create(
		const std::function<std::optional<FlowChannelWaterLevel>(double)> &Source,
		const std::span<const double> InitialDistances,
		const FlowWaterProfileSettings &Settings,
		const std::stop_token Stop,
		FlowWaterProfileDiagnostics *const Diagnostics
	) {
		FlowWaterProfileDiagnostics Local;
		FlowWaterProfileDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowWaterProfileFailure Failure) -> std::optional<FlowWaterProfile> {
			Report.Failure = Stop.stop_requested() ? FlowWaterProfileFailure::Cancelled : Failure;
			// 部分的な近似を公開しない返戻
			return {};
		};
		const bool Valid = Source && InitialDistances.size() > 1 && Settings.MaximumKnots > 1 && Settings.MaximumKnots < 65537 &&
		InitialDistances.size() <= Settings.MaximumKnots && Settings.MaximumPasses > 0 && Settings.MaximumPasses < 65 &&
		std::isfinite(Settings.HeightToleranceMeters) && Settings.HeightToleranceMeters > 0 && std::isfinite(Settings.SlopeTolerance) &&
		Settings.SlopeTolerance > 0;
		// 非有限又は無制限な入力への返戻
		if(!Valid || InitialDistances.front() || Stop.stop_requested()) return Reject(FlowWaterProfileFailure::Input);
		for(std::size_t I = 0; I < InitialDistances.size(); ++I) {
			if(!std::isfinite(InitialDistances[I]) || I && !(InitialDistances[I] > InitialDistances[I - 1])) {
				// 範囲外の節点を黙って並換えない返戻
				return Reject(FlowWaterProfileFailure::Input);
			}
		}
		std::map<double, FlowChannelWaterLevel> Samples;
		const auto Evaluate = [&](const double Distance) -> std::optional<FlowChannelWaterLevel> {
			// 準備済の同一距離の標本への返戻
			if(const std::map<double, FlowChannelWaterLevel>::const_iterator Found = Samples.find(Distance); Found != Samples.end()) {
				// 同じ元場の値を再照会しない返戻
				return Found->second;
			}
			// 元場の呼出しにも有限な保持予算を課す返戻
			if(Samples.size() >= 4 * Settings.MaximumKnots || Stop.stop_requested()) {
				Report.Failure = Stop.stop_requested() ? FlowWaterProfileFailure::Cancelled : FlowWaterProfileFailure::Budget;
				// 元場の失敗と保持量の不足を区別する返戻
				return {};
			}
			const std::optional<FlowChannelWaterLevel> Value = Source(Distance);
			++Report.Evaluations;
			// 非有限な元水位を一定面へ置換えない返戻
			if(!Value || !std::isfinite(Value->ElevationMeters) || !std::isfinite(Value->DerivativePerMeter)) {
				Report.Failure = FlowWaterProfileFailure::Source;
				// 元場の欠落と非有限値の返戻
				return {};
			}
			Samples.emplace(Distance, *Value);
			// 同じ節点の元水位の返戻
			return Value;
		};
		FlowWaterProfile Result;
		for(const double Distance : InitialDistances) {
			const std::optional<FlowChannelWaterLevel> Value = Evaluate(Distance);
			// 欠落した初期標本への返戻
			if(!Value) return Reject(Report.Failure);
			Result.Knots.push_back({ Distance, *Value, Value->DerivativePerMeter });
		}
		for(std::uint32_t Pass = 0; Pass < Settings.MaximumPasses; ++Pass) {
			Report.Passes = Pass + 1;
			Report.Knots = static_cast<std::uint32_t>(Result.Knots.size());
			Report.HeightErrorMeters = Report.SlopeError = 0;
			for(Knot &Point : Result.Knots) {
				// 正の元微分を補間で隠さない返戻
				if(Point.Source.DerivativePerMeter > 0) return Reject(FlowWaterProfileFailure::Rising);
				Point.Slope = Point.Source.DerivativePerMeter;
			}
			for(std::size_t I = 1; I < Result.Knots.size(); ++I) {
				Knot &A = Result.Knots[I - 1], &B = Result.Knots[I];
				const double Secant = (B.Source.ElevationMeters - A.Source.ElevationMeters) / (B.DistanceMeters - A.DistanceMeters);
				// 上昇する標本や演算範囲外を受理しない返戻
				if(!std::isfinite(Secant) || Secant > 0) return Reject(FlowWaterProfileFailure::Rising);
				// 両側区間が同じ節点微分を制限し，各区間の中間微分係数を非正に保つ
				A.Slope = std::max(A.Slope, 1.5 * Secant);
				B.Slope = std::max(B.Slope, 1.5 * Secant);
			}
			std::vector<Knot> Refined;
			Refined.reserve(Result.Knots.size());
			Refined.push_back(Result.Knots.front());
			bool Changed = false;
			for(std::size_t I = 1; I < Result.Knots.size(); ++I) {
				const Knot &A = Result.Knots[I - 1], &B = Result.Knots[I];
				// 係数の構成中にも中断へ応答する返戻
				if(Stop.stop_requested()) return Reject(FlowWaterProfileFailure::Cancelled);
				double HeightError = 0;
				double SlopeError = std::max(std::abs(A.Slope - A.Source.DerivativePerMeter), std::abs(B.Slope - B.Source.DerivativePerMeter));
				std::optional<FlowChannelWaterLevel> Middle;
				for(const double Fraction : { 0.25, 0.5, 0.75 }) {
					const double Distance =
					Fraction == 0.5 ? std::midpoint(A.DistanceMeters, B.DistanceMeters) : std::lerp(A.DistanceMeters, B.DistanceMeters, Fraction);
					// 浮動小数点で区別出来ない区間を無限分割しない返戻
					if(!(Distance > A.DistanceMeters && Distance < B.DistanceMeters)) return Reject(FlowWaterProfileFailure::Resolution);
					const std::optional<FlowChannelWaterLevel> Actual = Evaluate(Distance);
					// 準備の元場又は保持予算の失敗を保持する返戻
					if(!Actual) return Reject(Report.Failure);
					const bool Rising = Actual->DerivativePerMeter > 0 || Actual->ElevationMeters > A.Source.ElevationMeters ||
					Actual->ElevationMeters < B.Source.ElevationMeters;
					// 区間内の元場の逆転を補間で隠さない返戻
					if(Rising) return Reject(FlowWaterProfileFailure::Rising);
					const FlowChannelWaterLevel Fitted = Interpolate(A, B, Distance);
					if(!std::isfinite(Fitted.ElevationMeters) || !std::isfinite(Fitted.DerivativePerMeter) || Fitted.DerivativePerMeter > 0) {
						// 演算範囲外又は上昇する近似の返戻
						return Reject(FlowWaterProfileFailure::Resolution);
					}
					HeightError = std::max(HeightError, std::abs(Fitted.ElevationMeters - Actual->ElevationMeters));
					SlopeError = std::max(SlopeError, std::abs(Fitted.DerivativePerMeter - Actual->DerivativePerMeter));
					if(Fraction == 0.5) Middle = Actual;
				}
				Report.HeightErrorMeters = std::max(Report.HeightErrorMeters, HeightError);
				Report.SlopeError = std::max(Report.SlopeError, SlopeError);
				if(HeightError > Settings.HeightToleranceMeters * 0.25 || SlopeError > Settings.SlopeTolerance * 0.25) {
					// 新しい節点も準備予算内に収める返戻
					if(Refined.size() + Result.Knots.size() - I >= Settings.MaximumKnots) return Reject(FlowWaterProfileFailure::Budget);
					Refined.push_back({ std::midpoint(A.DistanceMeters, B.DistanceMeters), *Middle, Middle->DerivativePerMeter });
					Changed = true;
				}
				Refined.push_back(B);
			}
			// 完成した不変な係数だけを返し，準備時の元場と標本は解放する
			if(!Changed) return Result;
			Result.Knots = std::move(Refined);
		}
		// 精度未達の近似を予算終了で公開しない返戻
		return Reject(FlowWaterProfileFailure::Budget);
	}

	/**
	 * 準備した節点の範囲内で水位を評価する
	 * @param DistanceMeters 上流端からの公称距離
	 * @return 単調な水位と同じ三次式の微分；範囲外又は非有限では空
	 * @note 節点数 N に対して O(log N)，元の地形評価は行わない
	 */
	std::optional<FlowChannelWaterLevel> FlowWaterProfile::Sample(const double DistanceMeters) const {
		// 未定義の距離を端へ丸めない返戻
		if(!std::isfinite(DistanceMeters) || DistanceMeters < 0 || DistanceMeters > Knots.back().DistanceMeters) return {};
		const std::vector<Knot>::const_iterator Next = std::upper_bound(
			Knots.begin(),
			Knots.end(),
			DistanceMeters,
			[](const double Distance, const Knot &Point) {
				// 次の節点を求める比較結果の返戻
				return Distance < Point.DistanceMeters;
			}
		);
		const std::size_t Index = Next == Knots.end() ? Knots.size() - 1 : static_cast<std::size_t>(Next - Knots.begin());
		// 節点でも同じ共有微分を返す
		return Interpolate(Knots[Index - 1], Knots[Index], DistanceMeters);
	}

	/**
	 * 不変な近似の保持量を取得する
	 * @return 有限な節点数
	 */
	std::size_t FlowWaterProfile::KnotCount() const {
		return Knots.size();
	}
}
