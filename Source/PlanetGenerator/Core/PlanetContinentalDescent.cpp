#include "PlanetContinentalDescent.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <stop_token>

namespace PlanetCore {
	namespace {
		constexpr double CoastRoundoff = 64 * std::numeric_limits<double>::epsilon();

		/**
		 * 直線の正方向と円板境界の交差迄の距離を得る
		 * @param Point 円板内の開始点
		 * @param Heading 正規化済みの進行方向
		 * @return 正の交差距離，境界近くの相殺を避ける
		 */
		double DistanceToCoast(const ContinentalDescentPoint &Point, const std::array<double, 2> Heading) {
			const double Projection = Point.X * Heading[0] + Point.Y * Heading[1], Margin = 1 - (Point.X * Point.X + Point.Y * Point.Y);
			const double Root = std::sqrt(Projection * Projection + std::max(0.0, Margin));
			// 円板外への最初の交差距離の返戻
			return Projection > 0 ? Margin / (Root + Projection) : Root - Projection;
		}

		/**
		 * 交差計算の丸めだけを閉円板へ戻す
		 * @param Point 直線の候補終点
		 * @return 解像された円板内の座標，丸めの範囲外の場合は空
		 */
		std::optional<std::array<double, 2>> FitInsideDisk(std::array<double, 2> Point) {
			const double Square = Point[0] * Point[0] + Point[1] * Point[1];
			// 既に円板内に在る点の返戻
			if(Square <= 1) return Point;
			// 有限な交差の丸めを超える位置の返戻
			if(!std::isfinite(Square) || Square - 1 > CoastRoundoff) return {};
			const double Scale = 1 / std::sqrt(Square);
			Point[0] *= Scale;
			Point[1] *= Scale;
			for(unsigned i = 0; i < 4 && Point[0] * Point[0] + Point[1] * Point[1] > 1; ++i) {
				Point[0] = std::nextafter(Point[0], 0.0);
				Point[1] = std::nextafter(Point[1], 0.0);
			}
			// 表現出来る閉円板へ戻せない座標の返戻
			if(Point[0] * Point[0] + Point[1] * Point[1] > 1) return {};
			// 丸めだけを解決した候補座標の返戻
			return Point;
		}

		/**
		 * 境界と零標高の両方を数値的に解像した終点を判定する
		 * @param Point 公開候補の終点
		 * @param HeightRoundoff 準備尺度と源の個数から定めた丸め上界
		 * @return 境界と高さの残差が両方とも丸め内の場合に真
		 */
		bool IsNumericalCoast(const ContinentalDescentPoint &Point, const double HeightRoundoff) {
			// 大きな海岸残差を単なる境界近接だけで受理しない返戻
			return 1 - (Point.X * Point.X + Point.Y * Point.Y) <= CoastRoundoff && Point.PotentialHeightMeters <= HeightRoundoff;
		}
	}

	/**
	 * 勾配と負の曲率を使い，微分上界で下降を検証した区間だけを構成する
	 * @param Potential 同じ不変な円板地形
	 * @param Start 円板内の第１・第２座標
	 * @param Settings 区間長と資源の上限
	 * @param Error 準備失敗の原因
	 * @param Stop 中断要求
	 * @return 海岸の数値的閉包へ到達した経路，失敗又は中断時は空
	 */
	std::optional<ContinentalDescentPath> ContinentalDescentPath::Create(
		const ContinentalPotential &Potential,
		const std::array<double, 2> Start,
		const ContinentalDescentSettings &Settings,
		ContinentalDescentError &Error,
		const std::stop_token Stop
	) {
		Error = ContinentalDescentError::None;
		const auto Fail = [&](const ContinentalDescentError Reason) -> std::optional<ContinentalDescentPath> {
			Error = Reason;
			// 未到達の経路を正常な河川として公開しない返戻
			return {};
		};
		// 中断した準備の返戻
		if(Stop.stop_requested()) return Fail(ContinentalDescentError::Cancelled);
		const double MaximumLength = Settings.MaximumSegmentLength;
		const bool IsInvalid =
		!std::isfinite(MaximumLength) || MaximumLength <= 0 || MaximumLength > 0.25 || !Settings.MaximumSegments ||
		Settings.MaximumSegments > 65536;
		// 無界又は不正な準備設定の返戻
		if(IsInvalid) return Fail(ContinentalDescentError::InvalidSettings);
		const std::optional<ContinentalPotentialSample> Initial = Potential.Evaluate(Start[0], Start[1]);
		// 円板外又は解像出来ない開始点の返戻
		if(!Initial) return Fail(ContinentalDescentError::InvalidStart);
		// 経路の所有と同一入力識別子の準備
		ContinentalDescentPath Result;
		Result.Points.push_back({ Start[0], Start[1], Initial->HeightMeters });
		Result.Hash = Potential.GetDefinitionHash() ^ ContinentalDescentVersion;
		const auto AppendHash = [&](const std::uint64_t Bits) -> void {
			Result.Hash = (Result.Hash ^ Bits) * UINT64_C(1099511628211);
			return;
		};
		AppendHash(std::bit_cast<std::uint64_t>(Start[0]));
		AppendHash(std::bit_cast<std::uint64_t>(Start[1]));
		AppendHash(std::bit_cast<std::uint64_t>(MaximumLength));
		AppendHash(static_cast<std::uint64_t>(Settings.MaximumSegments));
		const double HeightRoundoff =
		128 * std::numeric_limits<double>::epsilon() * (static_cast<double>(Potential.GetSourceCount()) + 1) *
		Potential.GetAbsoluteHeightBoundMeters();
		// 有限な数値尺度を持てない比較経路の返戻
		if(!std::isfinite(HeightRoundoff)) return Fail(ContinentalDescentError::UnresolvedProgress);
		for(std::size_t i = 0; i <= Settings.MaximumSegments; ++i) {
			// ワーカーの中断を区間境界で確認した返戻
			if(Stop.stop_requested()) return Fail(ContinentalDescentError::Cancelled);
			const ContinentalDescentPoint Current = Result.Points.back();
			if(IsNumericalCoast(Current, HeightRoundoff)) {
				Result.Stats.EndBoundaryResidual = std::abs(1 - (Current.X * Current.X + Current.Y * Current.Y));
				Result.Stats.EndPotentialHeightMeters = Current.PotentialHeightMeters;
				// 数値的な境界と零標高を検証した完成経路の返戻
				return Result;
			}
			// 未到達のまま資源上限へ達した経路の返戻
			if(i == Settings.MaximumSegments) return Fail(ContinentalDescentError::SegmentBudget);
			const std::optional<ContinentalPotentialSample> Sample = Potential.Evaluate(Current.X, Current.Y);
			// 更新した候補点を解像出来ない経路の返戻
			if(!Sample) return Fail(ContinentalDescentError::UnresolvedProgress);
			const double GX = Sample->GradientPerUnit[0], GY = Sample->GradientPerUnit[1], GradientLength = std::hypot(GX, GY);
			const double XX = Sample->HessianPerUnitSquared[0], XY = Sample->HessianPerUnitSquared[1];
			const double YY = Sample->HessianPerUnitSquared[2];
			bool HasNext = false;
			for(
				const ContinentalDescentControl Control :
				{ ContinentalDescentControl::Gradient, ContinentalDescentControl::NegativeCurvature }
			) {
				// 零勾配には非増加の接線を仮造せず，負曲率の経路へ進む
				if(Control == ContinentalDescentControl::Gradient && !(GradientLength > 0)) continue;
				std::array<double, 2> Heading {};
				if(Control == ContinentalDescentControl::Gradient) Heading = { -GX / GradientLength, -GY / GradientLength };
				else {
					// Hessian の小さい固有値へ向かい，最初の微分が非正になる方位を選択
					const double HessianScale = std::max({ std::abs(XX), std::abs(XY), std::abs(YY) });
					// 表現出来る負曲率を持たない候補では別の方向を仮造しない
					if(!(HessianScale > 0)) continue;
					const double Angle =
					0.5 * std::atan2(2 * (XY / HessianScale), XX / HessianScale - YY / HessianScale) + 0.5 * std::numbers::pi_v<double>;
					Heading = { std::cos(Angle), std::sin(Angle) };
					if(GX * Heading[0] + GY * Heading[1] > 0) {
						Heading[0] = -Heading[0];
						Heading[1] = -Heading[1];
					}
				}
				double Length = std::min(MaximumLength, DistanceToCoast(Current, Heading));
				for(unsigned Halving = 0; Halving < 64; ++Halving, Length *= 0.5) {
					// 短い候補の探索中にも中断を反映した返戻
					if(Stop.stop_requested()) return Fail(ContinentalDescentError::Cancelled);
					const std::optional<std::array<double, 2>> Coordinates =
					FitInsideDisk({ Current.X + Heading[0] * Length, Current.Y + Heading[1] * Length });
					if(!Coordinates) continue;
					const double DX = (*Coordinates)[0] - Current.X, DY = (*Coordinates)[1] - Current.Y, ActualLength = std::hypot(DX, DY);
					// 丸めで同じ点になった候補では曲率の別経路を検討
					if(!(ActualLength > 0) || !std::isfinite(ActualLength)) break;
					const double HX = DX / ActualLength, HY = DY / ActualLength;
					const std::optional<ContinentalPotentialDerivativeBounds> Bounds =
					Potential.GetDerivativeBounds(Current.X, Current.Y, std::nextafter(ActualLength, std::numeric_limits<double>::infinity()));
					++Result.Stats.BoundsEvaluations;
					// 上界を解像出来ない区間を証明済みとして公開しない返戻
					if(!Bounds) return Fail(ContinentalDescentError::DerivativeBoundsRejected);
					const double First = GX * HX + GY * HY, Second = XX * HX * HX + 2 * (XY * HX) * HY + YY * HY * HY;
					double DerivativeUpper = 0, CurvatureUpper = 0, DropLower = 0;
					if(Control == ContinentalDescentControl::Gradient) {
						DerivativeUpper = First + Bounds->HessianNormPerUnitSquared * ActualLength;
						CurvatureUpper = Bounds->HessianNormPerUnitSquared;
						if(!(DerivativeUpper < 0)) continue;
						DropLower = -First * ActualLength - 0.5 * Bounds->HessianNormPerUnitSquared * ActualLength * ActualLength;
					} else {
						DerivativeUpper = First;
						CurvatureUpper = Second + Bounds->ThirdDerivativeNormPerUnitCubed * ActualLength;
						if(!(First <= 0 && CurvatureUpper < 0)) continue;
						DropLower = -First * ActualLength - 0.5 * Second * ActualLength * ActualLength -
						Bounds->ThirdDerivativeNormPerUnitCubed * ActualLength * ActualLength * ActualLength / 6;
					}
					const std::optional<double> NextHeight = Potential.EvaluateHeight((*Coordinates)[0], (*Coordinates)[1]);
					// 有効な区間の終点を解像出来ない経路の返戻
					if(!NextHeight) return Fail(ContinentalDescentError::UnresolvedProgress);
					const ContinentalDescentPoint Next { (*Coordinates)[0], (*Coordinates)[1], *NextHeight };
					// 勾配による減少が丸めに近い場合は，負曲率の経路へ進む
					if(!(DropLower > HeightRoundoff) && !IsNumericalCoast(Next, HeightRoundoff)) break;
					if(!(*NextHeight < Current.PotentialHeightMeters)) continue;
					Result.Points.push_back(Next);
					Result.Segments.push_back({ ActualLength, First, DerivativeUpper, CurvatureUpper, Control });
					if(Control == ContinentalDescentControl::NegativeCurvature) ++Result.Stats.CurvatureEscapes;
					HasNext = true;
					break;
				}
				if(HasNext) break;
			}
			// どちらの局所下降も解像出来ない開始点を，平らな出口で隠さない返戻
			if(!HasNext) return Fail(ContinentalDescentError::UnresolvedProgress);
		}
		// 準備資源の上限を超えた未到達経路の返戻
		return Fail(ContinentalDescentError::SegmentBudget);
	}

	/**
	 * 単調なポテンシャルと同じ円板座標の列を参照する
	 * @return 経路の寿命内で有効な不変ビュー
	 */
	std::span<const ContinentalDescentPoint> ContinentalDescentPath::GetPoints() const {
		return Points;
	}

	/**
	 * 直線区間の下降と負曲率の検証値を参照する
	 * @return 点の個数より１個少ない不変な区間列
	 */
	std::span<const ContinentalDescentSegment> ContinentalDescentPath::GetSegments() const {
		return Segments;
	}

	/**
	 * 数値的な海岸残差と準備費用を参照する
	 * @return 不変な構成統計，球面・景観の完成証明とは別
	 */
	const ContinentalDescentStatistics &ContinentalDescentPath::GetStatistics() const {
		return Stats;
	}

	/**
	 * 同じ入力と構成版の実行時識別子を取得する
	 * @return 地形，開始点，資源設定と版の識別子
	 */
	std::uint64_t ContinentalDescentPath::GetDefinitionHash() const {
		return Hash;
	}
}
