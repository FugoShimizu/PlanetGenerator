#include "PlanetSphericalWarp.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace PlanetCore {
	/**
	 * 座標変形の物理尺度と有限な変形量を検査する
	 * @return 有効な準備条件の場合に真
	 */
	bool SphericalWarpSettings::IsValid() const {
		// 対応半径から外れる入力の返戻
		if(!std::isfinite(RadiusMeters) || RadiusMeters < 1000 || RadiusMeters > 1E9) return false;
		const double MaximumBudget = ShouldUseContinentalWavelengths ? 12 : 3;
		// 有限な微分予算を持たない入力の返戻
		if(!std::isfinite(DerivativeBudget) || DerivativeBudget < 0 || DerivativeBudget > MaximumBudget) return false;
		// 正の有限な波長を持たない入力の返戻
		if(!(std::isfinite(LargestWavelengthMeters) && LargestWavelengthMeters > 0)) return false;
		const double MaximumWavelength = (ShouldUseContinentalWavelengths ? 6.283185307179586 : 0.5) * RadiusMeters;
		const double Frequency = 6.283185307179586 * RadiusMeters / std::min(LargestWavelengthMeters, MaximumWavelength);
		const double FrequencyMultiplier = ShouldUseContinentalWavelengths ? 256 : 16;
		// 最短波長も有限な評価範囲に収まる条件の返戻
		return std::isfinite(Frequency) && Frequency * FrequencyMultiplier <= 1E8;
	}

	/**
	 * 変形後の球面勾配を元の入力方向の接線へ引戻す
	 * @param SourceGradient 元地形の球面勾配
	 * @return 連鎖微分した入力方向の接線勾配
	 */
	Vec3 SphericalWarpMapping::PullbackGradient(const Vec3 SourceGradient) const {
		// 写像の転置微分を作用した返戻
		return { SourceGradient.Dot(Derivatives[0]), SourceGradient.Dot(Derivatives[1]), SourceGradient.Dot(Derivatives[2]) };
	}

	/**
	 * 固定幅整数から決定論的な位相用ビット列を得る
	 * @param Value Seed と通し番号
	 * @return SplitMix64 の最終化に依る混合値
	 */
	std::uint64_t PlanetSphericalWarp::Mix(const std::uint64_t Value) {
		std::uint64_t Result = Value + UINT64_C(0X9E3779B97F4A7C15);
		Result = (Result ^ (Result >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
		Result = (Result ^ (Result >> 27)) * UINT64_C(0X94D049BB133111EB);
		// 固定した位相系列の返戻
		return Result ^ Result >> 31;
	}

	/**
	 * 位相を半開区間へ写す
	 * @param Value Seed と通し番号
	 * @return ０以上２ π 未満の位相
	 */
	double PlanetSphericalWarp::Phase(const std::uint64_t Value) {
		// ５３ビットで定めた角度の返戻
		return static_cast<double>(Mix(Value) >> 11) * (6.283185307179586 / 9007199254740992.0);
	}

	/**
	 * 軸成分を変えない回転を適用する
	 * @param Point 回転するベクトル
	 * @param Axis 正規化済の回転軸
	 * @param Angle 回転角
	 * @return Rodrigues の式に依る回転結果
	 */
	Vec3 PlanetSphericalWarp::Rotate(const Vec3 Point, const Vec3 Axis, const double Angle) {
		const double Cosine = std::cos(Angle), Sine = std::sin(Angle);
		// 軸成分と垂直面の回転を合成した返戻
		return Point * Cosine + Axis.Cross(Point) * Sine + Axis * (Axis.Dot(Point) * (1 - Cosine));
	}

	/**
	 * 回転軸への射影だけから捩り角を求める
	 * @param Point 単位球面上の方向
	 * @return 角度と，軸射影に対する角度微分
	 */
	std::array<double, 2> PlanetSphericalWarp::Twist::AngleAndDerivative(const Vec3 Point) const {
		const double Argument = Frequency * Axis.Dot(Point);
		// 滑らかな有限帯域角度と其の厳密な微分の返戻
		return {
			Amplitude * (std::sin(Argument + Phase) + 0.25 * std::sin(2.3 * Argument + HarmonicPhase)),
			Amplitude * Frequency * (std::cos(Argument + Phase) + 0.575 * std::cos(2.3 * Argument + HarmonicPhase))
		};
	}

	/**
	 * 従来の３尺度，又は大陸の９尺度を，有限回の可逆写像として準備する
	 * @param Settings Seed・半径・最長波長・角度微分の総予算
	 * @return 球面の面積を保存する写像，不正な条件では空
	 */
	std::optional<PlanetSphericalWarp> PlanetSphericalWarp::Create(const SphericalWarpSettings &Settings) {
		// 不正な設定から写像を作らない返戻
		if(!Settings.IsValid()) return {};
		PlanetSphericalWarp Result;
		Result.TwistCount = Settings.ShouldUseContinentalWavelengths ? MaximumTwistCount : LegacyTwistCount;
		const double MaximumWavelength = (Settings.ShouldUseContinentalWavelengths ? 6.283185307179586 : 0.5) * Settings.RadiusMeters;
		const double Wavelength = std::min(Settings.LargestWavelengthMeters, MaximumWavelength);
		const double Frequency = 6.283185307179586 * Settings.RadiusMeters / Wavelength;
		for(unsigned Index = 0; Index < Result.TwistCount; ++Index) {
			const double Longitude = Phase(Settings.Seed + 4 * Index), Z = 2 * Phase(Settings.Seed + 4 * Index + 1) / 6.283185307179586 - 1;
			const double Ring = std::sqrt(std::max(0.0, 1 - Z * Z));
			const UnitDirection Axis = *UnitDirection::FromVector({ Ring * std::cos(Longitude), Ring * std::sin(Longitude), Z });
			const unsigned Octave = (Settings.ShouldUseContinentalWavelengths ? 1 : 2) * (Index / 3);
			const double CurrentFrequency = Frequency * static_cast<double>(1U << Octave);
			// 大陸規模へ７割，残る細部へ３割の微分予算を配分
			const double Share = Settings.ShouldUseContinentalWavelengths ? Index < 9 ? 0.7 / 9 : 0.3 / 18 : 1.0 / LegacyTwistCount;
			// 過小な波長で非有限な周波数を準備しない返戻
			if(!std::isfinite(CurrentFrequency) || CurrentFrequency > 1E8) return {};
			Result.Twists[Index] = {
				Axis.Vector(),
				CurrentFrequency,
				Settings.ShouldUseContinentalWavelengths ?
				Settings.DerivativeBudget * Share / (1.575 * CurrentFrequency) :
				Settings.DerivativeBudget / (1.575 * LegacyTwistCount * CurrentFrequency),
				Phase(Settings.Seed + 4 * Index + 2),
				Phase(Settings.Seed + 4 * Index + 3)
			};
			Result.MaximumFrequency = std::max(Result.MaximumFrequency, 2.3 * CurrentFrequency);
		}
		// 地形の値を変えず，評価方向を対応付ける準備済写像の返戻
		return Result;
	}

	/**
	 * 実惑星の方向を元地形の評価方向へ写す
	 * @param Direction 実惑星上の方向
	 * @return 元の C1 地形を評価する方向
	 */
	UnitDirection PlanetSphericalWarp::ToSource(const UnitDirection Direction) const {
		Vec3 Point = Direction.Vector();
		for(unsigned i = 0; i < TwistCount; ++i) {
			const Twist &Item = Twists[i];
			Point = Rotate(Point, Item.Axis, Item.AngleAndDerivative(Point)[0]);
		}
		// 丸めだけを正規化した球面方向の返戻
		return *UnitDirection::FromVector(Point);
	}

	/**
	 * 元地形上の河口・流路を実惑星の方向へ戻す
	 * @param Direction 元地形上の方向
	 * @return 反復解法を使わず逆順の捩りで得た実惑星方向
	 */
	UnitDirection PlanetSphericalWarp::ToWorld(const UnitDirection Direction) const {
		Vec3 Point = Direction.Vector();
		for(unsigned Index = TwistCount; Index > 0; --Index) {
			const Twist &Item = Twists[Index - 1];
			Point = Rotate(Point, Item.Axis, -Item.AngleAndDerivative(Point)[0]);
		}
		// 軸射影が各回転で不変である事を使った逆写像の返戻
		return *UnitDirection::FromVector(Point);
	}

	/**
	 * 指定された向きの可逆写像と，入力方向の接線微分を評価する
	 * @param Direction 入力座標の正規方向
	 * @param IsInverse 真なら元地形から実惑星へ戻す
	 * @return 対応する方向と３列の解析的な接線微分
	 */
	SphericalWarpMapping PlanetSphericalWarp::MapImpl(const UnitDirection Direction, const bool IsInverse) const {
		const Vec3 Initial = Direction.Vector();
		Vec3 Point = Initial;
		std::array<Vec3, 3> Derivatives {
			Vec3{ 1, 0, 0 } - Initial * Initial.X,
			Vec3{ 0, 1, 0 } - Initial * Initial.Y,
			Vec3{ 0, 0, 1 } - Initial * Initial.Z
		};
		for(unsigned i = 0; i < TwistCount; ++i) {
			const unsigned Index = IsInverse ? TwistCount - 1 - i : i;
			const Twist &Item = Twists[Index];
			std::array<double, 2> Angle = Item.AngleAndDerivative(Point);
			if(IsInverse) {
				Angle[0] = -Angle[0];
				Angle[1] = -Angle[1];
			}
			const Vec3 Next = Rotate(Point, Item.Axis, Angle[0]), Turn = Item.Axis.Cross(Next);
			for(Vec3 &Column : Derivatives) Column = Rotate(Column, Item.Axis, Angle[0]) + Turn * (Angle[1] * Item.Axis.Dot(Column));
			Point = Next;
		}
		const double Length = Point.Length();
		const UnitDirection Mapped = *UnitDirection::FromVector(Point);
		for(Vec3 &Column : Derivatives) Column = (Column - Mapped.Vector() * Mapped.Vector().Dot(Column)) * (1 / Length);
		// 正規化を含む連鎖微分の返戻
		return { Mapped, Derivatives };
	}

	/**
	 * 元地形の評価方向と解析的な接線写像を求める
	 * @param Direction 実惑星の方向
	 * @return 評価方向と，入力の三成分に対する球面接線微分
	 */
	SphericalWarpMapping PlanetSphericalWarp::MapWithDerivatives(const UnitDirection Direction) const {
		return MapImpl(Direction, false);
	}

	/**
	 * 元地形の曲線を実惑星へ戻す方向と解析微分を求める
	 * @param Direction 元地形の正規方向
	 * @return 実惑星方向と，元座標の３成分に対する球面接線微分
	 */
	SphericalWarpMapping PlanetSphericalWarp::MapToWorldWithDerivatives(const UnitDirection Direction) const {
		return MapImpl(Direction, true);
	}

	/**
	 * 独立差分に使う最短の角度尺度を参照する
	 * @return 高調波を含む最大周波数
	 */
	double PlanetSphericalWarp::GetMaximumFrequency() const {
		// 有限な最大周波数の返戻
		return MaximumFrequency;
	}
}
