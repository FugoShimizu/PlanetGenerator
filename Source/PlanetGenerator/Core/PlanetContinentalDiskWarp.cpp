#include "PlanetContinentalDiskWarp.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace PlanetCore {
	/**
	 * 曲線接線へ対応する方向微分を作用させる
	 * @param Tangent 入力円板内の接線
	 * @return 対応する出力の接線
	 */
	std::array<double, 2> ContinentalDiskWarpMapping::Pushforward(const std::array<double, 2> Tangent) const {
		// 同じ座標変形の解析微分を作用した返戻
		return { Derivatives[0] * Tangent[0] + Derivatives[1] * Tangent[1], Derivatives[2] * Tangent[0] + Derivatives[3] * Tangent[1] };
	}

	/**
	 * 元の高さ場の勾配を入力円板へ引戻す
	 * @param Gradient 変形後の円板での高さ勾配
	 * @return 実円板の座標に対する勾配
	 */
	std::array<double, 2> ContinentalDiskWarpMapping::Pullback(const std::array<double, 2> Gradient) const {
		// 転置の連鎖微分を作用した返戻
		return {
			Derivatives[0] * Gradient[0] + Derivatives[2] * Gradient[1],
			Derivatives[1] * Gradient[0] + Derivatives[3] * Gradient[1]
		};
	}

	/**
	 * 支持域が海岸と交差しない有限個の局所捩りだけを準備する
	 * @param Settings 中心・支持半径・角度の列
	 * @return 海岸を固定する可逆写像，不正又は過大な列では空
	 */
	std::optional<ContinentalDiskWarp> ContinentalDiskWarp::Create(const ContinentalDiskWarpSettings &Settings) {
		// 無界な構成列を公開しない返戻
		if(Settings.Twists.size() > MaximumTwists) return {};
		ContinentalDiskWarp Result;
		Result.Hash = UINT64_C(1469598103934665603) ^ ContinentalDiskWarpVersion;
		Result.Twists.reserve(Settings.Twists.size());
		double TotalAngle = 0;
		const auto AppendHash = [&](const double Value) -> void {
			Result.Hash = (Result.Hash ^ std::bit_cast<std::uint64_t>(Value)) * UINT64_C(1099511628211);
			return;
		};
		for(const ContinentalDiskTwist &Twist : Settings.Twists) {
			const double CenterLength = std::hypot(Twist.X, Twist.Y);
			const bool IsInvalid =
			!std::isfinite(CenterLength) || !std::isfinite(Twist.Radius) || !std::isfinite(Twist.AngleRadians) || Twist.Radius < 0.01 ||
			Twist.Radius > 0.4 || !(CenterLength + Twist.Radius < 1) || std::abs(Twist.AngleRadians) > 3;
			// 海岸を跨ぐ又は逆写像を解像出来ない捩りの返戻
			if(IsInvalid) return {};
			TotalAngle += std::abs(Twist.AngleRadians);
			// 重ねた写像の伸長予算を無界にしない返戻
			if(TotalAngle > 16) return {};
			Result.Twists.push_back(Twist);
			AppendHash(Twist.X);
			AppendHash(Twist.Y);
			AppendHash(Twist.Radius);
			AppendHash(Twist.AngleRadians);
		}
		Result.Hash = (Result.Hash ^ static_cast<std::uint64_t>(Result.Twists.size())) * UINT64_C(1099511628211);
		// 捩りの順序迄固定した不変な写像の返戻
		return Result;
	}

	/**
	 * 支持内部では角度を滑らかに零へ下げ，逆順の符号反転も同じ微分で評価する
	 * @param X 実円板又は隆起円板の第１座標
	 * @param Y 実円板又は隆起円板の第２座標
	 * @param IsInverse 隆起円板から実円板へ戻す場合に真
	 * @return 対応する円板座標と解析微分，円板外では空
	 */
	std::optional<ContinentalDiskWarpMapping> ContinentalDiskWarp::MapImpl(
		const double X,
		const double Y,
		const bool IsInverse
	) const {
		// 有限な閉円板外への写像を作らない返戻
		if(!std::isfinite(X) || !std::isfinite(Y) || X * X + Y * Y > 1) return {};
		ContinentalDiskWarpMapping Result { { X, Y }, { 1, 0, 0, 1 } };
		for(std::size_t i = 0; i < Twists.size(); ++i) {
			const std::size_t Index = IsInverse ? Twists.size() - 1 - i : i;
			const ContinentalDiskTwist &Twist = Twists[Index];
			const double VX = Result.Coordinates[0] - Twist.X, VY = Result.Coordinates[1] - Twist.Y;
			const double InverseRadiusSquare = 1 / (Twist.Radius * Twist.Radius), Q = (VX * VX + VY * VY) * InverseRadiusSquare;
			if(Q >= 1) continue;
			const double T = 1 - Q, T2 = T * T, T3 = T2 * T, Angle = (IsInverse ? -1 : 1) * Twist.AngleRadians * T2 * T2;
			const double AngleDerivative = (IsInverse ? 1 : -1) * 8 * Twist.AngleRadians * T3 * InverseRadiusSquare;
			const double Cosine = std::cos(Angle), Sine = std::sin(Angle), NextX = Twist.X + Cosine * VX - Sine * VY;
			const double NextY = Twist.Y + Sine * VX + Cosine * VY, TurnX = -(NextY - Twist.Y), TurnY = NextX - Twist.X;
			const double GX = AngleDerivative * VX, GY = AngleDerivative * VY;
			const std::array<double, 4> Previous = Result.Derivatives;
			const double FirstAngle = GX * Previous[0] + GY * Previous[2], SecondAngle = GX * Previous[1] + GY * Previous[3];
			Result.Derivatives = {
				Cosine * Previous[0] - Sine * Previous[2] + TurnX * FirstAngle,
				Cosine * Previous[1] - Sine * Previous[3] + TurnX * SecondAngle,
				Sine * Previous[0] + Cosine * Previous[2] + TurnY * FirstAngle,
				Sine * Previous[1] + Cosine * Previous[3] + TurnY * SecondAngle
			};
			Result.Coordinates = { NextX, NextY };
		}
		const bool IsFinite = std::isfinite(Result.Coordinates[0]) && std::isfinite(Result.Coordinates[1]) && std::all_of(
			Result.Derivatives.begin(),
			Result.Derivatives.end(),
			[](const double Value) {
				return std::isfinite(Value);
			}
		);
		// 非有限な写像値又は解析微分の返戻
		if(!IsFinite) return {};
		// 海岸を動かさない同じ円板の座標と微分の返戻
		return Result;
	}

	/**
	 * 大陸の実円板から有限源の円板へ写す
	 * @param X 実円板の第１座標
	 * @param Y 実円板の第２座標
	 * @return 対応する隆起座標と解析微分，無効入力では空
	 */
	std::optional<ContinentalDiskWarpMapping> ContinentalDiskWarp::MapToSource(const double X, const double Y) const {
		return MapImpl(X, Y, false);
	}

	/**
	 * 隆起源の排水曲線を実円板へ戻す
	 * @param X 隆起円板の第１座標
	 * @param Y 隆起円板の第２座標
	 * @return 実座標と解析微分，無効入力では空
	 */
	std::optional<ContinentalDiskWarpMapping> ContinentalDiskWarp::MapToWorld(const double X, const double Y) const {
		return MapImpl(X, Y, true);
	}

	/**
	 * 準備した局所捩りの個数を取得する
	 * @return 海岸内で支持が完結する捩りの数
	 */
	std::size_t ContinentalDiskWarp::GetTwistCount() const {
		// 有限個の捩りを保持した結果の返戻
		return Twists.size();
	}

	/**
	 * 中心・半径・角度・順序と構成版の識別子を取得する
	 * @return 同じ円板座標変形の識別子
	 */
	std::uint64_t ContinentalDiskWarp::GetDefinitionHash() const {
		// 構成入力を含む識別子の返戻
		return Hash;
	}
}
