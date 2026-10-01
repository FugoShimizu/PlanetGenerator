#include "PlanetConnectedOceanDomain.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	/**
	 * 非交差円板と海陸比を検証してから，連結した開海域を準備する
	 * 計算量：帽数 N に対して O(N²)，N は最大６４個
	 * @param Settings 陸域帽，半径・Seed・可逆変形，海岸の物理遷移幅
	 * @param Error 準備の失敗理由，成功時は None
	 * @param Stop 準備の中断要求
	 * @return 不変な領域場，無効入力・不十分な離隔・中断時は空
	 */
	std::optional<ConnectedOceanDomain> ConnectedOceanDomain::Create(
		const ConnectedOceanDomainSettings &Settings,
		ConnectedOceanDomainError &Error,
		const std::stop_token Stop
	) {
		Error = ConnectedOceanDomainError::None;
		// 失敗の理由を部分場の公開より先に記録する
		const auto Fail = [&](const ConnectedOceanDomainError Reason) -> std::optional<ConnectedOceanDomain> {
			Error = Reason;
			// 準備失敗と其の理由の返戻
			return {};
		};
		// 中断された入力の返戻
		if(Stop.stop_requested()) return Fail(ConnectedOceanDomainError::Cancelled);
		const std::size_t Count = Settings.Continents.size();
		const bool IsValidSettings = Count && Count < 65 && Settings.Warp.IsValid() && std::isfinite(Settings.CoastTransitionMeters) &&
		Settings.CoastTransitionMeters > 0;
		// 無効な上限又は非有限の設定の返戻
		if(!IsValidSettings) return Fail(ConnectedOceanDomainError::InvalidSettings);
		double LandFraction = 0, Separation = std::numbers::pi_v<double>;
		for(std::size_t i = 0; i < Count; ++i) {
			const ContinentalDomainCap &Cap = Settings.Continents[i];
			// 円板と同相でない半径又は非有限値の返戻
			if(!(std::isfinite(Cap.RadiusRadians) && Cap.RadiusRadians > 0 && Cap.RadiusRadians < std::numbers::pi_v<double> * 0.5)) {
				// 不正な陸域帽の返戻
				return Fail(ConnectedOceanDomainError::InvalidCap);
			}
			const double Sine = std::sin(Cap.RadiusRadians * 0.5);
			LandFraction += Sine * Sine;
			Separation = std::min(Separation, std::numbers::pi_v<double> - Cap.RadiusRadians);
			for(std::size_t j = 0; j < i; ++j) {
				const ContinentalDomainCap &Other = Settings.Continents[j];
				const Vec3 FirstCenter = Cap.Center.Vector(), SecondCenter = Other.Center.Vector();
				const double Distance = std::atan2(FirstCenter.Cross(SecondCenter).Length(), FirstCenter.Dot(SecondCenter));
				const double Gap = Distance - Cap.RadiusRadians - Other.RadiusRadians;
				// 接触だけの狭い通路や交差した円板を連結証拠としない返戻
				if(!(Gap > 1E-8)) return Fail(ConnectedOceanDomainError::TouchingContinents);
				Separation = std::min(Separation, Gap);
			}
			// 中断後に完成した領域を返さない返戻
			if(Stop.stop_requested()) return Fail(ConnectedOceanDomainError::Cancelled);
		}
		const double OceanFraction = 1 - LandFraction;
		// 要求された海陸比を構成出来ない入力の返戻
		if(OceanFraction < 0.6 || OceanFraction > 0.7) return Fail(ConnectedOceanDomainError::OceanAreaOutsideRange);
		ConnectedOceanDomain Result;
		Result.Warp = PlanetSphericalWarp::Create(Settings.Warp);
		// 正規化した設定でも写像を準備出来ない場合の返戻
		if(!Result.Warp) return Fail(ConnectedOceanDomainError::InvalidSettings);
		Result.Stats = { Count, OceanFraction, Separation, std::numbers::pi_v<double> };
		Result.Hash = UINT64_C(0X434F4E4E4F434E03);
		// 領域の版と全ての準備入力を固定幅の識別値へ含める
		const auto Include = [&](const std::uint64_t Bits) -> void {
			Result.Hash = (Result.Hash ^ Bits) * UINT64_C(0X100000001B3);
			return;
		};
		Include(Settings.Warp.Seed);
		Include(Settings.Warp.ShouldUseContinentalWavelengths ? 1 : 0);
		for(
			const double Value :
			{
				Settings.Warp.RadiusMeters,
				Settings.Warp.LargestWavelengthMeters,
				Settings.Warp.DerivativeBudget,
				Settings.CoastTransitionMeters
			}
		) Include(std::bit_cast<std::uint64_t>(Value));
		Result.Caps.reserve(Count);
		for(const ContinentalDomainCap &Cap : Settings.Continents) {
			const double Width =
			std::min({ Settings.CoastTransitionMeters / Settings.Warp.RadiusMeters, 0.25 * Cap.RadiusRadians, 0.25 * Separation });
			const double ChordWidth = 2 * std::sin(Cap.RadiusRadians + Width * 0.5) * std::sin(Width * 0.5);
			// 浮動小数点で海岸の幅を解像出来ない入力の返戻
			// 符号判定が丸め幅に隠れる設定の返戻
			if(!(ChordWidth > 128 * std::numeric_limits<double>::epsilon())) return Fail(ConnectedOceanDomainError::UnresolvedCoastWidth);
			const Vec3 Center = Cap.Center.Vector();
			Result.Caps.push_back({ Center, std::cos(Cap.RadiusRadians), 1 / ChordWidth });
			Result.Stats.MinimumCoastWidthRadians = std::min(Result.Stats.MinimumCoastWidthRadians, Width);
			for(const double Value : { Center.X, Center.Y, Center.Z, Cap.RadiusRadians }) Include(std::bit_cast<std::uint64_t>(Value));
		}
		Include(static_cast<std::uint64_t>(Count));
		const ContinentalDomainCap &First = Settings.Continents.front();
		const Vec3 Center = First.Center.Vector(), Axis = std::abs(Center.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 };
		const Vec3 Tangent = UnitDirection::FromVector(Axis.Cross(Center))->Vector();
		const double Angle = First.RadiusRadians + 0.5 * Separation;
		const UnitDirection Root = *UnitDirection::FromVector(Center * std::cos(Angle) + Tangent * std::sin(Angle));
		Result.Anchor = Result.Warp->ToWorld(Root).Vector();
		// 中断された準備結果の返戻
		if(Stop.stop_requested()) return Fail(ConnectedOceanDomainError::Cancelled);
		// 全陸域円板が離れ，同じ海域と面積を保つ準備結果の返戻
		return Result;
	}

	/**
	 * 同じ領域の C2 符号関数と解析接線勾配を評価する
	 * 遷移外を一定値へ接続し，互いに離れた支持域で積の過小化を避ける
	 * 計算量：帽数 N に対して O(N)，動的な割当は無
	 * @param Direction 実惑星の正規方向
	 * @return 無次元の海陸符号と１ラジアン当りの接線微分
	 */
	ConnectedOceanDomainSample ConnectedOceanDomain::Evaluate(const UnitDirection Direction) const {
		const SphericalWarpMapping Mapping = Warp->MapWithDerivatives(Direction);
		const ConnectedOceanDomainSample Sample = EvaluateInSource(Mapping.Direction);
		// 同じ海域符号と入力方向へ引戻した勾配の返戻
		return { Sample.CoastSign, Mapping.PullbackGradient(Sample.TangentialGradientPerRadian) };
	}

	/**
	 * 元座標の同じ領域を評価し，共有された座標写像の重複計算を避ける
	 * @param Direction 同じ設定の写像で得た元球面方向，実惑星方向とは区別
	 * @return C2 海陸符号と元球面の接線勾配
	 */
	ConnectedOceanDomainSample ConnectedOceanDomain::EvaluateInSource(const UnitDirection Direction) const {
		const Vec3 Point = Direction.Vector();
		for(const PreparedCap &Cap : Caps) {
			const double Coordinate = (Cap.CoastCosine - Cap.Center.Dot(Point)) * Cap.InverseTransitionWidth;
			// 内側の一定符号の返戻
			if(Coordinate <= -1) return { 1, {} };
			if(Coordinate >= 1) continue;
			const double Square = Coordinate * Coordinate, Support = 1 - Square;
			const double Value = Coordinate * (15 + Square * (-10 + 3 * Square)) * 0.125;
			const double Derivative = 1.875 * Support * Support * Cap.InverseTransitionWidth;
			const Vec3 Gradient = (Cap.Center - Point * Cap.Center.Dot(Point)) * Derivative;
			// 非交差支持域で唯一の海岸の C2 符号と元座標の微分の返戻
			return { -Value, Gradient };
		}
		// 全陸域円板の外に在る同じ開海域の返戻
		return { -1, {} };
	}

	/**
	 * 海岸を含めず，構成済みの同じ開海域への所属を判定する
	 * 計算量：帽数 N に対して O(N)
	 * @param Direction 実惑星の正規方向
	 * @return 大陸の閉円板の外に在る場合に真
	 */
	bool ConnectedOceanDomain::ContainsOcean(const UnitDirection Direction) const {
		const Vec3 Point = Warp->ToSource(Direction).Vector();
		// 陸域又は其の海岸に属する点の返戻
		for(const PreparedCap &Cap : Caps) if(Cap.Center.Dot(Point) >= Cap.CoastCosine) return false;
		// 海面以下の同じ連結領域への所属の返戻
		return true;
	}

	/**
	 * 確定した同じ海域の基準点を取得する
	 * @return 陸域円板からの正の離隔を持つ，実惑星上の大洋基準方向
	 */
	UnitDirection ConnectedOceanDomain::OceanAnchor() const {
		return *UnitDirection::FromVector(Anchor);
	}

	/**
	 * 領域の構成条件を取得する
	 * @return 構成式の海域比と検証した離隔・遷移幅
	 */
	const ConnectedOceanDomainStatistics &ConnectedOceanDomain::Statistics() const {
		return Stats;
	}

	/**
	 * 不変な領域の識別値を取得する
	 * @return 海域領域の版・帽・写像・物理幅を含む識別値
	 */
	std::uint64_t ConnectedOceanDomain::DefinitionHash() const {
		return Hash;
	}
}
