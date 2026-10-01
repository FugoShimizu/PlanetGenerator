#include "PlanetLakeSill.h"
#include <algorithm>
#include <cmath>
#include <optional>

namespace PlanetCore {
	/**
	 * 鞍点の主方向へ局所的な楕円状の開削を準備する
	 * @param Pass 同じ元場で確認した非退化な鞍点
	 * @param Settings 半径と支持範囲・中心開削量
	 * @return 不変の補正；寸法外・急過ぎる補正・無効な主方向の場合は空
	 * @note 半幅は 0.1 m 以上で半径の１０分の１以下，開削は２ m 以下且つ小さい半幅の１０分の１以下
	 */
	std::optional<LakeSillPatch> LakeSillPatch::Create(const TerrainCriticalPoint &Pass, const LakeSillSettings &Settings) {
		const double Length = Settings.HalfLengthMeters, Width = Settings.HalfWidthMeters, Depth = Settings.DepthMeters;
		const bool Valid = std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 &&
		std::isfinite(Length) && std::isfinite(Width) && Length >= 0.1 && Width >= 0.1 &&
		std::max(Length, Width) <= Settings.RadiusMeters * 0.1 && std::isfinite(Depth) && Depth > 0 && Depth <= 2 &&
		Depth <= std::min(Length, Width) * 0.1 && Pass.Kind == TerrainCriticalKind::Saddle && std::isfinite(Pass.ElevationMeters) &&
		std::isfinite(Pass.MinimumCurvaturePerMeter) && std::isfinite(Pass.MaximumCurvaturePerMeter) &&
		Pass.MinimumCurvaturePerMeter < 0 && Pass.MaximumCurvaturePerMeter > 0;
		// 非局所的な開削又は数値的に不正な定義への返戻
		if(!Valid) return {};
		const Vec3 Origin = Pass.Direction.Vector();
		const std::optional<UnitDirection> Axis =
		UnitDirection::FromVector(Pass.MinimumCurvatureDirection - Origin * Pass.MinimumCurvatureDirection.Dot(Origin));
		// 鞍点上の有効な接線主方向が無い場合の返戻
		if(!Axis) return {};
		LakeSillPatch Result;
		Result.Origin = Origin;
		Result.Along = Axis->Vector();
		Result.Across = Origin.Cross(Result.Along);
		Result.RadiusMeters = Settings.RadiusMeters;
		Result.InverseLengthSquared = 1 / (Length * Length);
		Result.InverseWidthSquared = 1 / (Width * Width);
		Result.DepthMeters = Depth;
		// 元場を変更せず準備した，足算用の負の補正の返戻
		return Result;
	}

	/**
	 * 支持範囲の外では厳密に零となる，標高補正と解析接線勾配を評価する
	 * @param Direction 同じ惑星上の照会方向
	 * @return 負の開削量とラジアン当たりの勾配；境界で値・一階・二階微分が零
	 * @note 一つの補正につき O(1)，支持境界迄の残量の三乗に比例する補正を用いる
	 */
	TerrainSample LakeSillPatch::EvaluateCorrection(const UnitDirection Direction) const {
		const Vec3 D = Direction.Vector();
		const double Z = D.Dot(Origin);
		// 反対側の半球に同じ補正を複製しない返戻
		if(!(Z > 0)) return {};
		const double U = D.Dot(Along), V = D.Dot(Across), X = RadiusMeters * U / Z, Y = RadiusMeters * V / Z;
		const double Q = X * X * InverseLengthSquared + Y * Y * InverseWidthSquared;
		// 楕円の外側では元場の値と勾配を変更しない返戻
		if(!(Q < 1)) return {};
		const double Remaining = 1 - Q, Squared = Remaining * Remaining;
		const Vec3 DX = (Along - Origin * (U / Z)) * (RadiusMeters / Z);
		const Vec3 DY = (Across - Origin * (V / Z)) * (RadiusMeters / Z);
		const Vec3 GradientQ = DX * (2 * X * InverseLengthSquared) + DY * (2 * Y * InverseWidthSquared);
		// 境界の二階微分迄が連続した負の補正の返戻
		return { -DepthMeters * Squared * Remaining, GradientQ * (3 * DepthMeters * Squared) };
	}
}
