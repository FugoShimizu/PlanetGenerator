#pragma once

#include "PlanetClimate.h"
#include <span>

namespace PlanetCore {
/** 季節の地表被覆を描画へ渡す，固定寸法の一時配列 */
class PlanetSeasonalMap {
private:
	std::vector<std::array<float, 4>> Pixels;

	/**
	 * 一層の双一次補間を評価する
	 * @param Direction 地点の単位方向
	 * @param Layer 配列の層番号
	 * @return 補間した四成分
	 */
	std::array<double, 4> SampleLayer(const UnitDirection Direction, const std::uint32_t Layer) const;

	/** 未準備の配列の外部生成を防ぐ */
	PlanetSeasonalMap() = default;

public:
	static constexpr std::uint32_t Width = 128, Height = 65, ConiferLayer = ClimatePeriodCount * 2, Layers = ConiferLayer + 1;

	/**
	 * 検証済み気候から１２期の地表被覆を準備する
	 * @param Climate 不変な全球気候
	 * @param Stop 中断要求
	 * @return 全地点を準備した配列，中断又は不正な応答の場合は空
	 */
	static std::optional<PlanetSeasonalMap> Create(const PlanetClimate &Climate, const std::stop_token Stop = {});

	/**
	 * テクセル中心を全球の方向へ対応させる
	 * @param X 経度方向の番号，周期的に折り返す
	 * @param Y 緯度方向の番号，両端を極点へ固定する
	 * @return 単位方向
	 */
	static UnitDirection PixelDirection(const std::uint32_t X, const std::uint32_t Y);

	/**
	 * GPU へ渡す配列を取得する
	 * @return 層，緯度，経度の順の四成分配列
	 */
	std::span<const std::array<float, 4>> Data() const;

	/**
	 * 描画と同じ空間・季節補間を CPU で評価する
	 * @param Direction 地点の単位方向
	 * @param SolarYearPhase 年周位相，任意の有限値を許容する
	 * @return 草・落葉・雪，氷・湿潤・砂漠適性・樹木適性の八成分，非有限な位相では空
	 */
	std::optional<std::array<double, 8>> Sample(const UnitDirection Direction, const double SolarYearPhase) const;

	/**
	 * 季節に依らないバイオーム別の針葉樹比率を取得する
	 * @param Direction 地点の単位方向
	 * @return ０以上１以下の針葉樹比率
	 */
	double SampleConiferFraction(UnitDirection Direction) const;
};
} // namespace PlanetCore
