#pragma once
#include "PlanetView.h"

namespace PlanetCore {
	/** 固定骨格と近景を保った，前回の水面分割の縮約候補 */
	struct WaterLodTrim {
		LodTree Tree;
		std::uint32_t CoarsenedParents = 0;
	};

	/**
	 * 遠方の不要な細分化を，平衡と全球被覆を維持して戻す
	 * @param Previous 前回の有効な水面分割
	 * @param View 現在の視点と焦点距離
	 * @param Focus 今回の高精度領域の中心
	 * @param NearRadiusMeters 維持する高精度領域の弦半径
	 * @param MinimumLevel 固定水文骨格の最低段階，０～６
	 * @param Stop 中断要求
	 * @return 再検査前の候補，不正・中断の場合は空
	 * @note 投影寸法は縮約の目安のみ．出力を公開する前に全ての水面・水際条件を再検査する
	 */
	std::optional<WaterLodTrim> TrimWaterLod(const LodTree &Previous, const LodView &View, UnitDirection Focus,
		double NearRadiusMeters, std::uint8_t MinimumLevel, std::stop_token Stop = {});
}
