#pragma once

#include "PlanetAeolianField.h"

namespace PlanetCore {
	enum class ErosionPreparationFailure : std::uint8_t {
		None,
		InvalidInput,
		Climate,
		Runoff,
		DiscreteErosion,
		Reconstruction,
		Aeolian,
		RiverProfiles,
		Cancelled
	};

	struct ErosionPreparationStatistics {
		ErosionPreparationFailure Failure = ErosionPreparationFailure::InvalidInput;
		double ClimateMilliseconds = 0, RunoffMilliseconds = 0, DiscreteMilliseconds = 0;
		double ReconstructionMilliseconds = 0, AeolianMilliseconds = 0, ProfileMilliseconds = 0, TotalMilliseconds = 0;
		FluvialErosionStatistics DiscreteSediment;
	};

	/** 生成時に一度準備し，表示・衝突・気候の利用側へ共有する不変な侵食基盤 */
	class PlanetErosionPreparation {
	private:
		std::shared_ptr<const PlanetClimate> Climate;
		std::shared_ptr<const PlanetRunoff> Runoff;
		std::shared_ptr<const PlanetErodedTerrain> Terrain;
		std::shared_ptr<const PlanetAeolianField> Aeolian;
		std::shared_ptr<const DrainageNetwork> Drainage;
		ErosionPreparationStatistics Statistics;
		PlanetErosionPreparation() = default;

	public:
		/**
		 * 海陸から気候，流量，侵食，固定砂丘，河川縦断を一度だけ準備する
		 * @param Ocean 同じ生成定義の接続済み海域
		 * @param OriginalDrainage 侵食前の固定排水
		 * @param Stop 中断要求
		 * @param Diagnostics 任意の段階別検査結果
		 * @return 全段階で整合した不変な準備結果
		 */
		static std::optional<PlanetErosionPreparation> Create(const std::shared_ptr<const OceanConnectivityField> &Ocean,
			const std::shared_ptr<const DrainageNetwork> &OriginalDrainage, const std::stop_token Stop = {},
			ErosionPreparationStatistics *const Diagnostics = nullptr);

		/**
		 * 共通の季節気候を取得する
		 * @return 準備済みの不変な気候
		 */
		const std::shared_ptr<const PlanetClimate> &GetClimate() const;

		/**
		 * 元の排水網から得た流量を取得する
		 * @return 準備済みの不変な流量
		 */
		const std::shared_ptr<const PlanetRunoff> &GetRunoff() const;

		/**
		 * 砂丘を加える前の水食地形を取得する
		 * @return 侵食後の基盤
		 */
		const std::shared_ptr<const PlanetErodedTerrain> &GetTerrain() const;

		/**
		 * 河川縦断と表示へ使う固定砂丘地形を取得する
		 * @return 不変な砂丘場
		 */
		const std::shared_ptr<const PlanetAeolianField> &GetAeolianField() const;

		/**
		 * 砂丘後の標高に合わせた排水縦断を取得する
		 * @return 更新済みの排水
		 */
		const std::shared_ptr<const DrainageNetwork> &GetDrainage() const;

		/**
		 * 段階ごとの所要時間と失敗段階を取得する
		 * @return 準備の統計
		 */
		const ErosionPreparationStatistics &GetStatistics() const;
	};
}
