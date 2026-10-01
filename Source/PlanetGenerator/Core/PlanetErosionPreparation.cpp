#include "PlanetErosionPreparation.h"
#include <chrono>

namespace PlanetCore {
	namespace {
		using Clock = std::chrono::steady_clock;

		/**
		 * 単調時計の経過時間をミリ秒へ換算する
		 * @param Start 開始時点
		 * @return 壁時計の経過時間
		 */
		double ElapsedMilliseconds(const Clock::time_point Start) {
			// 準備時間の返戻
			return std::chrono::duration<double, std::milli>(Clock::now() - Start).count();
		}
	}

	/**
	 * 同じ海陸から気候・流量・侵食を準備し，現在の地形に河川水位を接続する
	 * @param Ocean 侵食前の地質と接続済みの大洋
	 * @param OriginalDrainage 同じ海陸で準備した侵食前の固定排水
	 * @param Stop 中断要求
	 * @param Diagnostics 失敗段階を含む任意の出力先
	 * @return 全段階を準備した不変な結果，入力不正・失敗・中断時は空
	 */
	std::optional<PlanetErosionPreparation> PlanetErosionPreparation::Create(
		const std::shared_ptr<const OceanConnectivityField> &Ocean,
		const std::shared_ptr<const DrainageNetwork> &OriginalDrainage,
		const std::stop_token Stop,
		ErosionPreparationStatistics *const Diagnostics
	) {
		const Clock::time_point Start = Clock::now();
		ErosionPreparationStatistics Local;
		ErosionPreparationStatistics &Progress = Diagnostics ? *Diagnostics : Local;
		Progress = {};
		/**
		 * 未完成の結果を公開せず，段階と経過時間を残す
		 * @return 準備失敗を示す空の結果
		 */
		const auto Failed = [&]() -> std::optional<PlanetErosionPreparation> {
			if(Stop.stop_requested()) Progress.Failure = ErosionPreparationFailure::Cancelled;
			Progress.TotalMilliseconds = ElapsedMilliseconds(Start);
			// 失敗した準備の返戻
			return {};
		};
		// 異なる生成定義又は中断された入力の返戻
		if(
			!Ocean || !OriginalDrainage || Stop.stop_requested() || OriginalDrainage->SourceDefinitionHash() != Ocean->DefinitionHash()
		) return Failed();
		PlanetErosionPreparation Result;
		Progress.Failure = ErosionPreparationFailure::Climate;
		Clock::time_point PhaseStart = Clock::now();
		std::optional<PlanetClimate> Climate = PlanetClimate::Create(*Ocean, {}, Stop);
		Progress.ClimateMilliseconds = ElapsedMilliseconds(PhaseStart);
		// 気候の準備失敗の返戻
		if(!Climate) return Failed();
		Result.Climate = std::make_shared<const PlanetClimate>(std::move(*Climate));
		Progress.Failure = ErosionPreparationFailure::Runoff;
		PhaseStart = Clock::now();
		std::optional<PlanetRunoff> Runoff = PlanetRunoff::Create(*OriginalDrainage, *Result.Climate, Stop);
		Progress.RunoffMilliseconds = ElapsedMilliseconds(PhaseStart);
		// 流量の準備又は水収支の検査失敗の返戻
		if(!Runoff) return Failed();
		Result.Runoff = std::make_shared<const PlanetRunoff>(std::move(*Runoff));
		Progress.Failure = ErosionPreparationFailure::DiscreteErosion;
		PhaseStart = Clock::now();
		std::optional<PlanetFluvialErosion> Discrete = PlanetFluvialErosion::Create(*OriginalDrainage, *Result.Runoff, {}, Stop);
		Progress.DiscreteMilliseconds = ElapsedMilliseconds(PhaseStart);
		// 離散的な侵食又は土砂収支の検査失敗の返戻
		if(!Discrete) return Failed();
		Progress.DiscreteSediment = Discrete->GetStatistics();
		Progress.Failure = ErosionPreparationFailure::Reconstruction;
		PhaseStart = Clock::now();
		std::optional<PlanetErodedTerrain> Terrain = PlanetErodedTerrain::Create(Ocean, *OriginalDrainage, *Discrete, Stop);
		Progress.ReconstructionMilliseconds = ElapsedMilliseconds(PhaseStart);
		// 連続な地形への再構成が失敗した場合の返戻
		if(!Terrain) return Failed();
		Result.Terrain = std::make_shared<const PlanetErodedTerrain>(std::move(*Terrain));
		// 再構成後は地点評価に不要な離散配列を保持しない
		Discrete.reset();
		Progress.Failure = ErosionPreparationFailure::Aeolian;
		PhaseStart = Clock::now();
		std::optional<PlanetAeolianField> Aeolian =
		PlanetAeolianField::Create(Result.Terrain, *Result.Climate, Ocean->Base().Settings(), Stop);
		Progress.AeolianMilliseconds = ElapsedMilliseconds(PhaseStart);
		// 固定砂丘の準備失敗又は中断の返戻
		if(!Aeolian) return Failed();
		Result.Aeolian = std::make_shared<const PlanetAeolianField>(std::move(*Aeolian));
		Progress.Failure = ErosionPreparationFailure::RiverProfiles;
		PhaseStart = Clock::now();
		std::optional<DrainageNetwork> Drainage = OriginalDrainage->WithTerrainProfiles(
			/**
			 * 水位と河床の準備へ同じ砂丘後の地形を渡す
			 * @param Direction 地点の方向
			 * @return 砂丘後の標高
			 */
			[&Result](const UnitDirection Direction) {
				// 河床と共有する標高の返戻
				return Result.Aeolian->EvaluateElevation(Direction);
			},
			Stop
		);
		Progress.ProfileMilliseconds = ElapsedMilliseconds(PhaseStart);
		// 縦断形状を整合出来なかった場合の返戻
		if(!Drainage || Stop.stop_requested()) return Failed();
		Result.Drainage = std::make_shared<const DrainageNetwork>(std::move(*Drainage));
		Progress.Failure = ErosionPreparationFailure::None;
		Progress.TotalMilliseconds = ElapsedMilliseconds(Start);
		Result.Statistics = Progress;
		// 利用側で再準備せず共有出来る侵食基盤の返戻
		return Result;
	}

	/**
	 * 生成時の気候を共有する
	 * @return 季節の地表被覆にも使う同じ不変な気候
	 */
	const std::shared_ptr<const PlanetClimate> &PlanetErosionPreparation::GetClimate() const {
		// 準備済み気候の返戻
		return Climate;
	}

	/**
	 * 侵食に使った流量を共有する
	 * @return 固定排水に対応する年周期流量
	 */
	const std::shared_ptr<const PlanetRunoff> &PlanetErosionPreparation::GetRunoff() const {
		// 準備済み流量の返戻
		return Runoff;
	}

	/**
	 * 表示と衝突に共通の侵食基盤を共有する
	 * @return 地点毎に評価出来る不変な標高関数
	 */
	const std::shared_ptr<const PlanetErodedTerrain> &PlanetErosionPreparation::GetTerrain() const {
		// 準備済み地形の返戻
		return Terrain;
	}

	/**
	 * 侵食後に固定した砂丘を表示・衝突・河床の双方へ渡す
	 * @return 同じ定義に固定した砂丘場
	 */
	const std::shared_ptr<const PlanetAeolianField> &PlanetErosionPreparation::GetAeolianField() const {
		// 河床と表示に共有する砂丘場の返戻
		return Aeolian;
	}

	/**
	 * 砂丘後の地面に合わせた河川縦断を共有する
	 * @return 同じ流域を持つ更新済みの水位と河床
	 */
	const std::shared_ptr<const DrainageNetwork> &PlanetErosionPreparation::GetDrainage() const {
		// 準備済み排水の返戻
		return Drainage;
	}

	/**
	 * 各段階の準備時間と離散的な土砂収支を取得する
	 * @return 連続化後の体積保証とは区別した準備統計
	 */
	const ErosionPreparationStatistics &PlanetErosionPreparation::GetStatistics() const {
		// 準備統計の返戻
		return Statistics;
	}
}
