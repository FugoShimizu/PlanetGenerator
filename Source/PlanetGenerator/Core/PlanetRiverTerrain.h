#pragma once

#include "PlanetDrainage.h"
#include "PlanetMesoRelief.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t RiverTerrainVersion = 22;

	struct BasinPreparationStatistics;

	class PlanetBasinFill;
	class PlanetErosionPreparation;

	struct RiverSurfaceSample {
		TerrainSample Terrain;
		double ChannelCoverage = 0;
		double WaterElevationMeters = 0;
		// 川底形状の被覆とは別に，河川又は大洋の水位が定義される支持範囲
		double WaterSupport = 0;
	};

	/** 水面位置に必要な値だけを持ち，法線用の微分を含めない標本 */
	struct RiverWaterHeights {
		double TerrainElevationMeters = 0;
		double WaterElevationMeters = 0;
		double WaterSupport = 0;
	};

	struct RiverTerrainChannel {
		Vec3 Start;
		Vec3 End;
		Vec3 Tangent;
		Vec3 Center;
		Vec3 BoundsMin;
		Vec3 BoundsMax;
		Vec3 MesoBoundsMin;
		Vec3 MesoBoundsMax;
		std::uint32_t ReachIndex = 0;
		double ArcAngle = 0;
		std::optional<RiverPlanform> Planform;
		double StartBedMeters = 0;
		double EndBedMeters = 0;
		double BedHalfWidthMeters = 0;
		double InnerHalfWidthMeters = 0;
		double OuterHalfWidthMeters = 0;
		double BedToBankChordRatio = 0;
		double InnerChordSquared = 0;
		double OuterChordSquared = 0;
		double MesoInnerMeters = 0;
		double MesoOuterMeters = 0;
	};

	struct RiverTerrainStatistics {
		std::uint32_t Channels = 0;
		std::uint32_t IndexNodes = 0;
		double MaximumValleyHalfWidthMeters = 0;
		double MaximumPreparedCutMeters = 0;
	};

	/** 最終地形の各合成段階を同じ地点評価から取り出す診断値 */
	struct RiverElevationStages {
		double BeforeBasinMeters = 0;
		double AfterBasinMeters = 0;
		double AfterValleyMeters = 0;
		double AfterFineReliefMeters = 0;
		double FinalMeters = 0;
	};

	/** 不変な基盤へ，河道・氾濫原・谷を有限幅の連続な関数として合成する */
	class RiverTerrainField {
	private:

		struct IndexNode {
			Vec3 Min;
			Vec3 Max;
			std::uint32_t Begin = 0;
			std::uint32_t Count = 0;
			std::uint32_t Left = 0;
			std::uint32_t Right = 0;
		};

		std::shared_ptr<const OceanConnectivityField> Ocean;
		std::shared_ptr<const DrainageNetwork> Network;
		std::shared_ptr<const PlanetErosionPreparation> Erosion;
		std::shared_ptr<const PlanetBasinFill> Basins;
		PlanetMesoRelief Meso;
		std::vector<RiverTerrainChannel> Valleys;
		std::vector<std::uint32_t> Order;
		std::vector<IndexNode> Index;
		RiverTerrainStatistics Stats;
		double ReliefBoundMeters = 0;

		template<bool Derivatives> TerrainSample EvaluateImpl(
			const UnitDirection Direction,
			const bool Indexed,
			std::size_t *const Candidates=nullptr,
			RiverSurfaceSample *const Surface=nullptr,
			RiverElevationStages *const Stages=nullptr
		) const;

		void BuildIndex();

		RiverTerrainField() = default;

	public:

		static std::optional<RiverTerrainField> Create(
			std::shared_ptr<const OceanConnectivityField> Ocean,
			std::shared_ptr<const DrainageNetwork> Drainage,
			const std::stop_token Stop={}
		);

		static std::optional<RiverTerrainField> CreateWithErosion(
			const std::shared_ptr<const OceanConnectivityField> &Ocean,
			const std::shared_ptr<const DrainageNetwork> &Drainage,
			const std::stop_token Stop={}
		);

		static std::optional<RiverTerrainField> CreateWithBasins(
			const std::shared_ptr<const OceanConnectivityField> &Ocean,
			const std::shared_ptr<const DrainageNetwork> &Drainage,
			const std::stop_token Stop={},
			BasinPreparationStatistics *Preparation=nullptr,
			std::size_t HeightCacheCapacity=2097152
		);

		/**
		 * 生成の元に為った地質を参照する
		 * @return 惑星の定義と地質基盤
		 */
		inline const GeologyField &Base() const {
			// 地質基盤の返戻
			return Ocean->Base();
		}

		/**
		 * 侵食前の海陸を参照する
		 * @return 全球の海域接続を含む基盤
		 */
		inline const OceanConnectivityField &OceanBase() const {
			// 海陸基盤の返戻
			return *Ocean;
		}

		/**
		 * 此の地形と同じ河床・水位を参照する
		 * @return 公開する地形と対応した排水
		 */
		inline const DrainageNetwork &Drainage() const {
			// 対応する排水の返戻
			return *Network;
		}

		/**
		 * 河道合成の統計を参照する
		 * @return 索引と谷幅の準備統計
		 */
		inline const RiverTerrainStatistics &Statistics() const {
			// 準備統計の返戻
			return Stats;
		}

		/**
		 * 有限幅の河道を参照する
		 * @return 河道番号順の形状定義
		 */
		inline std::span<const RiverTerrainChannel> Channels() const {
			// 河道形状の返戻
			return Valleys;
		}

		/**
		 * 表示の被覆判定に使う保守的な起伏幅を取得する
		 * @return 全球の起伏の絶対値上限
		 */
		inline double AbsoluteReliefBoundMeters() const {
			// 起伏の上限の返戻
			return ReliefBoundMeters;
		}

		/**
		 * 同じ地形の気候と流量を再準備せず共有する
		 * @return 侵食を含む準備結果，比較基準の Create では空
		 */
		inline const std::shared_ptr<const PlanetErosionPreparation> &GetErosionPreparation() const {
			// 共有する侵食準備の返戻
			return Erosion;
		}

		TerrainSample Evaluate(const UnitDirection Direction) const;
		RiverSurfaceSample EvaluateSurface(const UnitDirection Direction) const;
		/** 同じ式の値だけを評価し，地形・水位・支持範囲の倍精度値を維持する */
		RiverWaterHeights EvaluateWater(const UnitDirection Direction) const;
		double EvaluateElevation(const UnitDirection Direction) const;
		/** 河谷・細部起伏・中距離起伏の標高を同じ評価で測る */
		RiverElevationStages EvaluateElevationStages(const UnitDirection Direction) const;
		/** 盛土の接続前の補間面を診断する
		 * 盛土を持たない通常場では空 */
		std::optional<TerrainSample> EvaluateBasinReconstruction(const UnitDirection Direction) const;
		TerrainSample EvaluateReference(const UnitDirection Direction) const;
		std::size_t CandidateCount(const UnitDirection Direction) const;
		/**
		 * 球面方向の包囲箱と交差する河道候補を既存の索引から集める
		 * @param Min 箱の各軸の下限
		 * @param Max 箱の各軸の上限
		 * @param Candidates 出力先，既存の内容を置換える
		 * @return 有限且つ順序が正しい箱を照会した場合に真
		 */
		bool GatherChannelCandidates(const Vec3 Min, const Vec3 Max, std::vector<std::uint32_t> &Candidates) const;
	};
} // namespace PlanetCore
