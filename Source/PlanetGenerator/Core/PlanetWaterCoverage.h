#pragma once

#include "PlanetGround.h"
#include "PlanetLod.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 排水骨格の固定段階と現在の近景を含む，完全且つ平衡な分割を準備する
	 * @param NearTree 保持する近景分割
	 * @param MinimumLevel 固定骨格の段階，最大６
	 * @param Stop 中断要求
	 * @return 私有コピー，予算超過・不正・中断は空
	 * @note 生成地形の保存ではなく，有界な共通頂点と接続の準備
	 */
	std::optional<LodTree> PrepareWaterSkeletonTree(
		const LodTree &NearTree,
		std::uint8_t MinimumLevel,
		std::stop_token Stop={}
	);

	/** 共通分割の頂点で評価した水面位置と符号付き水深，正の水深だけを描画する */
	struct WaterCoverageSample {
		Vec3 Position;
		double DepthMeters = 0;
	};

	/** 属性も同じ辺で補間する為，切出した頂点と元の頂点番号・補間率を保持する */
	struct WaterCoverageVertex {
		Vec3 Position;
		std::uint32_t SourceA = 0;
		std::uint32_t SourceB = 0;
		double Fraction = 0;
	};

	struct WaterCoverageMesh {
		std::vector<WaterCoverageVertex> Vertices;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
		std::vector<std::uint32_t> SourceTriangles;
	};

	/**
	 * 共通の三角形分割上の区分線形な水深から，一意な水際を共有する水面だけを切出す
	 * @param Samples 同じ座標基準の水面位置と符号付き水深
	 * @param Triangles 向きと共有辺を整合させた元の分割
	 * @param Stop 中断要求
	 * @return 有界な水面，不正入力・中断・予算超過・数値的退化時は空
	 * @note 連続な地形の未標本部分，孤立水域の排除，描画精度は呼出側で検証する
	 */
	std::optional<WaterCoverageMesh> BuildWaterCoverage(
		std::span<const WaterCoverageSample> Samples,
		std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop={}
	);

	/** 細分化後の参照元も保持し，水際の属性補間の寿命を保証する */
	struct AdaptiveWaterCoverage {
		SphereMesh Partition;
		WaterCoverageMesh Wet;
		std::optional<SphereRefinementHierarchy> Hierarchy;
		std::uint32_t RefinementPasses = 0;
		double MaximumFloatErrorMeters = 0;
	};

	/**
	 * 短い大円弧が球面三角形の内側又は境界を通るかを保守的に判定する
	 * @param Start 河道の始点方向
	 * @param End 河道の終点方向，始点との角度は円周の半分未満
	 * @param Triangle 同じ半球に収まる，非退化な球面三角形
	 * @return 交差又は数値境界に接する場合に真
	 */
	bool ArcIntersectsSphericalTriangle(UnitDirection Start, UnitDirection End, const std::array<UnitDirection, 3> &Triangle);

	/**
	 * 弧と三角形が交差する区間の全体が，頂点の線形水深で覆われるか調べる
	 * @param Start 弧の始点
	 * @param End 弧の終点
	 * @param Triangle 同じ半球の非退化な三角形
	 * @param Depths 各頂点の水深を実半径で割り，共通の正定数を掛けた射影水深
	 * @return 非交差又は区間全体の水深が正の場合に真
	 * @note 連続な元地形の水深保証ではなく，現在の区分線形分割の被覆判定
	 */
	bool ArcCoveredByLinearWater(
		UnitDirection Start,
		UnitDirection End,
		const std::array<UnitDirection, 3> &Triangle,
		const std::array<double, 3> &Depths
	);

	using WaterDetailEvaluator =
	std::function<double(UnitDirection, double, const std::array<UnitDirection, 3> &, const std::array<double, 3> &)>;

	/** 水際の画面上の細かさと，確実に視界を遮る惑星内部球 */
	struct WaterCoverageView {
		Vec3 ObserverMeters;
		double FocalLengthPixels = 540;
		double MaximumShoreEdgePixels = 8;
		double OccluderRadiusMeters = 0;
	};

	/** 有界な反復毎の最大水際，階層経路の葉段階，又は非階層経路の完全分割回数 */
	struct WaterRefinementPass {
		std::uint32_t Pass = 0;
		std::uint32_t RefinementLevel = 0;
		std::size_t SourceVertices = 0;
		std::size_t FailedTriangles = 0;
		double Pixels = 0;
		Vec3 CenterMeters;
	};

	/** 階層分割の拒否で最後に試した段階，他の拒否は Unspecified */
	enum class WaterRefinementStage { Unspecified, Complete, OwnerRefinement, MeshRebuild, OutputBudget };

	/** 精度不足と資源制限を区別する診断値，失敗時も呼出側へ残す */
	struct WaterRefinementDiagnostics {
		/** 初期標本・切出し・条件検査・細分化の累積時間，単位は ms */
		std::array<double, 4> StageMilliseconds {};
		std::uint32_t Pass = 0;
		WaterRefinementStage RefinementStage = WaterRefinementStage::Unspecified;
		LodEditStatus LastLodEditStatus = LodEditStatus::Applied;
		std::size_t FailedTriangles = 0;
		std::size_t FeatureTriangles = 0;
		std::size_t ProjectedEdges = 0;
		std::size_t SourceVertices = 0;
		std::size_t SourceTriangles = 0;
		std::size_t PlannedVertices = 0;
		std::size_t SourceCheckHits = 0;
		std::size_t SourceCheckEvaluations = 0;
		std::size_t VertexCheckHits = 0;
		std::size_t VertexCheckEvaluations = 0;
		Vec3 WorstShoreStartMeters;
		Vec3 WorstShoreEndMeters;
		std::array<Vec3, 3> WorstShoreSourceDirections;
		std::uint32_t WorstShoreRefinementLevel = 0;
		std::vector<WaterRefinementPass> Passes;
		double WaterErrorMeters = 0;
		double ShoreErrorMeters = 0;
		double FloatErrorMeters = 0;
		double MaximumShoreEdgePixels = 0;
	};

	/**
	 * 誤差が大きい面の所有葉を細分化し，共有辺の接合を再構築する
	 * @param Partition 向きと共有辺が整合した球面分割，階層を渡す場合は同じ木の頂点鍵を保持
	 * @param Frame 水面の局所座標
	 * @param Focus 精度を検査する中心
	 * @param NearRadiusMeters 有限標本で精度を検査する半径，零は近景検査無
	 * @param Evaluate 同じ不変場に基付く水面位置と符号付き水深
	 * @param Diagnostics 失敗時も残す最後の検査値
	 * @param Stop 中断要求
	 * @param View 水際の投影寸法を制限する視点，空は局所精度だけを検査
	 * @param RequiredEdge 球面の中心・包囲弦半径・三角形・射影水深から必要な辺長を返す，無制限は正の無限大
	 * @param Hierarchy 元の平衡分割と所有葉，空は解析入力用の共有中点分割
	 * @param ReuseSourceChecks 同一要求の同一面と水面頂点の検査結果を再利用，無効時は比較用の全検査
	 * @note Evaluate と RequiredEdge は同じ要求内で不変な結果を返す必要が有る
	 * @return 整合した水面と参照元，精度・資源制約を満たさない場合は空
	 * @note 有限標本に依る検査であり，未標本の細い流路の完全性は保証しない
	 */
	std::optional<AdaptiveWaterCoverage> BuildAdaptiveWaterCoverage(
		SphereMesh Partition,
		const GroundFrame &Frame,
		UnitDirection Focus,
		double NearRadiusMeters,
		const std::function<WaterCoverageSample(UnitDirection)> &Evaluate,
		WaterRefinementDiagnostics &Diagnostics,
		std::stop_token Stop={},
		const WaterCoverageView *View=nullptr,
		const WaterDetailEvaluator &RequiredEdge={},
		std::optional<SphereRefinementHierarchy> Hierarchy={},
		bool ReuseSourceChecks=true
	);
}
