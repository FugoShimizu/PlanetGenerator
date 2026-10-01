#pragma once

#include "PlanetOcean.h"
#include "PlanetRiverPlanform.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	class SphericalSplineField;

	inline constexpr std::uint32_t DrainageVersion = 11;

	struct DrainageSettings {
		// 惑星尺度と河道の物理条件
		double RadiusMeters = 1000000;
		double SeaLevelMeters = 0;
		double MinimumSlope = 1E-7;
		double BankFreeboardMeters = 0.3;
		double ChannelAreaFraction = 0.0005;
		bool PreferBreaching = false;
		bool UseCurvedPlanforms = false;
		std::uint64_t PlanformSeed = 0;
		bool IsValid() const;
	};

	struct DrainageSite {
		// 排水入力の方向・標高・面積
		Vec3 Direction;
		double ElevationMeters = 0;
		double AreaSquareMeters = 0;
		bool Ocean = false;
	};

	struct DrainageNode {
		// 出口までの共有水位と集水情報
		Vec3 Direction;
		double OriginalElevationMeters = 0;
		double WaterMeters = 0;
		double DepthMeters = 0;
		double ContributingAreaSquareMeters = 0;
		double DistanceToOceanMeters = 0;
		std::uint32_t Downstream = 0;
		std::uint32_t Outlet = 0;
		bool Ocean = false;
	};

	/** 水位の上限制約と地形の横断形状で共有する河床・岸の半幅 */
	struct RiverCrossSectionWidths {
		// 河床と内岸の幅
		double BedHalfWidthMeters = 0;
		double InnerHalfWidthMeters = 0;
	};

	struct RiverReach {
		// 河道の接続と海岸での切詰め
		std::uint32_t Upstream = 0;
		std::uint32_t Downstream = 0;
		double LengthMeters = 0;
		// 排水先は Downstream を保ち，形状だけを海岸の河口で切詰める
		double EndFraction = 1;
		Vec3 EndDirection;
		std::optional<RiverPlanform> Planform;
		double PlanformEndFraction = 1;

		/**
		 * 縦断の評価に用いる公称河道長を取得する
		 * @return 海岸での切詰めを含む公称長，蛇行の実接線長は位置毎に異なる
		 */
		inline double ProfileLengthMeters() const {
			// 実河道長の返戻
			return LengthMeters * EndFraction;
		}
	};

	struct ShoreCrossing {
		// 海岸交差の位置と絞込残差
		Vec3 Direction;
		double LandFraction = 0;
		double WetFraction = 0;
		double BracketWidthMeters = 0;
		double TerrainResidualMeters = 0;
	};

	std::optional<ShoreCrossing> FindShoreCrossing(
		const UnitDirection Start,
		const UnitDirection End,
		const double Radius,
		const double Sea,
		const std::function<double(UnitDirection)> &Height,
		const std::stop_token Stop={}
	);

	/** 曲線と海岸探索の公称長・微分上限を共有して，実距離の挟込みを保つ */
	std::optional<ShoreCrossing> FindShoreCrossingOnCurve(
		const std::function<UnitDirection(double)> &Curve,
		double ParameterLengthMeters,
		double MaximumStretch,
		double Radius,
		double Sea,
		const std::function<double(UnitDirection)> &Height,
		std::stop_token Stop={}
	);

	struct RiverMouth {
		// 河道番号と実海岸
		std::uint32_t Reach = 0;
		ShoreCrossing Shore;
	};

	struct RiverProfileSample {
		// 縦断標本の位置・水位・傾斜
		Vec3 Direction;
		Vec3 DownstreamTangent;
		double WaterMeters = 0;
		double BedMeters = 0;
		double DownstreamWaterSlope = 0;
	};

	struct RiverHeights {
		// 縦断の値と一次微分
		double WaterMeters = 0;
		double BedMeters = 0;
		double WaterSlope = 0;
		double BedSlope = 0;
	};

	struct RiverLongitudinalProfile {
		// 共有縦断の区間係数と端点
		static constexpr std::uint32_t Segments = 64;
		std::array<double, Segments + 1> Headroom {};
		std::array<double, Segments + 1> Derivative {};
		double GradeStart = 0;
		double GradeEnd = 0;
		double LengthMeters = 0;
		double StartWater = 0;
		double EndWater = 0;
		double StartDepth = 0;
		double EndDepth = 0;
		double MaximumSampledBedCutMeters = 0;
		RiverHeights Evaluate(const double T) const;
	};

	struct DrainageStatistics {
		// 準備量と物理残差の統計
		std::uint32_t Nodes = 0;
		std::uint32_t OceanNodes = 0;
		std::uint32_t LandNodes = 0;
		std::uint32_t Reaches = 0;
		std::uint32_t CoastalOutletNodes = 0;
		double LandAreaSquareMeters = 0;
		double OutletAreaSquareMeters = 0;
		double MaximumRequiredBankRaiseMeters = 0;
		double AreaWeightedBankRaiseMeters = 0;
		double MaximumRequiredWaterCutMeters = 0;
		double MaximumUnresolvedKnotBedGapMeters = 0;
		double MaximumProfileWaterLoweringMeters = 0;
		double MaximumSampledWaterAboveTerrainMeters = 0;
		std::uint32_t ProfileKnots = 0;
		std::uint64_t ProfileTerrainEvaluations = 0;
		double LongestRouteMeters = 0;
		double MaximumMouthBracketWidthMeters = 0;
		double MaximumMouthHeightResidualMeters = 0;
	};

	/**
	 * 不変な排水木，海岸で切詰めた河道曲線と共有合流点を持つ縦断曲線
	 * 此の木と曲線だけでは最終地形 H を変更せず，大洋の連結と実際の河床・景観は別に検証する
	 */
	class DrainageNetwork {
	private:

		enum class OceanConnectivityEvidence { Graph, ConnectedSourceField };
		// 不変な地形定義と共有排水データ
		std::uint64_t SourceHash = 0;
		DrainageSettings Definition;
		DrainageStatistics Stats;
		std::vector<DrainageNode> Graph;
		std::vector<RiverReach> Rivers;
		std::vector<RiverMouth> CoastalMouths;
		std::vector<RiverLongitudinalProfile> Longitudinal;
		bool BalancedProfiles = false;

		// 海域証拠の種類を明示する内部準備
		static std::optional<DrainageNetwork> SolveImpl(
			std::span<const DrainageSite> Sites,
			std::span<const std::array<std::uint32_t, 2>> Edges,
			const DrainageSettings &S,
			const OceanConnectivityEvidence Evidence,
			const std::stop_token Stop
		);

		static std::optional<DrainageNetwork> CreateImpl(
			const OceanConnectivityField &Ocean,
			const std::function<double(UnitDirection)> &Height,
			const bool SharedSourceHeight,
			const std::stop_token Stop
		);

		static std::optional<DrainageNetwork> CreateOnMesh(
			const OceanConnectivityField &Ocean,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::function<double(UnitDirection)> &Height,
			const bool SharedSourceHeight,
			const std::stop_token Stop
		);

	public:

		// 公開する生成経路
		static std::optional<DrainageNetwork> Create(const OceanConnectivityField &Ocean, const std::stop_token Stop={});

		static std::optional<DrainageNetwork> CreateWithRefinedEdges(
			const OceanConnectivityField &Ocean,
			const std::span<const std::array<std::uint32_t, 2>> Edges,
			const std::stop_token Stop={}
		);

		static std::optional<DrainageNetwork> CreateWithSamplingSites(
			const OceanConnectivityField &Ocean,
			const std::span<const UnitDirection> Samples,
			const std::stop_token Stop={}
		);

		static std::optional<DrainageNetwork> CreateForHeight(
			const OceanConnectivityField &Ocean,
			const std::function<double(UnitDirection)> &Height,
			const std::stop_token Stop={}
		);

		/** 同じ球面補間を海陸分類・下流・海岸・縦断の全段階へ用いる */
		static std::optional<DrainageNetwork> CreateForSphericalSurface(
			const OceanConnectivityField &Ocean,
			const SphericalSplineField &Surface,
			const std::stop_token Stop={}
		);

		/** 適応網を共有し，節点以外の海岸と縦断も同じ補間場へ接続する */
		static std::optional<DrainageNetwork> CreateForSphericalSurfaceOnMesh(
			const OceanConnectivityField &Ocean,
			const SphericalSplineField &Surface,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::stop_token Stop={}
		);

		static std::optional<DrainageNetwork> Solve(
			std::span<const DrainageSite> Sites,
			std::span<const std::array<std::uint32_t, 2>> Edges,
			const DrainageSettings &S,
			const std::stop_token Stop={}
		);

		/**
		 * 固定網の地点を参照する
		 * @return 地点番号順の排水
		 */
		inline std::span<const DrainageNode> Nodes() const {
			// 排水地点の返戻
			return Graph;
		}

		/**
		 * 選択した河道を参照する
		 * @return 下流で閉じた河道集合
		 */
		inline std::span<const RiverReach> Reaches() const {
			// 河道集合の返戻
			return Rivers;
		}

		/**
		 * 切詰めた河口を参照する
		 * @return 河口と海岸の挟込み
		 */
		inline std::span<const RiverMouth> Mouths() const {
			// 海岸に接続した河口の返戻
			return CoastalMouths;
		}

		/**
		 * 排水と縦断形状の統計を参照する
		 * @return 未解決の河床差も含む準備統計
		 */
		inline const DrainageStatistics &Statistics() const {
			// 準備統計の返戻
			return Stats;
		}

		/**
		 * 元の海陸の生成定義を識別する
		 * @return 地質の生成定義の識別値，任意グラフでは０
		 */
		inline std::uint64_t SourceDefinitionHash() const {
			// 海陸の識別値の返戻
			return SourceHash;
		}

		/**
		 * 排水に用いた設定を取得する
		 * @return 不変な設定
		 */
		inline const DrainageSettings &Settings() const {
			// 設定の返戻
			return Definition;
		}

		std::optional<DrainageNetwork> WithCoastalOutlets(
			const std::function<double(UnitDirection)> &Height,
			const std::stop_token Stop={}
		) const;

		std::optional<DrainageNetwork> WithPromotedTributaries(
			std::span<const std::uint32_t> UpstreamNodes,
			const OceanConnectivityField &Ocean,
			const std::stop_token Stop={}
		) const;

		std::optional<DrainageNetwork> WithTerrainProfiles(
			const std::function<double(UnitDirection)> &Height,
			const std::stop_token Stop={}
		) const;

		std::optional<DrainageNetwork> WithBalancedTerrainProfiles(
			const std::function<double(UnitDirection)> &Height,
			const std::stop_token Stop={}
		) const;

		/**
		 * 盛土を伴う縦断の再構成が必要か確認する
		 * @return 開削専用の地形合成へ渡せない場合に真
		 */
		inline bool RequiresBalancedTerrain() const {
			// 地形再構成が必要な縦断種別の返戻
			return BalancedProfiles;
		}

		std::optional<RiverHeights> SampleHeights(const std::size_t Index, const double T) const;
		std::optional<RiverCrossSectionWidths> ChannelWidths(const std::size_t Index) const;

		/**
		 * 準備済みの縦断曲線を参照する
		 * @return 河道番号順の区分曲線
		 */
		inline std::span<const RiverLongitudinalProfile> Profiles() const {
			// 縦断曲線の返戻
			return Longitudinal;
		}

		std::optional<RiverProfileSample> SampleReach(const std::size_t Index, const double T) const;
	};
}
