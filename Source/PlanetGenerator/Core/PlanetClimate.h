#pragma once

#include "PlanetBiome.h"
#include "PlanetMoistureTransport.h"
#include "PlanetOcean.h"
#include <array>

namespace PlanetCore {
	// 地球型の参照気候，幾何の実半径とは別の生成仕様
	inline constexpr std::uint32_t ClimateGeneratorVersion = 1;

	struct ClimateSettings {
		Vec3 NorthAxis{ 0, 0, 1 };
		double AxialTiltRadians = 0.4090928;
		double ReferenceZonalWindMetersPerSecond = 8;
		double AngularDiffusionPerDay = 0.00015;

		/**
		 * 球面気候の設定を検査する
		 * @return 有限且つ対応範囲内で，北極軸が単位長の場合に真
		 */
		bool IsValid() const;
	};

	// 公開を拒否した段階
	enum class ClimatePreparationFailure : std::uint8_t { None, InvalidSettings, Cancelled, Geometry, Source, Transport, Climate };

	// 成功した期間数と輸送誤差，失敗時にも呼出側へ残す
	struct ClimatePreparationStatistics {
		std::uint32_t Nodes = 0, Links = 0, CompletedPeriods = 0, MaximumSweeps = 0;
		ClimatePreparationFailure Failure = ClimatePreparationFailure::InvalidSettings;
		MoistureTransportFailure TransportFailure = MoistureTransportFailure::None;
		double MaximumResidualMillimetersPerDay = 0, MaximumRelativeBudgetError = 0;
		double SurfaceAreaSteradians = 0, OceanAreaFraction = 0, GlobalAnnualPrecipitationMillimeters = 0;
	};

	/**
	 * 海陸・標高から準備する，不変な全球の季節気候
	 */
	class PlanetClimate {
	private:

		// 双対辺を通る保存流束の幾何
		struct TransportEdge {
			std::uint32_t First = 0, Second = 0;
			Vec3 IntegratedNormal;
			double DiffusionFactor = 0;
		};

		// 一定の球面分割と有界な近傍索引
		std::vector<UnitDirection> Directions;
		std::vector<ClimateCycle> Cycles;
		std::vector<std::uint32_t> Order;
		std::array<std::uint32_t, 4097> Offsets{};
		ClimatePreparationStatistics Statistics;
		std::uint64_t DefinitionHash = 0;
		double SupportChord = 0;

		/**
		 * 端点で二階微分迄接続する遷移を評価する
		 * @param Lower 遷移の下端
		 * @param Upper 遷移の上端
		 * @param Value 評価値
		 * @return ０～１の遷移量
		 */
		static double SmoothTransition(const double Lower, const double Upper, const double Value);

		/**
		 * 単位球の三角形の面積を求める
		 * @param First 第１頂点の単位方向
		 * @param Second 第２頂点の単位方向
		 * @param Third 第３頂点の単位方向
		 * @return 球面過剰に依る面積［sr］
		 */
		static double SphericalArea(const Vec3 First, const Vec3 Second, const Vec3 Third);

		/**
		 * 全球の面積と輸送辺を準備する
		 * @param Sphere 固定した球面網
		 * @param Cells 面積の出力先
		 * @param Support 補間が途切れない支持範囲の出力先
		 * @return 閉じた輸送辺，不正な球面の場合は空
		 */
		static std::optional<std::vector<TransportEdge>> BuildTransportEdges(
			const SphereMesh &Sphere, std::vector<MoistureTransportCell> &Cells, double &Support
		);

		/**
		 * 単位球の座標を索引セルへ対応させる
		 * @param Coordinate 単位球又は検索境界の座標
		 * @return ０～１５のセル座標
		 */
		static std::uint32_t GridCoordinate(const double Coordinate);

		/**
		 * 有限範囲の連続な重みで気候を補間する
		 * @param Direction 評価方向
		 * @param IsIndexed 索引を使う場合に真，偽なら独立比較用の全点探索
		 * @return 正常に補間出来た気候，失敗時は空
		 */
		std::optional<ClimateCycle> SampleImpl(const UnitDirection Direction, const bool IsIndexed) const;

		/**
		 * 描画分割から独立した近傍索引を構築する
		 */
		void BuildIndex();

		/**
		 * 日射と地形から温度・蒸発散の周期を準備する
		 * @param Ocean 侵食前の海陸と標高
		 * @param Settings 参照気候の設定
		 * @param Heights 参照縮尺の陸上標高の出力先
		 * @param IsOcean 各地点が海かを示す出力先
		 * @param Stop 中断要求
		 * @return 有効な気候入力を準備出来た場合に真
		 */
		bool PrepareSeasons(
			const OceanConnectivityField &Ocean, const ClimateSettings &Settings, std::vector<double> &Heights,
			std::vector<bool> &IsOcean, const std::stop_token Stop
		);

		/** 未準備の気候場の外部生成を防ぐ */
		PlanetClimate() = default;

	public:

		/**
		 * 全球の１２期の気候を準備する
		 * @param Ocean 侵食前の海陸・標高
		 * @param Settings 地球型の参照気候の設定
		 * @param Stop 中断要求
		 * @param Diagnostics 成功又は失敗の輸送証跡，省略可能
		 * @return 全期間で収束した気候，拒否・中断・未収束時は空
		 */
		static std::optional<PlanetClimate> Create(
			const OceanConnectivityField &Ocean, const ClimateSettings &Settings = {}, const std::stop_token Stop = {},
			ClimatePreparationStatistics *const Diagnostics = nullptr
		);

		/**
		 * 日平均の日射投影を幾何から求める
		 * @param SineLatitude 緯度の正弦，－１～１
		 * @param SineDeclination 太陽赤緯の正弦，－１～１
		 * @return 単位法線への日平均投影，範囲外の引数では空
		 */
		static std::optional<double> DailyMeanSolarProjection(const double SineLatitude, const double SineDeclination);

		/**
		 * 任意方向の気候を評価する
		 * @param Direction 評価方向
		 * @return 補間した１２期の気候
		 */
		std::optional<ClimateCycle> Sample(const UnitDirection Direction) const;

		/**
		 * 索引を使わず独立比較用の気候を評価する
		 * @param Direction 評価方向
		 * @return 全点探索で補間した１２期の気候
		 */
		std::optional<ClimateCycle> SampleReference(const UnitDirection Direction) const;

		/**
		 * 補間した気候から地表のバイオーム応答を準備する
		 * @param Direction 評価方向
		 * @return 水収支の周期解を持つ応答，準備出来ない場合は空
		 */
		std::optional<BiomeProfile> SampleBiome(const UnitDirection Direction) const;

		/**
		 * 準備した場の生成定義を取得する
		 * @return 地質・海洋・気候の仕様版と全設定を含む識別値
		 */
		std::uint64_t IdentityHash() const;

		/**
		 * 準備時の収束証跡を取得する
		 * @return 全期間の輸送誤差と全球積算値
		 */
		const ClimatePreparationStatistics &PreparationStatistics() const;
	};
}
