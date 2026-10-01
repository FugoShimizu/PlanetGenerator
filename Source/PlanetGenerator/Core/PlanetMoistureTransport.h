#pragma once

#include <cstdint>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	// 大気柱の面積・水分供給・相互変換係数
	struct MoistureTransportCell {
		double AreaSteradians = 1;
		double EvaporationMillimetersPerDay = 0;
		double CondensationPerDay = 0.5;
		double ReevaporationPerDay = 10;
		double PrecipitationPerDay = 2;
	};

	// 隣接大気柱間の移流・拡散，体積流束は単位球面積／日
	struct MoistureTransportLink {
		std::uint32_t First = 0;
		std::uint32_t Second = 0;
		double ForwardSteradiansPerDay = 0;
		double ReverseSteradiansPerDay = 0;
	};

	// 定常解の収束条件
	struct MoistureTransportSettings {
		std::uint32_t MaximumSweeps = 4096;
		double ResidualToleranceMillimetersPerDay = 1.0E-9;
		double RelativeBudgetTolerance = 1.0E-8;
	};

	// 鉛直積算した水量
	struct MoistureTransportColumn {
		double VaporMillimeters = 0;
		double CloudMillimeters = 0;
	};

	// 失敗理由，未収束の列は描画・生成へ公開しない
	enum class MoistureTransportFailure : std::uint8_t { None, InvalidInput, Cancelled, NumericalFailure, NotConverged };

	struct MoistureTransportResult {
		// 計算結果と収束証跡
		MoistureTransportFailure Failure = MoistureTransportFailure::InvalidInput;
		std::vector<MoistureTransportColumn> Columns;
		std::uint32_t Sweeps = 0;
		double MaximumResidualMillimetersPerDay = 0;
		double EvaporationAreaIntegral = 0; // mm・sr／日，m³／日への換算は半径［m］の二乗と 0.001 を掛ける
		double PrecipitationAreaIntegral = 0;
		double RelativeBudgetError = 0;

		/**
		 * 計算結果を利用可能か判定する
		 * @return 全ての収束条件を満たした場合に真
		 */
		bool HasConverged() const;
	};

	/**
	 * 閉じた領域の水蒸気・雲水を輸送する
	 */
	class PlanetMoistureTransport {
	private:

		// 行列の非対角項，列番号は入力地点の正準順
		struct IncomingLink {
			std::uint32_t Source = 0;
			double PerDay = 0;
		};

		struct EquationRow {
			std::vector<IncomingLink> Incoming;
			double OutgoingPerDay = 0;
		};

		/**
		 * 計算条件を検査する
		 * @param Settings 反復上限と収束条件
		 * @return 条件が有限かつ対応範囲内の場合に真
		 */
		static bool HasValidSettings(const MoistureTransportSettings &Settings);

		/**
		 * 大気柱の係数を検査する
		 * @param Cell 大気柱の面積・供給・相互変換係数
		 * @return 正値性を保つ対応範囲内の場合に真
		 */
		static bool HasValidCell(const MoistureTransportCell &Cell);

		/**
		 * 輸送行列を準備する
		 * @param Cells 大気柱の正準配列
		 * @param Links 隣接対毎の双方向流束
		 * @param Rows 出力する行列
		 * @return 重複や不正な流束が無い場合に真
		 */
		static bool TryBuildRows(
			const std::span<const MoistureTransportCell> Cells,
			const std::span<const MoistureTransportLink> Links,
			std::vector<EquationRow> &Rows
		);

		/**
		 * 方程式と収支の誤差を更新する
		 * @param Cells 大気柱の係数
		 * @param Rows 輸送行列
		 * @param Result 検査対象と更新先
		 */
		static void MeasureResiduals(
			const std::span<const MoistureTransportCell> Cells,
			const std::vector<EquationRow> &Rows,
			MoistureTransportResult &Result
		);

	public:

		/**
		 * 閉じた輸送網の定常水分場を求める
		 * @param Cells 大気柱の正準配列，１～４０，９６２地点
		 * @param Links 重複しない隣接対と非負の流束
		 * @param Settings 反復上限と収束条件
		 * @param Stop 中断要求
		 * @return 水分場・収束証跡・失敗理由
		 */
		static MoistureTransportResult Solve(
			const std::span<const MoistureTransportCell> Cells,
			const std::span<const MoistureTransportLink> Links,
			const MoistureTransportSettings &Settings = {},
			const std::stop_token Stop = {}
		);
	};
}
