#include "PlanetMoistureTransport.h"
#include <algorithm>
#include <cmath>
#include <tuple>

namespace PlanetCore {
	bool MoistureTransportResult::HasConverged() const { return Failure == MoistureTransportFailure::None; }

	bool PlanetMoistureTransport::HasValidSettings(const MoistureTransportSettings &Settings) {
		// 対応範囲の判定結果の返戻
		return Settings.MaximumSweeps && Settings.MaximumSweeps < 16385 &&
			std::isfinite(Settings.ResidualToleranceMillimetersPerDay) &&
			Settings.ResidualToleranceMillimetersPerDay >= 1.0E-12 && Settings.ResidualToleranceMillimetersPerDay <= 1.0E-3 &&
			std::isfinite(Settings.RelativeBudgetTolerance) &&
			Settings.RelativeBudgetTolerance >= 1.0E-12 && Settings.RelativeBudgetTolerance <= 1.0E-3;
	}

	bool PlanetMoistureTransport::HasValidCell(const MoistureTransportCell &Cell) {
		// 係数と面積の判定結果の返戻
		return std::isfinite(Cell.AreaSteradians) && Cell.AreaSteradians >= 1.0E-12 && Cell.AreaSteradians <= 13 &&
			std::isfinite(Cell.EvaporationMillimetersPerDay) &&
			Cell.EvaporationMillimetersPerDay >= 0 && Cell.EvaporationMillimetersPerDay <= 10000 &&
			std::isfinite(Cell.CondensationPerDay) && Cell.CondensationPerDay >= 1.0E-6 && Cell.CondensationPerDay <= 1000 &&
			std::isfinite(Cell.ReevaporationPerDay) && Cell.ReevaporationPerDay >= 0 && Cell.ReevaporationPerDay <= 1000 &&
			std::isfinite(Cell.PrecipitationPerDay) && Cell.PrecipitationPerDay >= 1.0E-6 && Cell.PrecipitationPerDay <= 1000;
	}

	bool PlanetMoistureTransport::TryBuildRows(
		const std::span<const MoistureTransportCell> Cells,
		const std::span<const MoistureTransportLink> Links,
		std::vector<EquationRow> &Rows
	) {
		// 対の向きと順序を固定し，要求順に依る加算順の変更を防ぐ
		std::vector<MoistureTransportLink> Ordered(Links.begin(), Links.end());
		for(MoistureTransportLink &Link : Ordered) {
			// 不正な隣接対が有る場合の返戻
			if(Link.First >= Cells.size() || Link.Second >= Cells.size() || Link.First == Link.Second ||
				!std::isfinite(Link.ForwardSteradiansPerDay) || !std::isfinite(Link.ReverseSteradiansPerDay) ||
				Link.ForwardSteradiansPerDay < 0 || Link.ForwardSteradiansPerDay > 13 ||
				Link.ReverseSteradiansPerDay < 0 || Link.ReverseSteradiansPerDay > 13) {
				return false;
			}
			if(Link.First > Link.Second) {
				std::swap(Link.First, Link.Second);
				std::swap(Link.ForwardSteradiansPerDay, Link.ReverseSteradiansPerDay);
			}
		}
		// 地点番号に依る正準順の比較
		std::sort(Ordered.begin(), Ordered.end(),
			[](const MoistureTransportLink &First, const MoistureTransportLink &Second) -> bool {
				return std::tie(First.First, First.Second) < std::tie(Second.First, Second.Second);
			}
		);
		Rows.resize(Cells.size());
		for(std::size_t i = 0; i < Ordered.size(); ++i) {
			const MoistureTransportLink &Link = Ordered[i];
			// 重複した隣接対が有る場合の返戻
			if(i && Link.First == Ordered[i - 1].First && Link.Second == Ordered[i - 1].Second) return false;
			Rows[Link.First].OutgoingPerDay += Link.ForwardSteradiansPerDay / Cells[Link.First].AreaSteradians;
			Rows[Link.Second].OutgoingPerDay += Link.ReverseSteradiansPerDay / Cells[Link.Second].AreaSteradians;
			Rows[Link.First].Incoming.push_back({ Link.Second, Link.ReverseSteradiansPerDay / Cells[Link.First].AreaSteradians });
			Rows[Link.Second].Incoming.push_back({ Link.First, Link.ForwardSteradiansPerDay / Cells[Link.Second].AreaSteradians });
		}
		// 準備成功の返戻
		return true;
	}

	void PlanetMoistureTransport::MeasureResiduals(
		const std::span<const MoistureTransportCell> Cells,
		const std::vector<EquationRow> &Rows,
		MoistureTransportResult &Result
	) {
		Result.MaximumResidualMillimetersPerDay = 0;
		Result.EvaporationAreaIntegral = 0;
		Result.PrecipitationAreaIntegral = 0;
		for(std::size_t i = 0; i < Cells.size(); ++i) {
			// 大気柱毎の独立した水分収支
			const MoistureTransportCell &Cell = Cells[i];
			const EquationRow &Row = Rows[i];
			const MoistureTransportColumn &Column = Result.Columns[i];
			double IncomingVapor = 0, IncomingCloud = 0;
			for(const IncomingLink &Link : Row.Incoming) {
				IncomingVapor += Link.PerDay * Result.Columns[Link.Source].VaporMillimeters;
				IncomingCloud += Link.PerDay * Result.Columns[Link.Source].CloudMillimeters;
			}
			const double Exchange = Cell.CondensationPerDay * Column.VaporMillimeters -
				Cell.ReevaporationPerDay * Column.CloudMillimeters;
			const double Precipitation = Cell.PrecipitationPerDay * Column.CloudMillimeters;
			Result.MaximumResidualMillimetersPerDay = std::max(
				{
					Result.MaximumResidualMillimetersPerDay,
					std::abs(Cell.EvaporationMillimetersPerDay + IncomingVapor - Row.OutgoingPerDay * Column.VaporMillimeters - Exchange),
					std::abs(IncomingCloud + Exchange - Row.OutgoingPerDay * Column.CloudMillimeters - Precipitation)
				}
			);
			Result.EvaporationAreaIntegral += Cell.AreaSteradians * Cell.EvaporationMillimetersPerDay;
			Result.PrecipitationAreaIntegral += Cell.AreaSteradians * Precipitation;
		}
		Result.RelativeBudgetError = std::abs(Result.EvaporationAreaIntegral - Result.PrecipitationAreaIntegral) /
			std::max(1.0E-12, Result.EvaporationAreaIntegral);
		// 終了
		return;
	}

	MoistureTransportResult PlanetMoistureTransport::Solve(
		const std::span<const MoistureTransportCell> Cells,
		const std::span<const MoistureTransportLink> Links,
		const MoistureTransportSettings &Settings,
		const std::stop_token Stop
	) {
		// 例外を無効化した UE の非同期生成へ，拒否理由を値で伝える
		MoistureTransportResult Result;
		// 対応範囲外の場合の返戻
		if(!HasValidSettings(Settings) || Cells.empty() || Cells.size() > 40962 || Links.size() > (Cells.size() << 3)) return Result;
		for(const MoistureTransportCell &Cell : Cells) {
			// 不正な係数が有る場合の返戻
			if(!HasValidCell(Cell)) return Result;
		}
		std::vector<EquationRow> Rows;
		// 不正な輸送行列の場合の返戻
		if(!TryBuildRows(Cells, Links, Rows)) return Result;
		Result.Columns.resize(Cells.size());
		Result.Failure = MoistureTransportFailure::NotConverged;
		for(std::uint32_t Sweep = 0; Sweep < Settings.MaximumSweeps; ++Sweep) {
			if(Stop.stop_requested()) {
				Result.Failure = MoistureTransportFailure::Cancelled;
				// 中断結果の返戻
				return Result;
			}
			// 各地点の水蒸気・雲水を同時に解き，強い再蒸発でも負値を作らない
			for(std::size_t i = 0; i < Cells.size(); ++i) {
				const MoistureTransportCell &Cell = Cells[i];
				const EquationRow &Row = Rows[i];
				double VaporSource = Cell.EvaporationMillimetersPerDay, CloudSource = 0;
				for(const IncomingLink &Link : Row.Incoming) {
					VaporSource += Link.PerDay * Result.Columns[Link.Source].VaporMillimeters;
					CloudSource += Link.PerDay * Result.Columns[Link.Source].CloudMillimeters;
				}
				// 相殺する積を引かず，同値な正値項の和で行列式を評価する
				const double Determinant = Row.OutgoingPerDay * (Row.OutgoingPerDay + Cell.CondensationPerDay +
					Cell.ReevaporationPerDay + Cell.PrecipitationPerDay) + Cell.CondensationPerDay * Cell.PrecipitationPerDay;
				MoistureTransportColumn &Column = Result.Columns[i];
				Column.VaporMillimeters = ((Row.OutgoingPerDay + Cell.ReevaporationPerDay + Cell.PrecipitationPerDay) *
					VaporSource + Cell.ReevaporationPerDay * CloudSource) / Determinant;
				Column.CloudMillimeters = ((Row.OutgoingPerDay + Cell.CondensationPerDay) * CloudSource +
					Cell.CondensationPerDay * VaporSource) / Determinant;
				if(!std::isfinite(Column.VaporMillimeters) || !std::isfinite(Column.CloudMillimeters) ||
					Column.VaporMillimeters < 0 || Column.CloudMillimeters < 0) {
					Result.Failure = MoistureTransportFailure::NumericalFailure;
					// 数値異常の返戻
					return Result;
				}
			}
			Result.Sweeps = Sweep + 1;
			if(!(Result.Sweeps % 8) || Result.Sweeps == Settings.MaximumSweeps) {
				MeasureResiduals(Cells, Rows, Result);
				if(Result.MaximumResidualMillimetersPerDay <= Settings.ResidualToleranceMillimetersPerDay &&
					Result.RelativeBudgetError <= Settings.RelativeBudgetTolerance) {
					Result.Failure = MoistureTransportFailure::None;
					// 収束した水分場の返戻
					return Result;
				}
			}
		}
		// 未収束結果の返戻
		return Result;
	}
}
