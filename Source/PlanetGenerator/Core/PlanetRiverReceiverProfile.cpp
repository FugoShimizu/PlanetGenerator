#include "PlanetRiverReceiverProfile.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 確認済通路を細分化し，受入河川の実水位を終点として最小の下降水面を後退計算する
	 * @param Path 連続場で調べた元窪地から受入河川への折線
	 * @param Ground 表示・衝突と同じ最終地面の標高
	 * @param ReceiverWaterMeters 既存河川の実水位
	 * @param Settings 物理尺度・水深・勾配・標本予算
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @return 各標本で下降と水深を満たす水面；標本間や横断面の保証ではない
	 */
	std::optional<RiverReceiverProfile> PrepareRiverReceiverProfile(
		const std::span<const UnitDirection> Path,
		const std::function<double(UnitDirection)> &Ground,
		const double ReceiverWaterMeters,
		const RiverReceiverProfileSettings &Settings,
		RiverReceiverProfileError &Error,
		const std::stop_token Stop
	) {
		Error = RiverReceiverProfileError::None;
		if(Stop.stop_requested()) {
			Error = RiverReceiverProfileError::Cancelled;
			// 開始前の中断で部分的な縦断を公開しない返戻
			return {};
		}
		const bool Valid = Path.size() > 1 && Path.size() < 1000001 && Ground && std::isfinite(ReceiverWaterMeters) &&
		std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 &&
		std::isfinite(Settings.MaximumSampleStepMeters) && !(Settings.MaximumSampleStepMeters < 0.01) &&
		Settings.MaximumSampleStepMeters < Settings.RadiusMeters * 0.25 && std::isfinite(Settings.MinimumDepthMeters) &&
		Settings.MinimumDepthMeters > 0 && std::isfinite(Settings.MinimumDownstreamSlope) && Settings.MinimumDownstreamSlope > 0 &&
		Settings.MinimumDownstreamSlope < 1 && !(Settings.MaximumSamples < Path.size()) && Settings.MaximumSamples < 2000001;
		if(!Valid) {
			Error = RiverReceiverProfileError::InvalidInput;
			// 無効な経路又は尺度と予算への返戻
			return {};
		}
		RiverReceiverProfile Result;
		Result.Samples.reserve(std::min<std::size_t>(Settings.MaximumSamples, Path.size() * 4));
		const auto Append = [&](const UnitDirection Direction) -> bool {
			if(Stop.stop_requested() || Result.Samples.size() >= Settings.MaximumSamples) {
				Error = Stop.stop_requested() ? RiverReceiverProfileError::Cancelled : RiverReceiverProfileError::SampleBudget;
				// 中断又は標本上限の返戻
				return false;
			}
			double Distance = 0;
			if(!Result.Samples.empty()) {
				const Vec3 A = Result.Samples.back().Direction.Vector(), B = Direction.Vector();
				const double Length = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				if(!(Length > 0) || !std::isfinite(Length)) {
					Error = RiverReceiverProfileError::InvalidPath;
					// 重複又は非有限な折線への返戻
					return false;
				}
				Distance = Result.Samples.back().DistanceMeters + Length;
			}
			const double Elevation = Ground(Direction);
			if(!std::isfinite(Elevation)) {
				Error = RiverReceiverProfileError::InvalidTerrain;
				// 地形欠落の返戻
				return false;
			}
			Result.Samples.push_back({ Direction, Distance, Elevation, 0 });
			if(Result.Samples.size() == 1 || Elevation > Result.Statistics.MaximumGroundMeters) {
				Result.Statistics.MaximumGroundMeters = Elevation;
				Result.Statistics.CrestDistanceMeters = Distance;
			}
			if(Result.Samples.size() == 1 || Elevation < Result.Statistics.MinimumGroundMeters) {
				Result.Statistics.MinimumGroundMeters = Elevation;
			}
			// 有効な一点を追加した返戻
			return true;
		};
		for(std::size_t Edge = 1; Edge < Path.size(); ++Edge) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverProfileError::Cancelled;
				// 辺の途中で中断した準備への返戻
				return {};
			}
			const Vec3 A = Path[Edge - 1].Vector(), B = Path[Edge].Vector();
			const double Length = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
			if(!(Length > 0) || !std::isfinite(Length)) {
				Error = RiverReceiverProfileError::InvalidPath;
				// 停止又は不正な辺への返戻
				return {};
			}
			const double RequiredParts = std::ceil(Length / Settings.MaximumSampleStepMeters);
			if(!std::isfinite(RequiredParts) || RequiredParts > Settings.MaximumSamples - Result.Samples.size()) {
				Error = RiverReceiverProfileError::SampleBudget;
				// 過大な細分化予算への返戻
				return {};
			}
			const std::uint32_t Parts = std::max(1U, static_cast<std::uint32_t>(RequiredParts));
			for(std::uint32_t Part = 0; Part < Parts; ++Part) {
				const double T = static_cast<double>(Part) / Parts;
				const std::optional<UnitDirection> Direction = UnitDirection::FromVector(A * (1 - T) + B * T);
				if(!Direction) {
					Error = RiverReceiverProfileError::InvalidPath;
					// 球面上の補間が不可能な辺への返戻
					return {};
				}
				// 地形欠落又は中断に因る不完全な縦断への返戻
				if(!Append(*Direction)) return {};
			}
		}
		// 終点も水位境界として実地形上で照合する
		if(!Append(Path.back())) return {};
		RiverReceiverProfileSample &Terminal = Result.Samples.back();
		if(Terminal.GroundMeters + Settings.MinimumDepthMeters > ReceiverWaterMeters) {
			Error = RiverReceiverProfileError::ReceiverExposed;
			// 既存河川の水位で必要水深を確保出来ない返戻
			return {};
		}
		Terminal.WaterMeters = ReceiverWaterMeters;
		Result.Statistics.MaximumWaterDepthMeters = ReceiverWaterMeters - Terminal.GroundMeters;
		Result.Statistics.MinimumActualDownstreamSlope = std::numeric_limits<double>::infinity();
		for(std::size_t i = Result.Samples.size() - 1; i > 0; --i) {
			const double Length = Result.Samples[i].DistanceMeters - Result.Samples[i - 1].DistanceMeters;
			RiverReceiverProfileSample &Upstream = Result.Samples[i - 1];
			const RiverReceiverProfileSample &Downstream = Result.Samples[i];
			Upstream.WaterMeters = std::max(
				Upstream.GroundMeters + Settings.MinimumDepthMeters,
				Downstream.WaterMeters + Settings.MinimumDownstreamSlope * Length
			);
			const double ActualSlope = (Upstream.WaterMeters - Downstream.WaterMeters) / Length;
			const double Depth = Upstream.WaterMeters - Upstream.GroundMeters;
			if(!std::isfinite(Upstream.WaterMeters) || !(ActualSlope > 0) || !(Depth >= Settings.MinimumDepthMeters)) {
				Error = RiverReceiverProfileError::NumericFailure;
				// 有限で下降する水面を構成出来ない返戻
				return {};
			}
			Result.Statistics.MaximumDownstreamSlope = std::max(Result.Statistics.MaximumDownstreamSlope, ActualSlope);
			Result.Statistics.MinimumActualDownstreamSlope = std::min(Result.Statistics.MinimumActualDownstreamSlope, ActualSlope);
			Result.Statistics.MaximumWaterDepthMeters = std::max(Result.Statistics.MaximumWaterDepthMeters, Depth);
		}
		Result.Statistics.LengthMeters = Terminal.DistanceMeters;
		// 各標本の地面・水位・下降勾配を確認した縦断の返戻
		return Result;
	}

	/**
	 * 水深上限と下降を両立する最小の最大開削量を厳密な一次元境界で求める
	 * @param Original 同じ最終地形から細分化した縦断標本
	 * @param Settings 最小水深・最大水深・最低勾配・最大開削量
	 * @param Error 成立しない理由
	 * @param Diagnostics 不成立時にも残す水深と開削の下界
	 * @param Stop 中断要求
	 * @return 標本ごとに制約を満たす最小最大開削の地面と水位
	 */
	std::optional<RiverReceiverConditioning> ConditionRiverReceiverProfile(
		const std::span<const RiverReceiverProfileSample> Original,
		const RiverReceiverConditioningSettings &Settings,
		RiverReceiverConditioningError &Error,
		RiverReceiverConditioningDiagnostics *const Diagnostics,
		const std::stop_token Stop
	) {
		Error = RiverReceiverConditioningError::None;
		RiverReceiverConditioningDiagnostics Local;
		RiverReceiverConditioningDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		if(Stop.stop_requested()) {
			Error = RiverReceiverConditioningError::Cancelled;
			// 中断後に部分的な河床を公開しない返戻
			return {};
		}
		const bool Valid = Original.size() > 1 && Original.size() < 2000001 && std::isfinite(Settings.MinimumDepthMeters) &&
		Settings.MinimumDepthMeters > 0 && std::isfinite(Settings.MaximumDepthMeters) &&
		Settings.MaximumDepthMeters >= Settings.MinimumDepthMeters && std::isfinite(Settings.MinimumDownstreamSlope) &&
		Settings.MinimumDownstreamSlope > 0 && Settings.MinimumDownstreamSlope < 1 && std::isfinite(Settings.MaximumCutMeters) &&
		!(Settings.MaximumCutMeters < 0);
		if(!Valid) {
			Error = RiverReceiverConditioningError::InvalidInput;
			// 無効な水深・勾配・経路規模への返戻
			return {};
		}
		const double ReceiverWater = Original.back().WaterMeters, EndDistance = Original.back().DistanceMeters;
		if(!std::isfinite(ReceiverWater) || !std::isfinite(EndDistance) || !(EndDistance > 0)) {
			Error = RiverReceiverConditioningError::InvalidInput;
			// 未定義の受入境界又は距離への返戻
			return {};
		}
		for(std::size_t i = 0; i < Original.size(); ++i) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverConditioningError::Cancelled;
				// 入力検査中の中断への返戻
				return {};
			}
			if(
				!std::isfinite(Original[i].GroundMeters) || !std::isfinite(Original[i].DistanceMeters) || Original[i].DistanceMeters < 0 ||
				i && !(Original[i].DistanceMeters > Original[i - 1].DistanceMeters)
			) {
				Error = RiverReceiverConditioningError::InvalidInput;
				// 非有限又は重複した縦断標本への返戻
				return {};
			}
		}
		if(ReceiverWater < Original.back().GroundMeters + Settings.MinimumDepthMeters) {
			Error = RiverReceiverConditioningError::ReceiverExposed;
			// 河川合流点の実水位で必要水深を確保出来ない返戻
			return {};
		}
		double SuffixMaximum = -std::numeric_limits<double>::infinity();
		for(std::size_t i = Original.size(); i-- > 0;) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverConditioningError::Cancelled;
				// 下界計算中の中断への返戻
				return {};
			}
			const double Distance = Original[i].DistanceMeters, Ground = Original[i].GroundMeters;
			const double AnchorWater = ReceiverWater + Settings.MinimumDownstreamSlope * (EndDistance - Distance);
			Report.MinimumIrreducibleDepthMeters = std::max(Report.MinimumIrreducibleDepthMeters, AnchorWater - Ground);
			SuffixMaximum = std::max(SuffixMaximum, Ground + Settings.MinimumDepthMeters + Settings.MinimumDownstreamSlope * Distance);
			const double RequiredCut = SuffixMaximum - (Ground + Settings.MaximumDepthMeters + Settings.MinimumDownstreamSlope * Distance);
			Report.MinimumRequiredMaximumCutMeters = std::max(Report.MinimumRequiredMaximumCutMeters, RequiredCut);
		}
		if(!std::isfinite(Report.MinimumIrreducibleDepthMeters) || !std::isfinite(Report.MinimumRequiredMaximumCutMeters)) {
			Error = RiverReceiverConditioningError::NumericFailure;
			// 非有限な下界への返戻
			return {};
		}
		if(Report.MinimumIrreducibleDepthMeters > Settings.MaximumDepthMeters + 1E-9) {
			Error = RiverReceiverConditioningError::IrreducibleDepth;
			// 受入水位その物が作る深水域への返戻
			return {};
		}
		if(Report.MinimumRequiredMaximumCutMeters > Settings.MaximumCutMeters + 1E-9) {
			Error = RiverReceiverConditioningError::CutLimit;
			// 必要な最大開削量が許容値を超える返戻
			return {};
		}
		RiverReceiverConditioning Result;
		Result.Bounds = Report;
		Result.Samples.reserve(Original.size());
		for(const RiverReceiverProfileSample &Point : Original) {
			Result.Samples.push_back({ Point.Direction, Point.DistanceMeters, Point.GroundMeters, Point.GroundMeters, 0, 0 });
		}
		Result.Samples.back().WaterMeters = ReceiverWater;
		Result.MaximumWaterDepthMeters = ReceiverWater - Original.back().GroundMeters;
		const double MaximumCut = Report.MinimumRequiredMaximumCutMeters;
		for(std::size_t i = Result.Samples.size() - 1; i > 0; --i) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverConditioningError::Cancelled;
				// 地形と水面の構成中に中断した返戻
				return {};
			}
			RiverReceiverConditioningSample &Upstream = Result.Samples[i - 1];
			const RiverReceiverConditioningSample &Downstream = Result.Samples[i];
			const double Length = Downstream.DistanceMeters - Upstream.DistanceMeters;
			Upstream.WaterMeters = std::max(
				Upstream.OriginalGroundMeters - MaximumCut + Settings.MinimumDepthMeters,
				Downstream.WaterMeters + Settings.MinimumDownstreamSlope * Length
			);
			Upstream.ConditionedGroundMeters = std::min(Upstream.OriginalGroundMeters, Upstream.WaterMeters - Settings.MinimumDepthMeters);
			Upstream.CutMeters = Upstream.OriginalGroundMeters - Upstream.ConditionedGroundMeters;
			const double Depth = Upstream.WaterMeters - Upstream.ConditionedGroundMeters;
			const double WaterSlope = (Upstream.WaterMeters - Downstream.WaterMeters) / Length;
			if(
				!std::isfinite(WaterSlope) || !std::isfinite(Depth) || !(WaterSlope > 0) || Upstream.CutMeters > MaximumCut + 1E-9 ||
				Depth > Settings.MaximumDepthMeters + 1E-9 || Depth + 1E-9 < Settings.MinimumDepthMeters
			) {
				Error = RiverReceiverConditioningError::NumericFailure;
				// 下界から得た標本が実際の制約を満たさない返戻
				return {};
			}
			Result.MaximumWaterDepthMeters = std::max(Result.MaximumWaterDepthMeters, Depth);
			Result.MaximumActualCutMeters = std::max(Result.MaximumActualCutMeters, Upstream.CutMeters);
			Result.MaximumDownstreamSlope = std::max(Result.MaximumDownstreamSlope, WaterSlope);
			Result.CutAreaPerUnitWidthSquareMeters += 0.5 * (Upstream.CutMeters + Downstream.CutMeters) * Length;
		}
		// 最大水深と最大開削の両方を守った標本縦断の返戻
		return Result;
	}

	/**
	 * 予定水面に対する有限幅の河岸を，同じ最終地形から独立に測る
	 * @param Profile 受入水位と中心線の制約を満たした標本
	 * @param Ground 表示・衝突と同じ最終地形の標高
	 * @param Settings 惑星半径・物理半幅・余裕高・標本間引き
	 * @param Error 失敗理由
	 * @param Stop 中断要求
	 * @return 両岸が水没する標本数と最大不足；標本間の保証ではない
	 */
	std::optional<RiverReceiverBankStatistics> SurveyRiverReceiverBanks(
		const std::span<const RiverReceiverConditioningSample> Profile,
		const std::function<double(UnitDirection)> &Ground,
		const RiverReceiverBankSettings &Settings,
		RiverReceiverBankError &Error,
		const std::stop_token Stop
	) {
		Error = RiverReceiverBankError::None;
		const bool Valid = Profile.size() > 2 && Profile.size() < 2000001 && Ground && std::isfinite(Settings.RadiusMeters) &&
		!(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 && std::isfinite(Settings.FreeboardMeters) &&
		!(Settings.FreeboardMeters < 0) && Settings.SampleStride > 0 && Settings.SampleStride < 2000001;
		if(!Valid) {
			Error = RiverReceiverBankError::InvalidInput;
			// 不正な経路，尺度又は間引きへの返戻
			return {};
		}
		for(std::size_t Width = 0; Width < Settings.HalfWidthsMeters.size(); ++Width) {
			const double HalfWidth = Settings.HalfWidthsMeters[Width];
			if(
				!std::isfinite(HalfWidth) || !(HalfWidth > 0) || !(HalfWidth < Settings.RadiusMeters) ||
				Width && !(HalfWidth > Settings.HalfWidthsMeters[Width - 1])
			) {
				Error = RiverReceiverBankError::InvalidInput;
				// 非有限又は順序の崩れた物理幅への返戻
				return {};
			}
		}
		for(std::size_t i = 0; i < Profile.size(); ++i) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverBankError::Cancelled;
				// 入力確認中の中断で部分的な結果を公開しない返戻
				return {};
			}
			const RiverReceiverConditioningSample &Point = Profile[i];
			if(
				!std::isfinite(Point.DistanceMeters) || !std::isfinite(Point.WaterMeters) || !std::isfinite(Point.ConditionedGroundMeters) ||
				i && !(Point.DistanceMeters > Profile[i - 1].DistanceMeters)
			) {
				Error = RiverReceiverBankError::InvalidPath;
				// 順序又は有限性を失った縦断への返戻
				return {};
			}
		}
		RiverReceiverBankStatistics Result;
		for(std::size_t Station = 1; Station + 1 < Profile.size(); Station += Settings.SampleStride) {
			if(Stop.stop_requested()) {
				Error = RiverReceiverBankError::Cancelled;
				// 横断測定中の中断で部分的な結果を公開しない返戻
				return {};
			}
			const Vec3 P = Profile[Station].Direction.Vector();
			const Vec3 Along = Profile[Station + 1].Direction.Vector() - Profile[Station - 1].Direction.Vector();
			const std::optional<UnitDirection> Across = UnitDirection::FromVector(P.Cross(Along));
			if(!Across) {
				Error = RiverReceiverBankError::InvalidPath;
				// 横断方向の定義出来ない折線への返戻
				return {};
			}
			++Result.Stations;
			for(std::size_t Width = 0; Width < Settings.HalfWidthsMeters.size(); ++Width) {
				const double Angle = Settings.HalfWidthsMeters[Width] / Settings.RadiusMeters;
				for(const double Side : { -1.0, 1.0 }) {
					const std::optional<UnitDirection> Bank =
					UnitDirection::FromVector(P * std::cos(Angle) + Across->Vector() * (Side * std::sin(Angle)));
					if(!Bank) {
						Error = RiverReceiverBankError::InvalidPath;
						// 横断方向の補間が不能な標本への返戻
						return {};
					}
					const double BankHeight = Ground(*Bank);
					if(!std::isfinite(BankHeight)) {
						Error = RiverReceiverBankError::InvalidTerrain;
						// 非有限な最終地面の河岸への返戻
						return {};
					}
					const double Water = Profile[Station].WaterMeters, Deficit = Water + Settings.FreeboardMeters - BankHeight;
					if(Deficit > 0) ++Result.BelowFreeboardSides[Width];
					if(Water > BankHeight) ++Result.SubmergedSides[Width];
					Result.MaximumFreeboardDeficitMeters[Width] = std::max(Result.MaximumFreeboardDeficitMeters[Width], Deficit);
				}
			}
		}
		// 有限幅の横断標本を全て測った結果の返戻
		return Result;
	}
}
