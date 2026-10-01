#include "PlanetClimate.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <map>
#include <numbers>

namespace PlanetCore {
	/**
	 * 気候設定の数値範囲と極軸を検査する
	 * @return 全ての設定が有効な場合に真
	 */
	bool ClimateSettings::IsValid() const {
		// 有限な単位軸と対応範囲の返戻
		return std::isfinite(NorthAxis.Length()) && std::abs(NorthAxis.Length() - 1) < 1.0E-12 && std::isfinite(AxialTiltRadians) &&
		AxialTiltRadians >= 0 && AxialTiltRadians <= std::numbers::pi / 4 && std::isfinite(ReferenceZonalWindMetersPerSecond) &&
		ReferenceZonalWindMetersPerSecond >= 0 && ReferenceZonalWindMetersPerSecond <= 60 && std::isfinite(AngularDiffusionPerDay) &&
		AngularDiffusionPerDay >= 0 && AngularDiffusionPerDay <= 0.01;
	}

	/**
	 * 区間端で滑らかにつながる遷移率を求める
	 * @param Lower 遷移開始値
	 * @param Upper 遷移終了値
	 * @param Value 評価する値
	 * @return ０～１の遷移率
	 */
	double PlanetClimate::SmoothTransition(const double Lower, const double Upper, const double Value) {
		const double Fraction = std::clamp((Value - Lower) / (Upper - Lower), 0.0, 1.0);
		// 端点の二階微分がゼロと為る補間の返戻
		return Fraction * Fraction * Fraction * (10 + Fraction * (-15 + 6 * Fraction));
	}

	/**
	 * 球面方向を索引格子の成分へ写す
	 * @param Coordinate 単位球上の１成分
	 * @return 境界を含む格子座標
	 */
	std::uint32_t PlanetClimate::GridCoordinate(const double Coordinate) {
		// 球の端を含む有効な索引座標の返戻
		return static_cast<std::uint32_t>(std::clamp(std::floor((Coordinate + 1) * 8), 0.0, 15.0));
	}

	/**
	 * 固定した気候標本を空間索引へ格納する
	 */
	void PlanetClimate::BuildIndex() {
		Offsets.fill(0);
		for(const UnitDirection Direction : Directions) {
			const Vec3 Position = Direction.Vector();
			const std::uint32_t Cell = GridCoordinate(Position.X) + (GridCoordinate(Position.Y) << 4) + (GridCoordinate(Position.Z) << 8);
			++Offsets[Cell + 1];
		}
		for(std::size_t i = 1; i < Offsets.size(); ++i) Offsets[i] += Offsets[i - 1];
		std::array<std::uint32_t, 4097> Positions = Offsets;
		Order.resize(Directions.size());
		for(std::uint32_t i = 0; i < Directions.size(); ++i) {
			const Vec3 Position = Directions[i].Vector();
			const std::uint32_t Cell = GridCoordinate(Position.X) + (GridCoordinate(Position.Y) << 4) + (GridCoordinate(Position.Z) << 8);
			Order[Positions[Cell]++] = i;
		}
		// 終了
		return;
	}

	/**
	 * 単位球上の三角形の球面面積を求める
	 * @param First 第１頂点
	 * @param Second 第２頂点
	 * @param Third 第３頂点
	 * @return 球面面積（ステラジアン）
	 */
	double PlanetClimate::SphericalArea(const Vec3 First, const Vec3 Second, const Vec3 Third) {
		// 球面過剰に依る面積の返戻
		return 2 * std::atan2(std::abs(First.Dot(Second.Cross(Third))), 1 + First.Dot(Second) + Second.Dot(Third) + Third.Dot(First));
	}

	/**
	 * 球面網から水蒸気輸送の隣接辺を準備する
	 * @param Sphere 固定球面網
	 * @param Cells 輸送用の標本
	 * @param Support 沿岸・陸域の支持情報
	 * @return 輸送辺，入力不正時は空
	 */
	std::optional<std::vector<PlanetClimate::TransportEdge>> PlanetClimate::BuildTransportEdges(
		const SphereMesh &Sphere,
		std::vector<MoistureTransportCell> &Cells,
		double &Support
	) {
		// 双対多角形の実面積と，各境界の法線積分
		using EdgeKey = std::pair<std::uint32_t, std::uint32_t>;
		std::map<EdgeKey, std::pair<Vec3, std::uint32_t>> Measures;
		Cells.resize(Sphere.Directions.size());
		for(MoistureTransportCell &Cell : Cells) Cell.AreaSteradians = 0;
		Support = 0;
		for(const std::array<std::uint32_t, 3> &Triangle : Sphere.Triangles) {
			const Vec3 First = Sphere.Directions[Triangle[0]].Vector();
			const Vec3 Second = Sphere.Directions[Triangle[1]].Vector();
			const Vec3 Third = Sphere.Directions[Triangle[2]].Vector();
			const double Area = SphericalArea(First, Second, Third);
			const std::optional<UnitDirection> Center = UnitDirection::FromVector(First + Second + Third);
			// 不正な三角形の場合の返戻
			if(!Center || !std::isfinite(Area) || Area <= 0) return {};
			for(std::size_t i = 0; i < Triangle.size(); ++i) {
				const std::uint32_t Begin = Triangle[i], End = Triangle[(i + 1) % 3];
				const Vec3 BeginDirection = Sphere.Directions[Begin].Vector();
				const Vec3 EndDirection = Sphere.Directions[End].Vector();
				const UnitDirection Midpoint = *UnitDirection::FromVector(BeginDirection + EndDirection);
				Cells[Begin].AreaSteradians += SphericalArea(Center->Vector(), BeginDirection, Midpoint.Vector());
				Cells[End].AreaSteradians += SphericalArea(Center->Vector(), Midpoint.Vector(), EndDirection);
				const EdgeKey Key = std::minmax(Begin, End);
				const Vec3 Across = Sphere.Directions[Key.second].Vector() - Sphere.Directions[Key.first].Vector();
				Vec3 Normal = UnitDirection::FromVector(Center->Vector().Cross(Midpoint.Vector()))->Vector();
				if(Normal.Dot(Across) < 0) Normal = Normal * -1;
				std::pair<Vec3, std::uint32_t> &Measure = Measures[Key];
				Measure.first = Measure.first + Normal * std::acos(std::clamp(Center->Vector().Dot(Midpoint.Vector()), -1.0, 1.0));
				++Measure.second;
				Support = std::max(Support, (EndDirection - BeginDirection).Length() * 1.5);
			}
		}
		std::vector<TransportEdge> Edges;
		Edges.reserve(Measures.size());
		for(const std::pair<const EdgeKey, std::pair<Vec3, std::uint32_t>> &Entry : Measures) {
			// 閉じていない辺の場合の返戻
			if(Entry.second.second != 2 || Entry.second.first.Length() <= 0) return {};
			const Vec3 First = Sphere.Directions[Entry.first.first].Vector();
			const Vec3 Second = Sphere.Directions[Entry.first.second].Vector();
			const Vec3 Across = UnitDirection::FromVector(Second - First)->Vector();
			Edges.push_back(
				{
					Entry.first.first,
					Entry.first.second,
					Entry.second.first,
					Entry.second.first.Dot(Across) / std::acos(std::clamp(First.Dot(Second), -1.0, 1.0))
				}
			);
		}
		// 保存流束の幾何の返戻
		return Edges;
	}

	/**
	 * 緯度と赤緯から日平均の日射射影を算出する
	 * @param SineLatitude 緯度の正弦
	 * @param SineDeclination 太陽赤緯の正弦
	 * @return 日平均の射影，不正な入力時は空
	 */
	std::optional<double> PlanetClimate::DailyMeanSolarProjection(const double SineLatitude, const double SineDeclination) {
		// 範囲外の日射入力の場合の返戻
		if(
			!std::isfinite(SineLatitude) || !std::isfinite(SineDeclination) || std::abs(SineLatitude) > 1 || std::abs(SineDeclination) > 1
		) return {};
		const double Offset = SineLatitude * SineDeclination;
		const double Amplitude = std::sqrt((1 - SineLatitude * SineLatitude) * (1 - SineDeclination * SineDeclination));
		// 白夜と極夜の返戻，極点のゼロ除算も避ける
		if(Offset >= Amplitude) return Offset;
		if(Offset <= -Amplitude) return 0;
		const double SunsetAngle = std::acos(-Offset / Amplitude);
		// 水平面への日平均投影の返戻
		return (SunsetAngle * Offset + Amplitude * std::sin(SunsetAngle)) / std::numbers::pi;
	}

	/**
	 * 季節毎の気温・水蒸気輸送と降水を準備する
	 * @param Ocean 接続済み海陸
	 * @param Settings 気候設定
	 * @param Heights 球面網の標高
	 * @param IsOcean 標本毎の海陸区分
	 * @param Stop 中断要求
	 * @return 全季節を準備出来た場合に真
	 */
	bool PlanetClimate::PrepareSeasons(
		const OceanConnectivityField &Ocean,
		const ClimateSettings &Settings,
		std::vector<double> &Heights,
		std::vector<bool> &IsOcean,
		const std::stop_token Stop
	) {
		// 気候の地球型参照縮尺，実際の距離や重力へは適用しない
		constexpr double DaysPerPeriod = 365.2422 / ClimatePeriodCount;
		const GeologySettings &Geology = Ocean.Base().Settings();
		const double ReferenceReliefScale = (Geology.RadiusMeters + 250000) / Geology.RadiusMeters;
		Cycles.resize(Directions.size());
		Heights.resize(Directions.size());
		IsOcean.resize(Directions.size());
		for(std::size_t i = 0; i < Directions.size(); ++i) {
			// 中断要求の場合の返戻
			if(Stop.stop_requested()) return false;
			const double Height = Ocean.EvaluateElevation(Directions[i]) - Geology.SeaLevelMeters;
			// 不正な標高の場合の返戻
			if(!std::isfinite(Height)) return false;
			Heights[i] = std::max(0.0, Height) * ReferenceReliefScale;
			IsOcean[i] = Height < 0;
			const double SineLatitude = std::clamp(Settings.NorthAxis.Dot(Directions[i].Vector()), -1.0, 1.0);
			std::array<double, ClimatePeriodCount> Radiation {};
			double MeanRadiation = 0;
			for(std::size_t j = 0; j < Radiation.size(); ++j) {
				const double Declination = std::sin(Settings.AxialTiltRadians) * std::sin(2 * std::numbers::pi * j / ClimatePeriodCount);
				Radiation[j] = *DailyMeanSolarProjection(SineLatitude, Declination);
				MeanRadiation += Radiation[j] / ClimatePeriodCount;
			}
			// 海陸の熱慣性を持つ周期解，初期温度の仮定を翌年へ持ち越さない
			const double Memory = std::exp(-DaysPerPeriod / (IsOcean[i] ? 90 : 20));
			std::array<double, ClimatePeriodCount> Targets {};
			double PeriodicTemperature = 0;
			for(std::size_t j = 0; j < Targets.size(); ++j) {
				Targets[j] =
				std::clamp(15 + 220 * (MeanRadiation - 0.25) + 100 * (Radiation[j] - MeanRadiation) - 0.0065 * Heights[i], -120.0, 65.0);
				PeriodicTemperature = Memory * PeriodicTemperature + (1 - Memory) * Targets[j];
			}
			PeriodicTemperature /= 1 - std::pow(Memory, ClimatePeriodCount);
			for(std::size_t j = 0; j < Targets.size(); ++j) {
				PeriodicTemperature = Memory * PeriodicTemperature + (1 - Memory) * Targets[j];
				Cycles[i].Periods[j].TemperatureC = PeriodicTemperature;
				Cycles[i].Periods[j].PotentialEvapotranspirationMm =
				DaysPerPeriod * 4.5 * Radiation[j] / 0.3 * SmoothTransition(-15, 20, PeriodicTemperature);
			}
		}
		// 季節入力の準備結果の返戻
		return true;
	}

	/**
	 * 索引又は全走査から季節気候を補間する
	 * @param Direction 球面上の評価方向
	 * @param IsIndexed 空間索引を使用する場合に真
	 * @return 補間した気候，評価不能時は空
	 */
	std::optional<ClimateCycle> PlanetClimate::SampleImpl(const UnitDirection Direction, const bool IsIndexed) const {
		// 有限範囲の三乗重み，支持境界で二階微分迄ゼロ
		const Vec3 Position = Direction.Vector();
		const double SupportSquared = SupportChord * SupportChord;
		ClimateCycle Result;
		double WeightSum = 0;
		/**
		 * 一つの地点の気候を加算する
		 * @param Index 正準の地点番号
		 */
		const auto Accumulate = [&](const std::uint32_t Index) -> void {
			const Vec3 Difference = Directions[Index].Vector() - Position;
			const double Fraction = Difference.Dot(Difference) / SupportSquared;
			if(Fraction < 1) {
				const double Remaining = 1 - Fraction, Weight = Remaining * Remaining * Remaining;
				WeightSum += Weight;
				for(std::size_t i = 0; i < ClimatePeriodCount; ++i) {
					Result.Periods[i].TemperatureC += Weight * Cycles[Index].Periods[i].TemperatureC;
					Result.Periods[i].PrecipitationMm += Weight * Cycles[Index].Periods[i].PrecipitationMm;
					Result.Periods[i].PotentialEvapotranspirationMm += Weight * Cycles[Index].Periods[i].PotentialEvapotranspirationMm;
				}
			}
			// 終了
			return;
		};
		if(IsIndexed) {
			const std::uint32_t MinX = GridCoordinate(Position.X - SupportChord), MaxX = GridCoordinate(Position.X + SupportChord);
			const std::uint32_t MinY = GridCoordinate(Position.Y - SupportChord), MaxY = GridCoordinate(Position.Y + SupportChord);
			const std::uint32_t MinZ = GridCoordinate(Position.Z - SupportChord), MaxZ = GridCoordinate(Position.Z + SupportChord);
			for(std::uint32_t z = MinZ; z <= MaxZ; ++z) {
				for(std::uint32_t y = MinY; y <= MaxY; ++y) for(std::uint32_t x = MinX; x <= MaxX; ++x) {
					const std::uint32_t Cell = x + (y << 4) + (z << 8);
					for(std::uint32_t i = Offsets[Cell]; i < Offsets[Cell + 1]; ++i) Accumulate(Order[i]);
				}
			}
		} else for(std::uint32_t i = 0; i < Directions.size(); ++i) Accumulate(i);
		// 支持範囲が不足した場合の返戻，最近点への置換で不連続を隠さない
		if(WeightSum <= 0) return {};
		for(ClimatePeriod &Period : Result.Periods) {
			Period.TemperatureC /= WeightSum;
			Period.PrecipitationMm /= WeightSum;
			Period.PotentialEvapotranspirationMm /= WeightSum;
		}
		// 正常な補間結果の返戻
		return Result.IsValid() ? std::optional<ClimateCycle>(Result) : std::nullopt;
	}

	/**
	 * 空間索引から季節気候を評価する
	 * @param Direction 球面上の評価方向
	 * @return 補間した気候，評価不能時は空
	 */
	std::optional<ClimateCycle> PlanetClimate::Sample(const UnitDirection Direction) const {
		return SampleImpl(Direction, true);
	}

	/**
	 * 全標本走査で索引評価の参照結果を得る
	 * @param Direction 球面上の評価方向
	 * @return 補間した気候，評価不能時は空
	 */
	std::optional<ClimateCycle> PlanetClimate::SampleReference(const UnitDirection Direction) const {
		return SampleImpl(Direction, false);
	}

	/**
	 * 季節気候から連続したバイオーム比率を求める
	 * @param Direction 球面上の評価方向
	 * @return バイオーム比率，気候が無い場合は空
	 */
	std::optional<BiomeProfile> PlanetClimate::SampleBiome(const UnitDirection Direction) const {
		const std::optional<ClimateCycle> Climate = Sample(Direction);
		// 補間した実気候に依る応答の返戻
		return Climate ? BiomeProfile::Create(*Climate) : std::nullopt;
	}

	/**
	 * 気候場の生成定義を識別する
	 * @return 気候・海陸・季節を含む識別子
	 */
	std::uint64_t PlanetClimate::IdentityHash() const {
		return DefinitionHash;
	}

	/**
	 * 気候準備の統計を取得する
	 * @return 不変な準備統計
	 */
	const ClimatePreparationStatistics &PlanetClimate::PreparationStatistics() const {
		return Statistics;
	}

	/**
	 * 海陸と季節設定から全球気候場を準備する
	 * @param Ocean 接続済み海陸
	 * @param Settings 季節と気候の設定
	 * @param Stop 中断要求
	 * @param Diagnostics 失敗段階を受け取る任意の出力先
	 * @return 準備済み気候場，入力不正又は中断時は空
	 */
	std::optional<PlanetClimate> PlanetClimate::Create(
		const OceanConnectivityField &Ocean,
		const ClimateSettings &Settings,
		const std::stop_token Stop,
		ClimatePreparationStatistics *const Diagnostics
	) {
		// 出力先は呼出毎に初期化し，以前の成功状態を残さない
		PlanetClimate Result;
		ClimatePreparationStatistics &Progress = Diagnostics ? *Diagnostics : Result.Statistics;
		Progress = {};
		// 不正な設定の場合の返戻
		if(!Settings.IsValid()) return {};
		if(Stop.stop_requested()) {
			Progress.Failure = ClimatePreparationFailure::Cancelled;
			// 中断結果の返戻
			return {};
		}
		// 視点と描画の細分化に依存しない固定網
		Progress.Failure = ClimatePreparationFailure::Geometry;
		std::optional<SphereMesh> Sphere = BuildUniformSphere(4);
		// 網の準備に失敗した場合の返戻
		if(!Sphere) return {};
		std::vector<MoistureTransportCell> Cells;
		const std::optional<std::vector<TransportEdge>> Edges = BuildTransportEdges(*Sphere, Cells, Result.SupportChord);
		// 輸送幾何の準備に失敗した場合の返戻
		if(!Edges) return {};
		Result.Directions = std::move(Sphere->Directions);
		Progress.Nodes = static_cast<std::uint32_t>(Result.Directions.size());
		Progress.Links = static_cast<std::uint32_t>(Edges->size());
		Progress.Failure = ClimatePreparationFailure::Source;
		std::vector<double> Heights;
		std::vector<bool> IsOcean;
		if(!Result.PrepareSeasons(Ocean, Settings, Heights, IsOcean, Stop)) {
			if(Stop.stop_requested()) Progress.Failure = ClimatePreparationFailure::Cancelled;
			// 季節入力の準備失敗の返戻
			return {};
		}
		double TotalArea = 0;
		for(std::size_t i = 0; i < Cells.size(); ++i) {
			TotalArea += Cells[i].AreaSteradians;
			if(IsOcean[i]) Progress.OceanAreaFraction += Cells[i].AreaSteradians;
		}
		Progress.SurfaceAreaSteradians = TotalArea;
		Progress.OceanAreaFraction /= TotalArea;
		// 参照地球の風速を角速度へ換算，小半径の人体・地形寸法へは適用しない
		constexpr double ReferenceRadiusMeters = 6371000, DaysPerPeriod = 365.2422 / ClimatePeriodCount;
		std::vector<Vec3> Winds(Cells.size());
		std::vector<MoistureTransportLink> Links;
		Links.reserve(Edges->size());
		for(std::size_t Period = 0; Period < ClimatePeriodCount; ++Period) {
			Progress.Failure = ClimatePreparationFailure::Transport;
			const double Declination =
			std::asin(std::sin(Settings.AxialTiltRadians) * std::sin(2 * std::numbers::pi * Period / ClimatePeriodCount));
			for(std::size_t i = 0; i < Cells.size(); ++i) {
				const Vec3 Direction = Result.Directions[i].Vector();
				const double SineLatitude = std::clamp(Settings.NorthAxis.Dot(Direction), -1.0, 1.0), Latitude = std::asin(SineLatitude);
				const double AbsoluteLatitude = std::abs(Latitude);
				const double Zonal = -1 + 2 * SmoothTransition(0.4363323, 0.6108652, AbsoluteLatitude) -
				2 * SmoothTransition(0.9599311, 1.1344640, AbsoluteLatitude);
				Winds[i] = (
					Settings.NorthAxis.Cross(Direction) * (Settings.ReferenceZonalWindMetersPerSecond * Zonal) +
					(Settings.NorthAxis - Direction * SineLatitude) * (-3 * std::sin(2 * (Latitude - 0.7 * Declination)))
				) * (86400 / ReferenceRadiusMeters);
				const ClimatePeriod &Climate = Result.Cycles[i].Periods[Period];
				Cells[i].EvaporationMillimetersPerDay =
				IsOcean[i] ? Climate.PotentialEvapotranspirationMm / DaysPerPeriod * SmoothTransition(-4, 2, Climate.TemperatureC) : 0;
				const double Subtropical = (std::abs(Latitude - 0.7 * Declination) - 0.48) / 0.18;
				Cells[i].ReevaporationPerDay = 0.5 * (8 + 55 * std::exp(-Subtropical * Subtropical)) * std::exp(-Heights[i] / 2500);
			}
			Links.clear();
			for(const TransportEdge &Edge : *Edges) {
				const double Advection = ((Winds[Edge.First] + Winds[Edge.Second]) * 0.5).Dot(Edge.IntegratedNormal);
				const double Diffusion = Settings.AngularDiffusionPerDay * Edge.DiffusionFactor;
				Links.push_back({ Edge.First, Edge.Second, Diffusion + std::max(0.0, Advection), Diffusion + std::max(0.0, -Advection) });
			}
			const MoistureTransportResult Transport = PlanetMoistureTransport::Solve(Cells, Links, {}, Stop);
			Progress.TransportFailure = Transport.Failure;
			Progress.MaximumSweeps = std::max(Progress.MaximumSweeps, Transport.Sweeps);
			Progress.MaximumResidualMillimetersPerDay =
			std::max(Progress.MaximumResidualMillimetersPerDay, Transport.MaximumResidualMillimetersPerDay);
			Progress.MaximumRelativeBudgetError = std::max(Progress.MaximumRelativeBudgetError, Transport.RelativeBudgetError);
			if(!Transport.HasConverged()) {
				if(Stop.stop_requested()) Progress.Failure = ClimatePreparationFailure::Cancelled;
				// 未収束又は中断した気候を公開しない返戻
				return {};
			}
			for(std::size_t i = 0; i < Cells.size(); ++i) {
				const double Precipitation = DaysPerPeriod * Cells[i].PrecipitationPerDay * Transport.Columns[i].CloudMillimeters;
				Result.Cycles[i].Periods[Period].PrecipitationMm = Precipitation;
				Progress.GlobalAnnualPrecipitationMillimeters += Cells[i].AreaSteradians * Precipitation / TotalArea;
			}
			++Progress.CompletedPeriods;
		}
		// 生成定義の版と全設定を，固定のバイト順で識別
		Result.DefinitionHash = 1469598103934665603ULL;
		const std::array<std::uint64_t, 10> DefinitionValues {
			Ocean.DefinitionHash(),
			OceanConnectivityVersion,
			ClimateGeneratorVersion,
			BiomeVersion,
			std::bit_cast<std::uint64_t>(Settings.NorthAxis.X),
			std::bit_cast<std::uint64_t>(Settings.NorthAxis.Y),
			std::bit_cast<std::uint64_t>(Settings.NorthAxis.Z),
			std::bit_cast<std::uint64_t>(Settings.AxialTiltRadians),
			std::bit_cast<std::uint64_t>(Settings.ReferenceZonalWindMetersPerSecond),
			std::bit_cast<std::uint64_t>(Settings.AngularDiffusionPerDay)
		};
		for(const std::uint64_t Value : DefinitionValues) for(unsigned i = 0; i < 8; ++i) {
			Result.DefinitionHash ^= Value >> (i << 3) & 0XFFULL;
			Result.DefinitionHash *= 1099511628211ULL;
		}
		Progress.Failure = ClimatePreparationFailure::Climate;
		// 公開する全期間の入力検査が失敗した場合の返戻
		for(const ClimateCycle &Cycle : Result.Cycles) if(!Cycle.IsValid()) return {};
		if(Stop.stop_requested()) {
			Progress.Failure = ClimatePreparationFailure::Cancelled;
			// 公開前に到着した中断要求の返戻
			return {};
		}
		Result.BuildIndex();
		Progress.Failure = ClimatePreparationFailure::None;
		Result.Statistics = Progress;
		// 全期間で収束した不変な全球気候の返戻
		return Result;
	}
}
