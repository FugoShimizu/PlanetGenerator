#include "PlanetAeolianField.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace PlanetCore {
	namespace {
		constexpr double Pi = std::numbers::pi_v<double>, CellMeters = 256;
		constexpr double DesertSupportMeters = 130, CoastalSupportMeters = 180;
		constexpr double DesertCrossHalfWidthMeters = 90, CoastalCrossHalfWidthMeters = 150;
		constexpr double WindwardMeters = 90, CrestMeters = 30, LeeMeters = 30, CurvatureMeters = 20;

		/**
		 * 整数セルと Seed から固定した乱数語を得る
		 * @param Value 入力語
		 * @return 混合した乱数語
		 */
		std::uint64_t Mix(std::uint64_t Value) {
			Value += UINT64_C(0x9E3779B97F4A7C15);
			Value = (Value ^ (Value >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
			Value = (Value ^ (Value >> 27)) * UINT64_C(0x94D049BB133111EB);
			// 定義済みの整数演算だけによる混合結果
			return Value ^ (Value >> 31);
		}

		/**
		 * 開区間へ写した乱数を得る
		 * @param Value 混合した乱数語
		 * @return ０～１未満の倍精度値
		 */
		double Fraction(const std::uint64_t Value) {
			// 上位５３ビットから得た再現可能な割合
			return static_cast<double>(Value >> 11) * (1.0 / 9007199254740992.0);
		}

		/**
		 * 零階から二階迄が両端で接続する補間値を求める
		 * @param Value ０～１の座標
		 * @return ０～１の補間値
		 */
		double Smooth(const double Value) {
			const double T = std::clamp(Value, 0.0, 1.0);
			// 五次の滑らかな補間値
			return T * T * T * (10 + T * (-15 + 6 * T));
		}

		/**
		 * 五次補間の導関数を求める
		 * @param Value ０～１の座標
		 * @return 入力座標に対する微分
		 */
		double SmoothSlope(const double Value) {
			if(Value <= 0 || Value >= 1) return 0;
			// 両端で零となる解析微分
			return 30 * Value * Value * (1 - Value) * (1 - Value);
		}
	}

	/**
	 * 固定した気候格子から候補地点の砂漠適性を補間する
	 * @param Direction 候補地点の方向
	 * @return ０～１の砂漠適性
	 */
	double PlanetAeolianField::DesertWeight(const UnitDirection Direction) const {
		const Vec3 D = Direction.Vector();
		const double Longitude = (std::atan2(D.Y, D.X) + Pi) * (LongitudeCount / (2 * Pi)) - 0.5;
		const double Latitude = (D.Z + 1) * (LatitudeCount * 0.5) - 0.5;
		const int32_t X0 = static_cast<int32_t>(std::floor(Longitude));
		const int32_t Y0 = static_cast<int32_t>(std::floor(Latitude));
		const double TX = Longitude - X0, TY = std::clamp(Latitude - Y0, 0.0, 1.0);
		const auto At = [this](const int32_t X, const int32_t Y) {
			const unsigned LongitudeIndex = static_cast<unsigned>((X % static_cast<int32_t>(LongitudeCount) +
				static_cast<int32_t>(LongitudeCount)) % static_cast<int32_t>(LongitudeCount));
			const unsigned LatitudeIndex = static_cast<unsigned>(std::clamp(Y, 0, static_cast<int32_t>(LatitudeCount) - 1));
			// 周期経度と極側の端を維持した標本
			return DesertWeights[LatitudeIndex * LongitudeCount + LongitudeIndex];
		};
		// 候補の振幅だけを決め，照会方向で不連続なマスクを掛けない
		return std::lerp(std::lerp(At(X0, Y0), At(X0 + 1, Y0), TX),
			std::lerp(At(X0, Y0 + 1), At(X0 + 1, Y0 + 1), TX), TY);
	}

	namespace {
		/**
		 * 海面を横切る実地形が近くに有る緩い陸だけを前砂丘の候補にする
		 * @param Base 侵食後の地形
		 * @param Direction 候補地点
		 * @param RadiusMeters 惑星の半径
		 * @param SeaLevelMeters 一定の海面
		 * @param Landward 海から陸へ向かう近似接線
		 * @return 海岸への適性．零なら前砂丘を置かない
		 */
		double CoastalWeight(const PlanetErodedTerrain &Base, const UnitDirection Direction,
			const double RadiusMeters, const double SeaLevelMeters, Vec3 &Landward) {
			const TerrainSample Site = Base.Evaluate(Direction);
			const double AboveSea = Site.ElevationMeters - SeaLevelMeters;
			if(AboveSea <= 0.1 || AboveSea >= 6) return 0;
			const double GradientLength = Site.TangentialGradientPerRadian.Length();
			const double Grade = GradientLength / RadiusMeters;
			if(Grade <= 0.0005 || Grade >= 0.08) return 0;
			Landward = Site.TangentialGradientPerRadian * (1 / GradientLength);
			constexpr std::array<Vec3, 6> Axes{{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
				{0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
			const Vec3 Point = Direction.Vector();
			const UnitDirection Seaward = *UnitDirection::FromVector(Point - Landward * (600 / RadiusMeters));
			double Lowest = Base.EvaluateElevation(Seaward);
			if(Lowest >= SeaLevelMeters - 0.05) {
				for(const Vec3 Axis : Axes) {
					const UnitDirection Nearby = *UnitDirection::FromVector(Point + Axis * (300 / RadiusMeters));
					Lowest = std::min(Lowest, Base.EvaluateElevation(Nearby));
				}
			}
			if(Lowest >= SeaLevelMeters - 0.05) return 0;
			// 候補ごとに固定した重みであり，評価地点を移しても不連続な地形境界を作らない
			return (1 - Smooth((AboveSea - 3) / 3)) * Smooth((Grade - 0.0005) / 0.002);
		}
	}

	/**
	 * 侵食後の標高へ非対称な固定砂丘を加える
	 * @param Direction 評価方向
	 * @return 標高と要求時の解析接線勾配
	 */
	template <bool Derivatives>
	TerrainSample PlanetAeolianField::EvaluateImpl(const UnitDirection Direction) const {
		const TerrainSample Original = [&]() {
			if constexpr(Derivatives) return Base->Evaluate(Direction);
			else return TerrainSample{Base->EvaluateElevation(Direction), {}};
		}();
		const double ShoreStart = std::min(50.0, RadiusMeters * 0.001);
		const double ShoreWidth = std::min(50.0, RadiusMeters * 0.003);
		const double AboveSea = Original.ElevationMeters - SeaLevelMeters;
		const double T = (AboveSea - ShoreStart) / ShoreWidth;
		const bool BeachBand = RadiusMeters >= 100000 && AboveSea > 0.1 && AboveSea < 8;
		if(T <= 0 && !BeachBand) return Original;
		const Vec3 D = Direction.Vector(), P = D * RadiusMeters;
		const double SupportMeters = BeachBand ? CoastalSupportMeters : DesertSupportMeters;
		const double CrossHalfWidthMeters =
			BeachBand ? CoastalCrossHalfWidthMeters : DesertCrossHalfWidthMeters;
		const double MaximumChord = SupportMeters / RadiusMeters;
		const double Horizontal = std::hypot(D.X, D.Y);
		if(!BeachBand && Horizontal > 2 * MaximumChord) {
			const double MaximumWeightChange = LatitudeCount * 0.5 * MaximumChord +
				LongitudeCount * 0.5 * MaximumChord / (Horizontal - MaximumChord);
			// 有限支持内の候補地点でも乾燥度が閾値へ届かなければセル探索を省く
			if(DesertWeight(Direction) + MaximumWeightChange < 0.2) return Original;
		}
		const int32_t CX = static_cast<int32_t>(std::floor(P.X / CellMeters));
		const int32_t CY = static_cast<int32_t>(std::floor(P.Y / CellMeters));
		const int32_t CZ = static_cast<int32_t>(std::floor(P.Z / CellMeters));
		double Sum = 0;
		Vec3 SumGradient;
		for(unsigned CandidateIndex = 0; CandidateIndex < (BeachBand ? 4U : 1U); ++CandidateIndex) {
			for(int32_t Z = CZ - 1; Z <= CZ + 1; ++Z) {
				for(int32_t Y = CY - 1; Y <= CY + 1; ++Y) {
					for(int32_t X = CX - 1; X <= CX + 1; ++X) {
						std::uint64_t Random = Mix(Seed ^ static_cast<std::uint32_t>(X) ^
							(static_cast<std::uint64_t>(CandidateIndex) * UINT64_C(0xD1B54A32D192ED03)));
						Random = Mix(Random ^ static_cast<std::uint32_t>(Y));
						Random = Mix(Random ^ static_cast<std::uint32_t>(Z));
						const double JX = Fraction(Random);
						Random = Mix(Random);
						const double JY = Fraction(Random);
						Random = Mix(Random);
						const double JZ = Fraction(Random);
						const Vec3 Candidate{(X + JX) * CellMeters, (Y + JY) * CellMeters, (Z + JZ) * CellMeters};
						const double CandidateLength = Candidate.Length();
						// 砂漠１３０ｍ・海岸１８０ｍの支持弦長と球殻６０ｍの和は１セル幅未満で，隣接２７セルが完全な候補集合となる
						if(std::abs(CandidateLength - RadiusMeters) > 60) continue;
						const UnitDirection Site = *UnitDirection::FromVector(Candidate);
						const Vec3 SiteDirection = Site.Vector();
						const Vec3 Relative = P - SiteDirection * RadiusMeters;
						if(Relative.Length() > SupportMeters) continue;
						Vec3 Landward;
						const double Habitat = BeachBand ? CoastalWeight(*Base, Site, RadiusMeters, SeaLevelMeters, Landward) :
							DesertWeight(Site);
						if(Habitat < 0.2) continue;
						Vec3 East{-SiteDirection.Y, SiteDirection.X, 0};
						if(East.Length() < 1E-6) East = {1, 0, 0};
						East = East * (1 / East.Length());
						const Vec3 North = SiteDirection.Cross(East);
						Random = Mix(Random);
						const double WindAngle = (Fraction(Random) - 0.5) * (Pi / 3);
						const Vec3 Wind = BeachBand ? Landward * std::cos(WindAngle) +
							SiteDirection.Cross(Landward) * std::sin(WindAngle) :
							East * std::cos(WindAngle) + North * std::sin(WindAngle);
						const Vec3 Across = SiteDirection.Cross(Wind);
						const double U = Relative.Dot(Wind), V = Relative.Dot(Across);
						if(std::abs(V) >= CrossHalfWidthMeters) continue;
						const double NormalizedV = V / CrossHalfWidthMeters;
						const double CurvedU = U - CurvatureMeters * NormalizedV * NormalizedV;
						if(CurvedU <= -WindwardMeters || CurvedU >= CrestMeters + LeeMeters) continue;
						double Along = 0, AlongSlope = 0;
						if(CurvedU < CrestMeters) {
							const double Coordinate = (CurvedU + WindwardMeters) / (WindwardMeters + CrestMeters);
							Along = Smooth(Coordinate);
							AlongSlope = SmoothSlope(Coordinate) / (WindwardMeters + CrestMeters);
						} else {
							const double Coordinate = (CurvedU - CrestMeters) / LeeMeters;
							Along = 1 - Smooth(Coordinate);
							AlongSlope = -SmoothSlope(Coordinate) / LeeMeters;
						}
						const double Cross = 1 - NormalizedV * NormalizedV;
						const double Width = Cross * Cross * Cross;
						Random = Mix(Random);
						const double Amplitude = (BeachBand ? 0.8 + 1.8 * Fraction(Random) :
							2 + 4 * Fraction(Random)) * Habitat;
						Sum += Amplitude * Along * Width;
						if constexpr(Derivatives) {
							const double DWidth = -6 * V * Cross * Cross / (CrossHalfWidthMeters * CrossHalfWidthMeters);
							const double DCurvedU = -2 * CurvatureMeters * V /
								(CrossHalfWidthMeters * CrossHalfWidthMeters);
							const double DU = Amplitude * AlongSlope * Width;
							const double DV = Amplitude * (AlongSlope * DCurvedU * Width + Along * DWidth);
							SumGradient = SumGradient + (Wind - D * D.Dot(Wind)) * (RadiusMeters * DU) +
								(Across - D * D.Dot(Across)) * (RadiusMeters * DV);
						}
					}
				}
			}
		}
		const double HeightLimit = BeachBand ? 4 : std::min(MaximumAddedHeightMeters, RadiusMeters * 0.005);
		const double Saturation = 1 + Sum / HeightLimit;
		const double Dune = Sum / Saturation;
		const double BeachInland = Smooth((AboveSea - 0.1) / 0.9);
		const double BeachUpper = 1 - Smooth((AboveSea - 6) / 2);
		const double ShoreMask = BeachBand ? BeachInland * BeachUpper : Smooth(T);
		TerrainSample Result{Original.ElevationMeters + ShoreMask * Dune, {}};
		if constexpr(Derivatives) {
			const double ShoreSlope = BeachBand ?
				SmoothSlope((AboveSea - 0.1) / 0.9) * BeachUpper / 0.9 -
				BeachInland * SmoothSlope((AboveSea - 6) / 2) / 2 :
				SmoothSlope(T) / ShoreWidth;
			Result.TangentialGradientPerRadian =
				Original.TangentialGradientPerRadian * (1 + ShoreSlope * Dune) +
				SumGradient * (ShoreMask / (Saturation * Saturation));
		}
		// 同じ固定砂丘の値と解析勾配の返戻
		return Result;
	}

	/**
	 * 気候上の砂漠域を準備し，表示分割に依らない形状関数を作る
	 * @param BaseTerrain 侵食後の標高
	 * @param Climate 固定した気候
	 * @param Settings 惑星の定義
	 * @param Stop 中断要求
	 * @return 準備済みの砂丘場，中断又は不正時は空
	 */
	std::optional<PlanetAeolianField> PlanetAeolianField::Create(std::shared_ptr<const PlanetErodedTerrain> BaseTerrain,
		const PlanetClimate &Climate, const GeologySettings &Settings, const std::stop_token Stop) {
		if(!BaseTerrain || !Settings.IsValid() || Stop.stop_requested()) return {};
		PlanetAeolianField Result;
		Result.Base = std::move(BaseTerrain);
		Result.Seed = Settings.Seed ^ UINT64_C(0x41454F4C49414E01);
		Result.RadiusMeters = Settings.RadiusMeters;
		Result.SeaLevelMeters = Settings.SeaLevelMeters;
		const BiomeSite SandSite{SurfaceDomain::Land, 0, 1, 0};
		for(unsigned Y = 0; Y < LatitudeCount; ++Y) {
			for(unsigned X = 0; X < LongitudeCount; ++X) {
				if(Stop.stop_requested()) return {};
				const double Z = -1 + 2 * (Y + 0.5) / LatitudeCount;
				const double Longitude = -Pi + 2 * Pi * (X + 0.5) / LongitudeCount;
				const double Horizontal = std::sqrt(1 - Z * Z);
				const UnitDirection Direction = *UnitDirection::FromVector(
					{Horizontal * std::cos(Longitude), Horizontal * std::sin(Longitude), Z});
				const std::optional<BiomeProfile> Profile = Climate.SampleBiome(Direction);
				if(!Profile) return {};
				const std::optional<BiomeHabitat> Habitat = Profile->Habitat(SandSite);
				if(!Habitat) return {};
				const unsigned Hot = static_cast<unsigned>(LandBiome::HotDesert);
				const unsigned Cold = static_cast<unsigned>(LandBiome::ColdDesert);
				Result.DesertWeights[Y * LongitudeCount + X] =
					std::clamp(Habitat->Weights[Hot] + Habitat->Weights[Cold], 0.0, 1.0);
			}
		}
		// 定義に固定した形状場の返戻
		return Result;
	}

	/**
	 * 固定砂丘と侵食基盤の解析標高を評価する
	 * @param Direction 球面方向
	 * @return 標高と接線勾配
	 */
	TerrainSample PlanetAeolianField::Evaluate(const UnitDirection Direction) const {
		// 解析微分を含む地形の返戻
		return EvaluateImpl<true>(Direction);
	}

	/**
	 * 固定砂丘の標高だけを評価する
	 * @param Direction 球面方向
	 * @return 標高
	 */
	double PlanetAeolianField::EvaluateElevation(const UnitDirection Direction) const {
		// 同じ地点評価式の値の返戻
		return EvaluateImpl<false>(Direction).ElevationMeters;
	}
}
