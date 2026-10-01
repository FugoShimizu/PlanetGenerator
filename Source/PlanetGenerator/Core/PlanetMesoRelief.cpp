#include "PlanetMesoRelief.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>

namespace PlanetCore {
	namespace {
		/**
		 * 他の地質乱数列から独立した固定幅の擬似乱数を進める
		 * @param State 更新する乱数状態
		 * @return 更新後の固定幅乱数
		 */
		std::uint64_t NextBits(std::uint64_t &State) {
			std::uint64_t Value = State += UINT64_C(0X9E3779B97F4A7C15);
			Value = (Value ^ (Value >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
			Value = (Value ^ (Value >> 27)) * UINT64_C(0X94D049BB133111EB);
			// 更新後の乱数の返戻
			return Value ^ Value >> 31;
		}

		/**
		 * 53 bit の一様な区間値を返す
		 * @param State 更新する乱数状態
		 * @return ０以上１未満の値
		 */
		double NextFraction(std::uint64_t &State) {
			return static_cast<double>(NextBits(State) >> 11) * (1.0 / 9007199254740992.0);
		}
	} // namespace

	/**
	 * 半径に収まる波群と連続な稜線応答を準備する
	 * @param Seed 惑星の Seed
	 * @param RadiusMeters 惑星半径
	 * 1,000～10^9 m
	 * @return 準備済みの場
	 * 不正な半径では空
	 */
	std::optional<PlanetMesoRelief> PlanetMesoRelief::Create(const std::uint64_t Seed, const double RadiusMeters) {
		return CreateImpl(Seed, RadiusMeters, false);
	}

	/**
	 * 固定レベル６排水網が解像出来る全球波長の起伏を準備する
	 * @param Seed 惑星の Seed
	 * @param RadiusMeters 惑星半径（1,000～10^9 m）
	 * @return 全球波長の不変な場，不正な半径では空
	 */
	std::optional<PlanetMesoRelief> PlanetMesoRelief::CreateForDrainage(const std::uint64_t Seed, const double RadiusMeters) {
		return CreateImpl(Seed, RadiusMeters, true);
	}

	/**
	 * 同じ位相の波群を物理波長又は全球排水網の波長で構築する
	 * @param Seed 惑星の Seed
	 * @param RadiusMeters 惑星半径（1,000～10^9 m）
	 * @param GlobalWavelengths 全球網が解像出来る長い波長を選ぶ場合に真
	 * @return 準備済みの場，不正な半径では空
	 */
	std::optional<PlanetMesoRelief> PlanetMesoRelief::CreateImpl(
		const std::uint64_t Seed,
		const double RadiusMeters,
		const bool GlobalWavelengths
	) {
		// 不正な物理波長又は半径を公開しない返戻
		if(!std::isfinite(RadiusMeters) || RadiusMeters < 1000 || RadiusMeters > 1E9) return {};
		// 既定は16～2km帯，全球排水用は半径比0.9～0.1125の４帯域へ広げる
		constexpr std::array<double, 4> WavelengthMeters { 16000, 8000, 4000, 2000 }, RidgeAmplitudesMeters { 150, 100, 60, 25 };
		PlanetMesoRelief Result;
		Result.BandAmplitudesMeters = RidgeAmplitudesMeters;
		std::uint64_t State = Seed ^ UINT64_C(0X4D45534F52454C46);
		for(std::size_t Band = 0; Band < WavelengthMeters.size(); ++Band) {
			const double MaximumWavelength = RadiusMeters * 0.9 / (1U << Band);
			const double Wavelength = GlobalWavelengths ? MaximumWavelength : std::min(WavelengthMeters[Band], MaximumWavelength);
			const double Frequency = 2 * std::numbers::pi_v<double> * RadiusMeters / Wavelength;
			for(std::size_t Direction = 0; Direction < 3; ++Direction) {
				const double Z = 2 * NextFraction(State) - 1, Azimuth = 2 * std::numbers::pi_v<double> * NextFraction(State);
				const double Width = std::sqrt(std::max(0.0, 1 - Z * Z));
				Wave &Value = Result.Waves[Band * 3 + Direction];
				Value.Axis = { Width * std::cos(Azimuth), Width * std::sin(Azimuth), Z };
				Value.Frequency = Frequency;
				Value.Phase = 2 * std::numbers::pi_v<double> * NextFraction(State);
			}
		}
		// 固定した波群の返戻
		return Result;
	}

	/**
	 * 同じ位相で標高と必要時だけ接線勾配を評価する
	 * 計算量：O(1)（４帯域，各３波を固定回数評価）
	 * @tparam Derivatives 接線勾配を計算する場合に真
	 * @param Direction 球面上の正規方向
	 * @return 解析的な標高と指定時の勾配
	 */
	template<bool Derivatives> TerrainSample PlanetMesoRelief::EvaluateImpl(const UnitDirection Direction) const {
		constexpr double InverseRootThree = 0.5773502691896257645;
		const Vec3 Point = Direction.Vector();
		TerrainSample Result;
		for(std::size_t Band = 0; Band < BandAmplitudesMeters.size(); ++Band) {
			double Signal = 0;
			Vec3 SignalGradient;
			for(std::size_t Index = 0; Index < 3; ++Index) {
				const Wave &Value = Waves[Band * 3 + Index];
				const double Phase = Value.Frequency * Value.Axis.Dot(Point) + Value.Phase;
				Signal += std::sin(Phase) * InverseRootThree;
				if constexpr(Derivatives) SignalGradient = SignalGradient + Value.Axis * (Value.Frequency * std::cos(Phase) * InverseRootThree);
			}
			const double Square = Signal * Signal;
			// 支持端で値・１階・２階微分を零へ接続し，角張った絶対値の稜線を作らない
			if(Square >= 1) continue;
			const double Support = 1 - Square, Amplitude = BandAmplitudesMeters[Band];
			Result.ElevationMeters += Amplitude * Support * Support * Support;
			if constexpr(Derivatives) {
				Result.TangentialGradientPerRadian =
				Result.TangentialGradientPerRadian + SignalGradient * (-6 * Amplitude * Signal * Support * Support);
			}
		}
		if constexpr(Derivatives) {
			Result.TangentialGradientPerRadian = Result.TangentialGradientPerRadian - Point * Result.TangentialGradientPerRadian.Dot(Point);
		}
		// 高地マスクを適用する前の地形の返戻
		return Result;
	}

	/**
	 * 波群の標高と解析接線勾配を取得する
	 * @param Direction 球面上の正規方向
	 * @return 高地・河道マスク適用前の標高と勾配
	 */
	TerrainSample PlanetMesoRelief::Evaluate(const UnitDirection Direction) const {
		return EvaluateImpl<true>(Direction);
	}

	/**
	 * 勾配の算出を省いて同じ波群の標高を取得する
	 * @param Direction 球面上の正規方向
	 * @return Evaluate と同じ演算順の標高
	 */
	double PlanetMesoRelief::EvaluateElevation(const UnitDirection Direction) const {
		return EvaluateImpl<false>(Direction).ElevationMeters;
	}
} // namespace PlanetCore
