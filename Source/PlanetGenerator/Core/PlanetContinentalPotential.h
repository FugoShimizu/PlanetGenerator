#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t ContinentalPotentialVersion = 3;

	/** 正の隆起密度を持つ有限な円形源，座標と半径は単位円板の尺度 */
	struct ContinentalUpliftSource {
		double X = 0;
		double Y = 0;
		double Radius = 0.1;
		double WeightMeters = 100;
	};

	struct ContinentalPotentialSettings {
		/** 円板全体の厳密に負の Laplacian を保つ弱い背景隆起 */
		double BackgroundMeters = 1;
		std::vector<ContinentalUpliftSource> Sources;
	};

	struct ContinentalPotentialSample {
		double HeightMeters = 0;
		std::array<double, 2> GradientPerUnit {};
		/** 順に XX，XY，YY */
		std::array<double, 3> HessianPerUnitSquared {};
		double LaplacianPerUnitSquared = 0;
	};

	/** 円板内の指定近傍で成り立つ，微分の作用素ノルムの保守的な上界 */
	struct ContinentalPotentialDerivativeBounds {
		double HessianNormPerUnitSquared = 0;
		double ThirdDerivativeNormPerUnitCubed = 0;
	};

	/**
	 * 海岸が零の単位円板へ，正の有限源から滑らかな高さ場を構成する比較用の基盤
	 * 正の背景項と非負の源に依り，円板内部では厳密に超調和的
	 * 山頂・鞍点での経路選択，球面・海底・侵食・河道の統合は此の型の外で扱う
	 * 全河川の大洋到達又は自然な完成地形を証明した型ではない
	 */
	class ContinentalPotential {
	private:

		struct PreparedSource {
			double X = 0;
			double Y = 0;
			double SquaredMagnitude = 0;
			double SquaredRadius = 0;
			double InverseSquaredRadius = 0;
			double WeightMeters = 0;
			double Radius = 0;
			double Magnitude = 0;
		};

		std::vector<PreparedSource> Sources;
		std::vector<ContinentalPotential> Layers;
		double BackgroundMeters = 0;
		double BlendTemperatureMeters = 0;
		double AbsoluteHeightBoundMeters = 0;
		double GradientMagnitudeBound = 0;
		double LaplacianMagnitudeBound = 0;
		std::uint64_t Hash = 0;

		ContinentalPotential() = default;

		static bool IsValidPoint(double X, double Y);
		void TightenHeightBound();

		template<bool Derivatives> ContinentalPotentialSample EvaluateInside(double X, double Y) const;

	public:

		static constexpr std::size_t MaximumSources = 256;
		static constexpr std::size_t MaximumLayers = 4;

		static std::optional<ContinentalPotential> Create(
			const ContinentalPotentialSettings &Settings,
			std::stop_token Stop={}
		);

		static std::optional<ContinentalPotential> CreateBlend(
			const std::vector<ContinentalPotentialSettings> &LayerSettings,
			double TemperatureMeters,
			std::stop_token Stop={}
		);

		std::optional<double> EvaluateHeight(double X, double Y) const;
		std::optional<ContinentalPotentialSample> Evaluate(double X, double Y) const;
		std::optional<ContinentalPotentialDerivativeBounds> GetDerivativeBounds(double X, double Y, double NeighborhoodRadius) const;
		double GetAbsoluteHeightBoundMeters() const;
		double GetLaplacianMagnitudeBound() const;
		std::size_t GetSourceCount() const;
		std::uint64_t GetDefinitionHash() const;
	};
}
