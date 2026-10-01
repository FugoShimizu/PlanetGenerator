#pragma once

#include "PlanetFlowChannel.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/** 不変な水位近似の誤差と保持量，地面の形状には使用しない */
	struct FlowWaterProfileSettings {
		double HeightToleranceMeters = 1E-5;
		double SlopeTolerance = 1E-6;
		std::uint32_t MaximumKnots = 16384;
		std::uint32_t MaximumPasses = 32;
	};

	enum class FlowWaterProfileFailure : std::uint8_t { None, Input, Source, Rising, Budget, Resolution, Cancelled };

	struct FlowWaterProfileDiagnostics {
		FlowWaterProfileFailure Failure = FlowWaterProfileFailure::None;
		std::uint32_t Knots = 0;
		std::uint32_t Passes = 0;
		std::uint32_t Evaluations = 0;
		double HeightErrorMeters = 0;
		double SlopeError = 0;
	};

	/** 準備時の元水位を，単調な C1 三次曲線として有界な RAM に保持する */
	class FlowWaterProfile {
	private:

		struct Knot {
			double DistanceMeters = 0;
			FlowChannelWaterLevel Source;
			double Slope = 0;
		};

		std::vector<Knot> Knots;

		FlowWaterProfile() = default;

		static FlowChannelWaterLevel Interpolate(const Knot &A, const Knot &B, const double DistanceMeters);

	public:

		static std::optional<FlowWaterProfile> Create(
			const std::function<std::optional<FlowChannelWaterLevel>(double)> &Source,
			const std::span<const double> InitialDistances,
			const FlowWaterProfileSettings &Settings,
			const std::stop_token Stop= {},
			FlowWaterProfileDiagnostics *const Diagnostics=nullptr
		);

		std::optional<FlowChannelWaterLevel> Sample(const double DistanceMeters) const;
		std::size_t KnotCount() const;
	};
}
