#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct RiverReceiverProfileSettings {
		double RadiusMeters = 1000000;
		double MaximumSampleStepMeters = 25;
		double MinimumDepthMeters = 0.3;
		double MinimumDownstreamSlope = 1E-7;
		std::uint32_t MaximumSamples = 65536;
	};

	enum class RiverReceiverProfileError : std::uint8_t {
		None,
		InvalidInput,
		InvalidPath,
		InvalidTerrain,
		SampleBudget,
		ReceiverExposed,
		NumericFailure,
		Cancelled
	};

	struct RiverReceiverProfileSample {
		UnitDirection Direction;
		double DistanceMeters = 0;
		double GroundMeters = 0;
		double WaterMeters = 0;
	};

	struct RiverReceiverProfileStatistics {
		double LengthMeters = 0;
		double MinimumGroundMeters = 0;
		double MaximumGroundMeters = 0;
		double CrestDistanceMeters = 0;
		double MaximumWaterDepthMeters = 0;
		double MaximumDownstreamSlope = 0;
		double MinimumActualDownstreamSlope = 0;
	};

	/** 最終地形の確認済通路上に，受入河川の水位から下降する最小水面を準備する */
	struct RiverReceiverProfile {
		std::vector<RiverReceiverProfileSample> Samples;
		RiverReceiverProfileStatistics Statistics;
	};

	struct RiverReceiverConditioningSettings {
		double MinimumDepthMeters = 0.3;
		double MaximumDepthMeters = 5;
		double MinimumDownstreamSlope = 1E-7;
		double MaximumCutMeters = 20;
	};

	enum class RiverReceiverConditioningError : std::uint8_t {
		None,
		InvalidInput,
		ReceiverExposed,
		IrreducibleDepth,
		CutLimit,
		NumericFailure,
		Cancelled
	};

	struct RiverReceiverConditioningSample {
		UnitDirection Direction;
		double DistanceMeters = 0;
		double OriginalGroundMeters = 0;
		double ConditionedGroundMeters = 0;
		double WaterMeters = 0;
		double CutMeters = 0;
	};

	struct RiverReceiverConditioningDiagnostics {
		double MinimumIrreducibleDepthMeters = 0;
		double MinimumRequiredMaximumCutMeters = 0;
	};

	struct RiverReceiverConditioning {
		std::vector<RiverReceiverConditioningSample> Samples;
		RiverReceiverConditioningDiagnostics Bounds;
		double MaximumWaterDepthMeters = 0;
		double MaximumActualCutMeters = 0;
		double MaximumDownstreamSlope = 0;
		double CutAreaPerUnitWidthSquareMeters = 0;
	};

	struct RiverReceiverBankSettings {
		double RadiusMeters = 1000000;
		std::array<double, 3> HalfWidthsMeters { 20, 100, 500 };
		double FreeboardMeters = 0.3;
		std::uint32_t SampleStride = 100;
	};

	enum class RiverReceiverBankError : std::uint8_t { None, InvalidInput, InvalidPath, InvalidTerrain, Cancelled };

	struct RiverReceiverBankStatistics {
		std::uint32_t Stations = 0;
		std::array<std::uint32_t, 3> BelowFreeboardSides {};
		std::array<std::uint32_t, 3> SubmergedSides {};
		std::array<double, 3> MaximumFreeboardDeficitMeters {};
	};

	std::optional<RiverReceiverProfile> PrepareRiverReceiverProfile(
		std::span<const UnitDirection> Path,
		const std::function<double(UnitDirection)> &Ground,
		double ReceiverWaterMeters,
		const RiverReceiverProfileSettings &Settings,
		RiverReceiverProfileError &Error,
		std::stop_token Stop={}
	);

	std::optional<RiverReceiverConditioning> ConditionRiverReceiverProfile(
		std::span<const RiverReceiverProfileSample> Original,
		const RiverReceiverConditioningSettings &Settings,
		RiverReceiverConditioningError &Error,
		RiverReceiverConditioningDiagnostics *Diagnostics=nullptr,
		std::stop_token Stop={}
	);

	std::optional<RiverReceiverBankStatistics> SurveyRiverReceiverBanks(
		std::span<const RiverReceiverConditioningSample> Profile,
		const std::function<double(UnitDirection)> &Ground,
		const RiverReceiverBankSettings &Settings,
		RiverReceiverBankError &Error,
		std::stop_token Stop={}
	);
}
