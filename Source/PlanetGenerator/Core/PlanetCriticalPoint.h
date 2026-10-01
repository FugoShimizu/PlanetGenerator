#pragma once

#include "PlanetCore.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	// 非退化な停留点の種類
	enum class TerrainCriticalKind : std::uint8_t { Minimum, Saddle, Maximum };

	struct TerrainCriticalSettings {
		// 物理尺度と局所探索の範囲
		double RadiusMeters = 1000000;
		double DifferenceStepMeters = 0.5;
		double MaximumStepMeters = 100;
		double MaximumDisplacementMeters = 2000;
		double GradientTolerance = 1E-8;
		double PositionToleranceMeters = 1E-4;
		std::uint32_t MaximumIterations = 48;
		bool IsValid() const;
	};

	enum class TerrainCriticalFailure : std::uint8_t {
		None,
		InvalidInput,
		InvalidSource,
		Cancelled,
		Degenerate,
		NonFiniteStep,
		UnstableCurvature,
		Range,
		NoProgress,
		IterationLimit
	};

	struct TerrainCriticalDiagnostics {
		// 実際に照会した末尾位置と，未制限の Newton 変位・曲率の収束条件
		TerrainCriticalFailure Failure = TerrainCriticalFailure::None;
		Vec3 Direction;
		double ElevationMeters = 0;
		double GradientNorm = 0;
		double InitialNewtonStepMeters = 0;
		double NewtonStepMeters = 0;
		double MinimumCurvaturePerMeter = 0;
		double MaximumCurvaturePerMeter = 0;
		double CurvatureDifferencePerMeter = 0;
		double DisplacementMeters = 0;
		std::uint32_t Evaluations = 0;
		std::uint32_t Iterations = 0;
		std::optional<TerrainCriticalKind> AcceptedKind;
	};

	struct TerrainCriticalPoint {
		// 元場上の停留位置，物理勾配，曲率と主方向
		UnitDirection Direction;
		double ElevationMeters = 0;
		double GradientNorm = 0;
		double MinimumCurvaturePerMeter = 0;
		double MaximumCurvaturePerMeter = 0;
		Vec3 MinimumCurvatureDirection;
		TerrainCriticalKind Kind = TerrainCriticalKind::Saddle;
		std::uint32_t Evaluations = 0;
		std::uint32_t Iterations = 0;
	};

	// 同じ連続場の解析勾配を使う局所停留点探索
	std::optional<TerrainCriticalPoint> RefineTerrainCriticalPoint(
		const UnitDirection Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainCriticalSettings &Settings,
		const std::stop_token Stop= {},
		TerrainCriticalDiagnostics *Diagnostics=nullptr
	);
}
