#pragma once

#include "PlanetCore.h"
#include <cstdint>
#include <functional>
#include <optional>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	// 下降経路の停止理由は，全域排水の達成と区別する
	enum class TerrainDescentEnd : std::uint8_t { HeightLimit, GradientLimit, LengthLimit, StepLimit, BudgetLimit };

	struct TerrainDescentSettings {
		// 物理尺度，停止条件と有界な計算予算
		double RadiusMeters = 1000000;
		double StopElevationMeters = 0;
		double GradientTolerance = 1E-6;
		double MinimumStepMeters = 0.001;
		double MaximumStepMeters = 50;
		double PositionToleranceMeters = 0.02;
		double MaximumLengthMeters = 20000;
		std::uint32_t MaximumVertices = 4096;
		bool IsValid() const;
	};

	struct TerrainDescentSample {
		// 元場上の方向，標高と始点からの累積弧長
		UnitDirection Direction;
		double ElevationMeters = 0;
		double DistanceMeters = 0;
	};

	struct TerrainDescentPath {
		// 変更しない標高と計測費用，末尾の停止理由
		std::vector<TerrainDescentSample> Samples;
		std::uint32_t Evaluations = 0;
		std::uint32_t RejectedSteps = 0;
		double MaximumEstimatedErrorMeters = 0;
		TerrainDescentEnd End = TerrainDescentEnd::BudgetLimit;
	};

	// 解析勾配に沿う下降曲線を球面上で追跡する
	std::optional<TerrainDescentPath> TraceTerrainDescent(
		const UnitDirection Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainDescentSettings &Settings,
		const std::stop_token Stop= {},
		const std::function<double(UnitDirection)> &StopDomainDistance= {}
	);
}
