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
	// 正則な閉曲線と，追跡を完了出来ない停止理由
	enum class TerrainContourEnd : std::uint8_t { Closed, CriticalPoint, StepLimit, LengthLimit, VertexLimit, Terminal };

	struct TerrainContourSettings {
		// 一定の水位と，位置・高さ・折線近似の物理精度
		double RadiusMeters = 1000000;
		double LevelMeters = 0;
		double HeightToleranceMeters = 1E-6;
		double PositionToleranceMeters = 0.001;
		double ChordToleranceMeters = 0.05;
		double GradientTolerance = 1E-8;
		// 有界な追跡範囲と適応刻み
		double MinimumStepMeters = 0.001;
		double MaximumStepMeters = 50;
		double MaximumLengthMeters = 100000;
		std::uint32_t MaximumVertices = 32768;
		// 最大１０２４個の等高点の何れかへ達する部分経路；始点の閉鎖とは区別する
		std::vector<UnitDirection> Terminals;
		double TerminalRadiusMeters = 0.05;
		bool IsValid() const;
	};

	struct TerrainContourSample {
		// 元場の等高点と，始点からの累積球面弧長
		UnitDirection Direction;
		double ElevationMeters = 0;
		double DistanceMeters = 0;
	};

	struct TerrainContour {
		// 閉鎖時だけ末尾は先頭と同一；部分経路を閉じた水面として扱わない
		std::vector<TerrainContourSample> Samples;
		TerrainContourEnd End = TerrainContourEnd::VertexLimit;
		// 終点到達の場合だけ，入力の終点配列へ対応
		std::optional<std::uint32_t> TerminalIndex;
		std::uint32_t Evaluations = 0;
		std::uint32_t RejectedSteps = 0;
		double MaximumHeightResidualMeters = 0;
		double MaximumChordCorrectionMeters = 0;
	};

	// 閉鎖点を重複しない局所平面の岸の検査
	bool IsSimplePlanarContour(const std::span<const std::array<double, 2>> Vertices);
	// 閉曲線の向きを保持した，半球内の代数的な球面面積
	std::optional<double> MeasureTerrainContourArea(const TerrainContour &Contour, const double RadiusMeters);

	// 低い側を左に保ち，変更しない元地形の正則な等高線を追跡する
	std::optional<TerrainContour> TraceTerrainContour(
		const UnitDirection Initial,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const TerrainContourSettings &Settings,
		const std::stop_token Stop= {}
	);
}
