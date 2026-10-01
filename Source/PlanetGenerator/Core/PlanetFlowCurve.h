#pragma once

#include "PlanetCore.h"
#include <optional>

namespace PlanetCore {
	struct TerrainFlowCurveSample {
		// 球面上の位置と，公称距離に対する接線微分
		UnitDirection Direction;
		Vec3 DirectionDerivativePerMeter;
		Vec3 DirectionSecondDerivativePerMeterSquared;
	};

	/** 直交点又は制約された経路端の最近点，距離は単位球上の弦長の二乗 */
	struct TerrainFlowFootpoint {
		TerrainFlowCurveSample Curve;
		double Fraction = 0;
		double SquaredChordDistance = 0;
		Vec3 SquaredChordGradientPerRadian;
		Vec3 ParameterGradientMetersPerRadian;
	};

	/** 下降方向を共有する河道候補の一区間；水位・河床の制約は別に検査する */
	class TerrainFlowSegment {
	private:

		// 正規化前の局所差分形で，惑星規模の位置の相殺を避ける
		UnitDirection Start;
		UnitDirection End;
		Vec3 StartVelocity;
		Vec3 EndVelocity;
		Vec3 StartAcceleration;
		Vec3 EndAcceleration;
		bool HasSharedCurvature = false;
		double ParameterLengthMeters = 0;

		TerrainFlowSegment(
			const UnitDirection First,
			const UnitDirection Last,
			const Vec3 FirstVelocity,
			const Vec3 LastVelocity,
			const double Length
		);

	public:

		// 端点と接線を受け，描画解像度とは独立した連続曲線を準備する
		static std::optional<TerrainFlowSegment> Create(
			const UnitDirection First,
			const UnitDirection Last,
			const Vec3 FirstTangent,
			const Vec3 LastTangent,
			const double Radius
		);

		static std::optional<TerrainFlowSegment> CreateWithCurvature(
			const UnitDirection First,
			const UnitDirection Last,
			const Vec3 FirstTangent,
			const Vec3 LastTangent,
			const Vec3 FirstCurvature,
			const Vec3 LastCurvature,
			const double Radius
		);

		std::optional<TerrainFlowCurveSample> Evaluate(const double Fraction) const;
		std::optional<TerrainFlowFootpoint> FindInteriorFootpoint(const UnitDirection Query) const;
		double GetParameterLengthMeters() const;
	};
}
