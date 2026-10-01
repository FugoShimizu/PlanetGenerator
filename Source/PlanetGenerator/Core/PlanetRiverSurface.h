#pragma once

#include "PlanetRiverTerrain.h"

namespace PlanetCore {
	/** 断面が陸岸で閉じるか，別の水面へ続くかを区別する */
	enum class RiverWaterBoundary : std::uint8_t { DryBank, OpenWater, Ocean };

	struct RiverWaterEdge {
		Vec3 Direction;
		double WaterMeters = 0, GroundMeters = 0, DistanceMeters = 0, BracketWidthMeters = 0;
		RiverWaterBoundary Boundary = RiverWaterBoundary::OpenWater;
		bool bUsesProfileExtension = false;
	};

	struct RiverWaterSection {
		RiverWaterEdge Center, Left, Right;
		Vec3 DownstreamTangent;
		std::uint32_t Evaluations = 0;
	};

	std::optional<RiverWaterSection> SampleRiverWaterSection(
		const RiverTerrainField &Field, const std::size_t ReachIndex, const double Fraction, const std::stop_token Stop = {});
}
