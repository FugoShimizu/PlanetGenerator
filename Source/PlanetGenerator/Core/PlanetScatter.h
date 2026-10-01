#pragma once

#include "PlanetCore.h"
#include <stop_token>

namespace PlanetCore {
	/** 視点・地形の LOD から独立した球面配置セル */
	struct ScatterCell {
		std::uint64_t U = 0, V = 0;
		std::uint8_t Face = 0;
		bool operator==(const ScatterCell &) const = default;
	};

	/** 配置を再現するための定義，間隔は最小分離距離ではない */
	struct ScatterSettings {
		std::uint64_t Seed = 42, Layer = 0;
		double RadiusMeters = 1000000, SpacingMeters = 1;
	};

	/** 地質とバイオームの適性を適用する前の配置候補 */
	struct ScatterCandidate {
		ScatterCell Cell;
		UnitDirection Direction;
		double Selection = 0, RotationTurns = 0, Size = 1;
	};

	struct ScatterQueryResult {
		std::vector<ScatterCandidate> Candidates;
		std::uint64_t ExaminedCells = 0;
	};

	/** 全球を保存せず，必要な球面範囲だけを同じセルから再生成する */
	class PlanetScatter {
	private:

		ScatterSettings Settings;
		std::uint64_t CellsPerSide = 0;
		PlanetScatter() = default;

	public:

		static std::optional<PlanetScatter> Create(const ScatterSettings &Settings);
		std::optional<ScatterCandidate> Candidate(const ScatterCell Cell) const;
		std::optional<ScatterQueryResult> Query(
			const UnitDirection Center, const double ArcRadiusMeters, const std::uint64_t MaximumExaminedCells,
			const std::stop_token Stop = {}
		) const;
		std::uint64_t Resolution() const;
	};
}
