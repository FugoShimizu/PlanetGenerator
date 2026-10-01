#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>

namespace PlanetCore {
	/** 外向きの閉球面を検査する
	 * 外部適応網の既定は６５５３６点，内部の固定高密度網は追加後１６７９３８点迄 */
	bool IsClosedSphericalSamplingMesh(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop= {},
		const std::size_t MaximumNodes=65536
	);
}
