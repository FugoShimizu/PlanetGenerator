#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace PlanetCore {
	// 一つの不変な元場の準備にだけ属する，固定容量の厳密な地点標本
	class SpillSampleCache {
	private:

		struct Entry {
			std::array<std::uint64_t, 3> Key {};
			TerrainSample Value {};
			bool IsOccupied = false;
		};

		std::vector<Entry> Entries;
		std::uint32_t Used = 0;
		static std::array<std::uint64_t, 3> MakeKey(const UnitDirection Direction);
		static std::size_t FindSlot(const std::array<std::uint64_t, 3> &Key, const std::size_t Count);

		explicit SpillSampleCache(const std::uint32_t Capacity);

	public:

		// 構造体の実寸で，準備用の保持領域を６４ MiB 以内に制限
		static constexpr std::size_t MaximumByteCapacity = 64 * 1024 * 1024;
		static constexpr std::uint32_t MaximumCapacity = static_cast<std::uint32_t>(MaximumByteCapacity / sizeof(Entry));
		static std::optional<SpillSampleCache> Create(const std::uint32_t Capacity);
		std::optional<TerrainSample> Find(const UnitDirection Direction) const;
		std::uint32_t GetCount() const;
		std::size_t GetMemoryBytes() const;
		void Store(const UnitDirection Direction, const TerrainSample &Value);
	};
}
