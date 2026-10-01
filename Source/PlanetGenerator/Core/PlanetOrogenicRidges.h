#pragma once

#include "PlanetGeology.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	inline constexpr std::uint32_t OrogenicRidgePrototypeVersion = 1;

	/** 地質境界に沿った有限支持の山稜記述子 */
	struct OrogenicRidgeDescriptor {
		Vec3 Center;
		Vec3 Along;
		Vec3 Across;
		Vec3 BoundsMin;
		Vec3 BoundsMax;
		std::uint32_t SourceFeature = 0;
		double AlongSine = 0;
		double AcrossSine = 0;
		double SupportCosine = 1;
		double AmplitudeMeters = 0;
		double BendPhase = 0;
	};

	struct OrogenicRidgeStatistics {
		std::uint32_t SourceFeatures = 0;
		std::uint32_t Descriptors = 0;
		std::uint32_t IndexNodes = 0;
		double MaximumDescriptorAmplitudeMeters = 0;
	};

	/** 地質履歴から一定の山稜を準備し，表示順に依存せず標高と勾配を評価する研究試作 */
	class PlanetOrogenicRidges {
	private:

		struct IndexNode {
			Vec3 Min;
			Vec3 Max;
			std::uint32_t Begin = 0;
			std::uint32_t Count = 0;
			std::uint32_t Left = 0;
			std::uint32_t Right = 0;
		};

		std::vector<OrogenicRidgeDescriptor> Ridges;
		std::vector<std::uint32_t> Order;
		std::vector<IndexNode> Index;
		OrogenicRidgeStatistics Stats;
		std::uint64_t SourceDefinitionHash = 0;

		void BuildIndex();
		template<bool Derivatives>
		TerrainSample EvaluateImpl(UnitDirection Direction, bool Indexed, std::size_t *Candidates = nullptr) const;

	public:

		static std::optional<PlanetOrogenicRidges> Create(const GeologyField &Source, std::stop_token Stop = {});
		TerrainSample Evaluate(UnitDirection Direction) const;
		double EvaluateElevation(UnitDirection Direction) const;
		TerrainSample EvaluateReference(UnitDirection Direction) const;
		std::size_t CandidateCount(UnitDirection Direction) const;
		std::span<const OrogenicRidgeDescriptor> Descriptors() const;
		const OrogenicRidgeStatistics &Statistics() const;
		std::uint64_t DefinitionHash() const;
	};
} // namespace PlanetCore
