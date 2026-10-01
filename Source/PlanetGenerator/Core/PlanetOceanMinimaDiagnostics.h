#pragma once

#include <cstddef>
#include <cstdint>

#if WITH_DEV_AUTOMATION_TESTS

namespace PlanetCore {
	enum class OceanMinimaPreparationStage : std::uint8_t {
		Input,
		GeologySamples,
		OriginalWetSamples,
		CandidateDiscovery,
		WitnessInsertion,
		MeshValidation,
		GraphConnection,
		Complete
	};

	struct OceanMinimaPreparationDiagnostics {
		OceanMinimaPreparationStage Stage = OceanMinimaPreparationStage::Input;
		std::uint64_t Evaluations = 0;
		std::uint32_t CriticalCandidates = 0;
		std::size_t WetWitnesses = 0;
		bool SourceRejected = false;
		bool Completed = false;
	};

	// 検査専用
	// 同じスレッドの直前の準備記録だけを取得し，本番の共有状態へ使わない
	OceanMinimaPreparationDiagnostics InspectOceanMinimaPreparation();
}

#endif
