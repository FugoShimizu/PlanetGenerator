#include "Core/PlanetOrogenicRidges.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stop_token>
#include <utility>

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetOrogenicRidgesContracts, "PlanetGenerator.Geology.OrogenicRidgesContracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 地質方向付き山稜の再現性，支持索引，解析勾配を複数半径で検査する
 * @param Parameters UE 自動検査の未使用引数
 * @return 全条件に合格した場合に真
 */
bool FPlanetOrogenicRidgesContracts::RunTest(const FString &Parameters) {
	(void)Parameters;
	const double Started = FPlatformTime::Seconds();
	std::uint32_t TotalDescriptors = 0, DerivativeSamples = 0, IndexedSamples = 0;
	std::size_t MaximumCandidates = 0;
	for(const std::pair<std::uint64_t, double> &Case :
		std::array<std::pair<std::uint64_t, double>, 4>{{{42, 1000}, {42, 1000000}, {42, 1E9}, {7, 6371000}}}) {
		PlanetCore::GeologySettings Settings;
		Settings.Seed = Case.first;
		Settings.RadiusMeters = Case.second;
		const std::optional<PlanetCore::GeologyField> Geology = PlanetCore::GeologyField::Create(Settings);
		if(!TestTrue(TEXT("地質特徴を準備"), Geology.has_value())) return false;
		std::stop_source Cancelled;
		Cancelled.request_stop();
		if(!TestFalse(TEXT("中断済みの山稜準備を拒否"),
			PlanetCore::PlanetOrogenicRidges::Create(*Geology, Cancelled.get_token()).has_value())) return false;
		const std::optional<PlanetCore::PlanetOrogenicRidges> Field = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
		const std::optional<PlanetCore::PlanetOrogenicRidges> Repeat = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
		if(!TestTrue(TEXT("山稜場を再現して準備"), Field.has_value() && Repeat.has_value())) return false;
		if(!TestTrue(TEXT("同じ惑星定義を保持"), Field->DefinitionHash() == Settings.IdentityHash())) return false;
		const std::span<const PlanetCore::OrogenicRidgeDescriptor> Ridges = Field->Descriptors();
		if(!TestTrue(TEXT("地質から有効な山稜を抽出"), !Ridges.empty() && Ridges.size() == Repeat->Descriptors().size())) return false;
		if(!TestTrue(TEXT("山稜索引を準備"), Field->Statistics().IndexNodes > 0 &&
			Field->Statistics().Descriptors == Ridges.size())) return false;
		TotalDescriptors += static_cast<std::uint32_t>(Ridges.size());
		const std::size_t Samples = std::min<std::size_t>(Ridges.size(), 48);
		for(std::size_t Index = 0; Index < Samples; ++Index) {
			const PlanetCore::OrogenicRidgeDescriptor &Ridge = Ridges[Index];
			if(!TestTrue(TEXT("山稜は有限の高度と細長い形状"),
				std::isfinite(Ridge.AmplitudeMeters) && Ridge.AmplitudeMeters > 0 && Ridge.AmplitudeMeters <= 450 &&
				Ridge.AlongSine > Ridge.AcrossSine)) return false;
			const PlanetCore::Vec3 Probe = Ridge.Center + Ridge.Along * (0.45 * Ridge.AlongSine) +
				Ridge.Across * (0.2 * Ridge.AcrossSine);
			const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(Probe);
			const PlanetCore::TerrainSample Actual = Field->Evaluate(Direction);
			const PlanetCore::TerrainSample Reference = Field->EvaluateReference(Direction);
			const double Scalar = Field->EvaluateElevation(Direction), Repeated = Repeat->EvaluateElevation(Direction);
			if(!TestTrue(TEXT("索引・値・再準備の標高がビット一致"),
				std::bit_cast<std::uint64_t>(Actual.ElevationMeters) == std::bit_cast<std::uint64_t>(Reference.ElevationMeters) &&
				std::bit_cast<std::uint64_t>(Scalar) == std::bit_cast<std::uint64_t>(Actual.ElevationMeters) &&
				std::bit_cast<std::uint64_t>(Repeated) == std::bit_cast<std::uint64_t>(Scalar))) return false;
			const PlanetCore::Vec3 Gradient = Actual.TangentialGradientPerRadian - Reference.TangentialGradientPerRadian;
			if(!TestTrue(TEXT("索引と全走査の勾配が一致"), Gradient.Length() <= 1E-7)) return false;
			const PlanetCore::Vec3 Point = Direction.Vector();
			const PlanetCore::Vec3 Axis = Ridge.Across - Point * Point.Dot(Ridge.Across);
			const PlanetCore::Vec3 Tangent = Axis * (1 / Axis.Length());
			const double Step = std::max(0.03, Case.second * 1E-10) / Case.second;
			const PlanetCore::UnitDirection Before = *PlanetCore::UnitDirection::FromVector(Point - Tangent * Step);
			const PlanetCore::UnitDirection After = *PlanetCore::UnitDirection::FromVector(Point + Tangent * Step);
			const double Difference = (Field->EvaluateElevation(After) - Field->EvaluateElevation(Before)) / (2 * Step);
			const double Analytic = Actual.TangentialGradientPerRadian.Dot(Tangent);
			const double Tolerance = std::max(0.04, std::abs(Analytic) * 0.0005);
			if(!TestTrue(TEXT("山稜の解析接線勾配が独立差分へ収束"),
				std::isfinite(Difference) && std::isfinite(Analytic) && std::abs(Difference - Analytic) <= Tolerance)) return false;
			MaximumCandidates = std::max(MaximumCandidates, Field->CandidateCount(Direction));
			++DerivativeSamples;
		}
		for(unsigned Index = 0; Index < 128; ++Index) {
			const double Z = 1 - 2 * (Index + 0.5) / 128, Azimuth = Index * 2.399963229728653;
			const double Width = std::sqrt(1 - Z * Z);
			const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector(
				{Width * std::cos(Azimuth), Width * std::sin(Azimuth), Z});
			const PlanetCore::TerrainSample Actual = Field->Evaluate(Direction);
			const PlanetCore::TerrainSample Reference = Field->EvaluateReference(Direction);
			if(!TestTrue(TEXT("全球方向の索引と全走査が一致"),
				std::bit_cast<std::uint64_t>(Actual.ElevationMeters) == std::bit_cast<std::uint64_t>(Reference.ElevationMeters) &&
				(Actual.TangentialGradientPerRadian - Reference.TangentialGradientPerRadian).Length() <= 1E-7)) return false;
			MaximumCandidates = std::max(MaximumCandidates, Field->CandidateCount(Direction));
			++IndexedSamples;
		}
	}
	AddInfo(FString::Printf(TEXT("Orogenic ridge prototype: descriptors=%u derivativeSamples=%u ")
		TEXT("indexedSamples=%u maximumCandidates=%llu elapsed=%.3fs. Not integrated into final terrain."),
		TotalDescriptors, DerivativeSamples, IndexedSamples,
		static_cast<unsigned long long>(MaximumCandidates), FPlatformTime::Seconds() - Started));
	// 地質方向付きの候補場だけを検査した結果の返戻
	return true;
}
#endif
