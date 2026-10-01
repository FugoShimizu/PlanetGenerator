#pragma once

#include "PlanetSphericalRefinement.h"
#include "PlanetSphericalSpline.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stop_token>

namespace PlanetCore {
	class DrainageNetwork;
	class OceanConnectivityField;

	/** 補間へ渡す節点接線の準備方法 */
	enum class SphericalGradientMode : std::uint8_t { Analytic, Sampled };

	struct SphericalDrainageStatistics {
		// 比較場の節点・常駐量と海底復元尺度
		SphericalGradientMode GradientMode = SphericalGradientMode::Analytic;
		std::uint32_t Nodes = 0;
		std::uint32_t CoastalOutlets = 0;
		std::size_t SplineResidentBytes = 0;
		double SeabedBlendStartMeters = 0;
		double SeabedMinimumBlendWidthMeters = 0;
		SphericalReliefStatistics Relief;
	};

	/** 下降する共有節点の C1 基盤と，水没領域を変えない元海底の合成 */
	class PlanetSphericalDrainage {
	private:

		// 海底復元の未縮尺定数
		static constexpr double UnscaledSeabedBlendStartMeters = 62.5;
		static constexpr double UnscaledSeabedMinimumBlendWidthMeters = 62.5;

		// 同じ生成定義を共有する不変場
		SphericalSplineField Surface;
		std::shared_ptr<const OceanConnectivityField> Ocean;
		std::shared_ptr<const DrainageNetwork> Drainage;
		SphericalDrainageStatistics Stats;

		// 補間と共有場の内部準備
		static std::optional<SphericalSplineField> PrepareSurface(
			const DrainageNetwork &Network,
			const OceanConnectivityField &Ocean,
			const SphericalReliefSettings *const Refinement,
			SphericalReliefStatistics &RefinementStats,
			const std::stop_token Stop,
			const SphericalReliefSamplingMesh *const InitialMesh=nullptr,
			const SphericalGradientMode Mode=SphericalGradientMode::Analytic
		);
		static std::optional<PlanetSphericalDrainage> CreateImpl(
			std::shared_ptr<const OceanConnectivityField> Ocean,
			std::shared_ptr<const DrainageNetwork> Drainage,
			const SphericalReliefSettings *const Refinement,
			const std::stop_token Stop,
			const SphericalReliefSamplingMesh *const InitialMesh=nullptr,
			const SphericalGradientMode Mode=SphericalGradientMode::Analytic
		);
		template<bool HasDerivatives> TerrainSample EvaluateImpl(const UnitDirection Direction) const;

		PlanetSphericalDrainage(
			SphericalSplineField Surface,
			std::shared_ptr<const OceanConnectivityField> Ocean,
			std::shared_ptr<const DrainageNetwork> Drainage
		);

	public:

		// 公開する比較場の準備
		static std::optional<PlanetSphericalDrainage> Create(
			std::shared_ptr<const OceanConnectivityField> Ocean,
			std::shared_ptr<const DrainageNetwork> Drainage,
			const std::stop_token Stop={}
		);

		static std::optional<PlanetSphericalDrainage> CreateWithRefinedRelief(
			std::shared_ptr<const OceanConnectivityField> Ocean,
			std::shared_ptr<const DrainageNetwork> Drainage,
			const SphericalReliefSettings &Refinement,
			const std::stop_token Stop={}
		);

		/** 元起伏の適応標本化を先に行い，同じ網で排水と C1 基盤を準備する */
		static std::optional<PlanetSphericalDrainage> CreateWithAdaptiveDrainage(
			std::shared_ptr<const OceanConnectivityField> Ocean,
			const SphericalReliefSettings &Sampling,
			const std::stop_token Stop={}
		);

		/** 同じ適応排水の準備で，解析接線と標本尺度の推定接線を比較する */
		static std::optional<PlanetSphericalDrainage> CreateWithAdaptiveDrainage(
			std::shared_ptr<const OceanConnectivityField> Ocean,
			const SphericalReliefSettings &Sampling,
			const SphericalGradientMode Mode,
			const std::stop_token Stop={}
		);

		// 不変な準備結果の照会
		const DrainageNetwork &GetDrainageNetwork() const;
		std::shared_ptr<const OceanConnectivityField> GetOriginalOcean() const;

		TerrainSample EvaluateOriginal(const UnitDirection Direction) const;
		double EvaluateOriginalElevation(const UnitDirection Direction) const;
		TerrainSample Evaluate(const UnitDirection Direction) const;
		double EvaluateElevation(const UnitDirection Direction) const;
		const SphericalDrainageStatistics &GetStatistics() const;
	};
}
