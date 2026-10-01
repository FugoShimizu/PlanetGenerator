#pragma once

#include "PlanetCore.h"
#include "PlanetSphericalWarp.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct ContinentalDomainCap {
		UnitDirection Center;
		double RadiusRadians = 0;
	};

	struct ConnectedOceanDomainSettings {
		SphericalWarpSettings Warp;
		std::vector<ContinentalDomainCap> Continents;
		double CoastTransitionMeters = 20000;
	};

	enum class ConnectedOceanDomainError : std::uint8_t {
		None,
		Cancelled,
		InvalidSettings,
		InvalidCap,
		TouchingContinents,
		OceanAreaOutsideRange,
		UnresolvedCoastWidth
	};

	struct ConnectedOceanDomainSample {
		double CoastSign = 0;
		Vec3 TangentialGradientPerRadian;
	};

	struct ConnectedOceanDomainStatistics {
		std::size_t ContinentalCount = 0;
		double OceanFraction = 0;
		double MinimumSeparationRadians = 0;
		double MinimumCoastWidthRadians = 0;
	};

	/**
	 * 交差しない陸域円板の補集合と可逆写像で，連結海域を構成する比較用の不変場
	 * 符号は海域で負，陸域で正，海岸で零
	 * 標高・距離・完成した地質場ではない
	 * 最終地形と河川への統合は未実施
	 * 丸い大陸を完成した景観として公開しない
	 */
	class ConnectedOceanDomain {
	private:

		struct PreparedCap {
			Vec3 Center;
			double CoastCosine = 0;
			double InverseTransitionWidth = 0;
		};

		std::vector<PreparedCap> Caps;
		std::optional<PlanetSphericalWarp> Warp;
		ConnectedOceanDomainStatistics Stats;
		Vec3 Anchor;
		std::uint64_t Hash = 0;

		ConnectedOceanDomain() = default;

	public:

		static std::optional<ConnectedOceanDomain> Create(
			const ConnectedOceanDomainSettings &Settings,
			ConnectedOceanDomainError &Error,
			std::stop_token Stop={}
		);

		ConnectedOceanDomainSample Evaluate(UnitDirection Direction) const;
		ConnectedOceanDomainSample EvaluateInSource(UnitDirection Direction) const;
		bool ContainsOcean(UnitDirection Direction) const;
		UnitDirection OceanAnchor() const;
		const ConnectedOceanDomainStatistics &Statistics() const;
		std::uint64_t DefinitionHash() const;
	};
}
