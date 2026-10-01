#pragma once

#include "PlanetGeology.h"
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	class PlanetOrogenicRidges;
	class PlanetMesoRelief;

	// 生成手順の識別版
	inline constexpr std::uint32_t OceanConnectivityVersion = 4;
	inline constexpr std::uint32_t PredrainageMesoReliefVersion = 2;
	inline constexpr std::uint32_t PredrainageLandReliefVersion = 1;
	// 細部起伏の海岸接続と，水没符号を保存する証明に共通の物理高さ
	inline constexpr double PredrainageLandReliefStartMeters = 2;
	inline constexpr double PredrainageLandReliefFullMeters = 8;

	// 既定の生成定義を保ち，細部起伏の順序を明示的に比較する
	enum class PredrainageReliefMode : std::uint8_t { MesoOnly, MesoAndLand };
	/** 海岸と排水を同じ二次補間へ渡す場合だけ，元の排水網を接続対象とする */
	enum class OceanSamplingMode : std::uint8_t { Fine, SharedDrainage };

	struct OceanCorridor {
		// 海峡中心線の球面幾何
		Vec3 Start;
		Vec3 End;
		Vec3 Tangent;
		Vec3 Center;
		double ArcAngle = 0;
		// 内外の有限支持幅
		double InnerChordSquared = 0;
		double OuterChordSquared = 0;
		// 索引の保守境界
		Vec3 BoundsMin;
		Vec3 BoundsMax;
	};

	struct OceanConnectivityStatistics {
		// 実際に検査した網と接続成分
		std::uint32_t GraphNodes = 0;
		// 呼出側の元の網と，勾配候補から追加した実水没点を区別する
		std::uint32_t InputGraphNodes = 0;
		std::uint32_t SubcellCriticalCandidates = 0;
		std::uint32_t SubcellOceanNodes = 0;
		std::uint64_t SubcellEvaluations = 0;
		std::uint32_t ComponentsBefore = 0;
		std::uint32_t ComponentsAfter = 0;
		std::uint32_t CorridorCount = 0;
		std::uint32_t IndexNodeCount = 0;
		// 端点の水没だけでは通過出来なかった辺と，中間評価の費用
		std::uint32_t BlockedWetEdgesBefore = 0;
		std::uint32_t BlockedWetEdgesAfter = 0;
		std::uint64_t PassageEvaluations = 0;
		// 面積・独立推定と開削統計
		double GraphOceanAreaFraction = 0;
		double IndependentOceanFractionEstimate = 0;
		double MaximumSampleLoweringMeters = 0;
		double CorridorLengthMeters = 0;
		// 物理幅と中心線の最低水深
		double InnerHalfWidthMeters = 0;
		double OuterHalfWidthMeters = 0;
		double MinimumDepthMeters = 0;
	};

	// 初期の大洋接続の準備
	// 網上の証拠を最終連続地形の証明とは扱わず，侵食後も拘束を再検証する
	class OceanConnectivityField {
	private:

		struct IndexNode {
			// 空間索引の境界と葉又は子の番号
			Vec3 Min;
			Vec3 Max;
			std::uint32_t Begin = 0;
			std::uint32_t Count = 0;
			std::uint32_t Left = 0;
			std::uint32_t Right = 0;
		};

		// 元地形の共有定義と生成順
		std::shared_ptr<const GeologyField> Geology;
		std::shared_ptr<const PlanetOrogenicRidges> Orogenic;
		std::shared_ptr<const PlanetMesoRelief> Meso;
		bool PredrainageLandRelief = false;
		bool HasFineOceanGraph = false;
		std::uint64_t AdaptiveSamplingHash = 0;
		// 海峡の不変な幾何と照会索引
		OceanConnectivityStatistics Stats;
		// 最新の接続検査に実際に含めた水没節点，元の標本網を再構成して推測しない
		std::vector<UnitDirection> OceanAnchors;
		std::vector<OceanCorridor> Channels;
		std::vector<std::uint32_t> Order;
		std::vector<IndexNode> Index;
		// 海峡底と連続接続の標高幅
		double FloorMeters = 0;
		double TransitionMeters = 0;

		static std::optional<OceanConnectivityField> CreateImpl(
			std::shared_ptr<const GeologyField> Base,
			std::shared_ptr<const PlanetOrogenicRidges> Ridges,
			std::shared_ptr<const PlanetMesoRelief> Meso,
			const std::stop_token Stop,
			const bool IncludeLandRelief=false,
			const OceanSamplingMode Sampling=OceanSamplingMode::Fine
		);

		template<bool Derivatives> TerrainSample EvaluateSourceImpl(const UnitDirection Direction) const;
		template<bool Derivatives> TerrainSample EvaluateImpl(const UnitDirection Direction) const;

		void BuildIndex();

		static bool ConnectOnMesh(
			OceanConnectivityField &Field,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::stop_token Stop
		);

		static bool ConnectPreparedMesh(
			OceanConnectivityField &Field,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::stop_token Stop
		);

	public:

		static std::optional<OceanConnectivityField> Create(
			std::shared_ptr<const GeologyField> Base,
			const std::stop_token Stop= {}
		);

		static std::optional<OceanConnectivityField> CreateWithOrogeny(
			std::shared_ptr<const GeologyField> Base,
			std::shared_ptr<const PlanetOrogenicRidges> Ridges,
			const std::stop_token Stop= {}
		);

		static std::optional<OceanConnectivityField> CreateWithPredrainageRelief(
			std::shared_ptr<const GeologyField> Base,
			std::shared_ptr<const PlanetOrogenicRidges> Ridges,
			const std::stop_token Stop= {},
			const PredrainageReliefMode Mode=PredrainageReliefMode::MesoOnly,
			const OceanSamplingMode Sampling=OceanSamplingMode::Fine
		);

		std::optional<OceanConnectivityField> WithSamplingMesh(
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::stop_token Stop= {}
		) const;

		std::uint64_t DefinitionHash() const;
		bool HasPredrainageMesoRelief() const;
		bool HasPredrainageLandRelief() const;
		TerrainSample Evaluate(const UnitDirection Direction) const;
		double EvaluateElevation(const UnitDirection Direction) const;
		const GeologyField &Base() const;
		const OceanConnectivityStatistics &Statistics() const;
		std::span<const UnitDirection> ConnectedOceanAnchors() const;
		std::span<const OceanCorridor> Corridors() const;
	};
}
