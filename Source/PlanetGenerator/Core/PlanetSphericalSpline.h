#pragma once

#include "PlanetCore.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	struct SphericalSplineStatistics {
		std::uint32_t Vertices = 0;
		std::uint32_t Triangles = 0;
		std::uint32_t Pieces = 0;
		std::size_t ResidentBytes = 0;
	};

	/**
	 * 球面 Powell–Sabin 二次補間，既定の零接線と指定接線の比較経路
	 * 排水先を定める物ではなく，節点の高低関係を連続面へ渡す比較用の基盤
	 */
	class SphericalSplineField {
	private:
		struct SplitEdge {
			std::uint32_t A = 0;
			std::uint32_t B = 0;
			std::uint32_t Face = 0;
			unsigned Side = 0;
			bool IsForward = false;
		};

		struct Piece {
			std::array<Vec3, 3> Dual;
			std::array<double, 3> Quadratic;
			double ReferenceMeters = 0;
		};

		std::uint8_t Level = 0;
		bool HasVertexGradients = false;
		std::vector<std::array<Vec3, 3>> Bounds;
		std::vector<Piece> Pieces;
		std::vector<std::array<double, 2>> VertexJets;
		std::vector<std::array<Vec3, 3>> FaceBounds;
		std::vector<std::uint32_t> FaceOffsets;
		std::vector<std::uint32_t> FaceIndices;
		SphericalSplineStatistics Stats;

		static std::optional<SphericalSplineField> CreateImpl(
			const std::uint8_t Level,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::span<const double> ElevationsMeters,
			const std::span<const Vec3> Gradients,
			const std::stop_token Stop
		);

		static std::optional<std::array<Vec3, 3>> DualBasis(const std::array<Vec3, 3> &Corners);
		static std::size_t SelectTriangle(const std::span<const std::array<Vec3, 3>> Bounds, const Vec3 Point);

		std::size_t LocateBase(const Vec3 Point) const;
		TerrainSample SamplePiece(const std::size_t Index, const UnitDirection Direction) const;

		SphericalSplineField() = default;

	public:
		/** 指定階層の各三角形の内側だけを分割した，閉じた球面網を受取る */
		static std::optional<SphericalSplineField> CreateOnMesh(
			const std::uint8_t Level,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::span<const double> ElevationsMeters,
			const std::stop_token Stop={}
		);

		/** 有限な勾配の接線成分を節点で補間する．単調性や排水は別途拘束する必要が有る */
		static std::optional<SphericalSplineField> CreateOnMeshWithGradients(
			const std::uint8_t Level,
			const std::span<const UnitDirection> Directions,
			const std::span<const std::array<std::uint32_t, 3>> Triangles,
			const std::span<const double> ElevationsMeters,
			const std::span<const Vec3> Gradients,
			const std::stop_token Stop={}
		);

		static std::optional<SphericalSplineField> Create(
			const std::uint8_t Level,
			const std::span<const double> ElevationsMeters,
			const std::stop_token Stop={}
		);

		TerrainSample Evaluate(const UnitDirection Direction) const;

		/** 接続検査用の指定二次式，区画外でも同じ式を延長して返す */
		std::optional<TerrainSample> EvaluatePiece(const std::size_t Index, const UnitDirection Direction) const;
		std::optional<std::array<Vec3, 3>> GetPieceDomain(const std::size_t Index) const;
		const SphericalSplineStatistics &GetStatistics() const;
	};
}
