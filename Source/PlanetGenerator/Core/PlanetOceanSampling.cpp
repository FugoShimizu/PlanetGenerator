#include "PlanetOcean.h"
#include "PlanetSphericalMesh.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 新たに標本化した水没域も，同じ連続海峡の生成対象へ含める
	 * @param Directions 排水前に確定した元地形の節点
	 * @param Triangles 同じ節点番号を使う閉じた局所三角形
	 * @param Stop 中断要求
	 * @return 実際の網で全海域が接続した不変場，不正又は中断では空
	 */
	std::optional<OceanConnectivityField> OceanConnectivityField::WithSamplingMesh(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop
	) const {
		// 未準備の地質と不正な網を海峡探索へ渡さない返戻
		if(!Geology || !IsClosedSphericalSamplingMesh(Directions, Triangles, Stop)) return {};
		OceanConnectivityField Result = *this;
		// 既存海峡を維持し，追加した地点の接続を実際に再計算する
		if(!ConnectOnMesh(Result, Directions, Triangles, Stop) || Result.Stats.ComponentsAfter != 1) return {};
		std::uint64_t Hash = UINT64_C(0XAD0CE00000000001);
		const auto Append = [&Hash](const std::uint64_t Value) -> void {
			Hash = (Hash ^ Value) * UINT64_C(0X100000001B3);
			// 終了
			return;
		};
		const auto AppendDirection = [&Append](const Vec3 Direction) -> void {
			Append(std::bit_cast<std::uint64_t>(Direction.X));
			Append(std::bit_cast<std::uint64_t>(Direction.Y));
			Append(std::bit_cast<std::uint64_t>(Direction.Z));
			// 終了
			return;
		};
		Append(Directions.size());
		for(const UnitDirection Direction : Directions) AppendDirection(Direction.Vector());
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) for(const std::uint32_t Vertex : Triangle) Append(Vertex);
		// 同じ最終網でも，過去の追加網から保持した海峡が異なる場合を区別する
		Append(Result.Channels.size());
		for(const OceanCorridor &Channel : Result.Channels) {
			AppendDirection(Channel.Start);
			AppendDirection(Channel.End);
			Append(std::bit_cast<std::uint64_t>(Channel.InnerChordSquared));
			Append(std::bit_cast<std::uint64_t>(Channel.OuterChordSquared));
		}
		Result.AdaptiveSamplingHash = Hash ? Hash : 1;
		// 追加した網と実際の海峡を識別する場の返戻
		return Result;
	}
}
