#include "PlanetSphericalDrainageBaseline.h"
#include "PlanetSphericalMesh.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	/**
	 * 共有球面網の各乾いた節点と面内を，同じ大洋へ下る区分線形の基準面にする
	 * @param Directions 閉球面の方向
	 * @param InTriangles 外向き且つ共有辺の整合する面
	 * @param InElevationsMeters 最終の基準標高として使う有限な節点値
	 * @param SeaLevel 一定高度の球状海面
	 * @param Error 正規入力の拒否理由
	 * @param Stop 中断要求
	 * @return 全大洋・陸の下降条件を満たす不変な面，其の他は空
	 */
	std::optional<SphericalDrainageBaseline> SphericalDrainageBaseline::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> InTriangles,
		const std::span<const double> InElevationsMeters,
		const double SeaLevel,
		SphericalDrainageBaselineError &Error,
		const std::stop_token Stop
	) {
		Error = SphericalDrainageBaselineError::None;
		const auto Fail = [&](const SphericalDrainageBaselineError Reason) -> std::optional<SphericalDrainageBaseline> {
			Error = Reason;
			// 未完成の基準面を公開しない返戻
			return {};
		};
		// 中断を入力の欠陥として報告しない返戻
		if(Stop.stop_requested()) return Fail(SphericalDrainageBaselineError::Cancelled);
		const bool InvalidInput = !std::isfinite(SeaLevel) || Directions.size() != InElevationsMeters.size() ||
		Directions.size() > SphericalDrainageBaselineMaximumVertices || Directions.empty();
		// 標高と球面網の対応が無い入力の返戻
		if(InvalidInput) return Fail(SphericalDrainageBaselineError::InvalidInput);
		const bool IsClosed = IsClosedSphericalSamplingMesh(Directions, InTriangles, Stop, SphericalDrainageBaselineMaximumVertices);
		const SphericalDrainageBaselineError MeshError =
		Stop.stop_requested() ? SphericalDrainageBaselineError::Cancelled : SphericalDrainageBaselineError::InvalidMesh;
		// 共有辺又は閉包が壊れた網を，中断と区別して返戻
		if(!IsClosed) return Fail(MeshError);
		SphericalDrainageBaseline Result;
		Result.SeaLevelMeters = SeaLevel;
		Result.Triangles.assign(InTriangles.begin(), InTriangles.end());
		Result.ElevationsMeters.assign(InElevationsMeters.begin(), InElevationsMeters.end());
		Result.DownstreamVertices.resize(Directions.size());
		Result.Stats.Vertices = static_cast<std::uint32_t>(Directions.size());
		Result.Stats.Triangles = static_cast<std::uint32_t>(InTriangles.size());
		std::uint32_t FirstOcean = std::numeric_limits<std::uint32_t>::max();
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			const double Height = Result.ElevationsMeters[Index];
			// 非有限な節点値を順位付け又は大洋判定へ渡さない返戻
			if(!std::isfinite(Height)) return Fail(SphericalDrainageBaselineError::InvalidInput);
			if(Height < SeaLevel) {
				++Result.Stats.OceanVertices;
				if(FirstOcean == std::numeric_limits<std::uint32_t>::max()) FirstOcean = Index;
			} else ++Result.Stats.NonOceanVertices;
		}
		// 海域の無い地形を大洋へ下る面として受理しない返戻
		if(!Result.Stats.OceanVertices) return Fail(SphericalDrainageBaselineError::NoOcean);
		// 陸の無い球面を要求する基準面として受理しない返戻
		if(!Result.Stats.NonOceanVertices) return Fail(SphericalDrainageBaselineError::NoLand);
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(3 * InTriangles.size());
		for(const std::array<std::uint32_t, 3> &Face : InTriangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Edges.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		Result.Stats.Edges = static_cast<std::uint32_t>(Edges.size());
		std::vector<std::uint32_t> Offsets(Directions.size() + 1);
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			const double A = Result.ElevationsMeters[Edge[0]], B = Result.ElevationsMeters[Edge[1]];
			// 平坦な乾いた辺の途中に厳密下降の経路を仮定しない返戻
			if(A >= SeaLevel && B >= SeaLevel && A == B) return Fail(SphericalDrainageBaselineError::LevelLandEdge);
			++Offsets[Edge[0] + 1];
			++Offsets[Edge[1] + 1];
		}
		for(std::size_t Index = 1; Index < Offsets.size(); ++Index) Offsets[Index] += Offsets[Index - 1];
		std::vector<std::uint32_t> Cursors = Offsets, Neighbors(Offsets.back());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			Neighbors[Cursors[Edge[0]]++] = Edge[1];
			Neighbors[Cursors[Edge[1]]++] = Edge[0];
		}
		std::vector<bool> Visited(Directions.size());
		std::vector<std::uint32_t> Queue;
		Queue.reserve(Result.Stats.OceanVertices);
		Visited[FirstOcean] = true;
		Queue.push_back(FirstOcean);
		for(std::size_t Position = 0; Position < Queue.size(); ++Position) {
			// 海域の連結探索で中断された結果の返戻
			if(Stop.stop_requested()) return Fail(SphericalDrainageBaselineError::Cancelled);
			const std::uint32_t Current = Queue[Position];
			for(std::size_t j = Offsets[Current]; j < Offsets[Current + 1]; ++j) {
				const std::uint32_t Next = Neighbors[j];
				if(Visited[Next] || Result.ElevationsMeters[Next] >= SeaLevel) continue;
				Visited[Next] = true;
				Queue.push_back(Next);
			}
		}
		// 負の面が別の湖に属する場合の返戻
		if(Queue.size() != Result.Stats.OceanVertices) return Fail(SphericalDrainageBaselineError::DisconnectedOcean);
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			const double Height = Result.ElevationsMeters[Index];
			Result.DownstreamVertices[Index] = Index;
			if(Height < SeaLevel) continue;
			double Lower = Height;
			for(std::size_t j = Offsets[Index]; j < Offsets[Index + 1]; ++j) {
				const std::uint32_t Neighbor = Neighbors[j];
				const double Candidate = Result.ElevationsMeters[Neighbor];
				if(Candidate >= Lower) continue;
				Lower = Candidate;
				Result.DownstreamVertices[Index] = Neighbor;
			}
			// 最終の標高で下る辺が無い陸上停留点の返戻
			if(Result.DownstreamVertices[Index] == Index) return Fail(SphericalDrainageBaselineError::LandSink);
		}
		// 厳密に低い節点を有限回辿れば連結した大洋へ至る基準面の返戻
		return Result;
	}

	/**
	 * 球面の一つの面を，中心投影の非負な重みで線形評価する
	 * @param Face 検証済みの面番号
	 * @param Weights 合計１の非負な同次重み
	 * @return 面内の連続な基準標高，不正な面・重みでは空
	 */
	std::optional<double> SphericalDrainageBaseline::EvaluateFace(
		const std::size_t Face,
		const std::array<double, 3> Weights
	) const {
		// 定義域外の面に対する返戻
		if(Face >= Triangles.size()) return {};
		double Sum = 0;
		for(const double Weight : Weights) {
			// 非有限又は負の面外重みの返戻
			if(!std::isfinite(Weight) || Weight < 0) return {};
			Sum += Weight;
		}
		// 丸めでない大きな不正規化を補正して見掛け上受理しない返戻
		if(!std::isfinite(Sum) || std::abs(Sum - 1) > 1E-12) return {};
		const std::array<std::uint32_t, 3> &Corners = Triangles[Face];
		double Height = 0;
		for(unsigned Corner = 0; Corner < 3; ++Corner) Height += Weights[Corner] * ElevationsMeters[Corners[Corner]];
		// 有限な範囲に有る基準面の返戻
		return std::isfinite(Height) ? std::optional<double>{ Height / Sum } : std::nullopt;
	}

	/**
	 * 準備済み基準面の節点毎の下流を取得する
	 * @return 各節点の厳密に低い隣接点，海底では自分自身
	 */
	std::span<const std::uint32_t> SphericalDrainageBaseline::GetDownstreamVertices() const {
		// 不変な下流番号の返戻
		return DownstreamVertices;
	}

	/**
	 * 全球の検証規模を取得する
	 * @return 検証した全球節点・面・海域の個数
	 */
	const SphericalDrainageBaselineStatistics &SphericalDrainageBaseline::GetStatistics() const {
		// 不変な検証統計の返戻
		return Stats;
	}
}
