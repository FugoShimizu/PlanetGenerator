#include "PlanetOceanDistanceTerrain.h"
#include "PlanetSphericalMesh.h"
#include "PlanetSphericalDrainageBaseline.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		struct OceanDistanceLink {
			std::uint32_t Neighbor = 0;
			double WeightedLengthMeters = 0;
		};
	}

	/**
	 * 惑星と隆起の物理上限が有限である事を確かめる
	 * @return 半径・海面・相対起伏が安全な場合に真
	 */
	bool OceanDistanceTerrainSettings::IsValid() const {
		// 有界な全球距離から標高を作れる設定の返戻
		return std::isfinite(RadiusMeters) && RadiusMeters >= 1E3 && std::isfinite(SeaLevelMeters) &&
		std::isfinite(MaximumReliefMeters) && MaximumReliefMeters > 0 && MaximumReliefMeters <= 0.2 * RadiusMeters &&
		std::isfinite(RadiusMeters + SeaLevelMeters) && RadiusMeters + SeaLevelMeters > 0;
	}

	/**
	 * 連結大洋から正の辺費用を伝播させ，陸上節点の厳密な下降先を同時に構成する
	 * @param Directions 閉球面上の共有方向
	 * @param Triangles 外向きの共有面
	 * @param OriginalHeightsMeters 海陸分類と維持する海底標高
	 * @param TravelCosts 各節点の正の移動費用密度
	 * @param Settings 半径・球状海面・起伏の最大値
	 * @param Error 拒否理由
	 * @param Stop 中断要求
	 * @return 厳密下降の陸と一つの大洋を持つ不変な標高
	 */
	std::optional<OceanDistanceTerrain> OceanDistanceTerrain::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const std::span<const double> TravelCosts,
		const OceanDistanceTerrainSettings &Settings,
		OceanDistanceTerrainError &Error,
		const std::stop_token Stop
	) {
		Error = OceanDistanceTerrainError::None;
		const auto Fail = [&](const OceanDistanceTerrainError Reason) -> std::optional<OceanDistanceTerrain> {
			Error = Reason;
			// 部分的に準備した地形を公開しない返戻
			return {};
		};
		// 中断要求を不正入力と区別した返戻
		if(Stop.stop_requested()) return Fail(OceanDistanceTerrainError::Cancelled);
		const bool InvalidInput =
		!Settings.IsValid() || Directions.size() != OriginalHeightsMeters.size() || Directions.size() != TravelCosts.size() ||
		Directions.empty() || Directions.size() > SphericalDrainageBaselineMaximumVertices;
		// 球面・高さ・費用の不整合を使わない返戻
		if(InvalidInput) return Fail(OceanDistanceTerrainError::InvalidInput);
		const bool IsClosed = IsClosedSphericalSamplingMesh(Directions, Triangles, Stop, SphericalDrainageBaselineMaximumVertices);
		const OceanDistanceTerrainError MeshError =
		Stop.stop_requested() ? OceanDistanceTerrainError::Cancelled : OceanDistanceTerrainError::InvalidMesh;
		// 開いた網又は中断を区別する返戻
		if(!IsClosed) return Fail(MeshError);
		OceanDistanceTerrain Result;
		Result.Stats.Vertices = static_cast<std::uint32_t>(Directions.size());
		Result.HeightsMeters.assign(OriginalHeightsMeters.begin(), OriginalHeightsMeters.end());
		Result.DownstreamVertices.resize(Directions.size());
		std::vector<bool> IsOcean(Directions.size());
		std::uint32_t FirstOcean = std::numeric_limits<std::uint32_t>::max();
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			const double Height = OriginalHeightsMeters[Index], Cost = TravelCosts[Index];
			const bool Valid = std::isfinite(Height) && std::isfinite(Cost) && Cost > 0 && std::isfinite(Settings.RadiusMeters + Height) &&
			Settings.RadiusMeters + Height > 0;
			// 原標高と辺費用の非有限又は非物理値の返戻
			if(!Valid) return Fail(OceanDistanceTerrainError::InvalidInput);
			IsOcean[Index] = Height < Settings.SeaLevelMeters;
			Result.DownstreamVertices[Index] = Index;
			if(IsOcean[Index]) {
				++Result.Stats.OceanVertices;
				if(FirstOcean == std::numeric_limits<std::uint32_t>::max()) FirstOcean = Index;
			} else ++Result.Stats.LandVertices;
		}
		// 大洋の無い入力の返戻
		if(!Result.Stats.OceanVertices) return Fail(OceanDistanceTerrainError::NoOcean);
		// 陸の無い入力の返戻
		if(!Result.Stats.LandVertices) return Fail(OceanDistanceTerrainError::NoLand);
		std::vector<std::array<std::uint32_t, 2>> Edges;
		Edges.reserve(Triangles.size() * 3);
		for(const std::array<std::uint32_t, 3> &Face : Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Face[Side], B = Face[(Side + 1) % 3];
			Edges.push_back({ std::min(A, B), std::max(A, B) });
		}
		std::sort(Edges.begin(), Edges.end());
		Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
		Result.Stats.Edges = static_cast<std::uint32_t>(Edges.size());
		std::vector<std::uint32_t> Offsets(Directions.size() + 1);
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			++Offsets[Edge[0] + 1];
			++Offsets[Edge[1] + 1];
		}
		for(std::size_t Index = 1; Index < Offsets.size(); ++Index) Offsets[Index] += Offsets[Index - 1];
		std::vector<std::uint32_t> Cursors = Offsets;
		std::vector<OceanDistanceLink> Neighbors(Offsets.back());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			const std::uint32_t A = Edge[0], B = Edge[1];
			const Vec3 Difference = Directions[A].Vector() - Directions[B].Vector();
			const double Arc = 2 * std::asin(std::min(1.0, 0.5 * Difference.Length()));
			const double Cost = 0.5 * (TravelCosts[A] + TravelCosts[B]) * Settings.RadiusMeters * Arc;
			// 接続辺が非正又は非有限の費用を持つ場合の返戻
			if(!std::isfinite(Cost) || Cost <= 0) return Fail(OceanDistanceTerrainError::NumericFailure);
			Neighbors[Cursors[A]++] = { B, Cost };
			Neighbors[Cursors[B]++] = { A, Cost };
		}
		std::vector<bool> Visited(Directions.size());
		std::vector<std::uint32_t> OceanQueue;
		OceanQueue.reserve(Result.Stats.OceanVertices);
		Visited[FirstOcean] = true;
		OceanQueue.push_back(FirstOcean);
		for(std::size_t Position = 0; Position < OceanQueue.size(); ++Position) {
			// 海域の接続検査を中断した場合の返戻
			if(!(Position & 255) && Stop.stop_requested()) return Fail(OceanDistanceTerrainError::Cancelled);
			const std::uint32_t Current = OceanQueue[Position];
			for(std::size_t j = Offsets[Current]; j < Offsets[Current + 1]; ++j) {
				const std::uint32_t Next = Neighbors[j].Neighbor;
				if(!IsOcean[Next] || Visited[Next]) continue;
				Visited[Next] = true;
				OceanQueue.push_back(Next);
			}
		}
		// 異なる湖を同じ球状海面の大洋と見なさない返戻
		if(OceanQueue.size() != Result.Stats.OceanVertices) return Fail(OceanDistanceTerrainError::DisconnectedOcean);
		using FrontierEntry = std::pair<double, std::uint32_t>;
		std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, std::greater<FrontierEntry>> Frontier;
		std::vector<double> Distances(Directions.size(), std::numeric_limits<double>::infinity());
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) if(IsOcean[Index]) {
			Distances[Index] = 0;
			Frontier.emplace(0, Index);
		}
		std::vector<std::uint32_t> LandOrder;
		LandOrder.reserve(Result.Stats.LandVertices);
		while(!Frontier.empty()) {
			// 正の距離を伝播する途中で中断した場合の返戻
			if(!(LandOrder.size() & 255) && Stop.stop_requested()) return Fail(OceanDistanceTerrainError::Cancelled);
			const FrontierEntry Current = Frontier.top();
			Frontier.pop();
			if(Current.first != Distances[Current.second]) continue;
			if(!IsOcean[Current.second]) LandOrder.push_back(Current.second);
			for(std::size_t j = Offsets[Current.second]; j < Offsets[Current.second + 1]; ++j) {
				const OceanDistanceLink &Edge = Neighbors[j];
				if(IsOcean[Edge.Neighbor]) continue;
				const double Raw = Current.first + Edge.WeightedLengthMeters;
				const double Candidate = std::max(Raw, std::nextafter(Current.first, std::numeric_limits<double>::infinity()));
				// 距離を有限かつ厳密に増加させられない場合の返戻
				if(!std::isfinite(Candidate)) return Fail(OceanDistanceTerrainError::NumericFailure);
				if(Candidate >= Distances[Edge.Neighbor]) continue;
				Distances[Edge.Neighbor] = Candidate;
				Result.DownstreamVertices[Edge.Neighbor] = Current.second;
				Frontier.emplace(Candidate, Edge.Neighbor);
			}
		}
		// 閉球面上の全陸地点へ大洋から到達出来ない場合の返戻
		if(LandOrder.size() != Result.Stats.LandVertices) return Fail(OceanDistanceTerrainError::NumericFailure);
		const double MaximumDistance = Distances[LandOrder.back()];
		// 正の最高距離を標高尺度へ変換出来ない場合の返戻
		if(!std::isfinite(MaximumDistance) || MaximumDistance <= 0) return Fail(OceanDistanceTerrainError::NumericFailure);
		Result.Stats.MaximumWeightedDistanceMeters = MaximumDistance;
		const double Scale = Settings.MaximumReliefMeters / MaximumDistance;
		double PreviousHeight = Settings.SeaLevelMeters;
		Result.Stats.MinimumLandDropMeters = std::numeric_limits<double>::infinity();
		for(const std::uint32_t Index : LandOrder) {
			const double Requested = Settings.SeaLevelMeters + Scale * Distances[Index];
			const double Strict = std::nextafter(PreviousHeight, std::numeric_limits<double>::infinity());
			const double Height = std::max(Requested, Strict);
			// 浮動小数の順位が標高へ移せない場合の返戻
			if(!std::isfinite(Height) || !(Settings.RadiusMeters + Height > 0)) return Fail(OceanDistanceTerrainError::NumericFailure);
			Result.HeightsMeters[Index] = Height;
			PreviousHeight = Height;
			const std::uint32_t Parent = Result.DownstreamVertices[Index];
			const double Drop = Height - Result.HeightsMeters[Parent];
			// 大洋根へ至る辺が厳密に下降しない場合の返戻
			if(!(Drop > 0)) return Fail(OceanDistanceTerrainError::NumericFailure);
			Result.Stats.MinimumLandDropMeters = std::min(Result.Stats.MinimumLandDropMeters, Drop);
		}
		Result.Stats.MaximumLandHeightMeters = PreviousHeight;
		// 大洋と全陸の順位を共に確定した不変な高さ場の返戻
		return Result;
	}

	/**
	 * 海底を保持し，陸の重み付き大洋距離から作った標高を取得する
	 * @return 閉球面網の全節点の最終標高
	 */
	std::span<const double> OceanDistanceTerrain::GetHeightsMeters() const {
		// 準備済みの不変標高の返戻
		return HeightsMeters;
	}

	/**
	 * 各陸節点の厳密に低い下流を取得する
	 * @return 大洋根は自分自身，其の他は低い隣接節点
	 */
	std::span<const std::uint32_t> OceanDistanceTerrain::GetDownstreamVertices() const {
		// 準備済みの不変下流の返戻
		return DownstreamVertices;
	}

	/**
	 * 海域・流路と距離の準備規模を取得する
	 * @return 最終地形の統計
	 */
	const OceanDistanceTerrainStatistics &OceanDistanceTerrain::GetStatistics() const {
		// 不変な統計の返戻
		return Stats;
	}
}
