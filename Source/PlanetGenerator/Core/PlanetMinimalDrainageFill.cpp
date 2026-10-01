#include "PlanetMinimalDrainageFill.h"
#include "PlanetSphericalDrainageBaseline.h"
#include "PlanetSphericalMesh.h"
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
		struct FillLink {
			std::uint32_t Neighbor = 0;
			double LengthMeters = 0;
		};
	}

	/**
	 * 惑星半径・海面・最低勾配が有限な事を確認する
	 * @return 閉球面上の標高を安全に構成出来る場合に真
	 */
	bool MinimalDrainageFillSettings::IsValid() const {
		// 有限な半径・海面・傾斜の妥当性を返す
		return std::isfinite(RadiusMeters) && RadiusMeters >= 1E3 && std::isfinite(SeaLevelMeters) &&
		std::isfinite(RadiusMeters + SeaLevelMeters) && RadiusMeters + SeaLevelMeters > 0 && std::isfinite(MinimumLandSlope) &&
		MinimumLandSlope > 0 && !(MinimumLandSlope > 1);
	}

	/**
	 * 元の標高を下界にして，海へ向かう辺毎に最低勾配を持つ最小包絡を解く
	 * @param Directions 閉じた球面網の方向
	 * @param Triangles 共有辺と外向き面
	 * @param OriginalHeightsMeters 保存する海底と，盛土の下界になる陸
	 * @param Settings 半径・球状海面・最低勾配
	 * @param Error 不正入力又は構成不能の理由
	 * @param Stop 中断要求
	 * @return 大規模な盛土の適否判定前の有限網の条件付け結果
	 */
	std::optional<MinimalDrainageFill> MinimalDrainageFill::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> OriginalHeightsMeters,
		const MinimalDrainageFillSettings &Settings,
		MinimalDrainageFillError &Error,
		const std::stop_token Stop
	) {
		Error = MinimalDrainageFillError::None;
		const auto Fail = [&](const MinimalDrainageFillError Reason) -> std::optional<MinimalDrainageFill> {
			Error = Reason;
			// 部分的に準備した面を公開しない返戻
			return {};
		};
		// 中断済みの構成を開始しない返戻
		if(Stop.stop_requested()) return Fail(MinimalDrainageFillError::Cancelled);
		const bool InvalidInput = !Settings.IsValid() || Directions.empty() || Directions.size() != OriginalHeightsMeters.size() ||
		Directions.size() > SphericalDrainageBaselineMaximumVertices;
		// 不正な球面又は値の配列を使わない返戻
		if(InvalidInput) return Fail(MinimalDrainageFillError::InvalidInput);
		const bool Closed = IsClosedSphericalSamplingMesh(Directions, Triangles, Stop, SphericalDrainageBaselineMaximumVertices);
		// 共有辺の壊れた網と中断を区別する返戻
		if(!Closed) return Fail(Stop.stop_requested() ? MinimalDrainageFillError::Cancelled : MinimalDrainageFillError::InvalidMesh);
		MinimalDrainageFill Result;
		Result.Stats.Vertices = static_cast<std::uint32_t>(Directions.size());
		Result.HeightsMeters.assign(OriginalHeightsMeters.begin(), OriginalHeightsMeters.end());
		Result.DownstreamVertices.resize(Directions.size());
		std::vector<bool> IsOcean(Directions.size());
		std::uint32_t FirstOcean = std::numeric_limits<std::uint32_t>::max();
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) {
			const double Height = OriginalHeightsMeters[Index];
			if(!std::isfinite(Height) || !(Settings.RadiusMeters + Height > 0) || !std::isfinite(Settings.RadiusMeters + Height)) {
				// 有限性又は半径方向の条件を満たさない元地形の返戻
				return Fail(MinimalDrainageFillError::InvalidInput);
			}
			IsOcean[Index] = Height < Settings.SeaLevelMeters;
			Result.DownstreamVertices[Index] = Index;
			if(IsOcean[Index]) {
				++Result.Stats.OceanVertices;
				if(FirstOcean == std::numeric_limits<std::uint32_t>::max()) FirstOcean = Index;
			}
		}
		// 大洋の無い標高の返戻
		if(!Result.Stats.OceanVertices) return Fail(MinimalDrainageFillError::NoOcean);
		// 陸の無い標高の返戻
		if(Result.Stats.OceanVertices == Result.Stats.Vertices) return Fail(MinimalDrainageFillError::NoLand);
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
		std::vector<FillLink> Neighbors(Offsets.back());
		for(const std::array<std::uint32_t, 2> &Edge : Edges) {
			const std::uint32_t A = Edge[0], B = Edge[1];
			const Vec3 Difference = Directions[A].Vector() - Directions[B].Vector();
			const double Arc = 2 * std::asin(std::min(1.0, 0.5 * Difference.Length())), Length = Settings.RadiusMeters * Arc;
			// 辺長を有限な正数に出来ない網の返戻
			if(!std::isfinite(Length) || !(Length > 0)) return Fail(MinimalDrainageFillError::NumericFailure);
			Neighbors[Cursors[A]++] = { B, Length };
			Neighbors[Cursors[B]++] = { A, Length };
		}
		std::vector<bool> Seen(Directions.size());
		std::vector<std::uint32_t> OceanQueue;
		OceanQueue.reserve(Result.Stats.OceanVertices);
		Seen[FirstOcean] = true;
		OceanQueue.push_back(FirstOcean);
		for(std::size_t Position = 0; Position < OceanQueue.size(); ++Position) {
			// 大洋連結探索の中断時の返戻
			if(!(Position & 255) && Stop.stop_requested()) return Fail(MinimalDrainageFillError::Cancelled);
			const std::uint32_t Current = OceanQueue[Position];
			for(std::uint32_t At = Offsets[Current]; At < Offsets[Current + 1]; ++At) {
				const std::uint32_t Next = Neighbors[At].Neighbor;
				if(Seen[Next] || !IsOcean[Next]) continue;
				Seen[Next] = true;
				OceanQueue.push_back(Next);
			}
		}
		// 孤立した水没域を閉鎖水域として残さない返戻
		if(OceanQueue.size() != Result.Stats.OceanVertices) return Fail(MinimalDrainageFillError::DisconnectedOcean);
		using Entry = std::pair<double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		std::vector<double> Best(Directions.size(), std::numeric_limits<double>::infinity()), ParentLengthMeters(Directions.size());
		std::vector<bool> Settled(Directions.size());
		for(std::uint32_t Index = 0; Index < Directions.size(); ++Index) if(IsOcean[Index]) {
			Best[Index] = Settings.SeaLevelMeters;
			Frontier.emplace(Settings.SeaLevelMeters, Index);
		}
		double PreviousLandHeight = Settings.SeaLevelMeters;
		Result.Stats.MinimumLandDropMeters = std::numeric_limits<double>::infinity();
		std::uint32_t SettledLand = 0;
		while(!Frontier.empty()) {
			// 全陸標高の構成中断時の返戻
			if(!(SettledLand & 255) && Stop.stop_requested()) return Fail(MinimalDrainageFillError::Cancelled);
			const Entry Current = Frontier.top();
			Frontier.pop();
			const std::uint32_t Index = Current.second;
			if(Settled[Index] || Current.first != Best[Index]) continue;
			Settled[Index] = true;
			if(!IsOcean[Index]) {
				const std::uint32_t Parent = Result.DownstreamVertices[Index];
				const double ParentHeight = IsOcean[Parent] ? Settings.SeaLevelMeters : Result.HeightsMeters[Parent];
				const double Height = std::max(
					{
						Current.first,
						ParentHeight + Settings.MinimumLandSlope * ParentLengthMeters[Index],
						std::nextafter(PreviousLandHeight, std::numeric_limits<double>::infinity())
					}
				);
				// 有限性又は単調性を失った計算の返戻
				if(!std::isfinite(Height) || !(Settings.RadiusMeters + Height > 0)) return Fail(MinimalDrainageFillError::NumericFailure);
				Result.HeightsMeters[Index] = Height;
				PreviousLandHeight = Height;
				++SettledLand;
				const double Drop = Height - ParentHeight;
				// 下流への厳密下降を維持出来ない場合の返戻
				if(!(Drop > 0)) return Fail(MinimalDrainageFillError::NumericFailure);
				Result.Stats.MinimumLandDropMeters = std::min(Result.Stats.MinimumLandDropMeters, Drop);
				const double Raise = Height - OriginalHeightsMeters[Index];
				if(Raise > 0) {
					if(Raise > 1E-6) ++Result.Stats.RaisedLandVertices;
					Result.Stats.MaximumRaiseMeters = std::max(Result.Stats.MaximumRaiseMeters, Raise);
					Result.Stats.SumRaiseMeters += Raise;
				}
			}
			const double StartHeight = IsOcean[Index] ? Settings.SeaLevelMeters : Result.HeightsMeters[Index];
			for(std::uint32_t At = Offsets[Index]; At < Offsets[Index + 1]; ++At) {
				const FillLink &Link = Neighbors[At];
				const std::uint32_t Next = Link.Neighbor;
				if(IsOcean[Next] || Settled[Next]) continue;
				const double Candidate = std::max(
					{
						OriginalHeightsMeters[Next],
						StartHeight + Settings.MinimumLandSlope * Link.LengthMeters,
						std::nextafter(StartHeight, std::numeric_limits<double>::infinity())
					}
				);
				// 次の陸標高を有限に評価出来ない場合の返戻
				if(!std::isfinite(Candidate)) return Fail(MinimalDrainageFillError::NumericFailure);
				if(Candidate >= Best[Next]) continue;
				Best[Next] = Candidate;
				Result.DownstreamVertices[Next] = Index;
				ParentLengthMeters[Next] = Link.LengthMeters;
				Frontier.emplace(Candidate, Next);
			}
		}
		// 有限性又は単調性を失った計算の返戻
		if(SettledLand != Result.Stats.Vertices - Result.Stats.OceanVertices) return Fail(MinimalDrainageFillError::NumericFailure);
		// 大洋連結と全陸下流を備えた基準面の返戻
		return Result;
	}

	/**
	 * 最小盛土後の標高を取得する
	 * @return 海底と未変更の陸を元値に保つ標高
	 */
	std::span<const double> MinimalDrainageFill::GetHeightsMeters() const {
		// 不変な標高列の返戻
		return HeightsMeters;
	}

	/**
	 * 構成済みの下流を取得する
	 * @return 各陸節点の厳密に低い大洋方向の隣接地点
	 */
	std::span<const std::uint32_t> MinimalDrainageFill::GetDownstreamVertices() const {
		// 不変な下流列の返戻
		return DownstreamVertices;
	}

	/**
	 * 修正量と網の規模を取得する
	 * @return 盛土量，排水辺，網の規模
	 */
	const MinimalDrainageFillStatistics &MinimalDrainageFill::GetStatistics() const {
		// 不変な修正統計の返戻
		return Stats;
	}
}
