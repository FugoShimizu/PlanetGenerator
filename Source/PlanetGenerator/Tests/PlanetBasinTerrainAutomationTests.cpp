#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetDrainageRefinement.h"
#include "Core/PlanetDrainageSampling.h"
#include "Core/PlanetRiverTerrain.h"
#include "Core/PlanetLandRelief.h"
#include "Data/BasinStationarySeed42.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <span>
#include <stop_token>
#include <unordered_map>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	struct BasinGridSurvey {
		double MaximumSpillMeters = 0;
		double MaximumSlope = 0;
		PlanetCore::Vec3 MaximumSlopeDirection {};
		std::uint32_t RaisedSites = 0;
		std::uint32_t MaximumSpillIndex = 0;
		std::array<std::uint32_t, 3> LargeSpills {};
		std::uint32_t VisitedSites = 0;
		std::uint32_t InvalidSamples = 0;
		std::vector<std::uint32_t> LocalMinima;
		std::uint32_t AddedRouteNodes = 0;
		std::uint32_t AddedRouteEdges = 0;
		std::uint64_t RouteSamples = 0;
		std::uint64_t ObstructedRouteSamples = 0;
		double MaximumRouteObstructionMeters = 0;
		double MaximumEndpointJoinMeters = 0;
		double MaximumHeadDeficitMeters = 0;
		std::uint32_t MaximumHeadDeficitIndex = 0;
		std::vector<std::uint32_t> HeadDeficitSites;
		PlanetCore::Vec3 MaximumHeadDeficitDirection {};
		std::size_t EdgeInteriorSites = 0;
		std::size_t TriangleInteriorLinks = 0;
		std::size_t JoinedRiverSites = 0;
		std::size_t RejectedRiverJoins = 0;
		std::size_t RiverSegments = 0;
		double MaximumRiverJoinMeters = 0;
	};

	struct BasinSurveyMesh {
		std::vector<PlanetCore::UnitDirection> Directions;
		std::vector<std::array<std::uint32_t, 2>> ParentEdges;
		std::vector<std::array<std::uint32_t, 3>> Triangles;
	};

	struct BasinChannelDistance {
		double Meters = std::numeric_limits<double>::infinity();
		double Fraction = 0;
		std::size_t Reach = 0;
	};

	struct BasinRefinementCandidate {
		PlanetCore::DrainageRefinementMesh Mesh;
		PlanetCore::RiverTerrainField Terrain;
		std::size_t PromotedEdges = 0;
	};

	struct BasinSamplingCandidate {
		PlanetCore::DrainageSamplingMesh Mesh;
		PlanetCore::RiverTerrainField Terrain;
		std::size_t PromotedEdges = 0;
	};

	enum class BasinDescentEnd : unsigned { Ocean, River, Stalled, Limit, Invalid };

	struct BasinDescentTrace {
		BasinDescentEnd End = BasinDescentEnd::Limit;
		PlanetCore::Vec3 Direction {};
		double StartHeight = 0;
		double EndHeight = 0;
		double EndSlope = 0;
		double DistanceMeters = 0;
		unsigned Steps = 0;
		std::uint64_t Evaluations = 0;
	};

	struct BasinSamplingPreparation {
		const TCHAR *Phase = TEXT("mesh");
		std::size_t Nodes = 0;
		std::size_t Reaches = 0;
		std::size_t BankConstraints = 0;
	};

	/**
	 * 最悪地点と生成済み河道の距離を，全河道との球面距離で独立に調べる
	 * 計算量：O(n)，n は河道数
	 * @param Field 検査対象の最終地形
	 * @param Direction 調べる地点
	 * @return 最近傍の河道と位置，河道が無い場合の距離は正の無限大
	 */
	BasinChannelDistance FindBasinChannel(const PlanetCore::RiverTerrainField &Field, const PlanetCore::UnitDirection Direction) {
		BasinChannelDistance Result;
		const PlanetCore::Vec3 P = Direction.Vector();
		for(const PlanetCore::RiverTerrainChannel &Channel : Field.Channels()) {
			const double Angle = std::clamp(std::atan2(P.Dot(Channel.Tangent), P.Dot(Channel.Start)), 0.0, Channel.ArcAngle);
			const PlanetCore::Vec3 Closest = Channel.Start * std::cos(Angle) + Channel.Tangent * std::sin(Angle);
			const double Distance = std::atan2(P.Cross(Closest).Length(), P.Dot(Closest)) * Field.Base().Settings().RadiusMeters;
			if(Distance >= Result.Meters) continue;
			Result = { Distance, Angle / Channel.ArcAngle, Channel.ReachIndex };
		}
		// 索引や水面の判定に依存しない最近傍の返戻
		return Result;
	}

	/**
	 * 格子の辺に拘束せず解析勾配を下り，各採用区間の８標本が下降する事を確認する
	 * 最大4096歩，各歩32回迄の刻み幅縮小に制限し，停滞を出口と誤認しない
	 * @param Start 検査開始地点
	 * @param Radius 惑星の基準半径
	 * @param Sea 接続済み大洋の基準海面
	 * @param Sample 地形と解析勾配
	 * @param Height 同じ地形の値だけの照会
	 * @param ReachedRiver 実際の水没河床へ接続した場合だけ真を返す照会
	 * @return 標本化した下降の終端，停滞や上限は連続な閉鎖盆地の証明ではない
	 */
	BasinDescentTrace TraceBasinDescent(
		const PlanetCore::UnitDirection Start,
		const double Radius,
		const double Sea,
		const std::function<PlanetCore::TerrainSample(PlanetCore::UnitDirection)> &Sample,
		const std::function<double(PlanetCore::UnitDirection)> &Height,
		const std::function<bool(PlanetCore::UnitDirection)> &ReachedRiver
	) {
		BasinDescentTrace Result;
		PlanetCore::UnitDirection Point = Start;
		while(true) {
			const PlanetCore::TerrainSample Terrain = Sample(Point);
			++Result.Evaluations;
			Result.Direction = Point.Vector();
			Result.EndHeight = Terrain.ElevationMeters;
			if(!Result.Steps) Result.StartHeight = Terrain.ElevationMeters;
			const PlanetCore::Vec3 Gradient =
			Terrain.TangentialGradientPerRadian - Point.Vector() * Terrain.TangentialGradientPerRadian.Dot(Point.Vector());
			Result.EndSlope = Gradient.Length() / Radius;
			if(!std::isfinite(Result.EndHeight) || !std::isfinite(Result.EndSlope)) {
				Result.End = BasinDescentEnd::Invalid;
				// 非有限な値を下降と誤認しない返戻
				return Result;
			}
			if(Result.EndHeight < Sea) {
				Result.End = BasinDescentEnd::Ocean;
				// 接続済み大洋の地形へ達した標本経路の返戻
				return Result;
			}
			if(ReachedRiver(Point)) {
				Result.End = BasinDescentEnd::River;
				// 実河床への接続を別途確認した経路の返戻
				return Result;
			}
			// 制限を使い切った経路を成功としない返戻
			if(Result.Steps == 4096) return Result;
			const std::optional<PlanetCore::UnitDirection> Down = PlanetCore::UnitDirection::FromVector(Gradient * -1);
			if(!Down) {
				Result.End = BasinDescentEnd::Stalled;
				// 勾配０を未実装の地下流出で補わない返戻
				return Result;
			}
			bool Accepted = false;
			for(unsigned Trial = 0; Trial < 32; ++Trial) {
				const double Step = std::ldexp(std::min(100.0, Radius * 0.05), -static_cast<int>(Trial));
				double PreviousHeight = Result.EndHeight;
				PlanetCore::UnitDirection Candidate = Point;
				bool Descending = true;
				for(unsigned Substep = 1; Substep < 9; ++Substep) {
					const double Angle = Step * Substep / (8 * Radius);
					Candidate = *PlanetCore::UnitDirection::FromVector(Point.Vector() * std::cos(Angle) + Down->Vector() * std::sin(Angle));
					const double Elevation = Height(Candidate);
					++Result.Evaluations;
					if(!std::isfinite(Elevation)) {
						Result.End = BasinDescentEnd::Invalid;
						// 途中の非有限な値を飛越えない返戻
						return Result;
					}
					if(Elevation > PreviousHeight) {
						Descending = false;
						break;
					}
					PreviousHeight = Elevation;
				}
				if(!Descending || PreviousHeight >= Result.EndHeight || PreviousHeight > Result.EndHeight - 0.1 * Step * Result.EndSlope) {
					continue;
				}
				Point = Candidate;
				++Result.Steps;
				Result.DistanceMeters += Step;
				Accepted = true;
				break;
			}
			if(!Accepted) {
				Result.End = BasinDescentEnd::Stalled;
				// 刻み幅を縮めても下降を確認出来ない場合の返戻
				return Result;
			}
		}
	}

	/**
	 * 同じ実地形と河川内部への接続条件で，任意の開始点から下降を追跡する
	 * @param Field 生成済みの最終地形
	 * @param Start 実際の開始方向
	 * @return 水没河床又は大洋への到達，停滞又は上限の診断
	 */
	BasinDescentTrace TraceBasinTerrain(const PlanetCore::RiverTerrainField &Field, const PlanetCore::UnitDirection Start) {
		using PlanetCore::UnitDirection;
		// 地形・水位・横断の全照会を同じ場へ結び付けた追跡の返戻
		return TraceBasinDescent(
			Start,
			Field.Base().Settings().RadiusMeters,
			Field.Drainage().Settings().SeaLevelMeters,
			[&](const UnitDirection Point) -> PlanetCore::TerrainSample {
				// 実際の最終面と同じ解析勾配の返戻
				return Field.Evaluate(Point);
			},
			[&](const UnitDirection Point) -> double {
				// 段階合成を省略しない同じ標高の返戻
				return Field.EvaluateElevation(Point);
			},
			[&](const UnitDirection Point) -> bool {
				const PlanetCore::RiverWaterHeights Water = Field.EvaluateWater(Point);
				// 実際に水没して居ない地点を出口としない返戻
				if(Water.WaterSupport <= 0.0 || Water.WaterElevationMeters <= Water.TerrainElevationMeters) return false;
				const BasinChannelDistance Nearest = FindBasinChannel(Field, Point);
				const std::span<const PlanetCore::RiverTerrainChannel> Channels = Field.Channels();
				const std::span<const PlanetCore::RiverTerrainChannel>::iterator Channel = std::find_if(
					Channels.begin(),
					Channels.end(),
					[&](const PlanetCore::RiverTerrainChannel &Candidate) -> bool {
						// 全探索で求めた同じ河道かどうかの返戻
						return Candidate.ReachIndex == Nearest.Reach;
					}
				);
				// 広い谷の支持だけでは河川へ接続したとしない返戻
				if(Channel == Channels.end() || Nearest.Meters > Channel->BedHalfWidthMeters) return false;
				const std::optional<PlanetCore::RiverProfileSample> Profile = Field.Drainage().SampleReach(Nearest.Reach, Nearest.Fraction);
				// 公開縦断を照会出来ない場合の返戻
				if(!Profile) return false;
				for(unsigned Step = 1; Step < 9; ++Step) {
					const double Fraction = Step / 8.0;
					const UnitDirection Across = *UnitDirection::FromVector(Point.Vector() * (1 - Fraction) + Profile->Direction * Fraction);
					const double Height = Field.EvaluateElevation(Across);
					// 横断の途中に水位を超える障壁が有る場合の返戻
					if(!std::isfinite(Height) || Height > Water.WaterElevationMeters) return false;
				}
				// 実河川の水没する中心へ接続した結果の返戻
				return true;
			}
		);
	}

	/**
	 * 同じ地点の合成段階を一度の照会から取り出す
	 * @param Field 最終地形の定義
	 * @param Point 実際の照会方向
	 * @return 元地形・盛土・谷・細部・最終の標高
	 */
	std::array<double, 5> BasinStageHeights(const PlanetCore::RiverTerrainField &Field, const PlanetCore::UnitDirection Point) {
		const PlanetCore::RiverElevationStages Stages = Field.EvaluateElevationStages(Point);
		// 同じ順序で比較する段階標高の返戻
		return {
			Stages.BeforeBasinMeters,
			Stages.AfterBasinMeters,
			Stages.AfterValleyMeters,
			Stages.AfterFineReliefMeters,
			Stages.FinalMeters
		};
	}

	/**
	 * 停滞点での各合成段階の勾配と周囲の高さを独立な地点評価で記録する
	 * @param Test 診断を保持する UE 検査
	 * @param Label 流出口追加の前後を区別する名前
	 * @param Field 比較する最終地形
	 * @param Point 下降が止まった実地点
	 */
	void InspectBasinStages(
		FAutomationTestBase &Test,
		const TCHAR *Label,
		const PlanetCore::RiverTerrainField &Field,
		const PlanetCore::UnitDirection Point
	) {
		using PlanetCore::UnitDirection;
		using PlanetCore::Vec3;
		const Vec3 P = Point.Vector(), Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		const Vec3 Across = UnitDirection::FromVector(P.Cross(Axis))->Vector(), Along = P.Cross(Across);
		const double Radius = Field.Base().Settings().RadiusMeters;
		const std::array<double, 5> Heights = BasinStageHeights(Field, Point);
		std::array<Vec3, 5> Gradients {}, Errors {};
		for(const Vec3 Tangent : { Across, Along }) {
			std::array<std::array<double, 5>, 3> Difference {};
			for(unsigned Level = 0; Level < 3; ++Level) {
				const double Step = std::ldexp(0.04, -static_cast<int>(Level)), Angle = Step / Radius;
				const UnitDirection Low = *UnitDirection::FromVector(P * std::cos(Angle) - Tangent * std::sin(Angle));
				const UnitDirection High = *UnitDirection::FromVector(P * std::cos(Angle) + Tangent * std::sin(Angle));
				const std::array<double, 5> A = BasinStageHeights(Field, Low), B = BasinStageHeights(Field, High);
				for(unsigned Stage = 0; Stage < 5; ++Stage) Difference[Level][Stage] = (B[Stage] - A[Stage]) / (2 * Step);
			}
			for(unsigned Stage = 0; Stage < 5; ++Stage) {
				const double Coarse = (4 * Difference[1][Stage] - Difference[0][Stage]) / 3;
				const double Fine = (4 * Difference[2][Stage] - Difference[1][Stage]) / 3;
				Gradients[Stage] = Gradients[Stage] + Tangent * Fine;
				Errors[Stage] = Errors[Stage] + Tangent * (Fine - Coarse);
			}
		}
		for(unsigned Stage = 0; Stage < 5; ++Stage) {
			Test.TestTrue(TEXT("全合成段階の標高と差分勾配が有限"), std::isfinite(Heights[Stage]) && std::isfinite(Gradients[Stage].Length()));
			Test.AddInfo(
				FString::Printf(
					TEXT("Basin stage gradient: label=%s stage=%u height=%.12gm slope=%.12g gx=%.12g gy=%.12g gz=%.12g convergence=%.12g."),
					Label,
					Stage,
					Heights[Stage],
					Gradients[Stage].Length(),
					Gradients[Stage].X,
					Gradients[Stage].Y,
					Gradients[Stage].Z,
					Errors[Stage].Length()
				)
			);
		}
		for(const double Distance : { 1.0, 10.0, 100.0 }) {
			std::array<double, 5> Minimum;
			Minimum.fill(std::numeric_limits<double>::infinity());
			for(unsigned Side = 0; Side < 16; ++Side) {
				const double Azimuth = Side * std::acos(-1.0) / 8, Angle = Distance / Radius;
				const Vec3 Tangent = Across * std::cos(Azimuth) + Along * std::sin(Azimuth);
				const UnitDirection Ring = *UnitDirection::FromVector(P * std::cos(Angle) + Tangent * std::sin(Angle));
				const std::array<double, 5> Values = BasinStageHeights(Field, Ring);
				for(unsigned Stage = 0; Stage < 5; ++Stage) Minimum[Stage] = std::min(Minimum[Stage], Values[Stage] - Heights[Stage]);
			}
			Test.AddInfo(
				FString::Printf(
					TEXT("Basin stage ring: label=%s radius=%.9fm original=%.12gm filled=%.12gm valley=%.12gm fine=%.12gm final=%.12gm."),
					Label,
					Distance,
					Minimum[0],
					Minimum[1],
					Minimum[2],
					Minimum[3],
					Minimum[4]
				)
			);
		}
		// 全段階の独立照会を記録し終えた後の返戻
		return;
	}

	/**
	 * 指定した球面網を，共有辺の中点を一度だけ作って検査専用に細分化する
	 * 計算量：O(n) の期待値，記憶量：O(n)，n は元の三角形数
	 * @param Directions 元の球面頂点
	 * @param Triangles 元の共有三角形
	 * @return 一段細かい検査網，中点の準備失敗では空
	 */
	std::optional<BasinSurveyMesh> SubdivideBasinSurveyMesh(
		const std::span<const PlanetCore::UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles
	) {
		BasinSurveyMesh Fine;
		Fine.Directions.assign(Directions.begin(), Directions.end());
		Fine.ParentEdges.resize(Directions.size());
		Fine.Triangles.reserve(Triangles.size() * 4);
		std::unordered_map<std::uint64_t, std::uint32_t> Midpoints;
		Midpoints.reserve(Triangles.size() * 3 / 2);
		// 同じ無向辺へ一つだけの球面中点を割当てる
		const auto Midpoint = [&](const std::uint32_t A, const std::uint32_t B) -> std::uint32_t {
			const std::uint64_t Key = static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
			const std::unordered_map<std::uint64_t, std::uint32_t>::iterator Found = Midpoints.find(Key);
			// 既に共有して居る中点番号の返戻
			if(Found != Midpoints.end()) return Found->second;
			const std::uint32_t Index = static_cast<std::uint32_t>(Fine.Directions.size());
			const std::optional<PlanetCore::UnitDirection> Direction =
			PlanetCore::UnitDirection::FromVector(Fine.Directions[A].Vector() + Fine.Directions[B].Vector());
			// 対向する不正な辺を中点と誤認しない返戻
			if(!Direction) return std::numeric_limits<std::uint32_t>::max();
			Fine.Directions.push_back(*Direction);
			Fine.ParentEdges.push_back({ std::min(A, B), std::max(A, B) });
			Midpoints.emplace(Key, Index);
			// 一度だけ作った共有中点の返戻
			return Index;
		};
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) {
			const std::uint32_t A = Triangle[0], B = Triangle[1], C = Triangle[2];
			const std::uint32_t AB = Midpoint(A, B), BC = Midpoint(B, C), CA = Midpoint(C, A);
			const bool InvalidMidpoint =
			AB == std::numeric_limits<std::uint32_t>::max() || BC == std::numeric_limits<std::uint32_t>::max() ||
			CA == std::numeric_limits<std::uint32_t>::max();
			// 共有中点を作れない場合の返戻
			if(InvalidMidpoint) return {};
			Fine.Triangles.push_back({ A, AB, CA });
			Fine.Triangles.push_back({ AB, B, BC });
			Fine.Triangles.push_back({ CA, BC, C });
			Fine.Triangles.push_back({ AB, BC, CA });
		}
		// 元の描画・生成器の上限を変えない検査専用の網の返戻
		return Fine;
	}

	/**
	 * 保持する流出口を全て同じ局所網へ写し，必要な支流と盛土・河床を再構成する
	 * 計算量：排水・盛土の準備に加え O(s log e + n)，s は入口数，e は分割辺数，n は排水地点数
	 * @param Ocean 比較に共通の連続な海陸場
	 * @param Fine 元網を一度細分化した検査網と親辺
	 * @param CoarseCount 元網の地点数
	 * @param Sources 保持又は追加する入口の検査網番号
	 * @return 同じ地形評価を持つ局所候補，不正な対応・循環・準備失敗では空
	 */
	std::optional<BasinRefinementCandidate> PrepareBasinRefinement(
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> &Ocean,
		const BasinSurveyMesh &Fine,
		const std::size_t CoarseCount,
		const std::span<const std::uint32_t> Sources
	) {
		std::vector<std::array<std::uint32_t, 2>> Edges;
		for(const std::uint32_t Index : Sources) {
			// 元網にも追加地点にも存在しない入口を使用しない返戻
			if(Index >= Fine.Directions.size() || Index >= Fine.ParentEdges.size()) return {};
			if(Index >= CoarseCount) Edges.push_back(Fine.ParentEdges[Index]);
		}
		std::optional<PlanetCore::DrainageRefinementMesh> Mesh = PlanetCore::BuildDrainageRefinement(Edges);
		// 位相を共有出来ない局所網を使用しない返戻
		if(!Mesh) return {};
		std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::CreateWithRefinedEdges(*Ocean, Edges);
		// 準備に失敗した排水網を参照しない返戻
		if(!Network) return {};
		std::vector<std::uint8_t> Selected(Network->Nodes().size()), Needed(Network->Nodes().size());
		for(const PlanetCore::RiverReach &Reach : Network->Reaches()) Selected[Reach.Upstream] = 1;
		for(const std::uint32_t FineIndex : Sources) {
			std::uint32_t Index = FineIndex;
			if(FineIndex >= CoarseCount) {
				const std::vector<std::array<std::uint32_t, 2>>::const_iterator Edge =
				std::lower_bound(Mesh->SplitEdges.cbegin(), Mesh->SplitEdges.cend(), Fine.ParentEdges[FineIndex]);
				// 対応する共有中点を見付けられない場合の返戻
				if(Edge == Mesh->SplitEdges.cend() || *Edge != Fine.ParentEdges[FineIndex]) return {};
				Index = static_cast<std::uint32_t>(CoarseCount + (Edge - Mesh->SplitEdges.cbegin()));
			}
			for(std::size_t Steps = 0;; ++Steps) {
				// 範囲外又は循環を含む支流を使用しない返戻
				if(Index >= Network->Nodes().size() || Steps >= Network->Nodes().size()) return {};
				if(Network->Nodes()[Index].Ocean || Selected[Index] || Needed[Index]) break;
				Needed[Index] = 1;
				Index = Network->Nodes()[Index].Downstream;
			}
		}
		std::vector<std::uint32_t> Promotion;
		for(std::uint32_t Index = 0; Index < Needed.size(); ++Index) if(Needed[Index]) Promotion.push_back(Index);
		Network = Network->WithPromotedTributaries(Promotion, *Ocean);
		// 海岸へ接続する実支流を準備出来ない場合の返戻
		if(!Network) return {};
		std::optional<PlanetCore::RiverTerrainField> Terrain =
		PlanetCore::RiverTerrainField::CreateWithBasins(Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(*Network));
		// 支流と盛土を同じ地形へ反映出来ない場合の返戻
		if(!Terrain) return {};
		// 検査網と実河床の対応を保持した候補の返戻
		return BasinRefinementCandidate{ std::move(*Mesh), std::move(*Terrain), Promotion.size() };
	}

	/**
	 * 検出地点を移動せず追加し，既存の入口を含む全ての流出口を実河床へ接続する
	 * @param Ocean 同じ比較に使う海陸場
	 * @param Samples 保持する入口の球面位置
	 * @param Progress 拒否段階と実際の準備件数
	 * @return 全入口を持つ場，位置・下流・地形の準備失敗では空
	 */
	std::optional<BasinSamplingCandidate> PrepareSampledBasin(
		const std::shared_ptr<const PlanetCore::OceanConnectivityField> &Ocean,
		const std::span<const PlanetCore::UnitDirection> Samples,
		BasinSamplingPreparation &Progress
	) {
		Progress = {};
		std::optional<PlanetCore::DrainageSamplingMesh> Mesh = PlanetCore::BuildDrainageSamplingMesh(Samples);
		// 任意地点の局所網を準備出来ない場合の返戻
		if(!Mesh) return {};
		Progress.Nodes = Mesh->Directions.size();
		Progress.Phase = TEXT("drainage");
		std::optional<PlanetCore::DrainageNetwork> Network = PlanetCore::DrainageNetwork::CreateWithSamplingSites(*Ocean, Samples);
		// 指定した位置を持つ排水を準備出来ない場合の返戻
		if(!Network) return {};
		std::vector<std::uint8_t> Selected(Network->Nodes().size()), Needed(Network->Nodes().size());
		for(const PlanetCore::RiverReach &Reach : Network->Reaches()) Selected[Reach.Upstream] = 1;
		for(const std::uint32_t Source : Mesh->SampleNodes) {
			// 網と水系の頂点番号が対応しない場合の返戻
			if(Source >= Network->Nodes().size()) return {};
			const PlanetCore::Vec3 Difference = Network->Nodes()[Source].Direction - Mesh->Directions[Source].Vector();
			// 網と水系の頂点位置が対応しない場合の返戻
			if(Difference.Length() > 1E-14) return {};
			std::uint32_t Index = Source;
			for(std::size_t Steps = 0;; ++Steps) {
				// 循環又は範囲外へ向かう流出口を採用しない返戻
				if(Index >= Network->Nodes().size() || Steps >= Network->Nodes().size()) return {};
				if(Network->Nodes()[Index].Ocean || Selected[Index] || Needed[Index]) break;
				Needed[Index] = 1;
				Index = Network->Nodes()[Index].Downstream;
			}
		}
		std::vector<std::uint32_t> Promotion;
		for(std::uint32_t Index = 0; Index < Needed.size(); ++Index) if(Needed[Index]) Promotion.push_back(Index);
		Progress.Phase = TEXT("tributaries");
		Network = Network->WithPromotedTributaries(Promotion, *Ocean);
		// 全入口を含む連続な河川縦断を作れない場合の返戻
		if(!Network) return {};
		Progress.Reaches = Network->Reaches().size();
		Progress.Phase = TEXT("balanced profiles");
		Network = Network->WithBalancedTerrainProfiles(
			[&Ocean](const PlanetCore::UnitDirection Point) -> double {
				// 一体化する元地形と同じ標高の返戻
				return Ocean->EvaluateElevation(Point);
			}
		);
		// 共有縦断の準備が失敗した場合の返戻
		if(!Network) return {};
		Progress.Phase = TEXT("constraint enumeration");
		const double Sea = Network->Settings().SeaLevelMeters, Radius = Network->Settings().RadiusMeters;
		for(std::size_t Reach = 0; Reach < Network->Reaches().size(); ++Reach) {
			const std::optional<PlanetCore::RiverCrossSectionWidths> Widths = Network->ChannelWidths(Reach);
			// 河岸の正規幅を取得出来ない場合の返戻
			if(!Widths) return {};
			const double Angle = Widths->InnerHalfWidthMeters / Radius;
			for(unsigned Knot = 0; Knot < 65; ++Knot) {
				const std::optional<PlanetCore::RiverProfileSample> Sample = Network->SampleReach(Reach, Knot / 64.0);
				// 河岸の縦断を取得出来ない場合の返戻
				if(!Sample) return {};
				const PlanetCore::Vec3 Across =
				PlanetCore::UnitDirection::FromVector(Sample->Direction.Cross(Sample->DownstreamTangent))->Vector();
				for(const double Side : { -1.0, 0.0, 1.0 }) {
					const PlanetCore::UnitDirection Point =
					*PlanetCore::UnitDirection::FromVector(Sample->Direction * std::cos(Side * Angle) + Across * std::sin(Side * Angle));
					if(Ocean->EvaluateElevation(Point) > Sea) ++Progress.BankConstraints;
				}
			}
		}
		Progress.Phase = TEXT("basin field");
		std::optional<PlanetCore::RiverTerrainField> Terrain =
		PlanetCore::RiverTerrainField::CreateWithBasins(Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(*Network));
		// 同じ標高関数へ流出口を反映出来ない場合の返戻
		if(!Terrain) return {};
		Progress.Phase = TEXT("ready");
		// 指定位置と生成した地形の対応を保持した返戻
		return BasinSamplingCandidate{ std::move(*Mesh), std::move(*Terrain), Promotion.size() };
	}

	/**
	 * 最終の地点評価を独立な細網へ標本化し，大洋迄の経路で必要な最高標高を測る
	 * 計算量：O(n log n)，記憶量：O(n)，n は固定三角網の地点数
	 * @param Mesh 河道の準備網より細かい球面網
	 * @param Field 同じ最終地形を返す場
	 * @param IncludeRiverRoutes 実河床を257標本で評価した通路も加える，未標本区間は保証しない
	 * @param SampleEdgeInteriors 元の辺を中点で置換え，途中の障壁を飛越える短絡を除く
	 * @param LinkTriangleInteriors 中点同士も接続し，一段細分化した三角網を完成する
	 * @param JoinRiverInteriors 水没する河道内標本を，実断面を検査して縦断の同じ位置へ接続する
	 * @return 網上の越流高と最大勾配，有限網が細い河道を見落とす可能性は別に残る
	 */
	BasinGridSurvey SurveyBasinGrid(
		const BasinSurveyMesh &Mesh,
		const PlanetCore::RiverTerrainField &Field,
		const bool IncludeRiverRoutes=false,
		const bool SampleEdgeInteriors=false,
		const bool LinkTriangleInteriors=false,
		const bool JoinRiverInteriors=false
	) {
		const double Sea = Field.Base().Settings().SeaLevelMeters, Radius = Field.Base().Settings().RadiusMeters;
		using Link = std::pair<std::uint32_t, double>;
		std::vector<PlanetCore::UnitDirection> AddedDirections;
		std::span<const PlanetCore::UnitDirection> Directions = Mesh.Directions;
		std::vector<std::vector<Link>> Neighbors(Mesh.Directions.size());
		std::size_t InteriorLinks = 0;
		if(SampleEdgeInteriors) {
			std::vector<std::array<std::uint32_t, 2>> Edges;
			Edges.reserve(Mesh.Triangles.size() * 3);
			for(const std::array<std::uint32_t, 3> &Triangle : Mesh.Triangles) for(unsigned Corner = 0; Corner < 3; ++Corner) {
				const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
				Edges.push_back({ std::min(A, B), std::max(A, B) });
			}
			std::sort(Edges.begin(), Edges.end());
			Edges.erase(std::unique(Edges.begin(), Edges.end()), Edges.end());
			AddedDirections.assign(Mesh.Directions.begin(), Mesh.Directions.end());
			AddedDirections.reserve(Mesh.Directions.size() + Edges.size());
			Neighbors.resize(Mesh.Directions.size() + Edges.size());
			for(const std::array<std::uint32_t, 2> &Edge : Edges) {
				const std::uint32_t Midpoint = static_cast<std::uint32_t>(AddedDirections.size());
				const std::optional<PlanetCore::UnitDirection> Direction =
				PlanetCore::UnitDirection::FromVector(Mesh.Directions[Edge[0]].Vector() + Mesh.Directions[Edge[1]].Vector());
				if(!Direction) {
					BasinGridSurvey Invalid;
					Invalid.InvalidSamples = 1;
					// 不正な中点を飛越えて経路を作らない返戻
					return Invalid;
				}
				AddedDirections.push_back(*Direction);
				for(const std::uint32_t Endpoint : Edge) {
					Neighbors[Endpoint].emplace_back(Midpoint, -std::numeric_limits<double>::infinity());
					Neighbors[Midpoint].emplace_back(Endpoint, -std::numeric_limits<double>::infinity());
				}
			}
			if(LinkTriangleInteriors) {
				// 中点は共有辺の昇順で追加済みであり，全ての面で同じ番号を使う
				const auto MidpointIndex = [&](const std::uint32_t A, const std::uint32_t B) -> std::uint32_t {
					const std::array<std::uint32_t, 2> Key { std::min(A, B), std::max(A, B) };
					const std::vector<std::array<std::uint32_t, 2>>::const_iterator Found = std::lower_bound(Edges.cbegin(), Edges.cend(), Key);
					// 元の全三角形から一度だけ列挙した共有辺の中点番号の返戻
					return static_cast<std::uint32_t>(Mesh.Directions.size() + (Found - Edges.cbegin()));
				};
				for(const std::array<std::uint32_t, 3> &Triangle : Mesh.Triangles) {
					const std::array<std::uint32_t, 3> Inside {
						MidpointIndex(Triangle[0], Triangle[1]),
						MidpointIndex(Triangle[1], Triangle[2]),
						MidpointIndex(Triangle[2], Triangle[0])
					};
					for(unsigned Corner = 0; Corner < 3; ++Corner) {
						const std::uint32_t A = Inside[Corner], B = Inside[(Corner + 1) % 3];
						Neighbors[A].emplace_back(B, -std::numeric_limits<double>::infinity());
						Neighbors[B].emplace_back(A, -std::numeric_limits<double>::infinity());
						++InteriorLinks;
					}
				}
			}
			Directions = AddedDirections;
		} else for(const std::array<std::uint32_t, 3> &Triangle : Mesh.Triangles) for(unsigned Corner = 0; Corner < 3; ++Corner) {
			const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
			Neighbors[A].emplace_back(B, -std::numeric_limits<double>::infinity());
			Neighbors[B].emplace_back(A, -std::numeric_limits<double>::infinity());
		}
		const std::size_t Count = Directions.size();
		std::vector<double> Height(Count), Spill(Count, std::numeric_limits<double>::infinity());
		std::vector<double> AvailableHeads(JoinRiverInteriors ? Count : 0);
		using Entry = std::pair<double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		BasinGridSurvey Result;
		Result.EdgeInteriorSites = Count - Mesh.Directions.size();
		Result.TriangleInteriorLinks = InteriorLinks;
		Result.MaximumHeadDeficitDirection = Directions.front().Vector();
		Result.MaximumSlopeDirection = Directions.front().Vector();
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			PlanetCore::TerrainSample Sample;
			if(JoinRiverInteriors) {
				const PlanetCore::RiverSurfaceSample Surface = Field.EvaluateSurface(Directions[Index]);
				Sample = Surface.Terrain;
				if(!std::isfinite(Surface.WaterElevationMeters) || !std::isfinite(Surface.WaterSupport)) {
					++Result.InvalidSamples;
					// 不正な水面を河川への接続に使わない返戻
					return Result;
				}
				AvailableHeads[Index] =
				Surface.WaterSupport > 0 ? std::max(Sample.ElevationMeters, Surface.WaterElevationMeters) : Sample.ElevationMeters;
			} else Sample = Field.Evaluate(Directions[Index]);
			if(!std::isfinite(Sample.ElevationMeters) || !std::isfinite(Sample.TangentialGradientPerRadian.Length())) {
				++Result.InvalidSamples;
				// 非有限な地形を排水成功と誤認しない返戻
				return Result;
			}
			Height[Index] = Sample.ElevationMeters;
			const double Slope = Sample.TangentialGradientPerRadian.Length() / Radius;
			if(Slope > Result.MaximumSlope) {
				Result.MaximumSlope = Slope;
				Result.MaximumSlopeDirection = Directions[Index].Vector();
			}
			if(Height[Index] < Sea) {
				Spill[Index] = Sea;
				Frontier.emplace(Sea, Index);
			}
		}
		if(IncludeRiverRoutes) {
			using Coordinate = std::pair<double, std::uint32_t>;
			std::vector<Coordinate> Sorted;
			for(std::uint32_t Index = 0; Index < Count; ++Index) Sorted.emplace_back(Directions[Index].Vector().X, Index);
			std::sort(Sorted.begin(), Sorted.end());
			constexpr double MatchTolerance = 64 * std::numeric_limits<double>::epsilon();
			const std::span<const PlanetCore::DrainageNode> Nodes = Field.Drainage().Nodes();
			std::vector<std::uint32_t> Mapping(Nodes.size());
			for(std::size_t Index = 0; Index < Nodes.size(); ++Index) {
				const PlanetCore::Vec3 Direction = Nodes[Index].Direction;
				std::uint32_t Match = std::numeric_limits<std::uint32_t>::max();
				double Distance = MatchTolerance;
				std::vector<Coordinate>::const_iterator Candidate =
				std::lower_bound(Sorted.cbegin(), Sorted.cend(), Coordinate{ Direction.X - MatchTolerance, 0 });
				for(; Candidate != Sorted.cend() && Candidate->first <= Direction.X + MatchTolerance; ++Candidate) {
					const double Difference = (Directions[Candidate->second].Vector() - Direction).Length();
					if(Difference >= Distance) continue;
					Distance = Difference;
					Match = Candidate->second;
				}
				if(Match == std::numeric_limits<std::uint32_t>::max()) {
					Match = static_cast<std::uint32_t>(Height.size());
					const std::optional<PlanetCore::UnitDirection> Site = PlanetCore::UnitDirection::FromVector(Direction);
					if(!Site) {
						++Result.InvalidSamples;
						// 不正な河道端点を追加しない返戻
						return Result;
					}
					const double Elevation = Field.EvaluateElevation(*Site);
					if(!std::isfinite(Elevation)) {
						++Result.InvalidSamples;
						// 評価出来ない河道端点を接続しない返戻
						return Result;
					}
					Height.push_back(Elevation);
					Spill.push_back(Elevation < Sea ? Sea : std::numeric_limits<double>::infinity());
					if(Elevation < Sea) Frontier.emplace(Sea, Match);
					Neighbors.emplace_back();
					++Result.AddedRouteNodes;
				} else Result.MaximumEndpointJoinMeters = std::max(Result.MaximumEndpointJoinMeters, Distance * Radius);
				Mapping[Index] = Match;
			}
			using RiverKnot = std::pair<double, std::uint32_t>;
			std::vector<std::vector<RiverKnot>> RiverKnots(JoinRiverInteriors ? Field.Drainage().Reaches().size() : 0);
			if(JoinRiverInteriors) {
				for(std::size_t Reach = 0; Reach < RiverKnots.size(); ++Reach) {
					const PlanetCore::RiverReach &Edge = Field.Drainage().Reaches()[Reach];
					if(Edge.Upstream >= Mapping.size() || Edge.Downstream >= Mapping.size()) {
						++Result.InvalidSamples;
						// 対応しない端点を持つ河川へ接続しない返戻
						return Result;
					}
					RiverKnots[Reach] = { { 0, Mapping[Edge.Upstream] }, { 1, Mapping[Edge.Downstream] } };
				}
				std::vector<std::uint32_t> Candidates;
				for(std::uint32_t Index = 0; Index < Count; ++Index) {
					if(Height[Index] < Sea || AvailableHeads[Index] <= Height[Index]) continue;
					const PlanetCore::Vec3 Position = Directions[Index].Vector();
					if(!Field.GatherChannelCandidates(Position, Position, Candidates)) {
						++Result.InvalidSamples;
						// 候補照会の失敗を河道無と誤認しない返戻
						return Result;
					}
					BasinChannelDistance Nearest;
					for(const std::uint32_t CandidateIndex : Candidates) {
						const PlanetCore::RiverTerrainChannel &Channel = Field.Channels()[CandidateIndex];
						const double Angle = std::clamp(std::atan2(Position.Dot(Channel.Tangent), Position.Dot(Channel.Start)), 0.0, Channel.ArcAngle);
						const PlanetCore::Vec3 Closest = Channel.Start * std::cos(Angle) + Channel.Tangent * std::sin(Angle);
						const double Distance = std::atan2(Position.Cross(Closest).Length(), Position.Dot(Closest)) * Radius;
						if(Distance >= Nearest.Meters || Distance > Channel.BedHalfWidthMeters) continue;
						Nearest = { Distance, Angle / Channel.ArcAngle, Channel.ReachIndex };
					}
					if(!std::isfinite(Nearest.Meters)) continue;
					const std::optional<PlanetCore::RiverProfileSample> Profile = Field.Drainage().SampleReach(Nearest.Reach, Nearest.Fraction);
					if(!Profile) {
						++Result.InvalidSamples;
						// 河川内部の位置を照会出来ない場合の返戻
						return Result;
					}
					double Barrier = Height[Index], CenterHeight = Height[Index];
					for(unsigned Step = 1; Step < 9; ++Step) {
						const double Fraction = Step / 8.0;
						const std::optional<PlanetCore::UnitDirection> Point =
						PlanetCore::UnitDirection::FromVector(Position * (1 - Fraction) + Profile->Direction * Fraction);
						if(!Point) {
							++Result.InvalidSamples;
							// 不正な横断経路を短絡に使わない返戻
							return Result;
						}
						CenterHeight = Field.EvaluateElevation(*Point);
						if(!std::isfinite(CenterHeight)) {
							++Result.InvalidSamples;
							// 非有限な横断地形を通路としない返戻
							return Result;
						}
						Barrier = std::max(Barrier, CenterHeight);
					}
					if(Barrier > AvailableHeads[Index]) {
						++Result.RejectedRiverJoins;
						continue;
					}
					const double Obstruction = CenterHeight - Profile->WaterMeters;
					Result.MaximumRouteObstructionMeters = std::max(Result.MaximumRouteObstructionMeters, Obstruction);
					if(Obstruction > 0) ++Result.ObstructedRouteSamples;
					++Result.RouteSamples;
					std::vector<RiverKnot> &Knots = RiverKnots[Nearest.Reach];
					std::uint32_t Center = Knots.front().second;
					if(Nearest.Fraction >= 1.0) Center = Knots[1].second;
					else if(Nearest.Fraction > 0) {
						Center = static_cast<std::uint32_t>(Height.size());
						Height.push_back(CenterHeight);
						Spill.push_back(CenterHeight < Sea ? Sea : std::numeric_limits<double>::infinity());
						if(CenterHeight < Sea) Frontier.emplace(Sea, Center);
						Neighbors.emplace_back();
						Knots.emplace_back(Nearest.Fraction, Center);
						++Result.AddedRouteNodes;
					}
					if(Index != Center) {
						Neighbors[Index].emplace_back(Center, Barrier);
						Neighbors[Center].emplace_back(Index, Barrier);
					}
					++Result.JoinedRiverSites;
					Result.MaximumRiverJoinMeters = std::max(Result.MaximumRiverJoinMeters, Nearest.Meters);
				}
			}
			for(std::size_t Reach = 0; Reach < Field.Drainage().Reaches().size(); ++Reach) {
				const PlanetCore::RiverReach &Edge = Field.Drainage().Reaches()[Reach];
				if(
					Edge.Upstream >= Mapping.size() || Edge.Downstream >= Mapping.size() ||
					(Edge.EndDirection - Nodes[Edge.Downstream].Direction).Length() > MatchTolerance
				) {
					++Result.InvalidSamples;
					// 実河口と排水番号の異なる辺を短絡させない返戻
					return Result;
				}
				double Barrier = -std::numeric_limits<double>::infinity();
				std::array<double, 257> RiverHeights {};
				for(unsigned Knot = 0; Knot < 257; ++Knot) {
					const std::optional<PlanetCore::RiverProfileSample> Sample = Field.Drainage().SampleReach(Reach, Knot / 256.0);
					if(!Sample) {
						++Result.InvalidSamples;
						// 縦断を照会出来ない辺を連結としない返戻
						return Result;
					}
					const PlanetCore::RiverWaterHeights Water = Field.EvaluateWater(*PlanetCore::UnitDirection::FromVector(Sample->Direction));
					if(!std::isfinite(Water.TerrainElevationMeters) || !std::isfinite(Sample->WaterMeters)) {
						++Result.InvalidSamples;
						// 非有限な実河床を通路としない返戻
						return Result;
					}
					Barrier = std::max(Barrier, Water.TerrainElevationMeters);
					RiverHeights[Knot] = Water.TerrainElevationMeters;
					const double Obstruction = Water.TerrainElevationMeters - Sample->WaterMeters;
					Result.MaximumRouteObstructionMeters = std::max(Result.MaximumRouteObstructionMeters, Obstruction);
					if(Obstruction > 0) ++Result.ObstructedRouteSamples;
					++Result.RouteSamples;
				}
				if(JoinRiverInteriors) {
					std::vector<RiverKnot> &Knots = RiverKnots[Reach];
					std::sort(Knots.begin(), Knots.end());
					for(std::size_t Segment = 1; Segment < Knots.size(); ++Segment) {
						const RiverKnot A = Knots[Segment - 1], B = Knots[Segment];
						double SegmentBarrier = std::max(Height[A.second], Height[B.second]);
						const unsigned First = static_cast<unsigned>(std::ceil(A.first * 256)), Last = static_cast<unsigned>(std::floor(B.first * 256));
						for(unsigned Knot = First; Knot <= Last; ++Knot) SegmentBarrier = std::max(SegmentBarrier, RiverHeights[Knot]);
						Neighbors[A.second].emplace_back(B.second, SegmentBarrier);
						Neighbors[B.second].emplace_back(A.second, SegmentBarrier);
						++Result.RiverSegments;
					}
				} else {
					Neighbors[Mapping[Edge.Upstream]].emplace_back(Mapping[Edge.Downstream], Barrier);
					Neighbors[Mapping[Edge.Downstream]].emplace_back(Mapping[Edge.Upstream], Barrier);
				}
				++Result.AddedRouteEdges;
			}
		}
		while(!Frontier.empty()) {
			const auto [Level, Index] = Frontier.top();
			Frontier.pop();
			if(Level != Spill[Index]) continue;
			++Result.VisitedSites;
			for(const Link &Edge : Neighbors[Index]) {
				const std::uint32_t Neighbor = Edge.first;
				const double Next = std::max({ Level, Height[Neighbor], Edge.second });
				if(Next >= Spill[Neighbor]) continue;
				Spill[Neighbor] = Next;
				Frontier.emplace(Next, Neighbor);
			}
		}
		for(std::uint32_t Index = 0; Index < Count; ++Index) if(Height[Index] >= Sea) {
			bool LocalMinimum = Height[Index] > Sea + 0.01;
			for(const Link &Edge : Neighbors[Index]) LocalMinimum &= Height[Edge.first] > Height[Index] + 0.01;
			if(LocalMinimum) Result.LocalMinima.push_back(Index);
			const double Gap = Spill[Index] - Height[Index];
			if(IncludeRiverRoutes) {
				const PlanetCore::RiverWaterHeights Water = JoinRiverInteriors ?
				PlanetCore::RiverWaterHeights{ Height[Index], AvailableHeads[Index], 1 } :
				Field.EvaluateWater(Directions[Index]);
				if(!std::isfinite(Water.WaterElevationMeters) || !std::isfinite(Water.WaterSupport)) {
					++Result.InvalidSamples;
					// 不正な水位を通水可能な水頭と誤認しない返戻
					return Result;
				}
				const double AvailableHead = Water.WaterSupport > 0 ? std::max(Height[Index], Water.WaterElevationMeters) : Height[Index];
				const double Deficit = std::max(0.0, Spill[Index] - AvailableHead);
				if(Deficit > Result.MaximumHeadDeficitMeters) {
					Result.MaximumHeadDeficitMeters = Deficit;
					Result.MaximumHeadDeficitIndex = Index;
					Result.MaximumHeadDeficitDirection = Directions[Index].Vector();
				}
				if(Deficit > 0.02) Result.HeadDeficitSites.push_back(Index);
			}
			if(Gap > Result.MaximumSpillMeters) {
				Result.MaximumSpillMeters = Gap;
				Result.MaximumSpillIndex = Index;
			}
			const std::array<double, 3> Thresholds { 1, 10, 50 };
			for(std::size_t Limit = 0; Limit < Thresholds.size(); ++Limit) if(Gap > Thresholds[Limit]) ++Result.LargeSpills[Limit];
			if(Gap > 0.02) ++Result.RaisedSites;
		}
		// 細い連続流路の未標本化と区別する，固定網上の検査値の返戻
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinTerrainContracts, "PlanetGenerator.BasinReconstruction.Terrain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 盛土・河床の一体評価と正規・拒否経路を検査し，独立な細網の閉鎖盆地指標を比較する
 * @param Parameters 未使用の UE 検査引数
 * @return 構成と微分の条件を満たす場合に真，閉鎖盆地指標は別途改善すべき計測値として残す
 */
bool FPlanetBasinTerrainContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::RiverTerrainField;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	// 解析解を持つ開放斜面と閉じた凹面で，追跡器自身の成功・停滞を区別する
	const auto NoRiver = [](const UnitDirection) -> bool {
		// 合成した解析地形には河川出口を追加しない返戻
		return false;
	};
	for(const bool Closed : { false, true }) {
		const auto Analytic = [Closed](const UnitDirection Point) -> PlanetCore::TerrainSample {
			const Vec3 P = Point.Vector(), Axis { 0, 0, 1 };
			PlanetCore::TerrainSample Result;
			Result.ElevationMeters = Closed ? 10 + 100 * (1 - P.Z) : 100 * P.Z;
			Result.TangentialGradientPerRadian = (Axis - P * P.Z) * (Closed ? -100 : 100);
			// 一つの既知の斜面又は凹面の返戻
			return Result;
		};
		const BasinDescentTrace Trace = TraceBasinDescent(
			*UnitDirection::FromVector({ 1, 0, 1 }),
			1000,
			0,
			Analytic,
			[&](const UnitDirection Point) -> double {
				// 同じ解析解の標高の返戻
				return Analytic(Point).ElevationMeters;
			},
			NoRiver
		);
		TestTrue(TEXT("開いた斜面では大洋へ達し，閉じた凹面を成功としない"), Trace.End == (Closed ? BasinDescentEnd::Stalled : BasinDescentEnd::Ocean));
		TestTrue(TEXT("採用する経路で実際に標高が下がる"), Trace.EndHeight < Trace.StartHeight && Trace.Steps > 0);
		if(Closed) TestTrue(TEXT("既知の凹面では実際の底へ収束"), std::abs(Trace.EndHeight - 10) < 1E-8);
	}
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Settings);
	// 元の地質の準備失敗時の返戻
	if(!TestTrue(TEXT("最終地形比較の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(*PreparedGeology);
	const std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 先行する山地の準備失敗時の返戻
	if(!TestTrue(TEXT("排水前の山地を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*PreparedRidges);
	const std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges);
	// 海域の準備失敗時の返戻
	if(!TestTrue(TEXT("排水前の山地を含む同じ海陸を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedOcean);
	const std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	// 排水の準備失敗時の返戻
	if(!TestTrue(TEXT("比較に共通の排水を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedDrainage);
	const std::optional<PlanetCore::DrainageNetwork> NoSplits = PlanetCore::DrainageNetwork::CreateWithRefinedEdges(*Ocean, {});
	// 細分化を指定しない正規経路が変わった場合の返戻
	if(!TestTrue(TEXT("空の細分化要求も既存の排水を生成"), NoSplits.has_value())) return false;
	// 番号対応を失った二つの網を比較しない返戻
	if(!TestEqual(TEXT("細分化なしの地点数が一致"), NoSplits->Nodes().size(), Drainage->Nodes().size())) return false;
	for(std::size_t Index = 0; Index < Drainage->Nodes().size(); ++Index) {
		const PlanetCore::DrainageNode &A = Drainage->Nodes()[Index], &B = NoSplits->Nodes()[Index];
		TestEqual(TEXT("細分化なしの標高が一致"), A.OriginalElevationMeters, B.OriginalElevationMeters);
		TestEqual(TEXT("細分化なしの水位が一致"), A.WaterMeters, B.WaterMeters);
		TestEqual(TEXT("細分化なしの流量面積が一致"), A.ContributingAreaSquareMeters, B.ContributingAreaSquareMeters);
		TestEqual(TEXT("細分化なしの下流番号が一致"), A.Downstream, B.Downstream);
		TestEqual(TEXT("細分化なしの出口番号が一致"), A.Outlet, B.Outlet);
	}
	const std::optional<RiverTerrainField> Baseline = RiverTerrainField::Create(Ocean, Drainage);
	const double Started = FPlatformTime::Seconds();
	const std::optional<RiverTerrainField> Candidate = RiverTerrainField::CreateWithBasins(Ocean, Drainage);
	const double PreparedMilliseconds = (FPlatformTime::Seconds() - Started) * 1000;
	// 同条件の二つの合成場を作れない場合の返戻
	if(!TestTrue(TEXT("盛土と開削を同じ最終地点評価へ結合"), Baseline.has_value() && Candidate.has_value())) return false;
	TestFalse(TEXT("再構成済みの河床を開削未統合として残さない"), Candidate->Drainage().RequiresBalancedTerrain());
	TestFalse(TEXT("未指定の基盤を拒否"), RiverTerrainField::CreateWithBasins({}, Drainage).has_value());
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(TEXT("局所排水の中断を拒否"), PlanetCore::DrainageNetwork::CreateWithRefinedEdges(*Ocean, {}, Cancel.get_token()).has_value());
	TestFalse(TEXT("中断を拒否"), RiverTerrainField::CreateWithBasins(Ocean, Drainage, Cancel.get_token()).has_value());
	double MaximumNormalError = 0, MaximumBedError = 0, MaximumWaterError = 0, MaximumDepthError = 0;
	for(std::size_t Reach = 0; Reach < Candidate->Drainage().Reaches().size(); ++Reach) for(unsigned Knot = 0; Knot < 65; ++Knot) {
		const std::optional<PlanetCore::RiverProfileSample> Sample = Candidate->Drainage().SampleReach(Reach, Knot / 64.0);
		// 公開した河道の照会に失敗した場合の返戻
		if(!TestTrue(TEXT("地形と共有する公開河道を照会"), Sample.has_value())) return false;
		const UnitDirection Direction = *UnitDirection::FromVector(Sample->Direction);
		const PlanetCore::RiverWaterHeights Surface = Candidate->EvaluateWater(Direction);
		MaximumBedError = std::max(MaximumBedError, std::abs(Surface.TerrainElevationMeters - Sample->BedMeters));
		MaximumWaterError = std::max(MaximumWaterError, std::abs(Surface.WaterElevationMeters - Sample->WaterMeters));
		MaximumDepthError = std::max(
			MaximumDepthError,
			Surface.WaterElevationMeters - Surface.TerrainElevationMeters - (Sample->WaterMeters - Sample->BedMeters)
		);
		if(Reach % 53 || Knot != 27) continue;
		const Vec3 P = Direction.Vector(), Along = Sample->DownstreamTangent;
		const PlanetCore::TerrainSample Analytical = Candidate->Evaluate(Direction);
		const auto Difference = [&](const double Step) -> double {
			const UnitDirection Low = *UnitDirection::FromVector(P * std::cos(Step) - Along * std::sin(Step));
			const UnitDirection High = *UnitDirection::FromVector(P * std::cos(Step) + Along * std::sin(Step));
			// 異なる位置で最終地形を取得した独立差分の返戻
			return (Candidate->EvaluateElevation(High) - Candidate->EvaluateElevation(Low)) / (2 * Step);
		};
		const double Independent = (4 * Difference(5E-9) - Difference(1E-8)) / 3;
		const double Slope = Analytical.TangentialGradientPerRadian.Dot(Along) / Settings.RadiusMeters;
		const double Error = std::abs(std::atan(Slope) - std::atan(Independent / Settings.RadiusMeters));
		MaximumNormalError = std::max(MaximumNormalError, Error);
		TestTrue(TEXT("盛土と開削を合成した最終法線が独立差分と一致"), Error <= 1E-4);
		TestEqual(TEXT("最終標高の値だけの経路も一致"), Candidate->EvaluateElevation(Direction), Analytical.ElevationMeters);
	}
	const std::optional<PlanetCore::SphereMesh> SurveyBase = PlanetCore::BuildUniformSphere(6);
	// 検査の元網を準備出来ない場合の返戻
	if(!TestTrue(TEXT("比較用の元網を準備"), SurveyBase.has_value())) return false;
	const std::optional<BasinSurveyMesh> Fine = SubdivideBasinSurveyMesh(SurveyBase->Directions, SurveyBase->Triangles);
	// 独立な細網の準備失敗時の返戻
	if(!TestTrue(TEXT("排水準備より細かい独立な検査網を準備"), Fine.has_value())) return false;
	TestEqual(TEXT("共有辺を一度だけ細分化した地点数"), Fine->Directions.size(), std::size_t{ 163842 });
	TestEqual(TEXT("独立な細網の閉じた三角形数"), Fine->Triangles.size(), std::size_t{ 327680 });
	const BasinGridSurvey Before = SurveyBasinGrid(*Fine, *Baseline), After = SurveyBasinGrid(*Fine, *Candidate);
	TestTrue(TEXT("元地形の全標本と勾配が有限"), !Before.InvalidSamples);
	TestTrue(TEXT("盛土後の全標本と勾配が有限"), !After.InvalidSamples);
	TestEqual(TEXT("比較元で全ての地点へ越流経路を計算"), static_cast<std::size_t>(Before.VisitedSites), Fine->Directions.size());
	TestEqual(TEXT("盛土後も全ての地点へ越流経路を計算"), static_cast<std::size_t>(After.VisitedSites), Fine->Directions.size());
	AddInfo(
		FString::Printf(
			TEXT("Basin composed terrain: prepare=%.6fms reaches=%llu bedError=%.9fm waterError=%.9fm excessDepth=%.9fm ")
			TEXT("normalAngle=%.12grad. No erosion stage in this comparison."),
			PreparedMilliseconds,
			static_cast<unsigned long long>(Candidate->Drainage().Reaches().size()),
			MaximumBedError,
			MaximumWaterError,
			MaximumDepthError,
			MaximumNormalError
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Basin fixed-grid survey: sites=%llu baselineSpill=%.9fm candidateSpill=%.9fm baselineSites=%u ")
			TEXT("candidateSites=%u baselineSlope=%.9f candidateSlope=%.9f. Narrow unsampled channels are not represented."),
			static_cast<unsigned long long>(Fine->Directions.size()),
			Before.MaximumSpillMeters,
			After.MaximumSpillMeters,
			Before.RaisedSites,
			After.RaisedSites,
			Before.MaximumSlope,
			After.MaximumSlope
		)
	);
	const UnitDirection Sink = Fine->Directions[After.MaximumSpillIndex];
	const PlanetCore::RiverWaterHeights SinkWater = Candidate->EvaluateWater(Sink);
	const PlanetCore::RiverElevationStages SinkStages = Candidate->EvaluateElevationStages(Sink);
	const BasinChannelDistance NearestChannel = FindBasinChannel(*Candidate, Sink);
	const std::optional<PlanetCore::RiverProfileSample> Nearest =
	Candidate->Drainage().SampleReach(NearestChannel.Reach, NearestChannel.Fraction);
	// 最も近い実河道を照合出来ない場合の返戻
	if(!TestTrue(TEXT("最大窪地に最も近い実河道を照会"), Nearest.has_value())) return false;
	const UnitDirection NearestDirection = *UnitDirection::FromVector(Nearest->Direction);
	const PlanetCore::RiverWaterHeights NearestSurface = Candidate->EvaluateWater(NearestDirection);
	AddInfo(
		FString::Printf(
			TEXT("Basin spill distribution: baseline>1m=%u >10m=%u >50m=%u candidate>1m=%u >10m=%u >50m=%u."),
			Before.LargeSpills[0],
			Before.LargeSpills[1],
			Before.LargeSpills[2],
			After.LargeSpills[0],
			After.LargeSpills[1],
			After.LargeSpills[2]
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Basin worst location: node=%u source=%.9fm valley=%.9fm fine=%.9fm final=%.9fm water=%.9fm support=%.9g ")
			TEXT("nearestReach=%llu distance=%.9fm fraction=%.9f nearestTerrain=%.9fm nearestWater=%.9fm."),
			After.MaximumSpillIndex,
			Ocean->EvaluateElevation(Sink),
			SinkStages.AfterValleyMeters,
			SinkStages.AfterFineReliefMeters,
			SinkStages.FinalMeters,
			SinkWater.WaterElevationMeters,
			SinkWater.WaterSupport,
			static_cast<unsigned long long>(NearestChannel.Reach),
			NearestChannel.Meters,
			NearestChannel.Fraction,
			NearestSurface.TerrainElevationMeters,
			NearestSurface.WaterElevationMeters
		)
	);
	const std::optional<PlanetCore::SphereMesh> Coarse = PlanetCore::BuildUniformSphere(6);
	// 排水と同じ元網を作れない場合の返戻
	if(!TestTrue(TEXT("不足する支流の比較に元網を準備"), Coarse.has_value())) return false;
	std::vector<double> CoarseHeight(Coarse->Directions.size());
	std::vector<double> NeighborMinimum(Coarse->Directions.size(), std::numeric_limits<double>::infinity());
	std::vector<std::uint8_t> Selected(Drainage->Nodes().size()), MissingPath(Drainage->Nodes().size());
	for(const PlanetCore::RiverReach &Reach : Drainage->Reaches()) Selected[Reach.Upstream] = 1;
	for(std::size_t Index = 0; Index < CoarseHeight.size(); ++Index) {
		CoarseHeight[Index] = Candidate->EvaluateElevation(Coarse->Directions[Index]);
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Coarse->Triangles) for(unsigned Corner = 0; Corner < 3; ++Corner) {
		const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
		NeighborMinimum[A] = std::min(NeighborMinimum[A], CoarseHeight[B]);
		NeighborMinimum[B] = std::min(NeighborMinimum[B], CoarseHeight[A]);
	}
	std::uint32_t CoarseSinks = 0;
	for(std::uint32_t Index = 0; Index < CoarseHeight.size(); ++Index) {
		if(
			Drainage->Nodes()[Index].Ocean || CoarseHeight[Index] <= Settings.SeaLevelMeters + 0.01 ||
			NeighborMinimum[Index] - CoarseHeight[Index] <= 0.01
		) continue;
		++CoarseSinks;
		std::uint32_t Current = Index;
		for(std::size_t Steps = 0;; ++Steps) {
			// 範囲外や循環を含む経路を完了と誤認しない返戻
			if(!TestTrue(TEXT("支流の追加候補が有限な排水木を辿る"), Current < Drainage->Nodes().size() && Steps < Drainage->Nodes().size())) return false;
			if(Drainage->Nodes()[Current].Ocean || Selected[Current] || MissingPath[Current]) break;
			MissingPath[Current] = 1;
			Current = Drainage->Nodes()[Current].Downstream;
		}
	}
	std::vector<std::uint32_t> Promotion;
	for(std::uint32_t Index = 0; Index < MissingPath.size(); ++Index) if(MissingPath[Index]) Promotion.push_back(Index);
	// 原因候補の辺が無い比較を成功としない返戻
	if(!TestTrue(TEXT("盛土後に不足する物理的な排水辺を検出"), !Promotion.empty())) return false;
	const double PromotionStarted = FPlatformTime::Seconds();
	const std::optional<PlanetCore::DrainageNetwork> Promoted = Drainage->WithPromotedTributaries(Promotion, *Ocean);
	// 海岸・合流を保った支流を追加出来ない場合の返戻
	if(!TestTrue(TEXT("既存の接続契約で必要な支流だけを追加"), Promoted.has_value())) return false;
	const std::optional<RiverTerrainField> Connected =
	RiverTerrainField::CreateWithBasins(Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(*Promoted));
	const double PromotionMilliseconds = (FPlatformTime::Seconds() - PromotionStarted) * 1000;
	// 追加支流を連続な地形へ形成出来ない場合の返戻
	if(!TestTrue(TEXT("追加した支流を盛土と開削へ反映"), Connected.has_value())) return false;
	const BasinGridSurvey PromotedSurvey = SurveyBasinGrid(*Fine, *Connected);
	TestTrue(TEXT("支流追加後の標高と勾配が有限"), !PromotedSurvey.InvalidSamples);
	TestEqual(TEXT("支流追加後も全球の越流経路を計算"), static_cast<std::size_t>(PromotedSurvey.VisitedSites), Fine->Directions.size());
	const PlanetCore::RiverWaterHeights ConnectedSink = Connected->EvaluateWater(Sink);
	AddInfo(
		FString::Printf(
			TEXT("Basin tributary comparison: coarseSinks=%u addedEdges=%llu reaches=%llu prepare=%.6fms spill=%.9fm ")
			TEXT("sites=%u >1m=%u >10m=%u >50m=%u maxSlope=%.9f formerWorstTerrain=%.9fm formerWorstWater=%.9fm ")
			TEXT("formerWorstSupport=%.9g. Diagnostic, not acceptance."),
			CoarseSinks,
			static_cast<unsigned long long>(Promotion.size()),
			static_cast<unsigned long long>(Connected->Drainage().Reaches().size()),
			PromotionMilliseconds,
			PromotedSurvey.MaximumSpillMeters,
			PromotedSurvey.RaisedSites,
			PromotedSurvey.LargeSpills[0],
			PromotedSurvey.LargeSpills[1],
			PromotedSurvey.LargeSpills[2],
			PromotedSurvey.MaximumSlope,
			ConnectedSink.TerrainElevationMeters,
			ConnectedSink.WaterElevationMeters,
			ConnectedSink.WaterSupport
		)
	);
	const UnitDirection NewSink = Fine->Directions[PromotedSurvey.MaximumSpillIndex];
	const BasinChannelDistance NewNearest = FindBasinChannel(*Connected, NewSink);
	const PlanetCore::RiverWaterHeights NewSinkWater = Connected->EvaluateWater(NewSink);
	const PlanetCore::RiverElevationStages NewSinkStages = Connected->EvaluateElevationStages(NewSink);
	const std::optional<PlanetCore::RiverProfileSample> NewNearestSample =
	Connected->Drainage().SampleReach(NewNearest.Reach, NewNearest.Fraction);
	// 新しい最悪地点の周囲を照合出来ない場合の返戻
	if(!TestTrue(TEXT("支流追加後の最悪地点と河道を照合"), NewNearestSample.has_value())) return false;
	AddInfo(
		FString::Printf(
			TEXT("Basin promoted worst: node=%u source=%.9fm valley=%.9fm final=%.9fm water=%.9fm support=%.9g ")
			TEXT("nearestReach=%llu distance=%.9fm fraction=%.9f nearestWater=%.9fm."),
			PromotedSurvey.MaximumSpillIndex,
			Ocean->EvaluateElevation(NewSink),
			NewSinkStages.AfterValleyMeters,
			NewSinkStages.FinalMeters,
			NewSinkWater.WaterElevationMeters,
			NewSinkWater.WaterSupport,
			static_cast<unsigned long long>(NewNearest.Reach),
			NewNearest.Meters,
			NewNearest.Fraction,
			NewNearestSample->WaterMeters
		)
	);
	double MaximumRouteObstruction = 0, MinimumRouteSupport = 1;
	for(std::size_t Reach = 0; Reach < Connected->Drainage().Reaches().size(); ++Reach) for(unsigned Knot = 0; Knot < 129; ++Knot) {
		const std::optional<PlanetCore::RiverProfileSample> Sample = Connected->Drainage().SampleReach(Reach, Knot / 128.0);
		// 追加河道の物理的な通水箇所を照会出来ない場合の返戻
		if(!TestTrue(TEXT("支流の中間を独立な密度で照会"), Sample.has_value())) return false;
		const PlanetCore::RiverWaterHeights Water = Connected->EvaluateWater(*UnitDirection::FromVector(Sample->Direction));
		MaximumRouteObstruction = std::max(MaximumRouteObstruction, Water.TerrainElevationMeters - Sample->WaterMeters);
		MinimumRouteSupport = std::min(MinimumRouteSupport, Water.WaterSupport);
	}
	AddInfo(
		FString::Printf(
			TEXT("Basin promoted routes: sampledObstruction=%.9fm minimumWaterSupport=%.9g. ") TEXT("Unproven between samples."),
			MaximumRouteObstruction,
			MinimumRouteSupport
		)
	);
	// 既に開削した窪地も再準備の対象に保持し，改善済みの流出口を失わない
	std::vector<std::uint32_t> RefinedSources = PromotedSurvey.LocalMinima;
	RefinedSources.insert(RefinedSources.end(), Promotion.begin(), Promotion.end());
	std::sort(RefinedSources.begin(), RefinedSources.end());
	RefinedSources.erase(std::unique(RefinedSources.begin(), RefinedSources.end()), RefinedSources.end());
	const double RefineStarted = FPlatformTime::Seconds();
	const std::optional<BasinRefinementCandidate> Refined =
	PrepareBasinRefinement(Ocean, *Fine, Coarse->Directions.size(), RefinedSources);
	const double RefineMilliseconds = (FPlatformTime::Seconds() - RefineStarted) * 1000;
	// 局所排水を一体化した地形を作れない場合の返戻
	if(!TestTrue(TEXT("局所的な排水と同じ盛土・河床を準備"), Refined.has_value())) return false;
	const PlanetCore::DrainageRefinementMesh *RefinedMesh = &Refined->Mesh;
	const RiverTerrainField *RefinedTerrain = &Refined->Terrain;
	TestTrue(TEXT("粗網の外に有る窪地を局所網へ追加"), !RefinedMesh->SplitEdges.empty());
	AddInfo(
		FString::Printf(
			TEXT("Basin refinement input: meshSites=%llu splitEdges=%llu."),
			static_cast<unsigned long long>(RefinedMesh->Directions.size()),
			static_cast<unsigned long long>(RefinedMesh->SplitEdges.size())
		)
	);
	const BasinGridSurvey RefinedSurvey = SurveyBasinGrid(*Fine, *RefinedTerrain);
	TestTrue(TEXT("局所排水後も全ての標本と勾配が有限"), !RefinedSurvey.InvalidSamples);
	TestEqual(TEXT("局所排水後も全球の越流を照合"), static_cast<std::size_t>(RefinedSurvey.VisitedSites), Fine->Directions.size());
	const PlanetCore::RiverWaterHeights RefinedSink = RefinedTerrain->EvaluateWater(NewSink);
	const PlanetCore::RiverWaterHeights RetainedSink = RefinedTerrain->EvaluateWater(Sink);
	TestTrue(
		TEXT("局所追加した窪地に実河床と水面が有る"),
		RefinedSink.WaterSupport > 0.99 && RefinedSink.TerrainElevationMeters < RefinedSink.WaterElevationMeters
	);
	TestTrue(
		TEXT("以前に接続した窪地も通水を保持"),
		RetainedSink.WaterSupport > 0.99 && RetainedSink.TerrainElevationMeters < RetainedSink.WaterElevationMeters
	);
	AddInfo(
		FString::Printf(
			TEXT("Basin local refinement: addedNodes=%llu addedRoutes=%llu reaches=%llu prepare=%.6fms spill=%.9fm ")
			TEXT("sites=%u >1m=%u >10m=%u >50m=%u worstNode=%u formerWorstTerrain=%.9fm water=%.9fm support=%.9g."),
			static_cast<unsigned long long>(RefinedMesh->SplitEdges.size()),
			static_cast<unsigned long long>(Refined->PromotedEdges),
			static_cast<unsigned long long>(RefinedTerrain->Drainage().Reaches().size()),
			RefineMilliseconds,
			RefinedSurvey.MaximumSpillMeters,
			RefinedSurvey.RaisedSites,
			RefinedSurvey.LargeSpills[0],
			RefinedSurvey.LargeSpills[1],
			RefinedSurvey.LargeSpills[2],
			RefinedSurvey.MaximumSpillIndex,
			RefinedSink.TerrainElevationMeters,
			RefinedSink.WaterElevationMeters,
			RefinedSink.WaterSupport
		)
	);
	const UnitDirection RemainingSink = Fine->Directions[RefinedSurvey.MaximumSpillIndex];
	const BasinChannelDistance RemainingNearest = FindBasinChannel(*RefinedTerrain, RemainingSink);
	const PlanetCore::RiverWaterHeights RemainingWater = RefinedTerrain->EvaluateWater(RemainingSink);
	const PlanetCore::RiverElevationStages RemainingStages = RefinedTerrain->EvaluateElevationStages(RemainingSink);
	const std::optional<PlanetCore::RiverProfileSample> RemainingSample =
	RefinedTerrain->Drainage().SampleReach(RemainingNearest.Reach, RemainingNearest.Fraction);
	// 残る窪地に近い実河道を照合出来ない場合の返戻
	if(!TestTrue(TEXT("局所網の残差と実河道を照合"), RemainingSample.has_value())) return false;
	const PlanetCore::RiverWaterHeights RemainingChannel =
	RefinedTerrain->EvaluateWater(*UnitDirection::FromVector(RemainingSample->Direction));
	AddInfo(
		FString::Printf(
			TEXT("Basin refined worst: node=%u previousLocalMinimum=%u source=%.9fm valley=%.9fm final=%.9fm ")
			TEXT("water=%.9fm support=%.9g nearestReach=%llu distance=%.9fm fraction=%.9f ")
			TEXT("nearestTerrain=%.9fm nearestWater=%.9fm."),
			RefinedSurvey.MaximumSpillIndex,
			static_cast<unsigned>(
				std::binary_search(PromotedSurvey.LocalMinima.begin(), PromotedSurvey.LocalMinima.end(), RefinedSurvey.MaximumSpillIndex)
			),
			Ocean->EvaluateElevation(RemainingSink),
			RemainingStages.AfterValleyMeters,
			RemainingStages.FinalMeters,
			RemainingWater.WaterElevationMeters,
			RemainingWater.WaterSupport,
			static_cast<unsigned long long>(RemainingNearest.Reach),
			RemainingNearest.Meters,
			RemainingNearest.Fraction,
			RemainingChannel.TerrainElevationMeters,
			RemainingChannel.WaterElevationMeters
		)
	);
	const BasinGridSurvey WithRoutes = SurveyBasinGrid(*Fine, *RefinedTerrain, true);
	TestTrue(TEXT("実河床標本の有限性"), !WithRoutes.InvalidSamples);
	TestEqual(TEXT("全河道を照合"), static_cast<std::size_t>(WithRoutes.AddedRouteEdges), RefinedTerrain->Drainage().Reaches().size());
	TestEqual(
		TEXT("河口を含む到達数"),
		static_cast<std::size_t>(WithRoutes.VisitedSites),
		Fine->Directions.size() + WithRoutes.AddedRouteNodes
	);
	TestEqual(TEXT("実河道の途中に閉塞なし"), WithRoutes.ObstructedRouteSamples, std::uint64_t{ 0 });
	AddInfo(
		FString::Printf(
			TEXT("Basin actual-route survey: routes=%u additionalEndpoints=%u samples=%llu obstruction=%.9fm ")
			TEXT("endpointJoin=%.12gm spill=%.9fm sites=%u >1m=%u >10m=%u >50m=%u worstNode=%u. ")
			TEXT("Sampled channel barriers; not a continuous certificate."),
			WithRoutes.AddedRouteEdges,
			WithRoutes.AddedRouteNodes,
			static_cast<unsigned long long>(WithRoutes.RouteSamples),
			WithRoutes.MaximumRouteObstructionMeters,
			WithRoutes.MaximumEndpointJoinMeters,
			WithRoutes.MaximumSpillMeters,
			WithRoutes.RaisedSites,
			WithRoutes.LargeSpills[0],
			WithRoutes.LargeSpills[1],
			WithRoutes.LargeSpills[2],
			WithRoutes.MaximumSpillIndex
		)
	);
	const UnitDirection HeadSink = Fine->Directions[WithRoutes.MaximumHeadDeficitIndex];
	const PlanetCore::RiverWaterHeights HeadWater = RefinedTerrain->EvaluateWater(HeadSink);
	const BasinChannelDistance HeadNearest = FindBasinChannel(*RefinedTerrain, HeadSink);
	AddInfo(
		FString::Printf(
			TEXT("Basin available-head survey: deficit=%.9fm sites=%llu worstNode=%u terrain=%.9fm water=%.9fm ")
			TEXT("support=%.9g nearestDistance=%.9fm priorSource=%u. Raw spill remains recorded separately."),
			WithRoutes.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(WithRoutes.HeadDeficitSites.size()),
			WithRoutes.MaximumHeadDeficitIndex,
			HeadWater.TerrainElevationMeters,
			HeadWater.WaterElevationMeters,
			HeadWater.WaterSupport,
			HeadNearest.Meters,
			static_cast<unsigned>(std::binary_search(RefinedSources.begin(), RefinedSources.end(), WithRoutes.MaximumHeadDeficitIndex))
		)
	);
	std::vector<std::uint32_t> AccumulatedSources = RefinedSources;
	BasinGridSurvey FeedbackSurvey = SurveyBasinGrid(*Fine, *RefinedTerrain, true, false, false, true);
	TestTrue(TEXT("入口選定でも河川内部の接続を含む有限な地形"), !FeedbackSurvey.InvalidSamples);
	TestEqual(
		TEXT("入口選定でも全地点と河川内部を大洋へ照合"),
		static_cast<std::size_t>(FeedbackSurvey.VisitedSites),
		Fine->Directions.size() + FeedbackSurvey.AddedRouteNodes
	);
	TestEqual(TEXT("入口選定でも実河床標本に閉塞なし"), FeedbackSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
	AddInfo(
		FString::Printf(
			TEXT("Basin feedback input: endpointOnly=%.9fm endpointSites=%llu joined=%.9fm joinedSites=%llu riverJoins=%llu. ")
			TEXT("Subsequent outlet selection uses actual river interiors without moving survey sites."),
			WithRoutes.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(WithRoutes.HeadDeficitSites.size()),
			FeedbackSurvey.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(FeedbackSurvey.HeadDeficitSites.size()),
			static_cast<unsigned long long>(FeedbackSurvey.JoinedRiverSites)
		)
	);
	std::optional<BasinRefinementCandidate> FinalFeedback;
	for(unsigned Pass = 0; Pass < 4 && !FeedbackSurvey.HeadDeficitSites.empty(); ++Pass) {
		const std::size_t PreviousCount = AccumulatedSources.size();
		AccumulatedSources.insert(
			AccumulatedSources.end(),
			FeedbackSurvey.HeadDeficitSites.begin(),
			FeedbackSurvey.HeadDeficitSites.end()
		);
		std::sort(AccumulatedSources.begin(), AccumulatedSources.end());
		AccumulatedSources.erase(std::unique(AccumulatedSources.begin(), AccumulatedSources.end()), AccumulatedSources.end());
		if(AccumulatedSources.size() == PreviousCount) {
			AddInfo(TEXT("Basin feedback stopped: unresolved sites already have retained routes; topology alone is insufficient."));
			break;
		}
		const double FeedbackStarted = FPlatformTime::Seconds();
		std::optional<BasinRefinementCandidate> Feedback =
		PrepareBasinRefinement(Ocean, *Fine, Coarse->Directions.size(), AccumulatedSources);
		const double FeedbackMilliseconds = (FPlatformTime::Seconds() - FeedbackStarted) * 1000;
		// 新たな窪地の出口と既存の出口を共に再構成出来ない場合の返戻
		if(!TestTrue(TEXT("不足水頭の地点を保持する局所再構成"), Feedback.has_value())) return false;
		for(const std::uint32_t Index : AccumulatedSources) {
			const PlanetCore::RiverWaterHeights Water = Feedback->Terrain.EvaluateWater(Fine->Directions[Index]);
			if(Water.TerrainElevationMeters < Settings.SeaLevelMeters) continue;
			TestTrue(TEXT("保持した全入口に水位未満の実河床"), Water.WaterSupport > 0.99 && Water.TerrainElevationMeters < Water.WaterElevationMeters);
		}
		FeedbackSurvey = SurveyBasinGrid(*Fine, Feedback->Terrain, true, false, false, true);
		TestTrue(TEXT("再構成後の全標本が有限"), !FeedbackSurvey.InvalidSamples);
		TestEqual(
			TEXT("再構成後も全河道を照合"),
			static_cast<std::size_t>(FeedbackSurvey.AddedRouteEdges),
			Feedback->Terrain.Drainage().Reaches().size()
		);
		TestEqual(
			TEXT("再構成後も全地点から海への到達を照合"),
			static_cast<std::size_t>(FeedbackSurvey.VisitedSites),
			Fine->Directions.size() + FeedbackSurvey.AddedRouteNodes
		);
		TestEqual(TEXT("再構成後の実河道標本に閉塞なし"), FeedbackSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
		const PlanetCore::RiverWaterHeights Worst =
		Feedback->Terrain.EvaluateWater(Fine->Directions[FeedbackSurvey.MaximumHeadDeficitIndex]);
		AddInfo(
			FString::Printf(
				TEXT("Basin feedback: pass=%u sources=%llu newSources=%llu splits=%llu promoted=%llu reaches=%llu prepare=%.6fms ")
				TEXT("rawSpill=%.9fm headDeficit=%.9fm unresolved=%llu worstNode=%u terrain=%.9fm water=%.9fm support=%.9g ")
				TEXT("routeSamples=%llu obstruction=%.9fm. Fixed sampling does not certify continuous drainage."),
				Pass + 1,
				static_cast<unsigned long long>(AccumulatedSources.size()),
				static_cast<unsigned long long>(AccumulatedSources.size() - PreviousCount),
				static_cast<unsigned long long>(Feedback->Mesh.SplitEdges.size()),
				static_cast<unsigned long long>(Feedback->PromotedEdges),
				static_cast<unsigned long long>(Feedback->Terrain.Drainage().Reaches().size()),
				FeedbackMilliseconds,
				FeedbackSurvey.MaximumSpillMeters,
				FeedbackSurvey.MaximumHeadDeficitMeters,
				static_cast<unsigned long long>(FeedbackSurvey.HeadDeficitSites.size()),
				FeedbackSurvey.MaximumHeadDeficitIndex,
				Worst.TerrainElevationMeters,
				Worst.WaterElevationMeters,
				Worst.WaterSupport,
				static_cast<unsigned long long>(FeedbackSurvey.RouteSamples),
				FeedbackSurvey.MaximumRouteObstructionMeters
			)
		);
		FinalFeedback = std::move(Feedback);
	}
	const std::optional<BasinSurveyMesh> Holdout = SubdivideBasinSurveyMesh(Fine->Directions, Fine->Triangles);
	// 生成に使用していない中間標本を準備出来ない場合の返戻
	if(!TestTrue(TEXT("未使用の中間点を持つ独立な細網を準備"), Holdout.has_value())) return false;
	TestEqual(TEXT("未使用の中間点の追加数"), Holdout->Directions.size() - Fine->Directions.size(), std::size_t{ 491520 });
	const RiverTerrainField &FinalTerrain = FinalFeedback ? FinalFeedback->Terrain : *RefinedTerrain;
	const BasinGridSurvey HoldoutSurvey = SurveyBasinGrid(*Holdout, FinalTerrain, true, false, false, true);
	TestTrue(TEXT("未使用標本の有限性"), !HoldoutSurvey.InvalidSamples);
	TestEqual(
		TEXT("独立な細網でも全地点からの経路を照合"),
		static_cast<std::size_t>(HoldoutSurvey.VisitedSites),
		Holdout->Directions.size() + HoldoutSurvey.AddedRouteNodes
	);
	TestEqual(TEXT("独立な細網でも実河床の閉塞なし"), HoldoutSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
	const UnitDirection HoldoutSink = Holdout->Directions[HoldoutSurvey.MaximumHeadDeficitIndex];
	const PlanetCore::RiverWaterHeights HoldoutWater = FinalTerrain.EvaluateWater(HoldoutSink);
	const BasinChannelDistance HoldoutNearest = FindBasinChannel(FinalTerrain, HoldoutSink);
	AddInfo(
		FString::Printf(
			TEXT("Basin holdout: sites=%llu newSites=%llu rawSpill=%.9fm headDeficit=%.9fm unresolved=%llu ")
			TEXT("worstNode=%u terrain=%.9fm water=%.9fm support=%.9g nearestDistance=%.9fm. ")
			TEXT("No holdout site was fed back to the generator."),
			static_cast<unsigned long long>(Holdout->Directions.size()),
			static_cast<unsigned long long>(Holdout->Directions.size() - Fine->Directions.size()),
			HoldoutSurvey.MaximumSpillMeters,
			HoldoutSurvey.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(HoldoutSurvey.HeadDeficitSites.size()),
			HoldoutSurvey.MaximumHeadDeficitIndex,
			HoldoutWater.TerrainElevationMeters,
			HoldoutWater.WaterElevationMeters,
			HoldoutWater.WaterSupport,
			HoldoutNearest.Meters
		)
	);
	// 此処からは独立評価用だった細網を生成にも使用する
	// 上の独立評価を上書きしない
	BasinGridSurvey SamplingSurvey = HoldoutSurvey;
	std::optional<BasinSamplingCandidate> FinalSampled;
	for(unsigned Pass = 0; Pass < 6 && !SamplingSurvey.HeadDeficitSites.empty(); ++Pass) {
		const std::size_t PreviousCount = AccumulatedSources.size();
		AccumulatedSources.insert(
			AccumulatedSources.end(),
			SamplingSurvey.HeadDeficitSites.begin(),
			SamplingSurvey.HeadDeficitSites.end()
		);
		std::sort(AccumulatedSources.begin(), AccumulatedSources.end());
		AccumulatedSources.erase(std::unique(AccumulatedSources.begin(), AccumulatedSources.end()), AccumulatedSources.end());
		if(AccumulatedSources.size() == PreviousCount) {
			AddInfo(TEXT("Basin sampling stopped: all unresolved sites already retained; new topology does not resolve the barrier."));
			break;
		}
		std::vector<UnitDirection> Samples;
		for(const std::uint32_t Index : AccumulatedSources) Samples.push_back(Holdout->Directions[Index]);
		const double SamplingStarted = FPlatformTime::Seconds();
		BasinSamplingPreparation Preparation;
		std::optional<BasinSamplingCandidate> Sampled = PrepareSampledBasin(Ocean, Samples, Preparation);
		const double SamplingMilliseconds = (FPlatformTime::Seconds() - SamplingStarted) * 1000;
		AddInfo(
			FString::Printf(
				TEXT("Basin sampling preparation: pass=%u phase=%s sources=%llu nodes=%llu reaches=%llu bankConstraints=%llu prepare=%.6fms."),
				Pass + 1,
				Preparation.Phase,
				static_cast<unsigned long long>(Samples.size()),
				static_cast<unsigned long long>(Preparation.Nodes),
				static_cast<unsigned long long>(Preparation.Reaches),
				static_cast<unsigned long long>(Preparation.BankConstraints),
				SamplingMilliseconds
			)
		);
		// 検出地点其の物に実際の流路を形成出来ない場合の返戻
		if(!TestTrue(TEXT("検出した位置を移動しない排水再構成"), Sampled.has_value())) return false;
		for(const UnitDirection Point : Samples) {
			const PlanetCore::RiverWaterHeights Water = Sampled->Terrain.EvaluateWater(Point);
			if(Water.TerrainElevationMeters < Settings.SeaLevelMeters) continue;
			TestTrue(TEXT("任意の追加地点も既存の入口も通水を保持"), Water.WaterSupport > 0.99 && Water.TerrainElevationMeters < Water.WaterElevationMeters);
		}
		SamplingSurvey = SurveyBasinGrid(*Holdout, Sampled->Terrain, true, false, false, true);
		TestTrue(TEXT("任意地点追加後の全標本が有限"), !SamplingSurvey.InvalidSamples);
		TestEqual(
			TEXT("任意地点追加後も全ての経路を照合"),
			static_cast<std::size_t>(SamplingSurvey.VisitedSites),
			Holdout->Directions.size() + SamplingSurvey.AddedRouteNodes
		);
		TestEqual(TEXT("任意地点追加後の実河道標本に閉塞なし"), SamplingSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
		AddInfo(
			FString::Printf(
				TEXT("Basin sampling feedback: pass=%u sources=%llu newSources=%llu addedNodes=%llu promoted=%llu reaches=%llu ")
				TEXT("prepare=%.6fms rawSpill=%.9fm headDeficit=%.9fm unresolved=%llu worstNode=%u routeSamples=%llu ")
				TEXT("obstruction=%.9fm. This grid is now used for generation and is no longer a holdout."),
				Pass + 1,
				static_cast<unsigned long long>(Samples.size()),
				static_cast<unsigned long long>(AccumulatedSources.size() - PreviousCount),
				static_cast<unsigned long long>(Sampled->Mesh.Directions.size() - Coarse->Directions.size()),
				static_cast<unsigned long long>(Sampled->PromotedEdges),
				static_cast<unsigned long long>(Sampled->Terrain.Drainage().Reaches().size()),
				SamplingMilliseconds,
				SamplingSurvey.MaximumSpillMeters,
				SamplingSurvey.MaximumHeadDeficitMeters,
				static_cast<unsigned long long>(SamplingSurvey.HeadDeficitSites.size()),
				SamplingSurvey.MaximumHeadDeficitIndex,
				static_cast<unsigned long long>(SamplingSurvey.RouteSamples),
				SamplingSurvey.MaximumRouteObstructionMeters
			)
		);
		AddInfo(
			FString::Printf(
				TEXT("Basin sampling shape: pass=%u maxSlope=%.9f reliefBound=%.9fm."),
				Pass + 1,
				SamplingSurvey.MaximumSlope,
				Sampled->Terrain.AbsoluteReliefBoundMeters()
			)
		);
		if(SamplingSurvey.HeadDeficitSites.size() < 21) for(const std::uint32_t Index : SamplingSurvey.HeadDeficitSites) {
			const UnitDirection Point = Holdout->Directions[Index];
			const PlanetCore::RiverWaterHeights Water = Sampled->Terrain.EvaluateWater(Point);
			const BasinChannelDistance ResidualChannel = FindBasinChannel(Sampled->Terrain, Point);
			const PlanetCore::TerrainSample Terrain = Sampled->Terrain.Evaluate(Point);
			AddInfo(
				FString::Printf(
					TEXT("Basin remaining source: pass=%u node=%u retained=%u terrain=%.9fm water=%.9fm support=%.9g ")
					TEXT("slope=%.12g nearestReach=%llu nearestDistance=%.9fm."),
					Pass + 1,
					Index,
					static_cast<unsigned>(std::binary_search(AccumulatedSources.begin(), AccumulatedSources.end(), Index)),
					Water.TerrainElevationMeters,
					Water.WaterElevationMeters,
					Water.WaterSupport,
					Terrain.TangentialGradientPerRadian.Length() / Settings.RadiusMeters,
					static_cast<unsigned long long>(ResidualChannel.Reach),
					ResidualChannel.Meters
				)
			);
		}
		FinalSampled = std::move(Sampled);
	}
	const RiverTerrainField &IndependentTerrain = FinalSampled ? FinalSampled->Terrain : FinalTerrain;
	// 回帰用の生成入力だけを記録し，地形の標高配列や生成済みの面は保存しない
	AddInfo(
		FString::Printf(
			TEXT("Basin retained definition: seed=%llu radius=%.9fm oceanHash=%llu sources=%llu."),
			static_cast<unsigned long long>(Settings.Seed),
			Settings.RadiusMeters,
			static_cast<unsigned long long>(Ocean->DefinitionHash()),
			static_cast<unsigned long long>(AccumulatedSources.size())
		)
	);
	for(std::size_t Offset = 0; Offset < AccumulatedSources.size(); Offset += 128) {
		FString Sites;
		Sites.Reserve(1536);
		const std::size_t End = std::min(Offset + 128, AccumulatedSources.size());
		for(std::size_t Index = Offset; Index < End; ++Index) Sites.Appendf(TEXT("%u,"), AccumulatedSources[Index]);
		AddInfo(FString::Printf(TEXT("Basin retained sites: offset=%llu indices=%s"), static_cast<unsigned long long>(Offset), *Sites));
	}
	const double EdgeStarted = FPlatformTime::Seconds();
	const BasinGridSurvey EdgeSurvey = SurveyBasinGrid(*Holdout, IndependentTerrain, true, true);
	const double EdgeMilliseconds = (FPlatformTime::Seconds() - EdgeStarted) * 1000;
	TestTrue(TEXT("未使用の辺中点も有限な地形"), !EdgeSurvey.InvalidSamples);
	TestEqual(TEXT("各共有辺に独立な中点を一度だけ追加"), EdgeSurvey.EdgeInteriorSites, std::size_t{ 1966080 });
	TestEqual(
		TEXT("辺中点を飛越えず全地点の出口を照合"),
		static_cast<std::size_t>(EdgeSurvey.VisitedSites),
		Holdout->Directions.size() + EdgeSurvey.EdgeInteriorSites + EdgeSurvey.AddedRouteNodes
	);
	TestEqual(TEXT("独立な辺検査でも実河床標本に閉塞なし"), EdgeSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
	const UnitDirection EdgeSink = *UnitDirection::FromVector(EdgeSurvey.MaximumHeadDeficitDirection);
	const PlanetCore::RiverWaterHeights EdgeWater = IndependentTerrain.EvaluateWater(EdgeSink);
	const BasinChannelDistance EdgeNearest = FindBasinChannel(IndependentTerrain, EdgeSink);
	AddInfo(
		FString::Printf(
			TEXT("Basin independent edges: sites=%llu newSites=%llu survey=%.6fms rawSpill=%.9fm headDeficit=%.9fm unresolved=%llu ")
			TEXT("worstNode=%u terrain=%.9fm water=%.9fm support=%.9g nearestDistance=%.9fm maxSlope=%.9f. ")
			TEXT("No midpoint was used for generation. Edge-only paths omit triangle interiors and are not a continuous certificate."),
			static_cast<unsigned long long>(Holdout->Directions.size() + EdgeSurvey.EdgeInteriorSites),
			static_cast<unsigned long long>(EdgeSurvey.EdgeInteriorSites),
			EdgeMilliseconds,
			EdgeSurvey.MaximumSpillMeters,
			EdgeSurvey.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(EdgeSurvey.HeadDeficitSites.size()),
			EdgeSurvey.MaximumHeadDeficitIndex,
			EdgeWater.TerrainElevationMeters,
			EdgeWater.WaterElevationMeters,
			EdgeWater.WaterSupport,
			EdgeNearest.Meters,
			EdgeSurvey.MaximumSlope
		)
	);
	for(const Vec3 Position : { EdgeSurvey.MaximumHeadDeficitDirection, EdgeSurvey.MaximumSlopeDirection }) {
		const UnitDirection Point = *UnitDirection::FromVector(Position);
		const PlanetCore::TerrainSample Surface = IndependentTerrain.Evaluate(Point), OriginalSurface = Ocean->Evaluate(Point);
		const PlanetCore::RiverElevationStages Stages = IndependentTerrain.EvaluateElevationStages(Point);
		const PlanetCore::RiverWaterHeights Water = IndependentTerrain.EvaluateWater(Point);
		TestEqual(TEXT("診断経路の元地形が実際の場と一致"), Stages.BeforeBasinMeters, OriginalSurface.ElevationMeters);
		TestEqual(TEXT("診断経路の最終地形が実際の場と一致"), Stages.FinalMeters, Surface.ElevationMeters);
		TestTrue(
			TEXT("盆地底の段階も有限で元地形以上"),
			std::isfinite(Stages.AfterBasinMeters) && Stages.AfterBasinMeters >= Stages.BeforeBasinMeters
		);
		AddInfo(
			FString::Printf(
				TEXT("Basin edge stages: x=%.17g y=%.17g z=%.17g original=%.12gm filled=%.12gm valley=%.12gm ")
				TEXT("fine=%.12gm final=%.12gm originalSlope=%.12g finalSlope=%.12g water=%.12gm support=%.12g."),
				Position.X,
				Position.Y,
				Position.Z,
				Stages.BeforeBasinMeters,
				Stages.AfterBasinMeters,
				Stages.AfterValleyMeters,
				Stages.AfterFineReliefMeters,
				Stages.FinalMeters,
				OriginalSurface.TangentialGradientPerRadian.Length() / Settings.RadiusMeters,
				Surface.TangentialGradientPerRadian.Length() / Settings.RadiusMeters,
				Water.WaterElevationMeters,
				Water.WaterSupport
			)
		);
		const std::optional<UnitDirection> Along = UnitDirection::FromVector(Surface.TangentialGradientPerRadian);
		if(!Along) continue;
		for(const double StepMeters : { 1.0, 0.1, 0.01 }) {
			const double Step = StepMeters / Settings.RadiusMeters;
			const UnitDirection Low = *UnitDirection::FromVector(Position * std::cos(Step) - Along->Vector() * std::sin(Step));
			const UnitDirection High = *UnitDirection::FromVector(Position * std::cos(Step) + Along->Vector() * std::sin(Step));
			const PlanetCore::RiverElevationStages A = IndependentTerrain.EvaluateElevationStages(Low);
			const PlanetCore::RiverElevationStages B = IndependentTerrain.EvaluateElevationStages(High);
			AddInfo(
				FString::Printf(
					TEXT("Basin edge differences: step=%.6gm originalSlope=%.12g filledSlope=%.12g valleySlope=%.12g finalSlope=%.12g."),
					StepMeters,
					(B.BeforeBasinMeters - A.BeforeBasinMeters) / (2 * StepMeters),
					(B.AfterBasinMeters - A.AfterBasinMeters) / (2 * StepMeters),
					(B.AfterValleyMeters - A.AfterValleyMeters) / (2 * StepMeters),
					(B.FinalMeters - A.FinalMeters) / (2 * StepMeters)
				)
			);
		}
	}
	const double TriangleStarted = FPlatformTime::Seconds();
	const BasinGridSurvey TriangleSurvey = SurveyBasinGrid(*Holdout, IndependentTerrain, true, true, true);
	const double TriangleMilliseconds = (FPlatformTime::Seconds() - TriangleStarted) * 1000;
	TestTrue(TEXT("独立な三角網の全標本が有限"), !TriangleSurvey.InvalidSamples);
	TestEqual(TEXT("辺だけの検査と同じ未使用地点"), TriangleSurvey.EdgeInteriorSites, EdgeSurvey.EdgeInteriorSites);
	TestEqual(TEXT("各面の内部に３本の中点接続"), TriangleSurvey.TriangleInteriorLinks, Holdout->Triangles.size() * 3);
	TestEqual(
		TEXT("細分化した球面三角網の辺数"),
		TriangleSurvey.EdgeInteriorSites * 2 + TriangleSurvey.TriangleInteriorLinks,
		(Holdout->Directions.size() + TriangleSurvey.EdgeInteriorSites) * 3 - 6
	);
	TestEqual(
		TEXT("内部を接続した三角網で全地点の出口を照合"),
		static_cast<std::size_t>(TriangleSurvey.VisitedSites),
		Holdout->Directions.size() + TriangleSurvey.EdgeInteriorSites + TriangleSurvey.AddedRouteNodes
	);
	TestTrue(TEXT("経路の追加で必要水頭が増えない"), TriangleSurvey.MaximumHeadDeficitMeters <= EdgeSurvey.MaximumHeadDeficitMeters);
	TestTrue(TEXT("経路の追加で不足地点が増えない"), TriangleSurvey.HeadDeficitSites.size() <= EdgeSurvey.HeadDeficitSites.size());
	TestEqual(TEXT("三角網の検査でも実河床標本に閉塞なし"), TriangleSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
	const UnitDirection TriangleSink = *UnitDirection::FromVector(TriangleSurvey.MaximumHeadDeficitDirection);
	const PlanetCore::RiverWaterHeights TriangleWater = IndependentTerrain.EvaluateWater(TriangleSink);
	const BasinChannelDistance TriangleNearest = FindBasinChannel(IndependentTerrain, TriangleSink);
	AddInfo(
		FString::Printf(
			TEXT("Basin independent triangles: sites=%llu newSites=%llu interiorLinks=%llu survey=%.6fms rawSpill=%.9fm ")
			TEXT("headDeficit=%.9fm unresolved=%llu worstNode=%u x=%.17g y=%.17g z=%.17g terrain=%.9fm water=%.9fm ")
			TEXT("support=%.9g nearestDistance=%.9fm. Same unused points with complete midpoint triangulation; unsampled paths remain."),
			static_cast<unsigned long long>(Holdout->Directions.size() + TriangleSurvey.EdgeInteriorSites),
			static_cast<unsigned long long>(TriangleSurvey.EdgeInteriorSites),
			static_cast<unsigned long long>(TriangleSurvey.TriangleInteriorLinks),
			TriangleMilliseconds,
			TriangleSurvey.MaximumSpillMeters,
			TriangleSurvey.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(TriangleSurvey.HeadDeficitSites.size()),
			TriangleSurvey.MaximumHeadDeficitIndex,
			TriangleSurvey.MaximumHeadDeficitDirection.X,
			TriangleSurvey.MaximumHeadDeficitDirection.Y,
			TriangleSurvey.MaximumHeadDeficitDirection.Z,
			TriangleWater.TerrainElevationMeters,
			TriangleWater.WaterElevationMeters,
			TriangleWater.WaterSupport,
			TriangleNearest.Meters
		)
	);
	const double JoinedStarted = FPlatformTime::Seconds();
	const BasinGridSurvey JoinedSurvey = SurveyBasinGrid(*Holdout, IndependentTerrain, true, true, true, true);
	const double JoinedMilliseconds = (FPlatformTime::Seconds() - JoinedStarted) * 1000;
	TestTrue(TEXT("河川内部へ接続した全標本が有限"), !JoinedSurvey.InvalidSamples);
	TestEqual(TEXT("河川接続でも独立な地点を移動しない"), JoinedSurvey.EdgeInteriorSites, TriangleSurvey.EdgeInteriorSites);
	TestEqual(
		TEXT("河川内部の追加節点も全て大洋へ探索"),
		static_cast<std::size_t>(JoinedSurvey.VisitedSites),
		Holdout->Directions.size() + JoinedSurvey.EdgeInteriorSites + JoinedSurvey.AddedRouteNodes
	);
	TestEqual(
		TEXT("河川内部を接続しても全ての河道を保持"),
		JoinedSurvey.AddedRouteEdges,
		static_cast<std::uint32_t>(IndependentTerrain.Drainage().Reaches().size())
	);
	TestTrue(TEXT("水没する河川内部の独立地点を実際に接続"), JoinedSurvey.JoinedRiverSites > 0);
	TestTrue(TEXT("途中の接続位置で実河道を分割"), JoinedSurvey.RiverSegments > JoinedSurvey.AddedRouteEdges);
	TestEqual(TEXT("追加した河川中心でも実河床標本に閉塞なし"), JoinedSurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
	const UnitDirection JoinedSink = *UnitDirection::FromVector(JoinedSurvey.MaximumHeadDeficitDirection);
	const PlanetCore::RiverWaterHeights JoinedWater = IndependentTerrain.EvaluateWater(JoinedSink);
	const BasinChannelDistance JoinedNearest = FindBasinChannel(IndependentTerrain, JoinedSink);
	AddInfo(
		FString::Printf(
			TEXT("Basin joined rivers: sites=%llu joined=%llu rejected=%llu segments=%llu maxJoin=%.9fm survey=%.6fms ")
			TEXT("rawSpill=%.9fm headDeficit=%.9fm unresolved=%llu worstNode=%u x=%.17g y=%.17g z=%.17g ")
			TEXT("terrain=%.9fm water=%.9fm support=%.9g nearestDistance=%.9fm. ")
			TEXT("River interiors joined through eight terrain samples; unsampled paths remain."),
			static_cast<unsigned long long>(Holdout->Directions.size() + JoinedSurvey.EdgeInteriorSites),
			static_cast<unsigned long long>(JoinedSurvey.JoinedRiverSites),
			static_cast<unsigned long long>(JoinedSurvey.RejectedRiverJoins),
			static_cast<unsigned long long>(JoinedSurvey.RiverSegments),
			JoinedSurvey.MaximumRiverJoinMeters,
			JoinedMilliseconds,
			JoinedSurvey.MaximumSpillMeters,
			JoinedSurvey.MaximumHeadDeficitMeters,
			static_cast<unsigned long long>(JoinedSurvey.HeadDeficitSites.size()),
			JoinedSurvey.MaximumHeadDeficitIndex,
			JoinedSurvey.MaximumHeadDeficitDirection.X,
			JoinedSurvey.MaximumHeadDeficitDirection.Y,
			JoinedSurvey.MaximumHeadDeficitDirection.Z,
			JoinedWater.TerrainElevationMeters,
			JoinedWater.WaterElevationMeters,
			JoinedWater.WaterSupport,
			JoinedNearest.Meters
		)
	);
	const BasinDescentTrace Descent = TraceBasinTerrain(IndependentTerrain, JoinedSink);
	TestTrue(TEXT("残る最悪点の下降追跡も有限"), Descent.End != BasinDescentEnd::Invalid);
	const PlanetCore::RiverElevationStages DescentStages =
	IndependentTerrain.EvaluateElevationStages(*UnitDirection::FromVector(Descent.Direction));
	AddInfo(
		FString::Printf(
			TEXT("Basin sampled descent: end=%u steps=%u evaluations=%llu distance=%.9fm start=%.9fm final=%.9fm ")
			TEXT("slope=%.12g x=%.17g y=%.17g z=%.17g original=%.9fm filled=%.9fm valley=%.9fm fine=%.9fm. ")
			TEXT("End codes: ocean0 river1 stalled2 limit3 invalid4. Eight samples per accepted step; no continuous certificate."),
			static_cast<unsigned>(Descent.End),
			Descent.Steps,
			static_cast<unsigned long long>(Descent.Evaluations),
			Descent.DistanceMeters,
			Descent.StartHeight,
			Descent.EndHeight,
			Descent.EndSlope,
			Descent.Direction.X,
			Descent.Direction.Y,
			Descent.Direction.Z,
			DescentStages.BeforeBasinMeters,
			DescentStages.AfterBasinMeters,
			DescentStages.AfterValleyMeters,
			DescentStages.AfterFineReliefMeters
		)
	);
	for(const Vec3 Position : { Descent.Direction, Vec3{ -0.2025916655428705, -0.16657808879042132, -0.9649913768461853 } }) {
		const UnitDirection Point = *UnitDirection::FromVector(Position);
		const PlanetCore::TerrainSample FinalSample = IndependentTerrain.Evaluate(Point), BaseSample = Ocean->Evaluate(Point);
		const std::optional<UnitDirection> Along = UnitDirection::FromVector(FinalSample.TangentialGradientPerRadian);
		if(!Along) continue;
		const double FinalSlope = FinalSample.TangentialGradientPerRadian.Dot(Along->Vector()) / Settings.RadiusMeters;
		const double BaseSlope = BaseSample.TangentialGradientPerRadian.Dot(Along->Vector()) / Settings.RadiusMeters;
		for(const double StepMeters : { 1.0, 0.1, 0.01, 0.001 }) {
			const double Angle = StepMeters / Settings.RadiusMeters;
			const UnitDirection Low = *UnitDirection::FromVector(Position * std::cos(Angle) - Along->Vector() * std::sin(Angle));
			const UnitDirection High = *UnitDirection::FromVector(Position * std::cos(Angle) + Along->Vector() * std::sin(Angle));
			const double FinalDifference =
			(IndependentTerrain.EvaluateElevation(High) - IndependentTerrain.EvaluateElevation(Low)) / (2 * StepMeters);
			const double BaseDifference = (Ocean->EvaluateElevation(High) - Ocean->EvaluateElevation(Low)) / (2 * StepMeters);
			AddInfo(
				FString::Printf(
					TEXT("Basin descent differences: x=%.17g y=%.17g z=%.17g step=%.9fm ")
					TEXT("baseAnalytic=%.12g baseDifference=%.12g finalAnalytic=%.12g finalDifference=%.12g scalarGap=%.12gm."),
					Position.X,
					Position.Y,
					Position.Z,
					StepMeters,
					BaseSlope,
					BaseDifference,
					FinalSlope,
					FinalDifference,
					IndependentTerrain.EvaluateElevation(Point) - FinalSample.ElevationMeters
				)
			);
		}
	}
	if(Descent.End == BasinDescentEnd::Stalled) {
		InspectBasinStages(*this, TEXT("before outlet"), IndependentTerrain, *UnitDirection::FromVector(Descent.Direction));
		const UnitDirection Stationary = *UnitDirection::FromVector(Descent.Direction);
		const Vec3 Axis = std::abs(Descent.Direction.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		const Vec3 Across = UnitDirection::FromVector(Descent.Direction.Cross(Axis))->Vector();
		const Vec3 Along = Descent.Direction.Cross(Across);
		for(const double Distance : { 1.0, 10.0, 100.0 }) {
			double MinimumRise = std::numeric_limits<double>::infinity(), MaximumRise = -MinimumRise;
			for(unsigned Side = 0; Side < 16; ++Side) {
				const double Azimuth = Side * std::acos(-1.0) / 8, Angle = Distance / Settings.RadiusMeters;
				const Vec3 Tangent = Across * std::cos(Azimuth) + Along * std::sin(Azimuth);
				const UnitDirection Ring = *UnitDirection::FromVector(Descent.Direction * std::cos(Angle) + Tangent * std::sin(Angle));
				const double Rise = IndependentTerrain.EvaluateElevation(Ring) - Descent.EndHeight;
				MinimumRise = std::min(MinimumRise, Rise);
				MaximumRise = std::max(MaximumRise, Rise);
			}
			AddInfo(
				FString::Printf(
					TEXT("Basin stationary ring: radius=%.9fm minRise=%.12gm maxRise=%.12gm samples=16."),
					Distance,
					MinimumRise,
					MaximumRise
				)
			);
		}
		std::vector<UnitDirection> RetainedSources;
		for(const std::uint32_t Index : AccumulatedSources) RetainedSources.push_back(Holdout->Directions[Index]);
		RetainedSources.push_back(Stationary);
		BasinSamplingPreparation Progress;
		const double StationaryStarted = FPlatformTime::Seconds();
		const std::optional<BasinSamplingCandidate> Opened = PrepareSampledBasin(Ocean, RetainedSources, Progress);
		const double StationaryMilliseconds = (FPlatformTime::Seconds() - StationaryStarted) * 1000;
		AddInfo(
			FString::Printf(
				TEXT("Basin stationary preparation: phase=%s sources=%llu nodes=%llu reaches=%llu prepare=%.6fms."),
				Progress.Phase,
				static_cast<unsigned long long>(RetainedSources.size()),
				static_cast<unsigned long long>(Progress.Nodes),
				static_cast<unsigned long long>(Progress.Reaches),
				StationaryMilliseconds
			)
		);
		// 既存の入口を保持した停滞点の開通に失敗した場合の返戻
		if(!TestTrue(TEXT("格子点へ移さず実停滞点を再構成"), Opened.has_value())) return false;
		for(const UnitDirection Point : RetainedSources) {
			const PlanetCore::RiverWaterHeights Water = Opened->Terrain.EvaluateWater(Point);
			if(Water.TerrainElevationMeters < Settings.SeaLevelMeters) continue;
			TestTrue(TEXT("実停滞点と全ての既存入口を通水"), Water.WaterSupport > 0.99 && Water.TerrainElevationMeters < Water.WaterElevationMeters);
		}
		const BasinDescentTrace Repeated = TraceBasinTerrain(Opened->Terrain, JoinedSink);
		InspectBasinStages(*this, TEXT("after outlet"), Opened->Terrain, *UnitDirection::FromVector(Repeated.Direction));
		TestTrue(TEXT("元の開始点から再び追跡した結果も有限"), Repeated.End != BasinDescentEnd::Invalid);
		const double SurveyStarted = FPlatformTime::Seconds();
		const BasinGridSurvey StationarySurvey = SurveyBasinGrid(*Holdout, Opened->Terrain, true, true, true, true);
		const double SurveyMilliseconds = (FPlatformTime::Seconds() - SurveyStarted) * 1000;
		TestTrue(TEXT("実停滞点の追加後も全標本が有限"), !StationarySurvey.InvalidSamples);
		TestEqual(TEXT("実停滞点の追加後も実河道内部に閉塞なし"), StationarySurvey.ObstructedRouteSamples, std::uint64_t{ 0 });
		const PlanetCore::RiverWaterHeights Former = Opened->Terrain.EvaluateWater(Stationary);
		AddInfo(
			FString::Printf(
				TEXT("Basin stationary outlet: formerTerrain=%.9fm formerWater=%.9fm formerSupport=%.9g ")
				TEXT("descentEnd=%u steps=%u distance=%.9fm final=%.9fm slope=%.12g ")
				TEXT("headDeficit=%.9fm unresolved=%llu survey=%.6fms. ")
				TEXT("One actual stationary point reused from the previous survey; no independent acceptance."),
				Former.TerrainElevationMeters,
				Former.WaterElevationMeters,
				Former.WaterSupport,
				static_cast<unsigned>(Repeated.End),
				Repeated.Steps,
				Repeated.DistanceMeters,
				Repeated.EndHeight,
				Repeated.EndSlope,
				StationarySurvey.MaximumHeadDeficitMeters,
				static_cast<unsigned long long>(StationarySurvey.HeadDeficitSites.size()),
				SurveyMilliseconds
			)
		);
	}
	// 合成の検査と未解決の排水・景観指標を区別した結果の返戻
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetBasinStationaryOrderContracts, "PlanetGenerator.BasinReconstruction.StationaryOrder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 実測した停滞点を固定入力から再現し，同じ細部起伏を排水の前後へ配置して比較する
 * @param Parameters 未使用の UE 検査引数
 * @return 再現・段階順・有限性・河川条件を満たす場合に真，全陸地の排水保証ではない
 */
bool FPlanetBasinStationaryOrderContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::GeologySettings;
	using PlanetCore::GeologyField;
	using PlanetCore::PlanetOrogenicRidges;
	using PlanetCore::OceanConnectivityField;
	using PlanetCore::PredrainageReliefMode;
	using PlanetCore::BuildUniformSphere;
	using PlanetCore::UnitDirection;
	using PlanetCore::TerrainSample;
	using PlanetCore::SampleLandRelief;
	using PlanetCore::LandReliefAmplitudeMeters;
	using PlanetCore::Vec3;
	using PlanetCore::RiverWaterHeights;
	using PlanetCore::RiverElevationStages;
	using PlanetCore::TerrainJet;
	using PlanetCore::SphereMesh;
	GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	const std::optional<GeologyField> PreparedGeology = GeologyField::Create(Settings);
	// 比較する地質を用意出来ない場合の返戻
	if(!TestTrue(TEXT("固定した地質を準備"), PreparedGeology.has_value())) return false;
	const auto Geology = std::make_shared<const GeologyField>(*PreparedGeology);
	const std::optional<PlanetOrogenicRidges> PreparedRidges = PlanetOrogenicRidges::Create(*Geology);
	// 比較する造山場を用意出来ない場合の返戻
	if(!TestTrue(TEXT("同じ造山場を準備"), PreparedRidges.has_value())) return false;
	const auto Ridges = std::make_shared<const PlanetOrogenicRidges>(*PreparedRidges);
	TestFalse(
		TEXT("未定義の起伏順序を拒否"),
		OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges, {}, static_cast<PredrainageReliefMode>(255)).has_value()
	);
	std::stop_source Cancel;
	Cancel.request_stop();
	TestFalse(
		TEXT("細部起伏を含む正規経路も中断に従う"),
		OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges, Cancel.get_token(), PredrainageReliefMode::MesoAndLand)
		.has_value()
	);
	const std::optional<SphereMesh> BaseMesh = BuildUniformSphere(6);
	// 再現入力の基準網を用意出来ない場合の返戻
	if(!TestTrue(TEXT("元の再現網を準備"), BaseMesh.has_value())) return false;
	const std::optional<BasinSurveyMesh> Fine = SubdivideBasinSurveyMesh(BaseMesh->Directions, BaseMesh->Triangles);
	// 第１細分化を用意出来ない場合の返戻
	if(!TestTrue(TEXT("再現網の第１細分化"), Fine.has_value())) return false;
	const std::optional<BasinSurveyMesh> Holdout = SubdivideBasinSurveyMesh(Fine->Directions, Fine->Triangles);
	// 第２細分化を用意出来ない場合の返戻
	if(!TestTrue(TEXT("再現網の第２細分化"), Holdout.has_value())) return false;
	std::vector<UnitDirection> Sources;
	for(const std::uint32_t Index : PlanetStationaryFixture::SourceSites) {
		// 入力番号が元の網から外れた場合の返戻
		if(!TestTrue(TEXT("保存した入口が範囲内"), Index < Holdout->Directions.size())) return false;
		Sources.push_back(Holdout->Directions[Index]);
	}
	Sources.push_back(*UnitDirection::FromVector(PlanetStationaryFixture::Stationary));
	const UnitDirection Start = *UnitDirection::FromVector(PlanetStationaryFixture::DescentStart);
	std::shared_ptr<const OceanConnectivityField> BaselineOcean;
	for(const PredrainageReliefMode Mode : { PredrainageReliefMode::MesoOnly, PredrainageReliefMode::MesoAndLand }) {
		const bool Early = Mode == PredrainageReliefMode::MesoAndLand;
		const std::optional<OceanConnectivityField> PreparedOcean =
		OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges, {}, Mode);
		// 指定した順序で海域を準備出来ない場合の返戻
		if(!TestTrue(TEXT("比較する海域を準備"), PreparedOcean.has_value())) return false;
		const auto Ocean = std::make_shared<const OceanConnectivityField>(*PreparedOcean);
		TestEqual(TEXT("指定した起伏順序を保持"), Ocean->HasPredrainageLandRelief(), Early);
		if(!Early) {
			BaselineOcean = Ocean;
			TestEqual(TEXT("既定の生成識別子を変更しない"), Ocean->DefinitionHash(), PlanetStationaryFixture::OceanHash);
			const std::optional<PlanetCore::DrainageNetwork> BasicDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
			// 盛土を持たない通常場の排水を用意出来ない場合の返戻
			if(!TestTrue(TEXT("通常場の排水も準備"), BasicDrainage.has_value())) return false;
			const std::optional<PlanetCore::RiverTerrainField> WithoutBasins =
			PlanetCore::RiverTerrainField::Create(Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(*BasicDrainage));
			// 盛土を含まない正規経路を準備出来ない場合の返戻
			if(!TestTrue(TEXT("盛土なしの通常場も保持"), WithoutBasins.has_value())) return false;
			TestFalse(TEXT("盛土なしの場を元補間として返さない"), WithoutBasins->EvaluateBasinReconstruction(Start).has_value());
		} else {
			TestTrue(TEXT("順序の変更を定義で識別"), Ocean->DefinitionHash() != BaselineOcean->DefinitionHash());
			TestEqual(TEXT("前置した起伏でも準備網の大洋は接続"), Ocean->Statistics().ComponentsAfter, std::uint32_t{ 1 });
			double MaximumDifference = 0, MaximumGradientError = 0;
			for(std::size_t Index = 0; Index < Holdout->Directions.size(); Index += 257) {
				const UnitDirection Point = Holdout->Directions[Index];
				const TerrainSample Sample = Ocean->Evaluate(Point);
				const double Before = BaselineOcean->EvaluateElevation(Point);
				TestEqual(TEXT("前置した起伏の値と微分経路が一致"), Sample.ElevationMeters, Ocean->EvaluateElevation(Point));
				if(Geology->EvaluateElevation(Point) <= Settings.SeaLevelMeters) {
					TestTrue(TEXT("天然の海底を陸上へ押上げない"), Sample.ElevationMeters <= Settings.SeaLevelMeters);
				}
				const TerrainJet<true> Detail =
				SampleLandRelief<true>(Point.Vector(), Settings.RadiusMeters, Settings.Seed ^ UINT64_C(0X5355424752494401));
				const TerrainJet<false> Value =
				SampleLandRelief<false>(Point.Vector(), Settings.RadiusMeters, Settings.Seed ^ UINT64_C(0X5355424752494401));
				TestEqual(TEXT("共有した起伏の値と微分経路が一致"), Detail.V, Value.V);
				TestTrue(TEXT("同じ５ mの振幅を保持"), std::abs(Detail.V) <= LandReliefAmplitudeMeters);
				MaximumDifference = std::max(MaximumDifference, std::abs(Sample.ElevationMeters - Before));
				const Vec3 P = Point.Vector(), Axis = std::abs(P.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
				const Vec3 A = UnitDirection::FromVector(P.Cross(Axis))->Vector(), B = P.Cross(A);
				for(const Vec3 Tangent : { A, B }) {
					constexpr double Step = 0.05;
					const double Angle = Step / Settings.RadiusMeters;
					const UnitDirection Plus = *UnitDirection::FromVector(P * std::cos(Angle) + Tangent * std::sin(Angle));
					const UnitDirection Minus = *UnitDirection::FromVector(P * std::cos(Angle) - Tangent * std::sin(Angle));
					const double Difference = (Ocean->EvaluateElevation(Plus) - Ocean->EvaluateElevation(Minus)) / (2 * Step);
					const double Analytic = Sample.TangentialGradientPerRadian.Dot(Tangent) / Settings.RadiusMeters;
					MaximumGradientError = std::max(MaximumGradientError, std::abs(Difference - Analytic));
				}
			}
			TestTrue(TEXT("同じ振幅の細部起伏を実際に前置"), MaximumDifference > 1);
			TestTrue(TEXT("前置した起伏の接線微分が独立差分と一致"), MaximumGradientError < 1E-5);
			AddInfo(
				FString::Printf(
					TEXT("Basin early relief field: maxDifference=%.12gm maxGradientError=%.12g hash=%llu."),
					MaximumDifference,
					MaximumGradientError,
					static_cast<unsigned long long>(Ocean->DefinitionHash())
				)
			);
		}
		BasinSamplingPreparation Progress;
		const double Started = FPlatformTime::Seconds();
		const std::optional<BasinSamplingCandidate> Prepared = PrepareSampledBasin(Ocean, Sources, Progress);
		const double Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
		AddInfo(
			FString::Printf(
				TEXT("Basin ordering preparation: early=%d phase=%s sources=%llu nodes=%llu reaches=%llu time=%.6fms."),
				Early,
				Progress.Phase,
				static_cast<unsigned long long>(Sources.size()),
				static_cast<unsigned long long>(Progress.Nodes),
				static_cast<unsigned long long>(Progress.Reaches),
				Milliseconds
			)
		);
		// 準備拒否を排水成功へ数えない返戻
		if(!TestTrue(TEXT("同じ入口を持つ比較地形を準備"), Prepared.has_value())) return false;
		for(const UnitDirection Point : Sources) {
			const RiverWaterHeights Water = Prepared->Terrain.EvaluateWater(Point);
			if(Water.TerrainElevationMeters < Settings.SeaLevelMeters) continue;
			TestTrue(TEXT("固定した全ての入口を通水"), Water.WaterSupport > 0.99 && Water.TerrainElevationMeters < Water.WaterElevationMeters);
		}
		const BasinDescentTrace Trace = TraceBasinTerrain(Prepared->Terrain, Start);
		TestTrue(TEXT("同じ開始点からの下降を有限に評価"), Trace.End != BasinDescentEnd::Invalid);
		if(!Early) {
			TestEqual(TEXT("元の未解決の停滞を再現"), static_cast<unsigned>(Trace.End), static_cast<unsigned>(BasinDescentEnd::Stalled));
			TestTrue(TEXT("式の共有化後も元の停滞標高を再現"), std::abs(Trace.EndHeight - 912.096765987) < 1E-6);
		} else for(std::size_t Index = 0; Index < Holdout->Directions.size(); Index += 257) {
			const RiverElevationStages Stages = Prepared->Terrain.EvaluateElevationStages(Holdout->Directions[Index]);
			TestEqual(TEXT("前置した起伏を排水後に二重加算しない"), Stages.AfterValleyMeters, Stages.AfterFineReliefMeters);
		}
		const UnitDirection End = *UnitDirection::FromVector(Trace.Direction);
		InspectBasinStages(*this, Early ? TEXT("early relief endpoint") : TEXT("late relief endpoint"), Prepared->Terrain, End);
		AddInfo(
			FString::Printf(
				TEXT("Basin ordering descent: early=%d end=%u steps=%u evaluations=%llu distance=%.9fm start=%.9fm final=%.9fm slope=%.12g ")
				TEXT("direction=(%.17g,%.17g,%.17g)."),
				Early,
				static_cast<unsigned>(Trace.End),
				Trace.Steps,
				static_cast<unsigned long long>(Trace.Evaluations),
				Trace.DistanceMeters,
				Trace.StartHeight,
				Trace.EndHeight,
				Trace.EndSlope,
				Trace.Direction.X,
				Trace.Direction.Y,
				Trace.Direction.Z
			)
		);
		if(Early) {
			const double SurveyStarted = FPlatformTime::Seconds();
			const BasinGridSurvey Survey = SurveyBasinGrid(*Holdout, Prepared->Terrain, true, true, true, true);
			TestEqual(TEXT("前置後の河道内部に閉塞なし"), Survey.ObstructedRouteSamples, std::uint64_t{ 0 });
			TestEqual(TEXT("前置後の独立標本は全て有限"), Survey.InvalidSamples, std::uint32_t{ 0 });
			const UnitDirection Worst = *UnitDirection::FromVector(Survey.MaximumHeadDeficitDirection);
			const BasinDescentTrace WorstTrace = TraceBasinTerrain(Prepared->Terrain, Worst);
			const BasinChannelDistance Nearest = FindBasinChannel(Prepared->Terrain, Worst);
			const UnitDirection Stalled = *UnitDirection::FromVector(WorstTrace.Direction);
			const std::optional<TerrainSample> Floor = Prepared->Terrain.EvaluateBasinReconstruction(Stalled);
			// 補間面の無い場を診断成功へ数えない返戻
			if(!TestTrue(TEXT("盛土の元補間を取得"), Floor.has_value())) return false;
			const TerrainSample Original = Ocean->Evaluate(Stalled);
			const double GradientProduct = Original.TangentialGradientPerRadian.Length() * Floor->TangentialGradientPerRadian.Length();
			const double Alignment =
			GradientProduct > 0 ? Original.TangentialGradientPerRadian.Dot(Floor->TangentialGradientPerRadian) / GradientProduct : 0;
			const auto SampleFloor = [&Prepared](const UnitDirection Point) -> TerrainSample {
				// 同じ不変な盛土準備から補間値と微分を直接取得する返戻
				return *Prepared->Terrain.EvaluateBasinReconstruction(Point);
			};
			const BasinDescentTrace FloorTrace = TraceBasinDescent(
				Stalled,
				Settings.RadiusMeters,
				Settings.SeaLevelMeters,
				SampleFloor,
				[&SampleFloor](const UnitDirection Point) -> double {
					// 同じ補間面の値の返戻
					return SampleFloor(Point).ElevationMeters;
				},
				[](const UnitDirection) -> bool {
					// 補間面は実河床ではない為，実河川へ到達したと数えない返戻
					return false;
				}
			);
			TestTrue(TEXT("元補間面の診断も有限"), FloorTrace.End != BasinDescentEnd::Invalid);
			AddInfo(
				FString::Printf(
					TEXT("Basin raw reconstruction: atActualEnd=%.9fm original=%.9fm rawSlope=%.12g alignment=%.12g ")
					TEXT("rawEnd=%u steps=%u evaluations=%llu distance=%.9fm final=%.9fm slope=%.12g ")
					TEXT("endpoint=(%.17g,%.17g,%.17g). Raw interpolant only; not actual terrain drainage."),
					Floor->ElevationMeters,
					Original.ElevationMeters,
					Floor->TangentialGradientPerRadian.Length() / Settings.RadiusMeters,
					Alignment,
					static_cast<unsigned>(FloorTrace.End),
					FloorTrace.Steps,
					static_cast<unsigned long long>(FloorTrace.Evaluations),
					FloorTrace.DistanceMeters,
					FloorTrace.EndHeight,
					FloorTrace.EndSlope,
					FloorTrace.Direction.X,
					FloorTrace.Direction.Y,
					FloorTrace.Direction.Z
				)
			);
			TestTrue(TEXT("残る最大不足地点からの下降も有限"), WorstTrace.End != BasinDescentEnd::Invalid);
			InspectBasinStages(*this, TEXT("early relief worst start"), Prepared->Terrain, Worst);
			InspectBasinStages(
				*this,
				TEXT("early relief worst endpoint"),
				Prepared->Terrain,
				*UnitDirection::FromVector(WorstTrace.Direction)
			);
			AddInfo(
				FString::Printf(
					TEXT("Basin ordering worst: direction=(%.17g,%.17g,%.17g) nearest=%.9fm end=%u steps=%u evaluations=%llu ")
					TEXT("distance=%.9fm start=%.9fm final=%.9fm slope=%.12g endpoint=(%.17g,%.17g,%.17g)."),
					Worst.Vector().X,
					Worst.Vector().Y,
					Worst.Vector().Z,
					Nearest.Meters,
					static_cast<unsigned>(WorstTrace.End),
					WorstTrace.Steps,
					static_cast<unsigned long long>(WorstTrace.Evaluations),
					WorstTrace.DistanceMeters,
					WorstTrace.StartHeight,
					WorstTrace.EndHeight,
					WorstTrace.EndSlope,
					WorstTrace.Direction.X,
					WorstTrace.Direction.Y,
					WorstTrace.Direction.Z
				)
			);
			AddInfo(
				FString::Printf(
					TEXT("Basin ordering survey: sites=%u headDeficit=%.9fm unresolved=%llu maximumSlope=%.12g ")
					TEXT("joined=%llu rejected=%llu time=%.6fms. ")
					TEXT("Diagnostic comparison; remaining basins and continuous drainage are separate acceptance conditions."),
					Survey.VisitedSites,
					Survey.MaximumHeadDeficitMeters,
					static_cast<unsigned long long>(Survey.HeadDeficitSites.size()),
					Survey.MaximumSlope,
					static_cast<unsigned long long>(Survey.JoinedRiverSites),
					static_cast<unsigned long long>(Survey.RejectedRiverJoins),
					(FPlatformTime::Seconds() - SurveyStarted) * 1000
				)
			);
		}
	}
	// 実行・再現の条件と未解決の排水指標を区別した返戻
	return !HasAnyErrors();
}

#endif
