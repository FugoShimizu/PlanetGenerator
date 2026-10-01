#include "Core/PlanetDrainage.h"
#include "Core/PlanetDrainageFit.h"
#include "Core/PlanetErosionPreparation.h"
#include "Core/PlanetOcean.h"
#include "Core/PlanetOrogenicRidges.h"
#include "Core/PlanetRiverTerrain.h"
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
#include <numeric>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

namespace {
	struct FineDrainageMesh {
		std::vector<PlanetCore::UnitDirection> Directions;
		std::vector<std::vector<std::uint32_t>> Neighbors;
	};

	struct ExcavationRoute {
		std::vector<std::uint32_t> Path;
		std::uint32_t Edges = 0;
		double LengthMeters = 0;
		double MaximumCutMeters = 0;
		double IntegratedCutSquareMeters = 0;
	};

	struct WaterProfileAudit {
		double MaximumBankCutMeters = 0;
		double IntegratedBankCutSquareMeters = 0;
		double MaximumWaterSlope = 0;
		double WaterDropMeters = 0;
	};

	struct MixedProfileAudit {
		double MaximumCutMeters = 0;
		double MaximumFillMeters = 0;
		double IntegratedCutSquareMeters = 0;
		double IntegratedFillSquareMeters = 0;
		double SquaredChange = 0;
		double CutOnlySquaredChange = 0;
		double SourceChangeMeters = 0;
		bool CutOnlyFeasible = false;
		double MinimumSlope = std::numeric_limits<double>::infinity();
	};

	struct ParetoWaterRoute {
		ExcavationRoute Route;
		WaterProfileAudit Profile;
		double SearchMilliseconds = 0;
		double MaximumTurnDegrees = 0;
		std::uint32_t LabelsCreated = 0;
		bool BudgetExceeded = false;
	};

	struct SmoothedRouteAudit {
		std::uint32_t Points = 0;
		std::uint32_t SubmergedIntermediate = 0;
		std::uint32_t FirstSubmergedIndex = std::numeric_limits<std::uint32_t>::max();
		double MaximumTurnDegrees = 0;
		double MaximumTerrainMeters = 0;
		bool FirstSubmergedSourceOcean = false;
		std::optional<WaterProfileAudit> Profile;
		std::optional<WaterProfileAudit> FirstOceanProfile;
		std::optional<MixedProfileAudit> MixedProfile;
	};

	struct ContinuousMinimaxRoute {
		double MaximumTerrainMeters = 0;
		double LengthMeters = 0;
		double SearchMilliseconds = 0;
		double MaximumTurnDegrees = 0;
		std::uint32_t Edges = 0;
		std::uint32_t ExpandedNodes = 0;
		std::uint32_t SampledPoints = 0;
		std::uint32_t ReconstructedPoints = 0;
		bool Found = false;
		bool BudgetExceeded = false;
		std::optional<WaterProfileAudit> Profile;
		std::optional<MixedProfileAudit> MixedProfile;
	};

	struct ContinuousWaterRoute {
		std::optional<WaterProfileAudit> Profile;
		std::optional<MixedProfileAudit> MixedProfile;
		double LengthMeters = 0;
		double SearchMilliseconds = 0;
		double MaximumTurnDegrees = 0;
		std::uint32_t LabelsCreated = 0;
		std::uint32_t ExpandedLabels = 0;
		std::uint32_t SampledPoints = 0;
		std::uint32_t Edges = 0;
		std::uint32_t ReconstructedPoints = 0;
		bool Found = false;
		bool BudgetExceeded = false;
	};

	struct FineDrainageStatistics {
		std::uint32_t WetNodes = 0;
		std::uint32_t WetComponents = 0;
		std::uint32_t IsolatedWetNodes = 0;
		std::uint32_t DryNodes = 0;
		std::uint32_t DryWithoutLowerNeighbor = 0;
		std::uint32_t DryWithoutOceanRoute = 0;
		std::uint32_t SteepestRouteTrapped = 0;
		std::uint32_t SubmergedSourceLand = 0;
		std::uint32_t RaisedSourceOcean = 0;
		std::uint32_t IsolatedWetIndex = std::numeric_limits<std::uint32_t>::max();
		std::uint32_t MaximumDirectSinkIndex = std::numeric_limits<std::uint32_t>::max();
		std::uint32_t TrappedSpillOver20Meters = 0;
		std::uint32_t TrappedSpillOver100Meters = 0;
		std::uint32_t EscapePathEdges = 0;
		double MaximumDirectSinkDepthMeters = 0;
		double MaximumTrappedSpillMeters = 0;
		double IsolatedWetDepthMeters = 0;
		double IsolatedWetBarrierMeters = 0;
		double EscapePathMeters = 0;
		double EscapeMaximumCutMeters = 0;
		double EscapeIntegratedCutSquareMeters = 0;
		std::optional<ExcavationRoute> BoundedEscape;
		std::optional<WaterProfileAudit> TreeWaterProfile;
		std::optional<WaterProfileAudit> BoundedWaterProfile;
		std::optional<WaterProfileAudit> SampledBestWaterProfile;
		std::optional<ExcavationRoute> SampledBestRoute;
		std::optional<ParetoWaterRoute> ParetoRoute;
		std::uint32_t SampledRouteCount = 0;
		std::uint32_t RejectedSampledRoutes = 0;
		double SampledBestReferenceFraction = 0;
	};

	/**
	 * 排水網の球面三角形を一段だけ細分し，共有辺の中点を一度だけ作る
	 * @param Coarse 排水の固定球面網
	 * @return 元の頂点を保持した細分網，不正な辺又は方向では空
	 * 計算量：O(V + T + Σ d log d)，V は頂点数，T は面数，d は各点の隣接数
	 */
	std::optional<FineDrainageMesh> RefineDrainageMesh(const PlanetCore::SphereMesh &Coarse) {
		// 添字上限を越える元網を細分しない場合の返戻
		if(Coarse.Directions.size() > std::numeric_limits<std::uint32_t>::max()) return {};
		FineDrainageMesh Fine;
		Fine.Directions = Coarse.Directions;
		Fine.Neighbors.resize(Fine.Directions.size());
		std::unordered_map<std::uint64_t, std::uint32_t> Midpoints;
		Midpoints.reserve(Coarse.Triangles.size() * 3 / 2);
		// 共有辺の中点の一度だけの作成と再利用
		const auto Midpoint = [&Fine, &Midpoints](const std::uint32_t A, const std::uint32_t B) -> std::optional<std::uint32_t> {
			// 不正又は同一の端点で中点を作らない場合の返戻
			if(A >= Fine.Directions.size() || B >= Fine.Directions.size() || A == B) return {};
			const std::uint64_t Key = static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
			const std::unordered_map<std::uint64_t, std::uint32_t>::const_iterator Found = Midpoints.find(Key);
			// 既に作った共有中点を再利用する場合の返戻
			if(Found != Midpoints.end()) return Found->second;
			// 中点追加で添字上限を越えない場合の返戻
			if(Fine.Directions.size() >= std::numeric_limits<std::uint32_t>::max()) return {};
			const std::optional<PlanetCore::UnitDirection> Direction =
			PlanetCore::UnitDirection::FromVector(Fine.Directions[A].Vector() + Fine.Directions[B].Vector());
			// 有効な球面方向を構成出来ない場合の返戻
			if(!Direction) return {};
			const std::uint32_t Index = static_cast<std::uint32_t>(Fine.Directions.size());
			Fine.Directions.push_back(*Direction);
			Fine.Neighbors.emplace_back();
			Midpoints.emplace(Key, Index);
			// 共有辺に一度だけ追加した中点番号の返戻
			return Index;
		};
		// 細分辺の双方向隣接の追加
		const auto Connect = [&Fine](const std::uint32_t A, const std::uint32_t B) {
			Fine.Neighbors[A].push_back(B);
			Fine.Neighbors[B].push_back(A);
		};
		for(const std::array<std::uint32_t, 3> &Triangle : Coarse.Triangles) {
			const std::uint32_t A = Triangle[0], B = Triangle[1], C = Triangle[2];
			const std::optional<std::uint32_t> AB = Midpoint(A, B), BC = Midpoint(B, C), CA = Midpoint(C, A);
			// 三辺の共有中点を準備出来ない場合の返戻
			if(!AB || !BC || !CA) return {};
			Connect(A, *AB);
			Connect(*AB, B);
			Connect(B, *BC);
			Connect(*BC, C);
			Connect(C, *CA);
			Connect(*CA, A);
			Connect(*AB, *BC);
			Connect(*BC, *CA);
			Connect(*CA, *AB);
		}
		for(std::vector<std::uint32_t> &Adjacent : Fine.Neighbors) {
			std::sort(Adjacent.begin(), Adjacent.end());
			Adjacent.erase(std::unique(Adjacent.begin(), Adjacent.end()), Adjacent.end());
		}
		// 共有中点と隣接関係を揃えた細分網の返戻
		return Fine;
	}

	/**
	 * 最小越流高を越えない辺から，河源水位に対する開削量の積分が最小の出口を探す
	 * @param Mesh 共有中点を持つ球面網
	 * @param Heights 最終標高
	 * @param Component 水没領域の成分番号
	 * @param Main 大洋の成分番号
	 * @param Sink 河源とする閉鎖地点
	 * @param MaximumHeight 最小越流高
	 * @param CostReferenceWater 加算する開削費用の基準水位
	 * @param Radius 惑星半径
	 * @return 積分開削量を優先した大洋出口経路，入力不正又は経路無しなら空
	 * 計算量：O((V + E) log(V + E))，V は頂点数，E は辺数
	 */
	std::optional<ExcavationRoute> FindBoundedExcavationRoute(
		const FineDrainageMesh &Mesh,
		const std::vector<double> &Heights,
		const std::vector<std::uint32_t> &Component,
		const std::uint32_t Main,
		const std::uint32_t Sink,
		const double MaximumHeight,
		const double CostReferenceWater,
		const double Radius
	) {
		const std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		const bool HasInvalidInput =
		Sink >= Heights.size() || Mesh.Directions.size() != Heights.size() || Component.size() != Heights.size() ||
		!std::isfinite(MaximumHeight) || !std::isfinite(CostReferenceWater) || !std::isfinite(Radius) || Radius <= 0 ||
		Heights[Sink] > MaximumHeight;
		// 入力網・標高・尺度又は出口条件が不整合な場合の返戻
		if(HasInvalidInput) return {};
		using Entry = std::tuple<double, double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		std::vector<double> BestCut(Heights.size(), std::numeric_limits<double>::infinity());
		std::vector<double> BestLength(Heights.size(), std::numeric_limits<double>::infinity());
		std::vector<std::uint32_t> Parent(Heights.size(), Missing);
		BestCut[Sink] = BestLength[Sink] = 0;
		Parent[Sink] = Sink;
		Frontier.emplace(0, 0, Sink);
		std::uint32_t Outlet = Missing;
		while(!Frontier.empty()) {
			const Entry QueueEntry = Frontier.top();
			const double Cost = std::get<0>(QueueEntry), Length = std::get<1>(QueueEntry);
			const std::uint32_t Index = std::get<2>(QueueEntry);
			Frontier.pop();
			if(Cost != BestCut[Index] || Length != BestLength[Index]) continue;
			if(Component[Index] == Main) {
				Outlet = Index;
				break;
			}
			for(const std::uint32_t Neighbor : Mesh.Neighbors[Index]) {
				if(Heights[Neighbor] > MaximumHeight + 1E-9) continue;
				const double Cosine = std::clamp(Mesh.Directions[Index].Vector().Dot(Mesh.Directions[Neighbor].Vector()), -1.0, 1.0);
				const double EdgeMeters = Radius * std::acos(Cosine);
				// 非有限又は非正の辺長を採用しない場合の返戻
				if(!std::isfinite(EdgeMeters) || EdgeMeters <= 0) return {};
				const double Cut = std::max(0.0, Heights[Neighbor] - CostReferenceWater), CandidateCut = Cost + Cut * EdgeMeters;
				const double CandidateLength = Length + EdgeMeters;
				if(CandidateCut > BestCut[Neighbor] || CandidateCut == BestCut[Neighbor] && CandidateLength >= BestLength[Neighbor]) continue;
				BestCut[Neighbor] = CandidateCut;
				BestLength[Neighbor] = CandidateLength;
				Parent[Neighbor] = Index;
				Frontier.emplace(CandidateCut, CandidateLength, Neighbor);
			}
		}
		// 大洋出口が見付からない場合の返戻
		if(Outlet == Missing) return {};
		std::vector<std::uint32_t> ReversePath;
		for(std::uint32_t Current = Outlet; Current != Sink; Current = Parent[Current]) {
			// 欠落又は循環した親列を復元しない場合の返戻
			if(Current == Missing || ReversePath.size() >= Heights.size()) return {};
			ReversePath.push_back(Current);
		}
		ExcavationRoute Result;
		Result.Path.push_back(Sink);
		std::uint32_t Current = Sink;
		for(std::vector<std::uint32_t>::reverse_iterator It = ReversePath.rbegin(); It != ReversePath.rend(); ++It) {
			const double Cosine = std::clamp(Mesh.Directions[Current].Vector().Dot(Mesh.Directions[*It].Vector()), -1.0, 1.0);
			const double EdgeMeters = Radius * std::acos(Cosine);
			Result.LengthMeters += EdgeMeters;
			++Result.Edges;
			const double Limit = Heights[Sink] - 1E-7 * Result.LengthMeters, Cut = std::max(0.0, Heights[*It] - Limit);
			Result.MaximumCutMeters = std::max(Result.MaximumCutMeters, Cut);
			Result.IntegratedCutSquareMeters += Cut * EdgeMeters;
			Result.Path.push_back(*It);
			Current = *It;
		}
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}

	/**
	 * 河口で零へ収束する解析用の河岸余裕高
	 * @param Height 河岸の標高
	 * @param Sea 固定海面
	 * @return 河岸余裕高
	 */
	double DiagnosticFreeboard(const double Height, const double Sea) {
		return 0.3 * std::clamp((Height - Sea) / 2.0, 0.0, 1.0);
	}

	/**
	 * 河源から大洋まで下流へ厳密に下がる水位を，地面以下の上限で構築して開削を測る
	 * @param Mesh 球面方向の並び
	 * @param Heights 地表の標高
	 * @param Path 河源から最初の大洋地点までの経路
	 * @param Sea 球状海面の標高
	 * @param Radius 惑星半径
	 * @return 単調な水位と河岸開削量，接続出来なければ空
	 * 計算量：O(P)，P は経路の地点数
	 */
	std::optional<WaterProfileAudit> AuditDescendingWaterProfile(
		const FineDrainageMesh &Mesh,
		const std::vector<double> &Heights,
		const std::vector<std::uint32_t> &Path,
		const double Sea,
		const double Radius
	) {
		const bool HasInvalidPath =
		Path.size() < 2 || Mesh.Directions.size() != Heights.size() || !std::isfinite(Radius) || Radius <= 0 ||
		Path.front() >= Heights.size() || Path.back() >= Heights.size() || Heights[Path.front()] <= Sea || Heights[Path.back()] > Sea;
		// 経路又は分割条件が不正な入力を評価しない場合の返戻
		if(HasInvalidPath) return {};
		const double SourceWater = Heights[Path.front()] - DiagnosticFreeboard(Heights[Path.front()], Sea);
		double PreviousWater = SourceWater;
		WaterProfileAudit Result;
		for(std::size_t Step = 1; Step < Path.size(); ++Step) {
			const std::uint32_t A = Path[Step - 1], B = Path[Step];
			// 標高の無い端点を縦断へ含めない場合の返戻
			if(A >= Heights.size() || B >= Heights.size()) return {};
			const double Cosine = std::clamp(Mesh.Directions[A].Vector().Dot(Mesh.Directions[B].Vector()), -1.0, 1.0);
			const double EdgeMeters = Radius * std::acos(Cosine);
			// 非有限又は非正の辺長を採用しない場合の返戻
			if(!std::isfinite(EdgeMeters) || EdgeMeters <= 0) return {};
			const bool Mouth = Step + 1 == Path.size();
			const double BankFreeboard = DiagnosticFreeboard(Heights[B], Sea);
			const double Water = Mouth ? Sea : std::min(PreviousWater - 1E-7 * EdgeMeters, Heights[B] - BankFreeboard);
			// 非有限又は下降しない水位を採用しない場合の返戻
			if(!std::isfinite(Water) || !(Water < PreviousWater) || !Mouth && Water <= Sea) return {};
			const double BankCut = Mouth ? 0 : std::max(0.0, Heights[B] - BankFreeboard - Water);
			Result.MaximumBankCutMeters = std::max(Result.MaximumBankCutMeters, BankCut);
			Result.IntegratedBankCutSquareMeters += BankCut * EdgeMeters;
			Result.MaximumWaterSlope = std::max(Result.MaximumWaterSlope, (PreviousWater - Water) / EdgeMeters);
			PreviousWater = Water;
		}
		Result.WaterDropMeters = SourceWater - Sea;
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}

	/**
	 * 重み付き二乗変更を最小にする非増加列を PAVA で求める
	 * 計算量：O(n)，n は縦断標本数
	 * @param Targets 最低勾配を距離分だけ加えた目標水位
	 * @param Weights 正の長さ重み
	 * @param Lower 海口の変換後水位で定まる共通下限
	 * @return 下限以上の非増加列，不正又は非有限な入力では空
	 */
	std::optional<std::vector<double>> FitMixedProfile(
		const std::vector<double> &Targets,
		const std::vector<double> &Weights,
		const double Lower
	) {
		// 未定義又は不整合な縦断を拒否する返戻
		if(Targets.empty() || Targets.size() != Weights.size() || !std::isfinite(Lower)) return {};
		struct Block {
			std::size_t Begin = 0;
			std::size_t End = 0;
			double Weight = 0;
			double Moment = 0;
		};
		std::vector<Block> Blocks;
		Blocks.reserve(Targets.size());
		for(std::size_t Index = 0; Index < Targets.size(); ++Index) {
			// 非有限値や非正の費用重みを拒否する返戻
			if(!std::isfinite(Targets[Index]) || !std::isfinite(Weights[Index]) || !(Weights[Index] > 0)) return {};
			Blocks.push_back({ Index, Index + 1, Weights[Index], Weights[Index] * Targets[Index] });
			// 積の桁溢れを拒否する返戻
			if(!std::isfinite(Blocks.back().Moment)) return {};
			while(Blocks.size() > 1) {
				const Block Last = Blocks.back();
				Block &Previous = Blocks[Blocks.size() - 2];
				if(Previous.Moment / Previous.Weight >= Last.Moment / Last.Weight) break;
				Previous.End = Last.End;
				Previous.Weight += Last.Weight;
				Previous.Moment += Last.Moment;
				// 統合費用を有限に保持出来ない場合の返戻
				if(!std::isfinite(Previous.Weight) || !std::isfinite(Previous.Moment)) return {};
				Blocks.pop_back();
			}
		}
		std::vector<double> Result(Targets.size());
		for(const Block &Group : Blocks) {
			const double Level = std::max(Lower, Group.Moment / Group.Weight);
			std::fill(Result.begin() + Group.Begin, Result.begin() + Group.End, Level);
		}
		// 固定した海口下限を守る最小二乗の変換後水位の返戻
		return Result;
	}

	/**
	 * 同じ標本経路に対する開削と盛土の同時配分を測る
	 * 計算量：O(n)，n は河口を含む経路の標本数
	 * @param Directions 水源から最初の海口までの球面方向
	 * @param Heights 各方向の最終地形標高
	 * @param Sea 固定海面
	 * @param Radius 距離へ変換する惑星半径
	 * @return 下流水位と変更量の診断，海口や最低勾配を満たせない場合は空
	 */
	std::optional<MixedProfileAudit> AuditMixedProfile(
		const std::vector<PlanetCore::UnitDirection> &Directions,
		const std::vector<double> &Heights,
		const double Sea,
		const double Radius
	) {
		// 不整合又は海口を持たない経路を拒否する返戻
		const bool HasInvalidProfile =
		Directions.size() < 2 || Directions.size() != Heights.size() || !std::isfinite(Sea) || !std::isfinite(Radius) || Radius <= 0 ||
		!std::isfinite(Heights.back()) || Heights.back() > Sea;
		// 入力網・標高・尺度又は出口条件が不整合な場合の返戻
		if(HasInvalidProfile) return {};
		const std::size_t LandCount = Heights.size() - 1;
		constexpr double Grade = 1E-7;
		std::vector<double> Distance(Heights.size()), Edges(Heights.size());
		for(std::size_t Index = 1; Index < Heights.size(); ++Index) {
			const double Cosine = std::clamp(Directions[Index - 1].Vector().Dot(Directions[Index].Vector()), -1.0, 1.0);
			Edges[Index] = Radius * std::acos(Cosine);
			// 退化した経路区間を拒否する返戻
			if(!std::isfinite(Edges[Index]) || !(Edges[Index] > 0)) return {};
			Distance[Index] = Distance[Index - 1] + Edges[Index];
		}
		std::vector<double> Targets(LandCount), Weights(LandCount), OriginalWater(LandCount);
		for(std::size_t Index = 0; Index < LandCount; ++Index) {
			// 海を通過して陸へ戻る経路を河口までの単一路と誤認しない返戻
			if(!std::isfinite(Heights[Index]) || Heights[Index] <= Sea) return {};
			OriginalWater[Index] = Heights[Index] - DiagnosticFreeboard(Heights[Index], Sea);
			Targets[Index] = OriginalWater[Index] + Grade * Distance[Index];
			Weights[Index] = (Edges[Index] + Edges[Index + 1]) * 0.5;
		}
		const std::optional<std::vector<double>> Levels = FitMixedProfile(Targets, Weights, Sea + Grade * Distance.back());
		// 変換後水位を解けない場合の返戻
		if(!Levels) return {};
		MixedProfileAudit Result;
		double Previous = 0, CutOnly = OriginalWater.front();
		for(std::size_t Index = 0; Index < LandCount + 1; ++Index) {
			const double Water = Index == LandCount ? Sea : (*Levels)[Index] - Grade * Distance[Index];
			if(Index) {
				// 海口を含む全区間が下流へ下がらない場合の返戻
				if(!(Water < Previous)) return {};
				Result.MinimumSlope = std::min(Result.MinimumSlope, (Previous - Water) / Edges[Index]);
			}
			Previous = Water;
			if(Index == LandCount) continue;
			if(Index) CutOnly = std::min(CutOnly - Grade * Edges[Index], OriginalWater[Index]);
			const double Change = Water - OriginalWater[Index], Cut = std::max(0.0, -Change), Fill = std::max(0.0, Change);
			Result.MaximumCutMeters = std::max(Result.MaximumCutMeters, Cut);
			Result.MaximumFillMeters = std::max(Result.MaximumFillMeters, Fill);
			Result.IntegratedCutSquareMeters += Cut * Weights[Index];
			Result.IntegratedFillSquareMeters += Fill * Weights[Index];
			Result.SquaredChange += Change * Change * Weights[Index];
			const double CutOnlyChange = CutOnly - OriginalWater[Index];
			Result.CutOnlySquaredChange += CutOnlyChange * CutOnlyChange * Weights[Index];
			if(!Index) Result.SourceChangeMeters = Change;
		}
		Result.CutOnlyFeasible = CutOnly >= Sea + Grade * Edges.back();
		// 河幅や盆地面積を含まない縦断だけの変更量の返戻
		return Result;
	}

	/**
	 * 地点毎に費用・水位・経路長の非劣解を保持し，実際の下降水位で低開削の出口を探す
	 * @param Mesh 細分球面網
	 * @param Heights 最終標高
	 * @param Component 水没成分
	 * @param Main 大洋成分
	 * @param Sink 河源
	 * @param Sea 球状海面の標高
	 * @param MaximumHeight 最小越流経路から得た標高上限
	 * @param MaximumBankCut 既知の経路を越えない最大河岸開削
	 * @param Radius 惑星半径
	 * @return 実水位に基づく候補，資源上限到達時は其の状態，経路無しなら空
	 * 計算量：O(L² + Q log Q + P)，L は状態数，Q は隣接候補数，P は復元標本数
	 */
	std::optional<ParetoWaterRoute> FindParetoWaterRoute(
		const FineDrainageMesh &Mesh,
		const std::vector<double> &Heights,
		const std::vector<std::uint32_t> &Component,
		const std::uint32_t Main,
		const std::uint32_t Sink,
		const double Sea,
		const double MaximumHeight,
		const double MaximumBankCut,
		const double Radius
	) {
		constexpr std::uint32_t LabelBudget = 250000;
		constexpr double MaximumRouteMeters = 250000;
		const bool HasInvalidSource =
		Sink >= Heights.size() || Mesh.Directions.size() != Heights.size() || Component.size() != Heights.size() ||
		!std::isfinite(MaximumHeight) || !std::isfinite(MaximumBankCut) || MaximumBankCut < 0 || !std::isfinite(Radius) ||
		Radius <= 0 || Heights[Sink] <= Sea;
		// 入力網・標高・尺度又は出口条件が不整合な場合の返戻
		if(HasInvalidSource) return {};
		const double SearchStarted = FPlatformTime::Seconds();
		struct Label {
			std::uint32_t Node = 0;
			std::uint32_t Parent = 0;
			double Water = 0;
			double Cost = 0;
			double Length = 0;
			bool Active = true;
		};
		std::vector<Label> Labels;
		Labels.reserve(4096);
		Labels.push_back({ Sink, 0, Heights[Sink] - DiagnosticFreeboard(Heights[Sink], Sea), 0, 0, true });
		std::vector<std::vector<std::uint32_t>> ByNode(Heights.size());
		ByNode[Sink].push_back(0);
		using Entry = std::tuple<double, double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		Frontier.emplace(0, 0, 0);
		const PlanetCore::Vec3 SourceDirection = Mesh.Directions[Sink].Vector();
		std::uint32_t OutletLabel = std::numeric_limits<std::uint32_t>::max();
		while(!Frontier.empty()) {
			const Entry QueueEntry = Frontier.top();
			const double Cost = std::get<0>(QueueEntry), Length = std::get<1>(QueueEntry);
			const std::uint32_t LabelIndex = std::get<2>(QueueEntry);
			Frontier.pop();
			if(!Labels[LabelIndex].Active) continue;
			const Label Current = Labels[LabelIndex];
			// 待機候補と保持状態が不一致の場合に続行しない場合の返戻
			if(Cost != Current.Cost || Length != Current.Length) return {};
			if(Component[Current.Node] == Main) {
				OutletLabel = LabelIndex;
				break;
			}
			for(const std::uint32_t Neighbor : Mesh.Neighbors[Current.Node]) {
				if(Heights[Neighbor] > MaximumHeight + 1E-9) continue;
				const PlanetCore::Vec3 NeighborDirection = Mesh.Directions[Neighbor].Vector();
				const double SourceAngle = std::acos(std::clamp(SourceDirection.Dot(NeighborDirection), -1.0, 1.0));
				if(Radius * SourceAngle > MaximumRouteMeters) continue;
				const double EdgeAngle = std::acos(std::clamp(Mesh.Directions[Current.Node].Vector().Dot(NeighborDirection), -1.0, 1.0));
				const double EdgeMeters = Radius * EdgeAngle, CandidateLength = Current.Length + EdgeMeters;
				if(!std::isfinite(EdgeMeters) || EdgeMeters <= 0 || CandidateLength > MaximumRouteMeters) continue;
				const bool Mouth = Component[Neighbor] == Main;
				const double Freeboard = DiagnosticFreeboard(Heights[Neighbor], Sea);
				const double Water = Mouth ? Sea : std::min(Current.Water - 1E-7 * EdgeMeters, Heights[Neighbor] - Freeboard);
				if(!std::isfinite(Water) || !(Water < Current.Water) || !Mouth && Water <= Sea) continue;
				const double BankCut = Mouth ? 0 : std::max(0.0, Heights[Neighbor] - Freeboard - Water);
				if(BankCut > MaximumBankCut + 1E-6) continue;
				const double CandidateCost = Current.Cost + BankCut * EdgeMeters;
				std::vector<std::uint32_t> &Existing = ByNode[Neighbor];
				bool Dominated = false;
				for(const std::uint32_t Index : Existing) {
					const Label &Old = Labels[Index];
					if(Old.Active && Old.Cost <= CandidateCost && Old.Water >= Water && Old.Length <= CandidateLength) {
						Dominated = true;
						break;
					}
				}
				if(Dominated) continue;
				for(const std::uint32_t Index : Existing) {
					Label &Old = Labels[Index];
					if(Old.Active && CandidateCost <= Old.Cost && Water >= Old.Water && CandidateLength <= Old.Length) Old.Active = false;
				}
				if(Labels.size() >= LabelBudget) {
					ParetoWaterRoute Exhausted;
					Exhausted.LabelsCreated = static_cast<std::uint32_t>(Labels.size());
					Exhausted.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
					Exhausted.BudgetExceeded = true;
					// 上限到達を成功経路と区別した診断の返戻
					return Exhausted;
				}
				const std::uint32_t NextLabel = static_cast<std::uint32_t>(Labels.size());
				Labels.push_back({ Neighbor, LabelIndex, Water, CandidateCost, CandidateLength, true });
				Existing.push_back(NextLabel);
				Frontier.emplace(CandidateCost, CandidateLength, NextLabel);
			}
		}
		// 水位条件を満たす大洋出口が無い場合の返戻
		if(OutletLabel == std::numeric_limits<std::uint32_t>::max()) return {};
		ParetoWaterRoute Result;
		Result.LabelsCreated = static_cast<std::uint32_t>(Labels.size());
		for(std::uint32_t Index = OutletLabel;; Index = Labels[Index].Parent) {
			// 状態の循環で経路を復元出来ない場合の返戻
			if(Result.Route.Path.size() >= Labels.size()) return {};
			Result.Route.Path.push_back(Labels[Index].Node);
			if(!Index) break;
		}
		std::reverse(Result.Route.Path.begin(), Result.Route.Path.end());
		const std::optional<WaterProfileAudit> Profile = AuditDescendingWaterProfile(Mesh, Heights, Result.Route.Path, Sea, Radius);
		// 復元した縦断の費用が探索結果と一致しない場合の返戻
		if(!Profile || std::abs(Profile->IntegratedBankCutSquareMeters - Labels[OutletLabel].Cost) > 0.001) return {};
		Result.Profile = *Profile;
		Result.Route.Edges = static_cast<std::uint32_t>(Result.Route.Path.size() - 1);
		Result.Route.LengthMeters = Labels[OutletLabel].Length;
		Result.Route.MaximumCutMeters = Profile->MaximumBankCutMeters;
		Result.Route.IntegratedCutSquareMeters = Profile->IntegratedBankCutSquareMeters;
		for(std::size_t Step = 1; Step + 1 < Result.Route.Path.size(); ++Step) {
			const PlanetCore::Vec3 A = Mesh.Directions[Result.Route.Path[Step - 1]].Vector();
			const PlanetCore::Vec3 B = Mesh.Directions[Result.Route.Path[Step]].Vector();
			const PlanetCore::Vec3 C = Mesh.Directions[Result.Route.Path[Step + 1]].Vector();
			const PlanetCore::Vec3 Incoming = B - A - B * (B - A).Dot(B);
			const PlanetCore::Vec3 Outgoing = C - B - B * (C - B).Dot(B);
			const double Product = Incoming.Length() * Outgoing.Length();
			// 非有限又は退化した接線で曲率を測らない場合の返戻
			if(!std::isfinite(Product) || Product <= 0) return {};
			const double Cosine = std::clamp(Incoming.Dot(Outgoing) / Product, -1.0, 1.0);
			const double Turn = std::acos(Cosine) * 180.0 / std::acos(-1.0);
			// 非有限な曲がり角を測定へ含めない場合の返戻
			if(!std::isfinite(Turn)) return {};
			Result.MaximumTurnDegrees = std::max(Result.MaximumTurnDegrees, Turn);
		}
		Result.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}

	/**
	 * 球面上の候補曲線を実地形で採点し，下降水位の成立も検査する
	 * @param Directions 水源から大洋までの評価方向
	 * @param Terrain 最終地形の地点評価場
	 * @param Ocean 元の大洋接続場
	 * @param Sea 球状海面の標高
	 * @param Radius 球半径
	 * @return 地形と水位の診断値，不正な評価なら空
	 * 計算量：O(P)，P は方向列の長さ；各地点で元場を評価する
	 */
	std::optional<SmoothedRouteAudit> AuditSampledDirections(
		std::vector<PlanetCore::UnitDirection> Directions,
		const PlanetCore::RiverTerrainField &Terrain,
		const PlanetCore::OceanConnectivityField &Ocean,
		const double Sea,
		const double Radius
	) {
		FineDrainageMesh Diagnostic;
		Diagnostic.Directions = std::move(Directions);
		std::vector<double> Heights(Diagnostic.Directions.size());
		std::vector<std::uint32_t> Indices(Heights.size());
		std::iota(Indices.begin(), Indices.end(), 0U);
		SmoothedRouteAudit Result;
		Result.Points = static_cast<std::uint32_t>(Heights.size());
		Result.MaximumTerrainMeters = -std::numeric_limits<double>::infinity();
		for(std::size_t Step = 0; Step < Heights.size(); ++Step) {
			Heights[Step] = Terrain.EvaluateElevation(Diagnostic.Directions[Step]);
			// 非有限な地点標高を縦断へ含めない場合の返戻
			if(!std::isfinite(Heights[Step])) return {};
			Result.MaximumTerrainMeters = std::max(Result.MaximumTerrainMeters, Heights[Step]);
			if(Step && Step + 1 < Heights.size() && Heights[Step] <= Sea) {
				++Result.SubmergedIntermediate;
				if(Result.FirstSubmergedIndex == std::numeric_limits<std::uint32_t>::max()) {
					Result.FirstSubmergedIndex = static_cast<std::uint32_t>(Step);
					Result.FirstSubmergedSourceOcean = Ocean.EvaluateElevation(Diagnostic.Directions[Step]) <= Sea;
				}
			}
		}
		for(std::size_t Step = 1; Step + 1 < Heights.size(); ++Step) {
			const PlanetCore::Vec3 A = Diagnostic.Directions[Step - 1].Vector();
			const PlanetCore::Vec3 B = Diagnostic.Directions[Step].Vector();
			const PlanetCore::Vec3 C = Diagnostic.Directions[Step + 1].Vector();
			const PlanetCore::Vec3 Incoming = B - A - B * (B - A).Dot(B);
			const PlanetCore::Vec3 Outgoing = C - B - B * (C - B).Dot(B);
			const double Product = Incoming.Length() * Outgoing.Length();
			// 非有限又は退化した接線で曲率を測らない場合の返戻
			if(!std::isfinite(Product) || Product <= 0) return {};
			const double Turn = std::acos(std::clamp(Incoming.Dot(Outgoing) / Product, -1.0, 1.0)) * 180.0 / std::acos(-1.0);
			// 非有限な曲がり角を測定へ含めない場合の返戻
			if(!std::isfinite(Turn)) return {};
			Result.MaximumTurnDegrees = std::max(Result.MaximumTurnDegrees, Turn);
		}
		Result.Profile = AuditDescendingWaterProfile(Diagnostic, Heights, Indices, Sea, Radius);
		Result.MixedProfile = AuditMixedProfile(Diagnostic.Directions, Heights, Sea, Radius);
		if(Result.FirstSubmergedSourceOcean) {
			const std::vector<std::uint32_t> Prefix(Indices.begin(), Indices.begin() + Result.FirstSubmergedIndex + 1);
			Result.FirstOceanProfile = AuditDescendingWaterProfile(Diagnostic, Heights, Prefix, Sea, Radius);
		}
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}

	/**
	 * 離散出口経路の角を球面上で丸め，実際の最終地形と下降水位を測る
	 * @param Mesh 元の球面網
	 * @param Path 水源から大洋までの格子点
	 * @param Terrain 最終地形の地点評価場
	 * @param Ocean 元の大洋接続場
	 * @param Sea 球状海面の標高
	 * @param Radius 球半径
	 * @param Passes Chaikin 角丸めの回数
	 * @return 滑らかにした経路の診断値，補間不能なら空
	 * 計算量：O(P 2ᴷ)，P は元経路の地点数，K は角丸め回数
	 */
	std::optional<SmoothedRouteAudit> AuditSmoothedRoute(
		const FineDrainageMesh &Mesh,
		const std::vector<std::uint32_t> &Path,
		const PlanetCore::RiverTerrainField &Terrain,
		const PlanetCore::OceanConnectivityField &Ocean,
		const double Sea,
		const double Radius,
		const std::uint32_t Passes
	) {
		// 経路又は分割条件が不正な入力を評価しない場合の返戻
		if(Path.size() < 2 || !Passes || Passes > 3) return {};
		std::vector<PlanetCore::UnitDirection> Directions;
		Directions.reserve(Path.size());
		for(const std::uint32_t Index : Path) {
			// 元網の範囲外の方向で経路を作らない場合の返戻
			if(Index >= Mesh.Directions.size()) return {};
			Directions.push_back(Mesh.Directions[Index]);
		}
		for(std::uint32_t Pass = 0; Pass < Passes; ++Pass) {
			std::vector<PlanetCore::UnitDirection> Rounded;
			Rounded.reserve(Directions.size() * 2);
			Rounded.push_back(Directions.front());
			for(std::size_t Step = 1; Step < Directions.size(); ++Step) {
				const PlanetCore::Vec3 A = Directions[Step - 1].Vector();
				const PlanetCore::Vec3 B = Directions[Step].Vector();
				const std::optional<PlanetCore::UnitDirection> First = PlanetCore::UnitDirection::FromVector(A * 0.75 + B * 0.25);
				const std::optional<PlanetCore::UnitDirection> Second = PlanetCore::UnitDirection::FromVector(A * 0.25 + B * 0.75);
				// 球面の角丸め方向を準備出来ない場合の返戻
				if(!First || !Second) return {};
				Rounded.push_back(*First);
				Rounded.push_back(*Second);
			}
			Rounded.push_back(Directions.back());
			Directions = std::move(Rounded);
		}
		// 連続方向列を独立に測った縦断監査の返戻
		return AuditSampledDirections(std::move(Directions), Terrain, Ocean, Sea, Radius);
	}

	/**
	 * 元の格子辺の内部を補間し，節点だけの越流評価が妥当か測る
	 * @param Mesh 元の球面網
	 * @param Path 水源から大洋までの格子点
	 * @param Terrain 最終地形の地点評価場
	 * @param Ocean 元の大洋接続場
	 * @param Sea 球状海面の標高
	 * @param Radius 球半径
	 * @param SegmentsPerEdge 一辺あたりの評価区間数
	 * @return 格子辺の内部を含む診断値，補間不能なら空
	 * 計算量：O(P K)，P は経路の辺数，K は辺内区間数
	 */
	std::optional<SmoothedRouteAudit> AuditSubdividedRoute(
		const FineDrainageMesh &Mesh,
		const std::vector<std::uint32_t> &Path,
		const PlanetCore::RiverTerrainField &Terrain,
		const PlanetCore::OceanConnectivityField &Ocean,
		const double Sea,
		const double Radius,
		const std::uint32_t SegmentsPerEdge
	) {
		// 経路又は分割条件が不正な入力を評価しない場合の返戻
		if(Path.size() < 2 || SegmentsPerEdge < 2 || SegmentsPerEdge > 16) return {};
		std::vector<PlanetCore::UnitDirection> Directions;
		Directions.reserve((Path.size() - 1) * SegmentsPerEdge + 1);
		for(std::size_t Step = 1; Step < Path.size(); ++Step) {
			// 元網の範囲外の端点を細分しない場合の返戻
			if(Path[Step - 1] >= Mesh.Directions.size() || Path[Step] >= Mesh.Directions.size()) return {};
			const PlanetCore::Vec3 A = Mesh.Directions[Path[Step - 1]].Vector();
			const PlanetCore::Vec3 B = Mesh.Directions[Path[Step]].Vector();
			if(Step == 1) Directions.push_back(Mesh.Directions[Path.front()]);
			for(std::uint32_t Part = 1; Part <= SegmentsPerEdge; ++Part) {
				const double Fraction = static_cast<double>(Part) / SegmentsPerEdge;
				const std::optional<PlanetCore::UnitDirection> Direction =
				PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
				// 有効な球面方向を構成出来ない場合の返戻
				if(!Direction) return {};
				Directions.push_back(*Direction);
			}
		}
		// 連続方向列を独立に測った縦断監査の返戻
		return AuditSampledDirections(std::move(Directions), Terrain, Ocean, Sea, Radius);
	}

	/**
	 * 格子辺の内部標高を含め，最初の元大洋接触までの最大越流高を最小化する
	 * @param Mesh 細分球面網
	 * @param Heights 最終地形の格子点標高
	 * @param Terrain 格子辺内部を測る最終地形場
	 * @param Ocean 元の大洋接続場
	 * @param Sink 出発する閉鎖域の格子点
	 * @param Sea 球状海面の標高
	 * @param Radius 球半径
	 * @return 制限内で見付かった越流候補，又は探索未達の診断値
	 * 計算量：O((V + E) log(V + E) + S)，V は頂点数，E は辺数，S は元場の評価数
	 */
	std::optional<ContinuousMinimaxRoute> FindContinuousMinimaxRoute(
		const FineDrainageMesh &Mesh,
		const std::vector<double> &Heights,
		const PlanetCore::RiverTerrainField &Terrain,
		const PlanetCore::OceanConnectivityField &Ocean,
		const std::uint32_t Sink,
		const double Sea,
		const double Radius
	) {
		constexpr std::uint32_t SegmentsPerEdge = 8, SampleBudget = 250000;
		constexpr double SearchRadiusMeters = 250000;
		const bool HasInvalidSource = Mesh.Directions.size() != Heights.size() || Mesh.Neighbors.size() != Heights.size() ||
		Heights.size() > std::numeric_limits<std::uint32_t>::max() || Sink >= Heights.size() || !std::isfinite(Radius) || Radius <= 0 ||
		!std::isfinite(Heights[Sink]) || Heights[Sink] <= Sea;
		// 入力網・標高・尺度又は出口条件が不整合な場合の返戻
		if(HasInvalidSource) return {};
		const double SearchStarted = FPlatformTime::Seconds(), Infinity = std::numeric_limits<double>::infinity();
		std::vector<double> BestMaximum(Heights.size(), Infinity), BestLength(Heights.size(), Infinity);
		const std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		std::vector<std::uint32_t> BestEdges(Heights.size()), Parent(Heights.size(), Missing);
		Parent[Sink] = Sink;
		using Entry = std::tuple<double, double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		BestMaximum[Sink] = Heights[Sink];
		BestLength[Sink] = 0;
		Frontier.emplace(Heights[Sink], 0, Sink);
		const PlanetCore::Vec3 Source = Mesh.Directions[Sink].Vector();
		ContinuousMinimaxRoute Result;
		std::uint32_t GoalNode = Missing, GoalNeighbor = Missing, GoalPart = 0;
		while(!Frontier.empty()) {
			const Entry QueueEntry = Frontier.top();
			const double Maximum = std::get<0>(QueueEntry), Length = std::get<1>(QueueEntry);
			const std::uint32_t Node = std::get<2>(QueueEntry);
			Frontier.pop();
			if(Maximum != BestMaximum[Node] || Length != BestLength[Node]) continue;
			if(
				Result.Found &&
				(Maximum > Result.MaximumTerrainMeters || Maximum == Result.MaximumTerrainMeters && Length >= Result.LengthMeters)
			) break;
			++Result.ExpandedNodes;
			const PlanetCore::Vec3 A = Mesh.Directions[Node].Vector();
			for(const std::uint32_t Neighbor : Mesh.Neighbors[Node]) {
				// 標高の無い隣接点を探索しない場合の返戻
				if(Neighbor >= Heights.size()) return {};
				const PlanetCore::Vec3 B = Mesh.Directions[Neighbor].Vector();
				const double SourceAngle = std::acos(std::clamp(Source.Dot(B), -1.0, 1.0));
				if(Radius * SourceAngle > SearchRadiusMeters) continue;
				const double EdgeMeters = Radius * std::acos(std::clamp(A.Dot(B), -1.0, 1.0));
				// 非有限又は非正の辺長を採用しない場合の返戻
				if(!std::isfinite(EdgeMeters) || EdgeMeters <= 0) return {};
				double EdgeMaximum = Maximum;
				bool StoppedAtWater = false;
				for(std::uint32_t Part = 1; Part <= SegmentsPerEdge; ++Part) {
					if(Result.SampledPoints >= SampleBudget) {
						Result.BudgetExceeded = true;
						Result.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
						// 当該経路と測定を保持した診断結果の返戻
						return Result;
					}
					++Result.SampledPoints;
					const double Fraction = static_cast<double>(Part) / SegmentsPerEdge;
					const std::optional<PlanetCore::UnitDirection> Direction =
					PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
					// 有効な球面方向を構成出来ない場合の返戻
					if(!Direction) return {};
					const double Height = Part == SegmentsPerEdge ? Heights[Neighbor] : Terrain.EvaluateElevation(*Direction);
					// 非有限な元場標高を出口探索へ含めない場合の返戻
					if(!std::isfinite(Height)) return {};
					EdgeMaximum = std::max(EdgeMaximum, Height);
					if(Height > Sea) continue;
					StoppedAtWater = true;
					if(Ocean.EvaluateElevation(*Direction) > Sea) break;
					const double PartialMeters = Radius * std::acos(std::clamp(A.Dot(Direction->Vector()), -1.0, 1.0));
					const double CandidateLength = Length + PartialMeters;
					if(
						!Result.Found || EdgeMaximum < Result.MaximumTerrainMeters ||
						EdgeMaximum == Result.MaximumTerrainMeters && CandidateLength < Result.LengthMeters
					) {
						Result.Found = true;
						Result.MaximumTerrainMeters = EdgeMaximum;
						Result.LengthMeters = CandidateLength;
						Result.Edges = BestEdges[Node] + 1;
						GoalNode = Node;
						GoalNeighbor = Neighbor;
						GoalPart = Part;
					}
					break;
				}
				if(StoppedAtWater) continue;
				const double CandidateLength = Length + EdgeMeters;
				if(EdgeMaximum < BestMaximum[Neighbor] || EdgeMaximum == BestMaximum[Neighbor] && CandidateLength < BestLength[Neighbor]) {
					BestMaximum[Neighbor] = EdgeMaximum;
					BestLength[Neighbor] = CandidateLength;
					BestEdges[Neighbor] = BestEdges[Node] + 1;
					Parent[Neighbor] = Node;
					Frontier.emplace(EdgeMaximum, CandidateLength, Neighbor);
				}
			}
		}
		if(Result.Found) {
			// 大洋出口の端点又は区間が欠けている場合の返戻
			if(GoalNode == Missing || GoalNeighbor == Missing || !GoalPart) return {};
			std::vector<std::uint32_t> Chain;
			for(std::uint32_t Index = GoalNode;; Index = Parent[Index]) {
				// 範囲外又は循環した親列を復元しない場合の返戻
				if(Index >= Parent.size() || Chain.size() >= Parent.size()) return {};
				Chain.push_back(Index);
				if(Index == Sink) break;
				// 欠落した親から経路を復元しない場合の返戻
				if(Parent[Index] == Missing) return {};
			}
			std::reverse(Chain.begin(), Chain.end());
			// 復元した経路の辺数が一致しない場合の返戻
			if(Chain.size() != Result.Edges) return {};
			std::vector<PlanetCore::UnitDirection> Directions;
			Directions.reserve(Result.Edges * SegmentsPerEdge + 1);
			Directions.push_back(Mesh.Directions[Sink]);
			for(std::size_t Step = 0; Step < Chain.size(); ++Step) {
				const PlanetCore::Vec3 A = Mesh.Directions[Chain[Step]].Vector();
				const std::uint32_t Next = Step + 1 < Chain.size() ? Chain[Step + 1] : GoalNeighbor;
				const PlanetCore::Vec3 B = Mesh.Directions[Next].Vector();
				const std::uint32_t Parts = Step + 1 < Chain.size() ? SegmentsPerEdge : GoalPart;
				for(std::uint32_t Part = 1; Part <= Parts; ++Part) {
					const double Fraction = static_cast<double>(Part) / SegmentsPerEdge;
					const std::optional<PlanetCore::UnitDirection> Direction =
					PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
					// 有効な球面方向を構成出来ない場合の返戻
					if(!Direction) return {};
					Directions.push_back(*Direction);
				}
			}
			const std::optional<SmoothedRouteAudit> Audit = AuditSampledDirections(std::move(Directions), Terrain, Ocean, Sea, Radius);
			if(!Audit || Audit->SubmergedIntermediate || std::abs(Audit->MaximumTerrainMeters - Result.MaximumTerrainMeters) > 0.001) {
				// 独立な経路監査と保持済み結果が一致しない場合の返戻
				return {};
			}
			Result.Profile = Audit->Profile;
			Result.MixedProfile = Audit->MixedProfile;
			Result.MaximumTurnDegrees = Audit->MaximumTurnDegrees;
			Result.ReconstructedPoints = Audit->Points;
		}
		Result.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}

	/**
	 * 辺内部の標高と下流へ引き継ぐ水位を同時に状態へ含め，河岸開削費用の小さい出口を探す
	 * @param Mesh 細分球面網
	 * @param Heights 最終地形の格子点標高
	 * @param Terrain 辺内部を測る最終地形場
	 * @param Ocean 元の大洋接続場
	 * @param Sink 閉鎖域の水源地点
	 * @param Sea 球状海面の標高
	 * @param Radius 球半径
	 * @param MaximumBankCutMeters 許容する最大河岸開削
	 * @return 制限内で見付かった連続河道，又は探索未達の診断値
	 * 計算量：O(L² + Q log Q + S)，L は状態数，Q は隣接候補数，S は元場の評価数
	 */
	std::optional<ContinuousWaterRoute> FindContinuousWaterRoute(
		const FineDrainageMesh &Mesh,
		const std::vector<double> &Heights,
		const PlanetCore::RiverTerrainField &Terrain,
		const PlanetCore::OceanConnectivityField &Ocean,
		const std::uint32_t Sink,
		const double Sea,
		const double Radius,
		const double MaximumBankCutMeters
	) {
		constexpr std::uint32_t SegmentsPerEdge = 8, LabelBudget = 250000, SampleBudget = 1000000;
		constexpr double SearchRadiusMeters = 250000;
		const bool HasInvalidSource = Mesh.Directions.size() != Heights.size() || Mesh.Neighbors.size() != Heights.size() ||
		Heights.size() > std::numeric_limits<std::uint32_t>::max() || Sink >= Heights.size() || !std::isfinite(Radius) || Radius <= 0 ||
		!std::isfinite(Heights[Sink]) || Heights[Sink] <= Sea || !std::isfinite(MaximumBankCutMeters) || MaximumBankCutMeters <= 0;
		// 入力網・標高・尺度又は出口条件が不整合な場合の返戻
		if(HasInvalidSource) return {};
		const double SearchStarted = FPlatformTime::Seconds();
		struct Label {
			std::uint32_t Node = 0;
			std::uint32_t Parent = 0;
			double Water = 0;
			double Cost = 0;
			double Length = 0;
			bool Active = true;
		};
		std::vector<Label> Labels;
		Labels.reserve(4096);
		Labels.push_back({ Sink, 0, Heights[Sink] - DiagnosticFreeboard(Heights[Sink], Sea), 0, 0, true });
		std::vector<std::vector<std::uint32_t>> ByNode(Heights.size());
		ByNode[Sink].push_back(0);
		using Entry = std::tuple<double, double, std::uint32_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> Frontier;
		Frontier.emplace(0, 0, 0);
		const PlanetCore::Vec3 Source = Mesh.Directions[Sink].Vector();
		ContinuousWaterRoute Result;
		double GoalCost = std::numeric_limits<double>::infinity();
		std::uint32_t GoalLabel = 0, GoalNeighbor = 0, GoalPart = 0;
		while(!Frontier.empty()) {
			const Entry QueueEntry = Frontier.top();
			const double Cost = std::get<0>(QueueEntry), Length = std::get<1>(QueueEntry);
			const std::uint32_t LabelIndex = std::get<2>(QueueEntry);
			Frontier.pop();
			if(!Labels[LabelIndex].Active) continue;
			const Label Current = Labels[LabelIndex];
			// 待機候補と保持状態が不一致の場合に続行しない場合の返戻
			if(Cost != Current.Cost || Length != Current.Length) return {};
			if(Result.Found && (Cost > GoalCost || Cost == GoalCost && Length >= Result.LengthMeters)) break;
			++Result.ExpandedLabels;
			const PlanetCore::Vec3 A = Mesh.Directions[Current.Node].Vector();
			for(const std::uint32_t Neighbor : Mesh.Neighbors[Current.Node]) {
				// 標高の無い隣接点を探索しない場合の返戻
				if(Neighbor >= Heights.size()) return {};
				const PlanetCore::Vec3 B = Mesh.Directions[Neighbor].Vector();
				const double SourceAngle = std::acos(std::clamp(Source.Dot(B), -1.0, 1.0));
				if(Radius * SourceAngle > SearchRadiusMeters) continue;
				PlanetCore::Vec3 PreviousDirection = A;
				double Water = Current.Water, EdgeCost = 0, EdgeLength = 0;
				bool StopEdge = false;
				for(std::uint32_t Part = 1; Part <= SegmentsPerEdge; ++Part) {
					if(Result.SampledPoints >= SampleBudget) {
						Result.BudgetExceeded = true;
						Result.LabelsCreated = static_cast<std::uint32_t>(Labels.size());
						Result.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
						// 当該経路と測定を保持した診断結果の返戻
						return Result;
					}
					++Result.SampledPoints;
					const double Fraction = static_cast<double>(Part) / SegmentsPerEdge;
					const std::optional<PlanetCore::UnitDirection> Direction =
					PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
					// 有効な球面方向を構成出来ない場合の返戻
					if(!Direction) return {};
					const PlanetCore::Vec3 Point = Direction->Vector();
					const double SegmentMeters = Radius * std::acos(std::clamp(PreviousDirection.Dot(Point), -1.0, 1.0));
					// 非有限又は非正の内部区間を縦断へ加えない場合の返戻
					if(!std::isfinite(SegmentMeters) || SegmentMeters <= 0) return {};
					EdgeLength += SegmentMeters;
					PreviousDirection = Point;
					const double Height = Part == SegmentsPerEdge ? Heights[Neighbor] : Terrain.EvaluateElevation(*Direction);
					// 非有限な元場標高を出口探索へ含めない場合の返戻
					if(!std::isfinite(Height)) return {};
					if(Height <= Sea) {
						StopEdge = true;
						if(Ocean.EvaluateElevation(*Direction) <= Sea && Water > Sea) {
							const double CandidateCost = Current.Cost + EdgeCost, CandidateLength = Current.Length + EdgeLength;
							if(!Result.Found || CandidateCost < GoalCost || CandidateCost == GoalCost && CandidateLength < Result.LengthMeters) {
								Result.Found = true;
								GoalCost = CandidateCost;
								Result.LengthMeters = CandidateLength;
								GoalLabel = LabelIndex;
								GoalNeighbor = Neighbor;
								GoalPart = Part;
							}
						}
						break;
					}
					const double Freeboard = DiagnosticFreeboard(Height, Sea);
					const double NextWater = std::min(Water - 1E-7 * SegmentMeters, Height - Freeboard);
					if(!std::isfinite(NextWater) || NextWater <= Sea || !(NextWater < Water)) {
						StopEdge = true;
						break;
					}
					const double BankCut = std::max(0.0, Height - Freeboard - NextWater);
					if(BankCut > MaximumBankCutMeters) {
						StopEdge = true;
						break;
					}
					EdgeCost += BankCut * SegmentMeters;
					Water = NextWater;
				}
				if(StopEdge) continue;
				const double CandidateCost = Current.Cost + EdgeCost, CandidateLength = Current.Length + EdgeLength;
				std::vector<std::uint32_t> &Existing = ByNode[Neighbor];
				bool Dominated = false;
				for(const std::uint32_t Index : Existing) {
					const Label &Old = Labels[Index];
					if(Old.Active && Old.Cost <= CandidateCost && Old.Water >= Water && Old.Length <= CandidateLength) {
						Dominated = true;
						break;
					}
				}
				if(Dominated) continue;
				for(const std::uint32_t Index : Existing) {
					Label &Old = Labels[Index];
					if(Old.Active && CandidateCost <= Old.Cost && Water >= Old.Water && CandidateLength <= Old.Length) Old.Active = false;
				}
				if(Labels.size() >= LabelBudget) {
					Result.BudgetExceeded = true;
					Result.LabelsCreated = static_cast<std::uint32_t>(Labels.size());
					Result.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
					// 当該経路と測定を保持した診断結果の返戻
					return Result;
				}
				const std::uint32_t NextIndex = static_cast<std::uint32_t>(Labels.size());
				Labels.push_back({ Neighbor, LabelIndex, Water, CandidateCost, CandidateLength, true });
				Existing.push_back(NextIndex);
				Frontier.emplace(CandidateCost, CandidateLength, NextIndex);
			}
		}
		Result.LabelsCreated = static_cast<std::uint32_t>(Labels.size());
		if(Result.Found) {
			std::vector<std::uint32_t> Chain;
			for(std::uint32_t Index = GoalLabel;; Index = Labels[Index].Parent) {
				// 状態の親が範囲外又は循環している場合の返戻
				if(Index >= Labels.size() || Chain.size() >= Labels.size()) return {};
				Chain.push_back(Labels[Index].Node);
				if(!Index) break;
			}
			std::reverse(Chain.begin(), Chain.end());
			Result.Edges = static_cast<std::uint32_t>(Chain.size());
			std::vector<PlanetCore::UnitDirection> Directions;
			Directions.reserve(Result.Edges * SegmentsPerEdge + 1);
			Directions.push_back(Mesh.Directions[Sink]);
			for(std::size_t Step = 0; Step < Chain.size(); ++Step) {
				const PlanetCore::Vec3 A = Mesh.Directions[Chain[Step]].Vector();
				const std::uint32_t Next = Step + 1 < Chain.size() ? Chain[Step + 1] : GoalNeighbor;
				const PlanetCore::Vec3 B = Mesh.Directions[Next].Vector();
				const std::uint32_t Parts = Step + 1 < Chain.size() ? SegmentsPerEdge : GoalPart;
				for(std::uint32_t Part = 1; Part <= Parts; ++Part) {
					const double Fraction = static_cast<double>(Part) / SegmentsPerEdge;
					const std::optional<PlanetCore::UnitDirection> Direction =
					PlanetCore::UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
					// 有効な球面方向を構成出来ない場合の返戻
					if(!Direction) return {};
					Directions.push_back(*Direction);
				}
			}
			const std::optional<SmoothedRouteAudit> Audit = AuditSampledDirections(std::move(Directions), Terrain, Ocean, Sea, Radius);
			const bool FailedIndependentAudit = !Audit || Audit->SubmergedIntermediate || !Audit->Profile ||
			std::abs(Audit->Profile->IntegratedBankCutSquareMeters - GoalCost) > 0.01;
			// 独立に監査した連続経路が探索結果と一致しない場合の返戻
			if(FailedIndependentAudit) return {};
			Result.Profile = Audit->Profile;
			Result.MixedProfile = Audit->MixedProfile;
			Result.MaximumTurnDegrees = Audit->MaximumTurnDegrees;
			Result.ReconstructedPoints = Audit->Points;
		}
		Result.SearchMilliseconds = (FPlatformTime::Seconds() - SearchStarted) * 1000;
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}

	/**
	 * 元海域を排水と同じ粗網で再標本化し，解像度に依る接続の欠落を数える
	 * @param Ocean 接続を準備済みの不変な海域場
	 * @return 水没点数と成分数，球面網を作れない場合は空
	 * 計算量：O(V + E)，V は元網の頂点数，E は辺数
	 */
	std::optional<std::array<std::uint32_t, 2>> CountCoarseOceanComponents(const PlanetCore::OceanConnectivityField &Ocean) {
		const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(6);
		// 診断用の粗網を準備出来ない場合の返戻
		if(!Mesh) return {};
		std::vector<std::uint32_t> Parent(Mesh->Directions.size());
		std::iota(Parent.begin(), Parent.end(), 0);
		std::vector<bool> Wet(Parent.size());
		std::uint32_t WetCount = 0;
		for(std::size_t Index = 0; Index < Parent.size(); ++Index) {
			Wet[Index] = Ocean.EvaluateElevation(Mesh->Directions[Index]) < Ocean.Base().Settings().SeaLevelMeters;
			WetCount += Wet[Index] ? 1 : 0;
		}
		// 海域成分の代表への経路圧縮
		const auto Root = [&](std::uint32_t Index) {
			while(Parent[Index] != Index) {
				Parent[Index] = Parent[Parent[Index]];
				Index = Parent[Index];
			}
			// 粗網内の同じ水没成分を識別する代表番号の返戻
			return Index;
		};
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(unsigned Edge = 0; Edge < 3; ++Edge) {
			const std::uint32_t A = Triangle[Edge], B = Triangle[(Edge + 1) % 3];
			if(Wet[A] && Wet[B]) Parent[Root(A)] = Root(B);
		}
		std::vector<std::uint32_t> Roots;
		for(std::uint32_t Index = 0; Index < Parent.size(); ++Index) if(Wet[Index]) Roots.push_back(Root(Index));
		std::sort(Roots.begin(), Roots.end());
		Roots.erase(std::unique(Roots.begin(), Roots.end()), Roots.end());
		// 粗い標本だけで見える水没点数と接続成分数の返戻
		return std::array<std::uint32_t, 2>{ WetCount, static_cast<std::uint32_t>(Roots.size()) };
	}

	/**
	 * 細分網の最終海域連結と陸地からの厳密下降経路を調べる
	 * @param Mesh 球面網と共有辺
	 * @param Heights 同じ順序の最終標高
	 * @param SourceOcean 排水前の海陸分類
	 * @param Sea 一定の球状海面の標高
	 * @param Radius 球面経路の物理長へ変換する惑星半径
	 * @param Test 失敗段階を記録する検査
	 * @param Name 比較元と候補の識別名
	 * @return 最大海域への経路と水没成分の標本統計，不正な標高では空
	 */
	std::optional<FineDrainageStatistics> SurveyFineDrainage(
		const FineDrainageMesh &Mesh,
		const std::vector<double> &Heights,
		const std::vector<std::uint8_t> &SourceOcean,
		const double Sea,
		const double Radius,
		FAutomationTestBase &Test,
		const TCHAR *Name
	) {
		FineDrainageStatistics Result;
		// 検査段階の失敗理由の記録
		const auto Failure = [&](const TCHAR *Reason) -> std::optional<FineDrainageStatistics> {
			Test.AddError(
				FString::Printf(
					TEXT("Fine drainage survey %s failed: %s; wetComponents=%u ") TEXT("isolatedWet=%u direct=%.3fm sink=%u trappedSpill=%.3fm."),
					Name,
					Reason,
					Result.WetComponents,
					Result.IsolatedWetNodes,
					Result.MaximumDirectSinkDepthMeters,
					Result.MaximumDirectSinkIndex,
					Result.MaximumTrappedSpillMeters
				)
			);
			// 失敗段階と其れ迄の測定値を残した上で未完了の返戻
			return {};
		};
		const bool HasInvalidSource =
		Mesh.Directions.size() != Heights.size() || Heights.size() != SourceOcean.size() || Mesh.Neighbors.size() != Heights.size() ||
		Heights.size() > std::numeric_limits<std::uint32_t>::max() || !std::isfinite(Radius) || Radius <= 0;
		// 網・標高・尺度が不整合な場合の返戻
		if(HasInvalidSource) return Failure(TEXT("invalid input"));
		const std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		std::vector<std::uint32_t> Component(Heights.size(), Missing), ComponentSizes, Queue;
		for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
			// 非有限な標高の場合の返戻
			if(!std::isfinite(Heights[Index])) return Failure(TEXT("nonfinite height"));
			if(SourceOcean[Index] && Heights[Index] > Sea) ++Result.RaisedSourceOcean;
			if(!SourceOcean[Index] && Heights[Index] <= Sea) ++Result.SubmergedSourceLand;
			if(Heights[Index] > Sea || Component[Index] != Missing) continue;
			const std::uint32_t Id = static_cast<std::uint32_t>(ComponentSizes.size());
			Component[Index] = Id;
			Queue.clear();
			Queue.push_back(Index);
			for(std::size_t Position = 0; Position < Queue.size(); ++Position) {
				for(const std::uint32_t Neighbor : Mesh.Neighbors[Queue[Position]]) {
					if(Heights[Neighbor] > Sea || Component[Neighbor] != Missing) continue;
					Component[Neighbor] = Id;
					Queue.push_back(Neighbor);
				}
			}
			ComponentSizes.push_back(static_cast<std::uint32_t>(Queue.size()));
			Result.WetNodes += static_cast<std::uint32_t>(Queue.size());
		}
		// 大洋出口となる海域が無いの場合の返戻
		if(ComponentSizes.empty()) return Failure(TEXT("no sea"));
		Result.WetComponents = static_cast<std::uint32_t>(ComponentSizes.size());
		const std::uint32_t Main =
		static_cast<std::uint32_t>(std::max_element(ComponentSizes.begin(), ComponentSizes.end()) - ComponentSizes.begin());
		Result.IsolatedWetNodes = Result.WetNodes - ComponentSizes[Main];
		using SpillEntry = std::pair<double, std::uint32_t>;
		std::priority_queue<SpillEntry, std::vector<SpillEntry>, std::greater<SpillEntry>> Frontier;
		std::vector<double> SpillHeight(Heights.size(), std::numeric_limits<double>::infinity());
		std::vector<std::uint32_t> SpillParent(Heights.size(), Missing);
		for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
			if(Component[Index] != Main) continue;
			SpillHeight[Index] = Sea;
			SpillParent[Index] = Index;
			Frontier.emplace(Sea, Index);
		}
		while(!Frontier.empty()) {
			const SpillEntry Current = Frontier.top();
			Frontier.pop();
			if(Current.first != SpillHeight[Current.second]) continue;
			for(const std::uint32_t Neighbor : Mesh.Neighbors[Current.second]) {
				const double Candidate = std::max(Current.first, Heights[Neighbor]);
				if(Candidate >= SpillHeight[Neighbor]) continue;
				SpillHeight[Neighbor] = Candidate;
				SpillParent[Neighbor] = Current.second;
				Frontier.emplace(Candidate, Neighbor);
			}
		}
		for(std::uint32_t Index = 0; Index < Heights.size(); ++Index) {
			// 大洋へ到達する越流経路が無いの場合の返戻
			if(!std::isfinite(SpillHeight[Index])) return Failure(TEXT("unreachable spill"));
			if(Component[Index] == Missing || Component[Index] == Main) continue;
			if(Result.IsolatedWetIndex != Missing && Heights[Index] >= Heights[Result.IsolatedWetIndex]) continue;
			Result.IsolatedWetIndex = Index;
			Result.IsolatedWetDepthMeters = Sea - Heights[Index];
			Result.IsolatedWetBarrierMeters = SpillHeight[Index] - Sea;
		}
		std::vector<std::uint32_t> HeightOrder(Heights.size());
		std::iota(HeightOrder.begin(), HeightOrder.end(), 0U);
		std::sort(
			HeightOrder.begin(),
			HeightOrder.end(),
			[&Heights](const std::uint32_t A, const std::uint32_t B) {
				return Heights[A] < Heights[B] || Heights[A] == Heights[B] && A < B;
			}
		);
		std::vector<std::uint8_t> AnyOceanRoute(Heights.size()), SteepestOceanRoute(Heights.size());
		for(const std::uint32_t Index : HeightOrder) {
			if(Heights[Index] <= Sea) {
				AnyOceanRoute[Index] = SteepestOceanRoute[Index] = Component[Index] == Main;
				continue;
			}
			++Result.DryNodes;
			std::uint32_t Lowest = Missing;
			double NeighborMinimum = std::numeric_limits<double>::infinity();
			for(const std::uint32_t Neighbor : Mesh.Neighbors[Index]) {
				NeighborMinimum = std::min(NeighborMinimum, Heights[Neighbor]);
				if(Heights[Neighbor] >= Heights[Index]) continue;
				if(Lowest == Missing || Heights[Neighbor] < Heights[Lowest]) Lowest = Neighbor;
				if(AnyOceanRoute[Neighbor]) AnyOceanRoute[Index] = 1;
			}
			if(Lowest == Missing) {
				++Result.DryWithoutLowerNeighbor;
				const double Depth = NeighborMinimum - Heights[Index];
				if(Depth > Result.MaximumDirectSinkDepthMeters) {
					Result.MaximumDirectSinkDepthMeters = Depth;
					Result.MaximumDirectSinkIndex = Index;
				}
			} else SteepestOceanRoute[Index] = SteepestOceanRoute[Lowest];
			if(!AnyOceanRoute[Index]) {
				++Result.DryWithoutOceanRoute;
				const double Spill = SpillHeight[Index] - Heights[Index];
				Result.MaximumTrappedSpillMeters = std::max(Result.MaximumTrappedSpillMeters, Spill);
				if(Spill > 20) ++Result.TrappedSpillOver20Meters;
				if(Spill > 100) ++Result.TrappedSpillOver100Meters;
			}
			if(!SteepestOceanRoute[Index]) ++Result.SteepestRouteTrapped;
		}
		if(Result.MaximumDirectSinkIndex != Missing) {
			Result.BoundedEscape = FindBoundedExcavationRoute(
				Mesh,
				Heights,
				Component,
				Main,
				Result.MaximumDirectSinkIndex,
				SpillHeight[Result.MaximumDirectSinkIndex],
				Heights[Result.MaximumDirectSinkIndex],
				Radius
			);
			// 上限内に出口経路が無いの場合の返戻
			if(!Result.BoundedEscape) return Failure(TEXT("bounded route unavailable"));
			const double SinkHeight = Heights[Result.MaximumDirectSinkIndex];
			std::vector<std::uint32_t> TreePath { Result.MaximumDirectSinkIndex };
			std::uint32_t Current = Result.MaximumDirectSinkIndex;
			for(std::size_t Step = 0; Step < Heights.size(); ++Step) {
				const std::uint32_t Next = SpillParent[Current];
				// 越流親が欠落の場合の返戻
				if(Next == Missing) return Failure(TEXT("missing spill parent"));
				if(Next == Current) {
					// 越流親の根が大洋でないの場合の返戻
					if(Component[Current] != Main) return Failure(TEXT("spill root not ocean"));
					break;
				}
				const double Cosine = std::clamp(Mesh.Directions[Current].Vector().Dot(Mesh.Directions[Next].Vector()), -1.0, 1.0);
				const double Length = Radius * std::acos(Cosine);
				// 経路長が非有限又は非正の場合の返戻
				if(!std::isfinite(Length) || Length <= 0) return Failure(TEXT("invalid route length"));
				Result.EscapePathMeters += Length;
				++Result.EscapePathEdges;
				const double Limit = SinkHeight - 1E-7 * Result.EscapePathMeters, Cut = std::max(0.0, Heights[Next] - Limit);
				Result.EscapeMaximumCutMeters = std::max(Result.EscapeMaximumCutMeters, Cut);
				Result.EscapeIntegratedCutSquareMeters += Cut * Length;
				TreePath.push_back(Next);
				Current = Next;
			}
			// 復元経路が大洋へ至らないの場合の返戻
			if(Component[Current] != Main) return Failure(TEXT("spill route not ocean"));
			Result.TreeWaterProfile = AuditDescendingWaterProfile(Mesh, Heights, TreePath, Sea, Radius);
			Result.BoundedWaterProfile = AuditDescendingWaterProfile(Mesh, Heights, Result.BoundedEscape->Path, Sea, Radius);
			// 下降する縦断を準備出来ないの場合の返戻
			if(!Result.TreeWaterProfile || !Result.BoundedWaterProfile) return Failure(TEXT("descending water unavailable"));
			ExcavationRoute TreeRoute;
			TreeRoute.Edges = Result.EscapePathEdges;
			TreeRoute.LengthMeters = Result.EscapePathMeters;
			TreeRoute.MaximumCutMeters = Result.EscapeMaximumCutMeters;
			TreeRoute.IntegratedCutSquareMeters = Result.EscapeIntegratedCutSquareMeters;
			TreeRoute.Path = std::move(TreePath);
			Result.SampledBestRoute = std::move(TreeRoute);
			Result.SampledBestWaterProfile = Result.TreeWaterProfile;
			// 河岸開削費用と経路長による候補の選択
			const auto Consider = [&Result](const ExcavationRoute &Route, const WaterProfileAudit &Profile, const double Fraction) {
				++Result.SampledRouteCount;
				const bool HasWorseCandidate =
				Profile.IntegratedBankCutSquareMeters > Result.SampledBestWaterProfile->IntegratedBankCutSquareMeters ||
				Profile.IntegratedBankCutSquareMeters == Result.SampledBestWaterProfile->IntegratedBankCutSquareMeters &&
				Route.LengthMeters >= Result.SampledBestRoute->LengthMeters;
				// 保持済み候補より費用が高い経路を採用しない返戻
				if(HasWorseCandidate) return;
				Result.SampledBestRoute = Route;
				Result.SampledBestWaterProfile = Profile;
				Result.SampledBestReferenceFraction = Fraction;
			};
			Consider(*Result.BoundedEscape, *Result.BoundedWaterProfile, 1);
			constexpr std::array<double, 5> Fractions { 0.8, 0.6, 0.4, 0.2, 0.05 };
			for(const double Fraction : Fractions) {
				const double Reference = Sea + (SinkHeight - Sea) * Fraction;
				const std::optional<ExcavationRoute> Route = FindBoundedExcavationRoute(
					Mesh,
					Heights,
					Component,
					Main,
					Result.MaximumDirectSinkIndex,
					SpillHeight[Result.MaximumDirectSinkIndex],
					Reference,
					Radius
				);
				if(!Route) {
					++Result.RejectedSampledRoutes;
					continue;
				}
				const std::optional<WaterProfileAudit> Profile = AuditDescendingWaterProfile(Mesh, Heights, Route->Path, Sea, Radius);
				if(!Profile) {
					++Result.RejectedSampledRoutes;
					continue;
				}
				Consider(*Route, *Profile, Fraction);
			}
			Result.ParetoRoute = FindParetoWaterRoute(
				Mesh,
				Heights,
				Component,
				Main,
				Result.MaximumDirectSinkIndex,
				Sea,
				SpillHeight[Result.MaximumDirectSinkIndex],
				Result.TreeWaterProfile->MaximumBankCutMeters,
				Radius
			);
			// 水位と費用の探索結果を取得出来ないの場合の返戻
			if(!Result.ParetoRoute) return Failure(TEXT("Pareto search unavailable"));
		}
		// 当該経路と測定を保持した診断結果の返戻
		return Result;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetFineDrainageSurvey, "PlanetGenerator.FinalDrainage.FineSurvey",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 排水固定網の辺中点を含む全球網で，最終海域と下降経路の取りこぼしを探す
 * @param Parameters 未使用の UE 自動検査引数
 * @return 診断を完了出来た場合に真，連続面の排水合格は意味しない
 */
bool FPlanetFineDrainageSurvey::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	PlanetCore::GeologySettings Settings;
	Settings.Seed = 42;
	Settings.RadiusMeters = 1000000;
	const std::optional<PlanetCore::GeologyField> PreparedGeology = PlanetCore::GeologyField::Create(Settings);
	// 細分排水調査の地質を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分排水調査の地質を準備"), PreparedGeology.has_value())) return false;
	const std::shared_ptr<const PlanetCore::GeologyField> Geology =
	std::make_shared<const PlanetCore::GeologyField>(*PreparedGeology);
	const std::optional<PlanetCore::PlanetOrogenicRidges> PreparedRidges = PlanetCore::PlanetOrogenicRidges::Create(*Geology);
	// 細分排水調査の造山を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分排水調査の造山を準備"), PreparedRidges.has_value())) return false;
	const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> Ridges =
	std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*PreparedRidges);
	const double OceanStarted = FPlatformTime::Seconds();
	const std::optional<PlanetCore::OceanConnectivityField> PreparedOcean =
	PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(Geology, Ridges);
	const double OceanPreparationMilliseconds = (FPlatformTime::Seconds() - OceanStarted) * 1000;
	// 細分排水調査の海陸を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分排水調査の海陸を準備"), PreparedOcean.has_value())) return false;
	const std::shared_ptr<const PlanetCore::OceanConnectivityField> Ocean =
	std::make_shared<const PlanetCore::OceanConnectivityField>(*PreparedOcean);
	TestEqual(TEXT("選択式の海域接続網を一段細分化"), Ocean->Statistics().InputGraphNodes, std::uint32_t{ 163842 });
	TestEqual(TEXT("細分海域接続網で大洋を一体化"), Ocean->Statistics().ComponentsAfter, std::uint32_t{ 1 });
	const std::optional<PlanetCore::DrainageNetwork> PreparedDrainage = PlanetCore::DrainageNetwork::Create(*Ocean);
	const std::optional<std::array<std::uint32_t, 2>> CoarseOcean = CountCoarseOceanComponents(*Ocean);
	// 元海域の粗網成分を診断の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("元海域の粗網成分を診断"), CoarseOcean.has_value())) return false;
	AddInfo(
		FString::Printf(
			TEXT("Drainage source ocean graphs: oceanGraph=%u fineWetComponents=%u ")
			TEXT("coarseNodes=40962 coarseWet=%u coarseWetComponents=%u."),
			Ocean->Statistics().GraphNodes,
			Ocean->Statistics().ComponentsAfter,
			(*CoarseOcean)[0],
			(*CoarseOcean)[1]
		)
	);
	// 細分排水調査の排水木を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分排水調査の排水木を準備"), PreparedDrainage.has_value())) return false;
	const std::shared_ptr<const PlanetCore::DrainageNetwork> Drainage =
	std::make_shared<const PlanetCore::DrainageNetwork>(*PreparedDrainage);
	const std::optional<PlanetCore::RiverTerrainField> Baseline = PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, Drainage);
	// 細分排水調査の比較地形を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分排水調査の比較地形を準備"), Baseline.has_value())) return false;
	const std::optional<PlanetCore::SphereMesh> Coarse = PlanetCore::BuildUniformSphere(6);
	// 排水と同じ固定球面網を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("排水と同じ固定球面網を準備"), Coarse.has_value())) return false;
	// 球面網の節点が河口拡張前の排水先頭に保持されるの失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("球面網の節点が河口拡張前の排水先頭に保持される"), Drainage->Nodes().size() >= Coarse->Directions.size())) return false;
	std::vector<double> CoarseHeights(Coarse->Directions.size());
	std::vector<double> NeighborMinimum(Coarse->Directions.size(), std::numeric_limits<double>::infinity());
	std::vector<std::uint8_t> SelectedUpstream(Coarse->Directions.size()), MissingPath(Coarse->Directions.size());
	for(const PlanetCore::RiverReach &Reach : Drainage->Reaches()) SelectedUpstream[Reach.Upstream] = 1;
	for(std::size_t Index = 0; Index < CoarseHeights.size(); ++Index) {
		CoarseHeights[Index] = Baseline->EvaluateElevation(Coarse->Directions[Index]);
	}
	for(const std::array<std::uint32_t, 3> &Triangle : Coarse->Triangles) for(std::size_t Corner = 0; Corner < 3; ++Corner) {
		const std::uint32_t A = Triangle[Corner], B = Triangle[(Corner + 1) % 3];
		NeighborMinimum[A] = std::min(NeighborMinimum[A], CoarseHeights[B]);
		NeighborMinimum[B] = std::min(NeighborMinimum[B], CoarseHeights[A]);
	}
	std::uint32_t CoarseSinks = 0;
	for(std::uint32_t Index = 0; Index < CoarseHeights.size(); ++Index) {
		if(
			Drainage->Nodes()[Index].Ocean || CoarseHeights[Index] <= Settings.SeaLevelMeters + 0.01 ||
			NeighborMinimum[Index] - CoarseHeights[Index] <= 0.01
		) continue;
		++CoarseSinks;
		std::uint32_t Current = Index, Steps = 0;
		while(!Drainage->Nodes()[Current].Ocean && !SelectedUpstream[Current]) {
			MissingPath[Current] = 1;
			Current = Drainage->Nodes()[Current].Downstream;
			// 下流の親が循環して大洋へ辿れない場合の返戻
			if(++Steps > CoarseHeights.size()) return false;
		}
	}
	std::vector<std::uint32_t> PromotionEdges;
	for(std::uint32_t Index = 0; Index < MissingPath.size(); ++Index) if(MissingPath[Index]) PromotionEdges.push_back(Index);
	// 固定網の窪地と必要支流が存在の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("固定網の窪地と必要支流が存在"), CoarseSinks > 0 && !PromotionEdges.empty())) return false;
	const std::optional<PlanetCore::DrainageNetwork> Promoted = Drainage->WithPromotedTributaries(PromotionEdges, *Ocean);
	// 必要支流を追加の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("必要支流を追加"), Promoted.has_value())) return false;
	const std::optional<PlanetCore::RiverTerrainField> Candidate =
	PlanetCore::RiverTerrainField::CreateWithErosion(Ocean, std::make_shared<const PlanetCore::DrainageNetwork>(*Promoted));
	// 細分調査の候補地形を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分調査の候補地形を準備"), Candidate.has_value())) return false;
	const std::optional<FineDrainageMesh> Fine = RefineDrainageMesh(*Coarse);
	// 共有辺の中点を持つ細分網を準備の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("共有辺の中点を持つ細分網を準備"), Fine.has_value())) return false;
	// １段細分化後の球面地点数の失敗時に検査を続けない返戻
	if(!TestEqual(TEXT("１段細分化後の球面地点数"), Fine->Directions.size(), std::size_t{ 163842 })) return false;
	std::vector<double> BaselineHeights(Fine->Directions.size()), CandidateHeights(Fine->Directions.size());
	std::vector<std::uint8_t> SourceOcean(Fine->Directions.size());
	const double SurveyStarted = FPlatformTime::Seconds();
	for(std::size_t Index = 0; Index < Fine->Directions.size(); ++Index) {
		const PlanetCore::UnitDirection Direction = Fine->Directions[Index];
		SourceOcean[Index] = Ocean->EvaluateElevation(Direction) <= Settings.SeaLevelMeters;
		BaselineHeights[Index] = Baseline->EvaluateElevation(Direction);
		CandidateHeights[Index] = Candidate->EvaluateElevation(Direction);
	}
	const double SurveyMilliseconds = (FPlatformTime::Seconds() - SurveyStarted) * 1000;
	{
		// 本番の同じ排水木を使い，盛土を許した共有水位の変更量だけを測る
		const PlanetCore::DrainageNetwork &Network = Candidate->Drainage();
		const std::span<const PlanetCore::DrainageNode> Nodes = Network.Nodes();
		std::vector<std::uint32_t> Local(Nodes.size(), PlanetCore::NoDrainageFitDownstream);
		std::vector<double> Areas(Nodes.size());
		std::vector<PlanetCore::DrainageFitSite> Sites;
		for(std::uint32_t Index = 0; Index < Nodes.size(); ++Index) {
			if(Nodes[Index].Ocean) continue;
			Local[Index] = static_cast<std::uint32_t>(Sites.size());
			Sites.emplace_back();
			Areas[Index] = Nodes[Index].ContributingAreaSquareMeters;
		}
		for(const PlanetCore::DrainageNode &Node : Nodes) {
			if(!Node.Ocean && !Nodes[Node.Downstream].Ocean) Areas[Node.Downstream] -= Node.ContributingAreaSquareMeters;
		}
		const double Grade = Network.Settings().MinimumSlope;
		for(std::uint32_t Index = 0; Index < Nodes.size(); ++Index) {
			const PlanetCore::DrainageNode &Node = Nodes[Index];
			if(Node.Ocean) continue;
			PlanetCore::DrainageFitSite &Site = Sites[Local[Index]];
			Site.Downstream = Local[Node.Downstream];
			const double Height = Candidate->EvaluateElevation(*PlanetCore::UnitDirection::FromVector(Node.Direction));
			Site.TargetMeters = Height - DiagnosticFreeboard(Height, Settings.SeaLevelMeters) - Grade * Node.DistanceToOceanMeters;
			Site.Weight = Areas[Index];
		}
		const double Started = FPlatformTime::Seconds();
		const std::optional<PlanetCore::DrainageFitResult> Fitted = PlanetCore::FitDrainageTree(Sites, Settings.SeaLevelMeters);
		const double Milliseconds = (FPlatformTime::Seconds() - Started) * 1000;
		// 実排水木の合流を共有する変更最小化の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("実排水木の合流を共有する変更最小化"), Fitted.has_value())) return false;
		double MaximumCut = 0, MaximumFill = 0, AreaCut = 0, AreaFill = 0, TotalArea = 0;
		for(std::uint32_t Index = 0; Index < Sites.size(); ++Index) {
			const PlanetCore::DrainageFitSite &Site = Sites[Index];
			const double Floor =
			Site.Downstream == PlanetCore::NoDrainageFitDownstream ? Settings.SeaLevelMeters : Fitted->Levels[Site.Downstream];
			TestTrue(TEXT("実排水木の全下流制約を保持"), Fitted->Levels[Index] >= Floor);
			const double Change = Fitted->Levels[Index] - Site.TargetMeters, Cut = std::max(0.0, -Change), Fill = std::max(0.0, Change);
			MaximumCut = std::max(MaximumCut, Cut);
			MaximumFill = std::max(MaximumFill, Fill);
			AreaCut += Cut * Site.Weight;
			AreaFill += Fill * Site.Weight;
			TotalArea += Site.Weight;
		}
		AddInfo(
			FString::Printf(
				TEXT("Global mixed drainage fit: landNodes=%llu time=%.6fms maxCut=%.6fm ")
				TEXT("maxFill=%.6fm areaWeightedCut=%.6fm areaWeightedFill=%.6fm rmsChange=%.6fm moves=%llu pooled=%u."),
				static_cast<unsigned long long>(Sites.size()),
				Milliseconds,
				MaximumCut,
				MaximumFill,
				AreaCut / TotalArea,
				AreaFill / TotalArea,
				std::sqrt(Fitted->WeightedSquaredChange / TotalArea),
				static_cast<unsigned long long>(Fitted->HeapMoves),
				Fitted->PooledKnots
			)
		);
	}
	const std::optional<FineDrainageStatistics> Before = SurveyFineDrainage(
		*Fine,
		BaselineHeights,
		SourceOcean,
		Settings.SeaLevelMeters,
		Settings.RadiusMeters,
		*this,
		TEXT("baseline")
	);
	const std::optional<FineDrainageStatistics> After = SurveyFineDrainage(
		*Fine,
		CandidateHeights,
		SourceOcean,
		Settings.SeaLevelMeters,
		Settings.RadiusMeters,
		*this,
		TEXT("candidate")
	);
	// 細分網の比較と候補を調査の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("細分網の比較と候補を調査"), Before.has_value() && After.has_value())) return false;
	// 最深窪地の地点は細分網内の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("最深窪地の地点は細分網内"), After->MaximumDirectSinkIndex < Fine->Directions.size())) return false;
	const std::uint32_t Sink = After->MaximumDirectSinkIndex;
	const PlanetCore::GeologySample SinkGeology = Geology->Evaluate(Fine->Directions[Sink]);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage geology sink: index=%u height=%.3fm ") TEXT("coordinate=%.6f tectonic=%.3fm resistance=%.6f."),
			Sink,
			SinkGeology.Terrain.ElevationMeters,
			SinkGeology.ContinentalCoordinate,
			SinkGeology.TectonicDisplacementMeters,
			SinkGeology.RockResistance
		)
	);
	for(const std::uint32_t Neighbor : Fine->Neighbors[Sink]) {
		const PlanetCore::GeologySample Nearby = Geology->Evaluate(Fine->Directions[Neighbor]);
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage geology neighbor: index=%u height=%.3fm ")
				TEXT("rise=%.3fm coordinate=%.6f deltaCoordinate=%.6f ")
				TEXT("tectonic=%.3fm deltaTectonic=%.3fm resistance=%.6f."),
				Neighbor,
				Nearby.Terrain.ElevationMeters,
				Nearby.Terrain.ElevationMeters - SinkGeology.Terrain.ElevationMeters,
				Nearby.ContinentalCoordinate,
				Nearby.ContinentalCoordinate - SinkGeology.ContinentalCoordinate,
				Nearby.TectonicDisplacementMeters,
				Nearby.TectonicDisplacementMeters - SinkGeology.TectonicDisplacementMeters,
				Nearby.RockResistance
			)
		);
	}
	// 最深地点に寄与する地質成分の測定
	const auto DiagnoseTectonics = [&](std::uint32_t Index) {
		const PlanetCore::Vec3 Direction = Fine->Directions[Index].Vector();
		std::array<double, 8> Weighted {}, Weights {}, Contributions {};
		for(const PlanetCore::GeologicFeature &Feature : Geology->Features()) {
			if(Feature.Center.Dot(Direction) <= Feature.SupportCosine) continue;
			const double U = Feature.Along.Dot(Direction) / Feature.AlongSine, V = Feature.Across.Dot(Direction) / Feature.AcrossSine;
			const double Q2 = U * U + V * V;
			if(Q2 >= 1) continue;
			const double Q = std::sqrt(Q2), S = 1 - Q, Kernel = S * S * S * S * (1 + 4 * Q);
			const std::size_t Kind = static_cast<std::size_t>(Feature.Kind);
			Weighted[Kind] += Kernel * Feature.AmplitudeMeters;
			Weights[Kind] += Kernel;
		}
		const double ReliefScale = Settings.RadiusMeters / (Settings.RadiusMeters + 250000);
		double Sum = 0;
		for(std::size_t Kind = 1; Kind < Contributions.size(); ++Kind) {
			Contributions[Kind] = Weighted[Kind] / (1 + Weights[Kind]) * ReliefScale;
			Sum += Contributions[Kind];
		}
		const double Published = Geology->Evaluate(Fine->Directions[Index]).TectonicDisplacementMeters;
		TestTrue(TEXT("要素別造構寄与の再構成"), std::abs(Sum - Published) < 1E-6);
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage tectonic kinds: index=%u collision=%.3fm ")
				TEXT("spreading=%.3fm trench=%.3fm outerRise=%.3fm forearc=%.3fm ")
				TEXT("volcanicArc=%.3fm transform=%.3fm sum=%.3fm."),
				Index,
				Contributions[1],
				Contributions[2],
				Contributions[3],
				Contributions[4],
				Contributions[5],
				Contributions[6],
				Contributions[7],
				Sum
			)
		);
	};
	DiagnoseTectonics(93899);
	for(const std::uint32_t Neighbor : Fine->Neighbors[93899]) DiagnoseTectonics(Neighbor);
	if(Sink != 93899) DiagnoseTectonics(Sink);
	// 最深地点の海溝成分の測定
	const auto ReportTrenches = [&](std::uint32_t Index) {
		const PlanetCore::Vec3 Direction = Fine->Directions[Index].Vector();
		const std::span<const PlanetCore::GeologicFeature> Features = Geology->Features();
		for(std::size_t FeatureIndex = 0; FeatureIndex < Features.size(); ++FeatureIndex) {
			const PlanetCore::GeologicFeature &Feature = Features[FeatureIndex];
			if(Feature.Kind != PlanetCore::GeologicFeatureKind::Trench || Feature.Center.Dot(Direction) <= Feature.SupportCosine) continue;
			const double U = Feature.Along.Dot(Direction) / Feature.AlongSine, V = Feature.Across.Dot(Direction) / Feature.AcrossSine;
			const double Q2 = U * U + V * V;
			if(Q2 >= 1) continue;
			const double Q = std::sqrt(Q2), S = 1 - Q, Weight = S * S * S * S * (1 + 4 * Q);
			if(Weight < 0.01) continue;
			const PlanetCore::UnitDirection Center = *PlanetCore::UnitDirection::FromVector(Feature.Center);
			const double CenterCoordinate = Geology->Evaluate(Center).ContinentalCoordinate;
			AddInfo(
				FString::Printf(
					TEXT("Fine drainage trench feature: node=%u id=%u weight=%.6f ")
					TEXT("amplitude=%.3fm weighted=%.3fm age=%.3fMyr centerCoordinate=%.6f."),
					Index,
					static_cast<std::uint32_t>(FeatureIndex),
					Weight,
					Feature.AmplitudeMeters,
					Weight * Feature.AmplitudeMeters,
					Feature.AgeMillionYears,
					CenterCoordinate
				)
			);
		}
	};
	ReportTrenches(93899);
	ReportTrenches(94223);
	TestEqual(TEXT("細分後の最終海域は一体"), After->WetComponents, std::uint32_t{ 1 });
	TestEqual(TEXT("細分後の最終海域に孤立水域なし"), After->IsolatedWetNodes, std::uint32_t{ 0 });
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage setup: nodes=%u coarseSinks=%u promotedEdges=%u ")
			TEXT("oceanPreparation=%.3fms sampleMilliseconds=%.3f. Survey only."),
			static_cast<std::uint32_t>(Fine->Directions.size()),
			CoarseSinks,
			static_cast<std::uint32_t>(PromotionEdges.size()),
			OceanPreparationMilliseconds,
			SurveyMilliseconds
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage baseline: wet=%u components=%u isolatedWet=%u ")
			TEXT("dry=%u noLower=%u noOceanRoute=%u steepestTrapped=%u sourceLandWet=%u sourceOceanRaised=%u."),
			Before->WetNodes,
			Before->WetComponents,
			Before->IsolatedWetNodes,
			Before->DryNodes,
			Before->DryWithoutLowerNeighbor,
			Before->DryWithoutOceanRoute,
			Before->SteepestRouteTrapped,
			Before->SubmergedSourceLand,
			Before->RaisedSourceOcean
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage candidate: wet=%u components=%u isolatedWet=%u ")
			TEXT("dry=%u noLower=%u noOceanRoute=%u steepestTrapped=%u sourceLandWet=%u sourceOceanRaised=%u."),
			After->WetNodes,
			After->WetComponents,
			After->IsolatedWetNodes,
			After->DryNodes,
			After->DryWithoutLowerNeighbor,
			After->DryWithoutOceanRoute,
			After->SteepestRouteTrapped,
			After->SubmergedSourceLand,
			After->RaisedSourceOcean
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage baseline depth: direct=%.3fm trappedSpill=%.3fm ")
			TEXT("trappedOver20/100=%u/%u isolatedIndex=%u isolatedDepth=%.3fm barrier=%.3fm."),
			Before->MaximumDirectSinkDepthMeters,
			Before->MaximumTrappedSpillMeters,
			Before->TrappedSpillOver20Meters,
			Before->TrappedSpillOver100Meters,
			Before->IsolatedWetIndex,
			Before->IsolatedWetDepthMeters,
			Before->IsolatedWetBarrierMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage candidate depth: direct=%.3fm trappedSpill=%.3fm ")
			TEXT("trappedOver20/100=%u/%u isolatedIndex=%u isolatedDepth=%.3fm barrier=%.3fm."),
			After->MaximumDirectSinkDepthMeters,
			After->MaximumTrappedSpillMeters,
			After->TrappedSpillOver20Meters,
			After->TrappedSpillOver100Meters,
			After->IsolatedWetIndex,
			After->IsolatedWetDepthMeters,
			After->IsolatedWetBarrierMeters
		)
	);
	TestTrue(TEXT("最深窪地の越流親経路を復元"), Before->EscapePathEdges > 0 && After->EscapePathEdges > 0);
	TestTrue(TEXT("開削下界は直接障壁以上"), After->EscapeMaximumCutMeters + 1E-6 >= After->MaximumDirectSinkDepthMeters);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage escape baseline: edges=%u length=%.3fm ") TEXT("maximumCut=%.3fm integratedCutProxy=%.3fm2."),
			Before->EscapePathEdges,
			Before->EscapePathMeters,
			Before->EscapeMaximumCutMeters,
			Before->EscapeIntegratedCutSquareMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage escape candidate: edges=%u length=%.3fm ") TEXT("maximumCut=%.3fm integratedCutProxy=%.3fm2."),
			After->EscapePathEdges,
			After->EscapePathMeters,
			After->EscapeMaximumCutMeters,
			After->EscapeIntegratedCutSquareMeters
		)
	);
	// 最小越流高内の代替経路を復元の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("最小越流高内の代替経路を復元"), Before->BoundedEscape.has_value() && After->BoundedEscape.has_value())) return false;
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage bounded escape baseline: edges=%u length=%.3fm ") TEXT("maximumCut=%.3fm integratedCutProxy=%.3fm2."),
			Before->BoundedEscape->Edges,
			Before->BoundedEscape->LengthMeters,
			Before->BoundedEscape->MaximumCutMeters,
			Before->BoundedEscape->IntegratedCutSquareMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage bounded escape candidate: edges=%u length=%.3fm ") TEXT("maximumCut=%.3fm integratedCutProxy=%.3fm2."),
			After->BoundedEscape->Edges,
			After->BoundedEscape->LengthMeters,
			After->BoundedEscape->MaximumCutMeters,
			After->BoundedEscape->IntegratedCutSquareMeters
		)
	);
	const bool FailedProfileComparison = !TestTrue(
		TEXT("両出口経路の下流水位を再構築"),
		Before->TreeWaterProfile.has_value() && Before->BoundedWaterProfile.has_value() && After->TreeWaterProfile.has_value() &&
		After->BoundedWaterProfile.has_value()
	);
	// 両出口経路の下流水位を再構築の失敗時に検査を続けない返戻
	if(FailedProfileComparison) return false;
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage descending water tree: drop=%.3fm maxSlope=%.6f ") TEXT("maximumBankCut=%.3fm integratedBankCut=%.3fm2."),
			After->TreeWaterProfile->WaterDropMeters,
			After->TreeWaterProfile->MaximumWaterSlope,
			After->TreeWaterProfile->MaximumBankCutMeters,
			After->TreeWaterProfile->IntegratedBankCutSquareMeters
		)
	);
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage descending water bounded: drop=%.3fm maxSlope=%.6f ")
			TEXT("maximumBankCut=%.3fm integratedBankCut=%.3fm2."),
			After->BoundedWaterProfile->WaterDropMeters,
			After->BoundedWaterProfile->MaximumWaterSlope,
			After->BoundedWaterProfile->MaximumBankCutMeters,
			After->BoundedWaterProfile->IntegratedBankCutSquareMeters
		)
	);
	const bool FailedRouteComparison = !TestTrue(
		TEXT("複数基準の下降水位候補を選択"),
		Before->SampledBestRoute.has_value() && Before->SampledBestWaterProfile.has_value() && After->SampledBestRoute.has_value() &&
		After->SampledBestWaterProfile.has_value()
	);
	// 複数基準の下降水位候補を選択の失敗時に検査を続けない返戻
	if(FailedRouteComparison) return false;
	AddInfo(
		FString::Printf(
			TEXT("Fine drainage sampled water route: referenceFraction=%.3f ")
			TEXT("sampled=%u rejected=%u edges=%u length=%.3fm maximumBankCut=%.3fm ")
			TEXT("integratedBankCut=%.3fm2 maxSlope=%.6f."),
			After->SampledBestReferenceFraction,
			After->SampledRouteCount,
			After->RejectedSampledRoutes,
			After->SampledBestRoute->Edges,
			After->SampledBestRoute->LengthMeters,
			After->SampledBestWaterProfile->MaximumBankCutMeters,
			After->SampledBestWaterProfile->IntegratedBankCutSquareMeters,
			After->SampledBestWaterProfile->MaximumWaterSlope
		)
	);
	// 水位状態を含む出口探索を完了の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("水位状態を含む出口探索を完了"), Before->ParetoRoute.has_value() && After->ParetoRoute.has_value())) return false;
	TestFalse(TEXT("水位状態探索のラベル上限に到達しない"), After->ParetoRoute->BudgetExceeded);
	if(!After->ParetoRoute->BudgetExceeded) {
		TestTrue(
			TEXT("水位状態経路は既知の越流木の費用以下"),
			After->ParetoRoute->Profile.IntegratedBankCutSquareMeters <= After->TreeWaterProfile->IntegratedBankCutSquareMeters + 0.001
		);
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage Pareto water route: labels=%u edges=%u length=%.3fm ")
				TEXT("maximumBankCut=%.3fm integratedBankCut=%.3fm2 maxSlope=%.6f ")
				TEXT("search=%.3fms maxTurn=%.3fdeg."),
				After->ParetoRoute->LabelsCreated,
				After->ParetoRoute->Route.Edges,
				After->ParetoRoute->Route.LengthMeters,
				After->ParetoRoute->Profile.MaximumBankCutMeters,
				After->ParetoRoute->Profile.IntegratedBankCutSquareMeters,
				After->ParetoRoute->Profile.MaximumWaterSlope,
				After->ParetoRoute->SearchMilliseconds,
				After->ParetoRoute->MaximumTurnDegrees
			)
		);
		for(const double BankLimit : std::array<double, 6>{ 600, 500, 450, 400, 350, 300 }) {
			const std::optional<ContinuousWaterRoute> ContinuousWater = FindContinuousWaterRoute(
				*Fine,
				CandidateHeights,
				*Candidate,
				*Ocean,
				After->MaximumDirectSinkIndex,
				Settings.SeaLevelMeters,
				Settings.RadiusMeters,
				BankLimit
			);
			// 辺内部水位を含む開削探索を診断の失敗時に検査を続けない返戻
			if(!TestTrue(TEXT("辺内部水位を含む開削探索を診断"), ContinuousWater.has_value())) return false;
			if(ContinuousWater->Found) {
				// 同じ連続経路で開削と盛土の縦断を比較の失敗時に検査を続けない返戻
				if(!TestTrue(TEXT("同じ連続経路で開削と盛土の縦断を比較"), ContinuousWater->MixedProfile.has_value())) return false;
				const MixedProfileAudit &Mixed = *ContinuousWater->MixedProfile;
				TestTrue(TEXT("比較元の開削専用も同じ最低勾配と海口を満たす"), Mixed.CutOnlyFeasible);
				TestTrue(TEXT("混合縦断の二乗変更は同じ重みの開削専用以下"), Mixed.SquaredChange <= Mixed.CutOnlySquaredChange + 1E-3);
				TestTrue(TEXT("混合縦断も全区間で最低下り勾配を保持"), Mixed.MinimumSlope >= 1E-7 * (1 - 1E-6));
				AddInfo(
					FString::Printf(
						TEXT("Mixed outlet profile: routeLimit=%.0fm maxCut=%.6fm maxFill=%.6fm ")
						TEXT("cutIntegral=%.6fm2 fillIntegral=%.6fm2 sourceChange=%.6fm ")
						TEXT("squaredChange=%.6f cutOnlySquaredChange=%.6f minSlope=%.12g."),
						BankLimit,
						Mixed.MaximumCutMeters,
						Mixed.MaximumFillMeters,
						Mixed.IntegratedCutSquareMeters,
						Mixed.IntegratedFillSquareMeters,
						Mixed.SourceChangeMeters,
						Mixed.SquaredChange,
						Mixed.CutOnlySquaredChange,
						Mixed.MinimumSlope
					)
				);
			}
			const bool FailedBoundedOceanRoute = BankLimit == 600 && !TestTrue(
				TEXT("既知の開削上限で大洋へ到達"),
				ContinuousWater->Found && !ContinuousWater->BudgetExceeded && ContinuousWater->Profile.has_value()
			);
			// 既知の開削上限で大洋へ到達の失敗時に検査を続けない返戻
			if(FailedBoundedOceanRoute) return false;
			if(ContinuousWater->Found && !ContinuousWater->BudgetExceeded && ContinuousWater->Profile) {
				TestTrue(TEXT("実際の最大開削は指定上限以下"), ContinuousWater->Profile->MaximumBankCutMeters <= BankLimit + 0.001);
			}
			AddInfo(
				FString::Printf(
					TEXT("Fine drainage continuous water route: limit=%.3fm ")
					TEXT("found=%u budget=%u labels=%u expanded=%u samples=%u edges=%u ")
					TEXT("points=%u length=%.3fm search=%.3fms waterValid=%u ")
					TEXT("maxBankCut=%.3fm integratedBankCut=%.3fm2 maxSlope=%.6f maxTurn=%.3fdeg."),
					BankLimit,
					ContinuousWater->Found ? 1U : 0U,
					ContinuousWater->BudgetExceeded ? 1U : 0U,
					ContinuousWater->LabelsCreated,
					ContinuousWater->ExpandedLabels,
					ContinuousWater->SampledPoints,
					ContinuousWater->Edges,
					ContinuousWater->ReconstructedPoints,
					ContinuousWater->LengthMeters,
					ContinuousWater->SearchMilliseconds,
					ContinuousWater->Profile.has_value() ? 1U : 0U,
					ContinuousWater->Profile ? ContinuousWater->Profile->MaximumBankCutMeters : 0,
					ContinuousWater->Profile ? ContinuousWater->Profile->IntegratedBankCutSquareMeters : 0,
					ContinuousWater->Profile ? ContinuousWater->Profile->MaximumWaterSlope : 0,
					ContinuousWater->MaximumTurnDegrees
				)
			);
		}
		const std::optional<ContinuousMinimaxRoute> Continuous = FindContinuousMinimaxRoute(
			*Fine,
			CandidateHeights,
			*Candidate,
			*Ocean,
			After->MaximumDirectSinkIndex,
			Settings.SeaLevelMeters,
			Settings.RadiusMeters
		);
		// 辺内部標高を含む越流探索を診断の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("辺内部標高を含む越流探索を診断"), Continuous.has_value())) return false;
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage continuous minimax: found=%u budget=%u ")
				TEXT("expanded=%u samples=%u edges=%u length=%.3fm maxTerrain=%.3fm ")
				TEXT("sourceTerrain=%.3fm rise=%.3fm search=%.3fms ")
				TEXT("waterValid=%u points=%u maxBankCut=%.3fm ")
				TEXT("integratedBankCut=%.3fm2 maxSlope=%.6f maxTurn=%.3fdeg."),
				Continuous->Found ? 1U : 0U,
				Continuous->BudgetExceeded ? 1U : 0U,
				Continuous->ExpandedNodes,
				Continuous->SampledPoints,
				Continuous->Edges,
				Continuous->LengthMeters,
				Continuous->MaximumTerrainMeters,
				CandidateHeights[After->MaximumDirectSinkIndex],
				Continuous->MaximumTerrainMeters - CandidateHeights[After->MaximumDirectSinkIndex],
				Continuous->SearchMilliseconds,
				Continuous->Profile.has_value() ? 1U : 0U,
				Continuous->ReconstructedPoints,
				Continuous->Profile ? Continuous->Profile->MaximumBankCutMeters : 0,
				Continuous->Profile ? Continuous->Profile->IntegratedBankCutSquareMeters : 0,
				Continuous->Profile ? Continuous->Profile->MaximumWaterSlope : 0,
				Continuous->MaximumTurnDegrees
			)
		);
		for(std::uint32_t Segments = 2; Segments < 17; Segments *= 2) {
			const std::optional<SmoothedRouteAudit> Along = AuditSubdividedRoute(
				*Fine,
				After->ParetoRoute->Route.Path,
				*Candidate,
				*Ocean,
				Settings.SeaLevelMeters,
				Settings.RadiusMeters,
				Segments
			);
			// 格子辺内部の河床を診断の失敗時に検査を続けない返戻
			if(!TestTrue(TEXT("格子辺内部の河床を診断"), Along.has_value())) return false;
			AddInfo(
				FString::Printf(
					TEXT("Fine drainage edge interiors: segments=%u points=%u ")
					TEXT("maxTurn=%.3fdeg maxTerrain=%.3fm submergedIntermediate=%u ")
					TEXT("waterValid=%u maxBankCut=%.3fm integratedBankCut=%.3fm2."),
					Segments,
					Along->Points,
					Along->MaximumTurnDegrees,
					Along->MaximumTerrainMeters,
					Along->SubmergedIntermediate,
					Along->Profile.has_value() ? 1U : 0U,
					Along->Profile ? Along->Profile->MaximumBankCutMeters : 0,
					Along->Profile ? Along->Profile->IntegratedBankCutSquareMeters : 0
				)
			);
			AddInfo(
				FString::Printf(
					TEXT("Fine drainage edge first sea: segments=%u index=%u ")
					TEXT("sourceOcean=%u truncatedWaterValid=%u truncatedMaxBankCut=%.3fm."),
					Segments,
					Along->FirstSubmergedIndex,
					Along->FirstSubmergedSourceOcean ? 1U : 0U,
					Along->FirstOceanProfile.has_value() ? 1U : 0U,
					Along->FirstOceanProfile ? Along->FirstOceanProfile->MaximumBankCutMeters : 0
				)
			);
		}
		for(std::uint32_t Passes = 1; Passes < 4; ++Passes) {
			const std::optional<SmoothedRouteAudit> Rounded = AuditSmoothedRoute(
				*Fine,
				After->ParetoRoute->Route.Path,
				*Candidate,
				*Ocean,
				Settings.SeaLevelMeters,
				Settings.RadiusMeters,
				Passes
			);
			// 球面河道の角丸めを診断の失敗時に検査を続けない返戻
			if(!TestTrue(TEXT("球面河道の角丸めを診断"), Rounded.has_value())) return false;
			AddInfo(
				FString::Printf(
					TEXT("Fine drainage smoothed route: passes=%u points=%u ")
					TEXT("maxTurn=%.3fdeg maxTerrain=%.3fm submergedIntermediate=%u ")
					TEXT("waterValid=%u maxBankCut=%.3fm integratedBankCut=%.3fm2."),
					Passes,
					Rounded->Points,
					Rounded->MaximumTurnDegrees,
					Rounded->MaximumTerrainMeters,
					Rounded->SubmergedIntermediate,
					Rounded->Profile.has_value() ? 1U : 0U,
					Rounded->Profile ? Rounded->Profile->MaximumBankCutMeters : 0,
					Rounded->Profile ? Rounded->Profile->IntegratedBankCutSquareMeters : 0
				)
			);
			AddInfo(
				FString::Printf(
					TEXT("Fine drainage smooth first sea: passes=%u index=%u ")
					TEXT("sourceOcean=%u truncatedWaterValid=%u truncatedMaxBankCut=%.3fm."),
					Passes,
					Rounded->FirstSubmergedIndex,
					Rounded->FirstSubmergedSourceOcean ? 1U : 0U,
					Rounded->FirstOceanProfile.has_value() ? 1U : 0U,
					Rounded->FirstOceanProfile ? Rounded->FirstOceanProfile->MaximumBankCutMeters : 0
				)
			);
		}
	} else {
		AddInfo(
			FString::Printf(TEXT("Fine drainage Pareto water route: label budget reached at %u."), After->ParetoRoute->LabelsCreated)
		);
	}
	const std::shared_ptr<const PlanetCore::PlanetErosionPreparation> Preparation = Candidate->GetErosionPreparation();
	// 候補の侵食段階を参照の失敗時に検査を続けない返戻
	if(!TestTrue(TEXT("候補の侵食段階を参照"), static_cast<bool>(Preparation))) return false;
	// 各地形合成段階の標高の比較
	const auto ReportLayers = [&](const TCHAR *Name, const std::uint32_t Index) {
		const PlanetCore::UnitDirection Direction = Fine->Directions[Index];
		const PlanetCore::Vec3 Vector = Direction.Vector();
		const PlanetCore::RiverElevationStages Stage = Candidate->EvaluateElevationStages(Direction);
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage %s index=%u direction=%.7f/%.7f/%.7f ") TEXT("source=%.3fm eroded=%.3fm aeolian=%.3fm."),
				Name,
				Index,
				Vector.X,
				Vector.Y,
				Vector.Z,
				Ocean->EvaluateElevation(Direction),
				Preparation->GetTerrain()->EvaluateElevation(Direction),
				Preparation->GetAeolianField()->EvaluateElevation(Direction)
			)
		);
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage %s layers: baseline=%.3fm valley=%.3fm ") TEXT("fine=%.3fm final=%.3fm."),
				Name,
				BaselineHeights[Index],
				Stage.AfterValleyMeters,
				Stage.AfterFineReliefMeters,
				Stage.FinalMeters
			)
		);
	};
	if(After->MaximumDirectSinkIndex != std::numeric_limits<std::uint32_t>::max()) {
		const std::uint32_t Index = After->MaximumDirectSinkIndex;
		ReportLayers(TEXT("worstSink"), Index);
		const std::optional<PlanetCore::PlanetMesoRelief> PreparedMeso =
		PlanetCore::PlanetMesoRelief::CreateForDrainage(Settings.Seed, Settings.RadiusMeters);
		// 最深窪地の中規模起伏を準備の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("最深窪地の中規模起伏を準備"), PreparedMeso.has_value())) return false;
		// 起伏追加に使う五次補間の再現
		const auto SmoothGate = [](const double T) {
			return T * T * T * (10 + T * (-15 + 6 * T));
		};
		// 同一地点の生成段階と独立再構成値の取得
		const auto Layers = [&](const std::uint32_t Site) {
			const PlanetCore::UnitDirection Direction = Fine->Directions[Site];
			const double Geological = Geology->EvaluateElevation(Direction);
			const double RidgeGate = std::clamp((Geological - Settings.SeaLevelMeters) / 500.0, 0.0, 1.0);
			const double Orogenic = Geological + SmoothGate(RidgeGate) * Ridges->EvaluateElevation(Direction);
			const double MesoGate = std::clamp((Orogenic - Settings.SeaLevelMeters - 150.0) / 850.0, 0.0, 1.0);
			const double Scale = Settings.RadiusMeters / (Settings.RadiusMeters + 250000.0);
			const double Reconstructed = Orogenic + Scale * SmoothGate(MesoGate) * PreparedMeso->EvaluateElevation(Direction);
			const PlanetCore::RiverElevationStages Stage = Candidate->EvaluateElevationStages(Direction);
			// 同一地点における各地形合成段階の返戻
			return std::array<double, 9>{
				Geological,
				Orogenic,
				Reconstructed,
				Ocean->EvaluateElevation(Direction),
				Preparation->GetTerrain()->EvaluateElevation(Direction),
				Preparation->GetAeolianField()->EvaluateElevation(Direction),
				Stage.AfterValleyMeters,
				Stage.AfterFineReliefMeters,
				Stage.FinalMeters
			};
		};
		const std::array<double, 9> Center = Layers(Index);
		std::array<double, 9> LayerNeighborMinima;
		LayerNeighborMinima.fill(std::numeric_limits<double>::infinity());
		double MaximumSourceResidual = std::abs(Center[2] - Center[3]);
		for(const std::uint32_t Neighbor : Fine->Neighbors[Index]) {
			const std::array<double, 9> Values = Layers(Neighbor);
			MaximumSourceResidual = std::max(MaximumSourceResidual, std::abs(Values[2] - Values[3]));
			for(std::size_t Layer = 0; Layer < Values.size(); ++Layer) {
				LayerNeighborMinima[Layer] = std::min(LayerNeighborMinima[Layer], Values[Layer]);
			}
		}
		AddInfo(
			FString::Printf(
				TEXT("Fine drainage worstSink neighbor rise: geology=%.3fm orogenic=%.3fm ")
				TEXT("reconstructed=%.3fm source=%.3fm eroded=%.3fm aeolian=%.3fm ")
				TEXT("valley=%.3fm fine=%.3fm final=%.3fm sourceResidual=%.6fm neighbors=%u."),
				LayerNeighborMinima[0] - Center[0],
				LayerNeighborMinima[1] - Center[1],
				LayerNeighborMinima[2] - Center[2],
				LayerNeighborMinima[3] - Center[3],
				LayerNeighborMinima[4] - Center[4],
				LayerNeighborMinima[5] - Center[5],
				LayerNeighborMinima[6] - Center[6],
				LayerNeighborMinima[7] - Center[7],
				LayerNeighborMinima[8] - Center[8],
				MaximumSourceResidual,
				static_cast<std::uint32_t>(Fine->Neighbors[Index].size())
			)
		);
	}
	if(After->IsolatedWetIndex != std::numeric_limits<std::uint32_t>::max()) {
		ReportLayers(TEXT("isolatedWet"), After->IsolatedWetIndex);
	}
	const std::array<std::pair<std::uint64_t, double>, 4> OceanCases {
		{ { 0, 6371000 }, { 90210, 6371000 }, { 42, 1000 }, { 42, 1E9 } }
	};
	for(const std::pair<std::uint64_t, double> &Case : OceanCases) {
		const std::uint64_t Seed = Case.first;
		const double Radius = Case.second;
		PlanetCore::GeologySettings OceanSettings;
		OceanSettings.Seed = Seed;
		OceanSettings.RadiusMeters = Radius;
		const std::optional<PlanetCore::GeologyField> PreparedCaseGeology = PlanetCore::GeologyField::Create(OceanSettings);
		// 別条件の地質を準備の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("別条件の地質を準備"), PreparedCaseGeology.has_value())) return false;
		const std::shared_ptr<const PlanetCore::GeologyField> CaseGeology =
		std::make_shared<const PlanetCore::GeologyField>(*PreparedCaseGeology);
		const std::optional<PlanetCore::PlanetOrogenicRidges> PreparedCaseRidges =
		PlanetCore::PlanetOrogenicRidges::Create(*CaseGeology);
		// 別条件の造山を準備の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("別条件の造山を準備"), PreparedCaseRidges.has_value())) return false;
		const std::shared_ptr<const PlanetCore::PlanetOrogenicRidges> CaseRidges =
		std::make_shared<const PlanetCore::PlanetOrogenicRidges>(*PreparedCaseRidges);
		const std::optional<PlanetCore::OceanConnectivityField> CaseOcean =
		PlanetCore::OceanConnectivityField::CreateWithPredrainageRelief(CaseGeology, CaseRidges);
		// 別条件の細分海域を準備の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("別条件の細分海域を準備"), CaseOcean.has_value())) return false;
		TestEqual(TEXT("別条件も一段細分化"), CaseOcean->Statistics().InputGraphNodes, std::uint32_t{ 163842 });
		TestEqual(TEXT("別条件の海域も一体"), CaseOcean->Statistics().ComponentsAfter, std::uint32_t{ 1 });
		const std::optional<std::array<std::uint32_t, 2>> CoarseCase = CountCoarseOceanComponents(*CaseOcean);
		// 別条件の粗い海域接続を診断の失敗時に検査を続けない返戻
		if(!TestTrue(TEXT("別条件の粗い海域接続を診断"), CoarseCase.has_value())) return false;
		const std::optional<PlanetCore::DrainageNetwork> CaseDrainage = PlanetCore::DrainageNetwork::Create(*CaseOcean);
		TestTrue(TEXT("元海域場の接続証拠で別条件の排水を準備"), CaseDrainage.has_value());
		AddInfo(
			FString::Printf(
				TEXT("Source ocean resolution: seed=%llu radius=%.0fm fineComponents=%u ")
				TEXT("coarseWet=%u coarseComponents=%u drainage=%u."),
				static_cast<unsigned long long>(Seed),
				Radius,
				CaseOcean->Statistics().ComponentsAfter,
				(*CoarseCase)[0],
				(*CoarseCase)[1],
				CaseDrainage.has_value() ? 1U : 0U
			)
		);
		const double OceanFraction = CaseOcean->Statistics().IndependentOceanFractionEstimate;
		TestTrue(TEXT("別条件の海域割合も指定範囲"), OceanFraction >= 0.60 && OceanFraction <= 0.70);
		AddInfo(
			FString::Printf(
				TEXT("Fine ocean case: seed=%llu radius=%.0fm componentsBefore=%u ") TEXT("componentsAfter=%u oceanFraction=%.6f."),
				static_cast<unsigned long long>(Seed),
				Radius,
				CaseOcean->Statistics().ComponentsBefore,
				CaseOcean->Statistics().ComponentsAfter,
				OceanFraction
			)
		);
	}
	// 要求した全比較の実行を終えた結果の返戻；地形品質の合格とは区別する
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetMixedProfileContracts, "PlanetGenerator.FinalDrainage.MixedProfileContracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 縦断の単調回帰を全連続区間分割の独立な最小二乗解と比較する
 * @param Parameters 未使用の UE 自動検査引数
 * @return 不正値拒否と全比較を満たす場合に真
 */
bool FPlanetMixedProfileContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	TestFalse(TEXT("空の縦断を拒否"), FitMixedProfile({}, {}, 0).has_value());
	TestFalse(TEXT("重み数の不整合を拒否"), FitMixedProfile({ 1 }, {}, 0).has_value());
	TestFalse(TEXT("非正の重みを拒否"), FitMixedProfile({ 1 }, { 0 }, 0).has_value());
	TestFalse(TEXT("負の重みを拒否"), FitMixedProfile({ 1 }, { -1 }, 0).has_value());
	TestFalse(TEXT("非有限の下限を拒否"), FitMixedProfile({ 1 }, { 1 }, std::numeric_limits<double>::quiet_NaN()).has_value());
	TestFalse(TEXT("非有限の水位を拒否"), FitMixedProfile({ std::numeric_limits<double>::infinity() }, { 1 }, 0).has_value());
	std::uint32_t Cases = 0;
	for(std::uint32_t Count = 1; Count < 7; ++Count) {
		std::uint32_t Combinations = 1;
		for(std::uint32_t Index = 0; Index < Count; ++Index) Combinations *= 3;
		for(std::uint32_t Code = 0; Code < Combinations; ++Code) {
			std::vector<double> Targets(Count), Weights(Count);
			std::uint32_t Digits = Code;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				Targets[Index] = static_cast<double>(Digits % 3) * 2 - 1;
				Weights[Index] = Index % 3 + 1;
				Digits /= 3;
			}
			constexpr double Lower = 0.25;
			const std::optional<std::vector<double>> Actual = FitMixedProfile(Targets, Weights, Lower);
			// 有限の正重み縦断を準備の失敗時に検査を続けない返戻
			if(!TestTrue(TEXT("有限の正重み縦断を準備"), Actual.has_value())) return false;
			double Best = std::numeric_limits<double>::infinity();
			// PAVA を用いず，全ての連続区間分割について平均と可否を直接計算する
			for(std::uint32_t Mask = 0; Mask < (1U << (Count - 1)); ++Mask) {
				double Previous = std::numeric_limits<double>::infinity(), Cost = 0;
				bool Feasible = true;
				for(std::uint32_t Begin = 0; Begin < Count;) {
					std::uint32_t End = Begin + 1;
					while(End < Count && !(Mask & (1U << (End - 1)))) ++End;
					double Moment = 0, Weight = 0;
					for(std::uint32_t Index = Begin; Index < End; ++Index) {
						Moment += Targets[Index] * Weights[Index];
						Weight += Weights[Index];
					}
					const double Level = std::max(Lower, Moment / Weight);
					Feasible &= Level <= Previous;
					Previous = Level;
					for(std::uint32_t Index = Begin; Index < End; ++Index) {
						const double Difference = Targets[Index] - Level;
						Cost += Weights[Index] * Difference * Difference;
					}
					Begin = End;
				}
				if(Feasible) Best = std::min(Best, Cost);
			}
			double ActualCost = 0;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				TestTrue(TEXT("海口下限以上の非増加水位"), (*Actual)[Index] >= Lower && (!Index || (*Actual)[Index - 1] >= (*Actual)[Index]));
				const double Difference = Targets[Index] - (*Actual)[Index];
				ActualCost += Weights[Index] * Difference * Difference;
			}
			TestTrue(TEXT("全分割列挙と最小二乗費用が一致"), std::abs(ActualCost - Best) <= 1E-10);
			++Cases;
		}
	}
	AddInfo(FString::Printf(TEXT("Mixed profile contracts: %u weighted sequences checked against every partition."), Cases));
	// 実行した全比較の結果の返戻
	return !HasAnyErrors();
}

#endif
