#include "PlanetSpillNetwork.h"
#include "PlanetSpillPassage.h"
#include "PlanetSpillSampleCache.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 実際にコンパイルした準備手順の版を返し，利用側のヘッダと混同しない
	 * @return 此の実装に対応する版
	 */
	std::uint32_t GetSpillNetworkVersion() {
		// コンパイルした生成手順の版の返戻
		return 46;
	}

	namespace {
		/**
		 * 同じ元場の停留点を物理距離と標高で照合する
		 * @param A 一方の確認済停留点
		 * @param B 他方の確認済停留点
		 * @param Radius 物理半径
		 * @return 種類・位置・標高が既存の統合精度内で一致する場合に真
		 */
		bool SameNetworkCritical(const TerrainCriticalPoint &A, const TerrainCriticalPoint &B, const double Radius) {
			const Vec3 P = A.Direction.Vector(), Q = B.Direction.Vector();
			// 別の窪地を水位だけで同一視しない返戻
			return A.Kind == B.Kind && Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q)) <= 0.001 &&
			std::abs(A.ElevationMeters - B.ElevationMeters) <= 1E-6;
		}
	}

	/**
	 * 確認済の極小から，包含全体の実流出口と受入水位を取得する
	 * @param Network 大洋接続を確認した同じ元場の有限網
	 * @param Site 網に登録された陸上の極小
	 * @return 下流の実枝と水位；大洋・範囲外・不整合の場合は空
	 */
	std::optional<SpillNetworkOutlet> GetSpillNetworkOutlet(const SpillNetwork &Network, const std::uint32_t Site) {
		const ContinuousSpillGraph &Graph = Network.Graph;
		const DepressionHierarchy &Hierarchy = Graph.Hierarchy;
		// 大洋又は未登録の地点は陸上の流出口を持たない返戻
		if(Site >= Graph.Sites.size() || Site >= Hierarchy.SiteRoots.size() || Graph.Sites[Site].IsOcean) return {};
		const std::uint32_t Root = Hierarchy.SiteRoots[Site];
		// 未確定の包含根への返戻
		if(!Root || Root >= Hierarchy.Nodes.size()) return {};
		const DepressionHierarchyNode &Basin = Hierarchy.Nodes[Root];
		const bool HasOutlet = Basin.OutletEdge < Graph.Edges.size() && Basin.OutletEdge < Graph.EdgeConnections.size() &&
		Basin.OutletSite < Hierarchy.SiteRoots.size() && Basin.ReceivingSite < Hierarchy.SiteRoots.size();
		// 対応する通路を失った場合の返戻
		if(!HasOutlet) return {};
		const std::uint32_t Receiver = Hierarchy.SiteRoots[Basin.ReceivingSite];
		const std::uint32_t Connection = Graph.EdgeConnections[Basin.OutletEdge];
		const bool HasReceiver =
		Receiver < Hierarchy.Nodes.size() && Receiver != Root && Hierarchy.SiteRoots[Basin.OutletSite] == Root &&
		Connection < Network.Connections.size() && Connection < Graph.BranchSites.size();
		// 同じ水域へ戻る偽の下流又は未対応の枝への返戻
		if(!HasReceiver) return {};
		const DepressionHierarchyNode &Receiving = Hierarchy.Nodes[Receiver];
		const std::array<std::uint32_t, 2> &Branches = Graph.BranchSites[Connection];
		const bool Positive = Branches[1] == Basin.ReceivingSite;
		const bool HasBranch =
		(Positive ? Branches[0] : Branches[1]) == Basin.OutletSite && (Positive ? Branches[1] : Branches[0]) == Basin.ReceivingSite;
		const bool HasWater =
		std::isfinite(Basin.SpillMeters) && std::isfinite(Receiving.SpillMeters) && !(Basin.SpillMeters < Receiving.SpillMeters) &&
		Receiving.OceanLinkRank < Basin.OceanLinkRank && Basin.SpillMeters == Network.Connections[Connection].Pass.ElevationMeters;
		// 枝の向き又は下降する水位と接続順が一致しない場合の返戻
		if(!HasBranch || !HasWater) return {};
		// 同水位の接続は湖の連結として保持し，正の河川勾配へ捏造しない返戻
		return SpillNetworkOutlet{ Root, Receiver, Connection, Positive, Basin.SpillMeters, Receiving.SpillMeters };
	}

	/**
	 * 初期鞍点から実接続を発見し，水没通路で大洋を確認して排水階層を準備する
	 * @param InitialPass 不変な元場の初期鞍点候補
	 * @param Source 同じ尺度の不変な標高と解析接線勾配
	 * @param OceanAnchors 同じ元場で所属を確認可能な海域標本の候補
	 * @param IsConnectedOceanAnchor 標本が共通の大洋に属する事を別途確認する述語
	 * @param Settings 半径・海面と有限の評価・保持予算
	 * @param Stop 中断要求
	 * @param Diagnostics 実際の費用と失敗理由
	 * @return 入力の水系全体を同じ大洋へ結んだ有限網；部分的な成功は返さない
	 */
	std::optional<SpillNetwork> PrepareSpillNetwork(
		const UnitDirection InitialPass,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::span<const UnitDirection> OceanAnchors,
		const std::function<bool(UnitDirection)> &IsConnectedOceanAnchor,
		const SpillNetworkSettings &Settings,
		const std::stop_token Stop,
		SpillNetworkDiagnostics *const Diagnostics
	) {
		SpillNetworkDiagnostics Local;
		SpillNetworkDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		bool HasFiniteSource = true, HasCompleteEvaluations = true;
		const auto Reject = [&](const SpillNetworkFailure Failure) -> std::optional<SpillNetwork> {
			Report.Failure = Stop.stop_requested() ?
			SpillNetworkFailure::Cancelled :
			!HasCompleteEvaluations ? SpillNetworkFailure::BudgetLimit : !HasFiniteSource ? SpillNetworkFailure::InvalidSource : Failure;
			// 未完成の水系を公開しない返戻
			return {};
		};
		const bool Valid = Source && IsConnectedOceanAnchor && !OceanAnchors.empty() && OceanAnchors.size() < 1000001 &&
		std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 &&
		std::isfinite(Settings.SeaLevelMeters) && Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 100000001 &&
		Settings.MaximumSteps > 0 && Settings.MaximumSteps < 4097 && Settings.MaximumConnections > 0 &&
		Settings.MaximumConnections < 4097 && Settings.MaximumBranchSamples > 0 && Settings.MaximumBranchSamples < 1048577 &&
		Settings.MaximumCachedSamples <= SpillSampleCache::MaximumCapacity;
		// 無効な設定又は開始前の中断への返戻
		if(!Valid || Stop.stop_requested()) return Reject(SpillNetworkFailure::InvalidInput);
		// 準備終了時に解放し，最終場や次の生成定義へ再利用枠を持越さない
		std::optional<SpillSampleCache> Cached = SpillSampleCache::Create(Settings.MaximumCachedSamples);
		// 無効な保持容量を参照前に拒否する返戻
		if(!Cached) return Reject(SpillNetworkFailure::InvalidInput);
		const auto Sample = [&](const UnitDirection D) -> TerrainSample {
			// 中断後は保持済の値も返さない
			if(Stop.stop_requested()) {
				HasCompleteEvaluations = false;
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 中断時の未評価値の返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			if(const std::optional<TerrainSample> Stored = Cached->Find(D)) {
				++Report.CacheHits;
				// 同じ元場を同じ方向で評価済の場合だけ，標高と全微分を其の儘返す
				return *Stored;
			}
			if(Stop.stop_requested() || Report.Evaluations >= Settings.MaximumEvaluations) {
				HasCompleteEvaluations = false;
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 共通予算後に元場を呼出さない返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Report.Evaluations;
			const TerrainSample Value = Source(D);
			HasFiniteSource =
			HasFiniteSource && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			if(HasFiniteSource) Cached->Store(D, Value);
			Report.CachedSamples = Cached->GetCount();
			// 衝突又は未保持の場合も同じ元場の実評価の返戻
			return Value;
		};
		// 初期の実鞍点と両下降枝
		SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Settings.RadiusMeters;
		ConnectionSettings.SeaLevelMeters = Settings.SeaLevelMeters;
		const std::optional<SpillConnection> Initial = PrepareSpillConnection(InitialPass, Sample, ConnectionSettings, Stop);
		// 初期の部分経路への返戻
		if(!Initial) return Reject(SpillNetworkFailure::InitialPass);
		SpillNetwork Result;
		std::vector<TerrainCriticalPoint> Minima;
		std::size_t BranchSamples = 0;
		bool HasSeaTerminal = false;
		const auto Append = [&](const SpillConnection &Connection) -> bool {
			for(const SpillConnection &Existing : Result.Connections) {
				// 同じ実鞍点の重複追加を行わない返戻
				if(SameNetworkCritical(Existing.Pass, Connection.Pass, Settings.RadiusMeters)) return true;
			}
			const std::size_t AddedSamples = Connection.Branches[0].Path.Samples.size() + Connection.Branches[1].Path.Samples.size();
			// 全探索段階を合算した保持量の検査
			if(Result.Connections.size() >= Settings.MaximumConnections || BranchSamples + AddedSamples > Settings.MaximumBranchSamples) {
				// 保持予算を超える接続を追加しない返戻
				return false;
			}
			BranchSamples += AddedSamples;
			Result.Connections.push_back(Connection);
			for(const SpillConnectionBranch &Branch : Connection.Branches) {
				if(Branch.Kind == SpillTerminalKind::SeaLevel) HasSeaTerminal = true;
				if(!Branch.Minimum) continue;
				const bool Known = std::any_of(
					Minima.begin(),
					Minima.end(),
					[&](const TerrainCriticalPoint &P) -> bool {
						// 同じ元場の共有極小の照合の返戻
						return SameNetworkCritical(P, *Branch.Minimum, Settings.RadiusMeters);
					}
				);
				if(!Known) {
					// 次の探索へ渡せない大きさの集合を保持しない返戻
					if(Minima.size() >= Settings.MaximumConnections + 1) return false;
					Minima.push_back(*Branch.Minimum);
				}
			}
			Report.PreparedConnections = static_cast<std::uint32_t>(Result.Connections.size());
			Report.PreparedMinima = static_cast<std::uint32_t>(Minima.size());
			Report.RetainedBranchSamples = static_cast<std::uint32_t>(BranchSamples);
			// 予算内の実接続を保持した返戻
			return true;
		};
		// 初期接続も全体の保持予算に含める返戻
		if(!Append(*Initial)) return Reject(SpillNetworkFailure::BudgetLimit);
		// 目標方向と大洋所属の証拠を別々に準備
		std::vector<UnitDirection> Anchors;
		double NearestOcean = std::numeric_limits<double>::infinity();
		std::optional<UnitDirection> Target;
		for(const UnitDirection D : OceanAnchors) {
			// 所属照会前の中断への返戻
			if(Stop.stop_requested()) return Reject(SpillNetworkFailure::Cancelled);
			if(!IsConnectedOceanAnchor(D)) continue;
			const TerrainSample Value = Sample(D);
			// 元場の欠落又は共通予算の超過への返戻
			if(!HasFiniteSource || !HasCompleteEvaluations) return Reject(SpillNetworkFailure::InvalidSource);
			if(!(Value.ElevationMeters < Settings.SeaLevelMeters)) continue;
			Anchors.push_back(D);
			const Vec3 A = InitialPass.Vector(), B = D.Vector();
			const double Distance = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
			if(Distance < NearestOcean) {
				NearestOcean = Distance;
				Target = D;
			}
		}
		// 海面以下の未所属地点を大洋へ置換えない返戻
		if(!Target) return Reject(SpillNetworkFailure::NoOceanAnchor);
		Report.InitialOceanDistanceMeters = NearestOcean;
		SpillDiscoverySettings DiscoverySettings;
		DiscoverySettings.RadiusMeters = Settings.RadiusMeters;
		DiscoverySettings.MaximumKnownMinima = Settings.MaximumConnections + 1;
		DiscoverySettings.SeaLevelMeters = Settings.SeaLevelMeters;
		// 格子は候補探索に限定し，元場の実鞍点と下降枝は同じ精度で確認
		DiscoverySettings.HalfCells = 64;
		DiscoverySettings.CandidateSpacingMeters = 16;
		DiscoverySettings.MaximumClimbingAttempts = 8;
		// 前線を進める時，成分内の低い別通路も保持して水域の包含を失わない
		DiscoverySettings.IncludeInternalConnections = Settings.UseClimbingFrontier;
		DiscoverySettings.InitialHalfWidthMeters = std::min(256.0, Settings.RadiusMeters * 0.0625);
		DiscoverySettings.MaximumHalfWidthMeters = std::min(262144.0, Settings.RadiusMeters * 0.25);
		DiscoverySettings.MaximumEvaluations = std::min(Settings.MaximumEvaluations, 4000000U);
		DiscoverySettings.OutletTarget = Target;
		// 元場と目標方向が不変な間，同じ有限方向で未発見だった中心を繰返さない
		std::vector<bool> InitialWindowsScanned, ClimbingScanned, WindowsScanned;
		// 極小毎の最寄りの確認済海域を一度だけ求め，固定初期目標への遠回りを避ける
		std::vector<UnitDirection> FrontierTargets;
		std::vector<double> FrontierOceanSquaredChords;
		const auto Discover = [&]() -> std::optional<SpillDiscoveryBatch> {
			if(!Settings.UseClimbingFrontier) {
				++Report.CenterQueries;
				// 比較用の従来の窓・登坂・緩和の順序を保持した返戻
				return FindSpillComponentExits(Minima, Sample, DiscoverySettings, Stop, &Report.Discovery, Result.Connections);
			}
			while(FrontierTargets.size() < Minima.size()) {
				const Vec3 D = Minima[FrontierTargets.size()].Direction.Vector();
				UnitDirection Nearest = Anchors.front();
				double Best = std::numeric_limits<double>::infinity();
				for(const UnitDirection Anchor : Anchors) {
					// 所属を確認済の海域標本の走査中にも中断を反映する返戻
					if(Stop.stop_requested()) return {};
					const Vec3 Delta = D - Anchor.Vector();
					const double Squared = Delta.Dot(Delta);
					if(Squared < Best) {
						Best = Squared;
						Nearest = Anchor;
					}
				}
				FrontierTargets.push_back(Nearest);
				FrontierOceanSquaredChords.push_back(Best);
			}
			InitialWindowsScanned.resize(Minima.size(), false);
			ClimbingScanned.resize(Minima.size(), false);
			WindowsScanned.resize(Minima.size(), false);
			const auto Key = [&](const std::size_t Index) -> std::tuple<double, double, double, double, double> {
				const TerrainCriticalPoint &Minimum = Minima[Index];
				const Vec3 D = Minimum.Direction.Vector();
				// 同じ入力成分内で最寄りの確認済大洋へ近い前線を選ぶ；近点でも差の桁を失わない返戻
				return { FrontierOceanSquaredChords[Index], Minimum.ElevationMeters, D.X, D.Y, D.Z };
			};
			const std::size_t Nearest = static_cast<std::size_t>(
				std::min_element(FrontierOceanSquaredChords.begin(), FrontierOceanSquaredChords.end()) - FrontierOceanSquaredChords.begin()
			);
			const Vec3 A = Minima[Nearest].Direction.Vector(), B = FrontierTargets[Nearest].Vector();
			Report.ClosestOceanDistanceMeters = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
			while(true) {
				std::size_t Center = Minima.size();
				unsigned Phase = 0;
				// 費用の小さい全中心の窓を先行し，確認済の前線から広窓へ進む
				for(unsigned Mode = 0; Mode < 3; ++Mode) {
					const std::vector<bool> &Scanned = !Mode ? InitialWindowsScanned : Mode == 1 ? ClimbingScanned : WindowsScanned;
					for(std::size_t I = 0; I < Minima.size(); ++I) {
						if(Scanned[I]) continue;
						if(Center == Minima.size() || Key(I) < Key(Center)) Center = I;
					}
					if(Center < Minima.size()) {
						Phase = Mode;
						break;
					}
				}
				// 全ての有限な中心照会で未発見だった場合の返戻
				if(Center == Minima.size() || !HasFiniteSource || !HasCompleteEvaluations || Stop.stop_requested()) return {};
				DiscoverySettings.PreferredCenter = Minima[Center].Direction;
				DiscoverySettings.OutletTarget = FrontierTargets[Center];
				DiscoverySettings.InitialWindowOnly = !Phase;
				DiscoverySettings.ClimbingOnly = Phase == 1;
				++Report.CenterQueries;
				std::optional<SpillDiscoveryBatch> Batch =
				FindSpillComponentExits(Minima, Sample, DiscoverySettings, Stop, &Report.Discovery, Result.Connections);
				// 元場で新たな実接続を確認出来た結果の返戻
				if(Batch) return Batch;
				const bool CanAdvance =
				Report.Discovery.Failure == SpillDiscoveryFailure::NoIncidentExit && HasFiniteSource && HasCompleteEvaluations &&
				!Stop.stop_requested();
				// 元場・中断・全予算の不足を別の中心へ移して隠さない返戻
				if(!CanAdvance) return {};
				if(!Phase) {
					InitialWindowsScanned[Center] = true;
					++Report.ExhaustedInitialWindowCenters;
				} else if(Phase == 1) {
					ClimbingScanned[Center] = true;
					++Report.ExhaustedClimbingCenters;
				} else {
					WindowsScanned[Center] = true;
					++Report.ExhaustedWindowCenters;
				}
			}
		};
		while(!HasSeaTerminal && Report.Steps < Settings.MaximumSteps) {
			++Report.Steps;
			Report.ClosestFrontierDistanceMeters = std::numeric_limits<double>::infinity();
			for(const TerrainCriticalPoint &Minimum : Minima) {
				const Vec3 A = Minimum.Direction.Vector(), B = Target->Vector();
				const double Distance = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				Report.ClosestFrontierDistanceMeters = std::min(Report.ClosestFrontierDistanceMeters, Distance);
			}
			const std::optional<SpillDiscoveryBatch> Batch = Discover();
			// 探索の途中失敗を部分的な水系として公開しない返戻
			if(!Batch) return Reject(SpillNetworkFailure::Discovery);
			for(const SpillConnection &Connection : Batch->Connections) {
				// 保持量を超えた場合の返戻
				if(!Append(Connection)) return Reject(SpillNetworkFailure::BudgetLimit);
			}
		}
		// 探索段階の上限に達した場合の返戻
		if(!HasSeaTerminal) return Reject(SpillNetworkFailure::BudgetLimit);
		// 個々の海面終端から，共通の大洋へ水没した通路を実際に確認
		for(const SpillConnection &Connection : Result.Connections) for(const SpillConnectionBranch &Branch : Connection.Branches) {
			if(Branch.Kind != SpillTerminalKind::SeaLevel) continue;
			const UnitDirection Terminal = Branch.Path.Samples.back().Direction;
			UnitDirection Anchor = Anchors.front();
			double Nearest = std::numeric_limits<double>::infinity();
			for(const UnitDirection D : Anchors) {
				const Vec3 A = Terminal.Vector(), B = D.Vector();
				const double Distance = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
				if(Distance < Nearest) {
					Nearest = Distance;
					Anchor = D;
				}
			}
			bool Verified = false;
			double PreviousOceanWidth = 0;
			for(const double HalfWidth : { 1000.0, 2000.0, 4000.0, 8000.0, 16000.0, 32000.0, 64000.0 }) {
				const double ActualWidth = std::min(HalfWidth, Settings.RadiusMeters * 0.25);
				if(ActualWidth == PreviousOceanWidth) continue;
				PreviousOceanWidth = ActualWidth;
				SpillPassageSettings WetSettings;
				WetSettings.RadiusMeters = Settings.RadiusMeters;
				WetSettings.SpacingMeters = std::min(50.0, Settings.RadiusMeters * 0.025);
				WetSettings.HalfWidthMeters = ActualWidth;
				WetSettings.MaximumEvaluations = std::min(Settings.MaximumEvaluations, 4000000U);
				// 通常の所属確認にも，長い候補通路の保持量だけの調整を使う
				WetSettings.AdaptCandidateSpacingToBudget = true;
				WetSettings.UseEndpointNeighborRing = true;
				++Report.OceanPassages;
				const std::optional<SpillPassage> Wet = FindSpillPassage(Terminal, Anchor, Sample, WetSettings, Stop);
				// 海域検査の未計測を別の幅で隠さない返戻
				if(!HasFiniteSource || !HasCompleteEvaluations || Stop.stop_requested()) return Reject(SpillNetworkFailure::OceanPassage);
				if(!Wet) continue;
				const double CheckedMaximum = Wet->CrestMeters + Wet->MaximumIndependentExcessMeters;
				if(CheckedMaximum <= Settings.SeaLevelMeters) {
					Result.OceanWitnesses.push_back({ Terminal, Anchor, CheckedMaximum });
					Verified = true;
					break;
				}
			}
			// 孤立した水没域を大洋の代表へ結ばない返戻
			if(!Verified) return Reject(SpillNetworkFailure::OceanPassage);
		}
		const auto IsVerifiedOcean = [&](const UnitDirection D) -> bool {
			const Vec3 A = D.Vector();
			for(const SpillOceanWitness &Witness : Result.OceanWitnesses) {
				const Vec3 B = Witness.Terminal.Vector();
				// 検査した実終端だけを受理する返戻
				if(A.X == B.X && A.Y == B.Y && A.Z == B.Z) return true;
			}
			// 水位だけ一致する未検査の地点への返戻
			return false;
		};
		ContinuousSpillSettings GraphSettings;
		GraphSettings.RadiusMeters = Settings.RadiusMeters;
		GraphSettings.SeaLevelMeters = Settings.SeaLevelMeters;
		std::optional<ContinuousSpillGraph> Graph =
		BuildContinuousSpillGraph(Result.Connections, IsVerifiedOcean, GraphSettings, Stop, &Report.Graph);
		// 一部の極小を捨てて接続したと見做さない返戻
		if(!Graph) return Reject(SpillNetworkFailure::Graph);
		Result.Graph = std::move(*Graph);
		// 全準備の終了時にも中断を確認する返戻
		if(Stop.stop_requested()) return Reject(SpillNetworkFailure::Cancelled);
		// 実際に大洋へ結んだ不変データの準備結果の返戻
		return Result;
	}
}
