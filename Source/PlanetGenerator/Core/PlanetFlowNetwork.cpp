#include "PlanetFlowNetwork.h"
#include "PlanetFlowNetworkPreparation.h"
#include "PlanetLakeTopology.h"
#include "PlanetLakeSill.h"
#include "PlanetSpillPassage.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	struct FlowNetworkPreparation::Impl {
		// 受入先は必ず先に保持した下流側に属し，別の古い版を蓄積しない
		struct Entry {
			std::uint32_t Connection = MissingSpillIndex;
			Vec3 Pass;
			Vec3 MinimumDirection;
			double UpstreamWaterMeters = 0;
			double DownstreamWaterMeters = 0;
			bool IsPositiveDownstream = false;
			std::shared_ptr<const SpillLakeShore> Receiving;
			PreparedFlowChannel Channel;
			FlowPreparationDiagnostics Diagnostics;

			bool HasSameInputs(
				const SpillNetworkOutlet &Outlet,
				const Vec3 CurrentPass,
				const Vec3 CurrentMinimumDirection,
				const std::shared_ptr<const SpillLakeShore> &ReceivingLake
			) const;
		};

		const std::function<TerrainSample(UnitDirection)> Source;
		const std::function<double(UnitDirection)> SourceHeight;
		const FlowNetworkSettings Settings;
		std::vector<Entry> Prefix;
		static std::uint64_t MakeChannelSeed(const std::uint64_t Seed, const TerrainCriticalPoint &Pass);
	};

	/**
	 * 同じ不変な元場の実峠・水位・受入先を，符号付き零も区別して照合する
	 * @param Outlet 現在の網で確認済の流出口
	 * @param CurrentPass 現在の実峠の方向
	 * @param CurrentMinimumDirection 枝の符号に対応する元の主方向
	 * @param ReceivingLake 先に準備した同じ受入岸，省略は大洋
	 * @return 入力の全成分と所有する受入先が一致する場合に真
	 */
	bool FlowNetworkPreparation::Impl::Entry::HasSameInputs(
		const SpillNetworkOutlet &Outlet,
		const Vec3 CurrentPass,
		const Vec3 CurrentMinimumDirection,
		const std::shared_ptr<const SpillLakeShore> &ReceivingLake
	) const {
		// 近接値・符号付き零を同一視せず，元の倍精度成分で照合
		const auto IsSame = [](const double A, const double B) -> bool {
			return std::bit_cast<std::uint64_t>(A) == std::bit_cast<std::uint64_t>(B);
		};
		// 不変な定義の全入力と受入岸の一致の返戻
		return Connection == Outlet.Connection && IsSame(Pass.X, CurrentPass.X) && IsSame(Pass.Y, CurrentPass.Y) &&
		IsSame(Pass.Z, CurrentPass.Z) && IsSame(MinimumDirection.X, CurrentMinimumDirection.X) &&
		IsSame(MinimumDirection.Y, CurrentMinimumDirection.Y) && IsSame(MinimumDirection.Z, CurrentMinimumDirection.Z) &&
		IsSame(UpstreamWaterMeters, Outlet.UpstreamWaterMeters) && IsSame(DownstreamWaterMeters, Outlet.DownstreamWaterMeters) &&
		IsPositiveDownstream == Outlet.IsPositiveDownstream && Receiving.get() == ReceivingLake.get();
	}

	/**
	 * 接続の投入番号を使わず，同じ実峠の物理成分から河道の乱数を固定する
	 * @param Seed 不変な惑星の生成条件
	 * @param Pass 元場で確認した同じ実峠
	 * @return 方向３成分と標高の全ビットに対応する有限な乱数系列の種
	 * @note 別の精査開始点から得た停留点の丸め差を正準化する物ではない
	 */
	std::uint64_t FlowNetworkPreparation::Impl::MakeChannelSeed(const std::uint64_t Seed, const TerrainCriticalPoint &Pass) {
		const Vec3 D = Pass.Direction.Vector();
		const std::array<std::uint64_t, 4> Values {
			std::bit_cast<std::uint64_t>(D.X),
			std::bit_cast<std::uint64_t>(D.Y),
			std::bit_cast<std::uint64_t>(D.Z),
			std::bit_cast<std::uint64_t>(Pass.ElevationMeters)
		};
		std::uint64_t State = Seed ^ UINT64_C(0X4348414E4E454C32);
		for(const std::uint64_t Component : Values) {
			std::uint64_t Value = Component + UINT64_C(0X9E3779B97F4A7C15);
			Value = (Value ^ (Value >> 30)) * UINT64_C(0XBF58476D1CE4E5B9);
			Value = (Value ^ (Value >> 27)) * UINT64_C(0X94D049BB133111EB);
			State = std::rotl(State, 17) ^ (Value ^ Value >> 31);
		}
		// 同じ物理流出口と生成条件に固定した系列の返戻
		return State;
	}

	/**
	 * 不変な元場と生成条件を固定し，別の生成定義へ準備を持出さない
	 * @param Source 所有する不変場を捕捉した標高と勾配
	 * @param SourceHeight 同じ不変場の標高専用照会，省略可
	 * @param Settings 準備全体に固定する半径・乱数条件と保持予算
	 */
	FlowNetworkPreparation::FlowNetworkPreparation(
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::function<double(UnitDirection)> &SourceHeight,
		const FlowNetworkSettings &Settings
	) : State(std::make_unique<Impl>(Impl{ Source, SourceHeight, Settings })) {
		// 終了
		return;
	}

	/** 所有する準備中の下流側だけを解放する */
	FlowNetworkPreparation::~FlowNetworkPreparation() = default;

	/**
	 * 確認済の水系の全流出口を下流順に準備し，受入湖の岸を同じ定義で共有する
	 * @param Network 同じ元場で大洋への接続を確認した有限網
	 * @param Stop 追跡・岸・縦断準備にも伝播する中断要求
	 * @param Diagnostics 拒否した流域と，実際に準備出来た範囲
	 * @return 全流出口を準備した有限網；一つでも未達なら空
	 * @note 地点数 N，辺数 E，流出口数 K，補正数 S の管理費用は O((N+K) log(K+1)+E log(S+1)+S*(S+K))
	 * @note 各河道・湖岸・通水の元場照会を除く；補正場の照会は元場に加えて O(S)，補正は最大１０２４個
	 */
	std::optional<PreparedFlowNetwork> FlowNetworkPreparation::Prepare(
		const SpillNetwork &Network,
		const std::stop_token Stop,
		FlowNetworkDiagnostics *const Diagnostics
	) {
		const std::function<TerrainSample(UnitDirection)> &Source = State->Source;
		const std::function<double(UnitDirection)> &SourceHeight = State->SourceHeight;
		const FlowNetworkSettings &Settings = State->Settings;
		FlowNetworkDiagnostics Local;
		FlowNetworkDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		const auto Reject = [&](const FlowNetworkFailure Failure) -> std::optional<PreparedFlowNetwork> {
			Report.Failure = Stop.stop_requested() ? FlowNetworkFailure::Cancelled : Failure;
			// 部分的な河道群を完成した水系として公開しない返戻
			return {};
		};
		const ContinuousSpillGraph &Graph = Network.Graph;
		const double Radius = Settings.Channel.Outlet.RadiusMeters;
		const bool Valid = Source && !Graph.Sites.empty() && Graph.Sites.size() == Graph.Hierarchy.SiteRoots.size() &&
		Graph.Minima.size() == Graph.Sites.size() && !Graph.Hierarchy.Nodes.empty() && std::isfinite(Radius) && Radius >= 1000 &&
		Radius <= 1E9 && Settings.MaximumChannels > 0 && Settings.MaximumChannels < 1025 && Settings.MaximumGeometrySamples > 0 &&
		Settings.MaximumGeometrySamples < 1048577;
		// 未設定・無制限・中断済の入力への返戻
		if(!Valid || Stop.stop_requested()) return Reject(FlowNetworkFailure::InvalidInput);
		const std::optional<LakeTopology> Lakes = BuildLakeTopology(Graph.Hierarchy, Stop);
		// 元の包含と水域の対応が不整合なら，元場の高価な追跡を開始しない返戻
		if(!Lakes) return Reject(FlowNetworkFailure::InvalidInput);
		Report.LakeComplexes = static_cast<std::uint32_t>(Lakes->Complexes.size());
		for(const LakeComplex &Complex : Lakes->Complexes) {
			Report.EqualWaterConnections += static_cast<std::uint32_t>(Complex.InternalSpillBasins.size());
			if(Complex.Basins.size() > 1) {
				++Report.CompoundWaterBodies;
				for(const std::uint32_t Basin : Complex.Basins) if(Basin && !Report.Basin) Report.Basin = Basin;
			}
		}
		// 複数の外部鞍点又は海岸を持つ水域は，単一の外岸へ置換えない
		for(const LakeComplex &Complex : Lakes->Complexes) {
			if(Complex.Basins.size() > 1 && (Complex.IsOcean || Complex.OutletBasins.size() != 1)) {
				// 別の流出口を削除して成功にしない返戻
				return Reject(FlowNetworkFailure::EqualWaterConnection);
			}
		}
		std::vector<std::uint32_t> Representatives(Graph.Hierarchy.Nodes.size(), MissingSpillIndex);
		for(const LakeComplex &Complex : Lakes->Complexes) {
			if(Complex.IsOcean || Complex.OutletBasins.empty()) continue;
			for(const std::uint32_t Basin : Complex.Basins) Representatives[Basin] = Complex.OutletBasins.front();
		}
		// 同じ包含水域の流出口を一つに集約し，湖内の子を別の河川にしない
		std::map<std::uint32_t, std::vector<std::uint32_t>> Members;
		std::vector<SpillNetworkOutlet> Outlets;
		std::set<std::uint32_t> OutletRoots;
		for(std::uint32_t Site = 0; Site < Graph.Sites.size(); ++Site) {
			// 収集中の中断への返戻
			if(Stop.stop_requested()) return Reject(FlowNetworkFailure::Cancelled);
			const std::uint32_t Root = Graph.Hierarchy.SiteRoots[Site];
			if(!Root) continue;
			const std::optional<SpillNetworkOutlet> Outlet = GetSpillNetworkOutlet(Network, Site);
			// 不完全な接続を収集対象から除外しない返戻
			if(!Outlet) return Reject(FlowNetworkFailure::Outlet);
			const std::uint32_t Representative = Representatives[Root];
			// 水深の無い根を，正の水深を持つ湖へ置換えない返戻
			if(Representative == MissingSpillIndex) return Reject(FlowNetworkFailure::Outlet);
			Members[Representative].push_back(Site);
			if(Representative != Root || !OutletRoots.insert(Root).second) continue;
			Report.Basin = Root;
			Outlets.push_back(*Outlet);
		}
		Report.RequiredChannels = static_cast<std::uint32_t>(Outlets.size());
		if(Outlets.size() > Settings.MaximumChannels) {
			Report.Basin = Outlets[Settings.MaximumChannels].Basin;
			// 河道の高価な準備前に，全水系の必要数と明示した保持量を照合する返戻
			return Reject(FlowNetworkFailure::ChannelBudget);
		}
		// 同水位の内部鞍点を，全ての元通路から集める；下流木に採用されない辺も含む
		std::vector<LakeSillPatch> Sills;
		std::vector<Vec3> SillCenters;
		std::set<std::uint32_t> SillConnections;
		LakeSillSettings SillSettings;
		SillSettings.RadiusMeters = Radius;
		for(std::size_t Index = 0; Index < Graph.Edges.size(); ++Index) {
			// 補正収集中の中断への返戻
			if(Stop.stop_requested()) return Reject(FlowNetworkFailure::Cancelled);
			const SpillGraphEdge &Edge = Graph.Edges[Index];
			// 壊れた元通路を補正対象外として隠さない返戻
			if(Edge.Nodes[0] >= Graph.Sites.size() || Edge.Nodes[1] >= Graph.Sites.size()) return Reject(FlowNetworkFailure::Outlet);
			const std::uint32_t ComplexIndex = Lakes->SiteComplexes[Edge.Nodes[0]];
			if(ComplexIndex == MissingSpillIndex || ComplexIndex != Lakes->SiteComplexes[Edge.Nodes[1]]) continue;
			const LakeComplex &Complex = Lakes->Complexes[ComplexIndex];
			if(Complex.IsOcean || Complex.Basins.size() < 2 || Edge.CrestMeters != Complex.WaterMeters) continue;
			// 同じ包含根の内部に在る低い鞍点は水没済であり，開削対象にしない
			if(Graph.Hierarchy.SiteRoots[Edge.Nodes[0]] == Graph.Hierarchy.SiteRoots[Edge.Nodes[1]]) continue;
			// 実鞍点への対応が無い辺の返戻
			if(Index >= Graph.EdgeConnections.size()) return Reject(FlowNetworkFailure::Outlet);
			const std::uint32_t Connection = Graph.EdgeConnections[Index];
			// 存在しない元通路への返戻
			if(Connection >= Network.Connections.size()) return Reject(FlowNetworkFailure::Outlet);
			if(!SillConnections.insert(Connection).second) continue;
			const TerrainCriticalPoint &Pass = Network.Connections[Connection].Pass;
			// 元の水位と有限な補正数を保持する返戻
			if(Pass.ElevationMeters != Complex.WaterMeters || Sills.size() > 1023) return Reject(FlowNetworkFailure::LakeSill);
			const std::optional<LakeSillPatch> Sill = LakeSillPatch::Create(Pass, SillSettings);
			// 非退化でない鞍点又は重なる支持域では，開削量の上限を失わない返戻
			if(!Sill) return Reject(FlowNetworkFailure::LakeSill);
			for(const Vec3 Center : SillCenters) {
				// 二つの支持域が重なる候補への返戻
				if((Center - Pass.Direction.Vector()).Length() * Radius <= 24.1) return Reject(FlowNetworkFailure::LakeSill);
			}
			// 外部流出口の局所的な曲率評価範囲も保持
			for(const SpillNetworkOutlet &Outlet : Outlets) {
				const Vec3 External = Network.Connections[Outlet.Connection].Pass.Direction.Vector();
				// 外部流出口を変更する候補への返戻
				if((External - Pass.Direction.Vector()).Length() * Radius <= 24.1) return Reject(FlowNetworkFailure::LakeSill);
			}
			Sills.push_back(*Sill);
			SillCenters.push_back(Pass.Direction.Vector());
		}
		std::function<TerrainSample(UnitDirection)> PreparedSource = Source;
		std::function<double(UnitDirection)> PreparedHeight = SourceHeight;
		if(!Sills.empty()) {
			const std::shared_ptr<const std::vector<LakeSillPatch>> Corrections =
			std::make_shared<const std::vector<LakeSillPatch>>(std::move(Sills));
			PreparedSource = [Source, Corrections](const UnitDirection Direction) -> TerrainSample {
				TerrainSample Result = Source(Direction);
				for(const LakeSillPatch &Patch : *Corrections) {
					const TerrainSample Delta = Patch.EvaluateCorrection(Direction);
					Result.ElevationMeters += Delta.ElevationMeters;
					Result.TangentialGradientPerRadian = Result.TangentialGradientPerRadian + Delta.TangentialGradientPerRadian;
				}
				// 河道・湖岸・表示・衝突で共有する補正済の元場の返戻
				return Result;
			};
			PreparedHeight = [Source, SourceHeight, Corrections](const UnitDirection Direction) -> double {
				double Height = SourceHeight ? SourceHeight(Direction) : Source(Direction).ElevationMeters;
				for(const LakeSillPatch &Patch : *Corrections) Height += Patch.EvaluateCorrection(Direction).ElevationMeters;
				// 同じ順序で加算した補正済標高の返戻
				return Height;
			};
			for(const std::uint32_t Connection : SillConnections) {
				// 通水検査前の中断への返戻
				if(Stop.stop_requested()) return Reject(FlowNetworkFailure::Cancelled);
				const TerrainCriticalPoint &Pass = Network.Connections[Connection].Pass;
				const Vec3 Up = Pass.Direction.Vector();
				const Vec3 Along =
				UnitDirection::FromVector(Pass.MinimumCurvatureDirection - Up * Pass.MinimumCurvatureDirection.Dot(Up))->Vector();
				const Vec3 Across = Up.Cross(Along);
				// 支持の外側迄，幅１メートルの通水標本を合成後の元場で確認
				for(int X = -16; X < 17; ++X) for(int Y = -2; Y < 3; ++Y) {
					const UnitDirection Direction = *UnitDirection::FromVector(Up + Along * (X / Radius) + Across * (Y * 0.25 / Radius));
					const double Height = PreparedHeight(Direction);
					// 有限幅の通水標本を満たさない候補への返戻
					if(!std::isfinite(Height) || !(Height < Pass.ElevationMeters - 0.01)) return Reject(FlowNetworkFailure::LakeSill);
				}
				++Report.LoweredSills;
			}
		}
		// 空の陸上水系を河道生成の成功としない返戻
		if(Outlets.empty()) return Reject(FlowNetworkFailure::Outlet);
		std::sort(
			Outlets.begin(),
			Outlets.end(),
			[&](const SpillNetworkOutlet &A, const SpillNetworkOutlet &B) -> bool {
				const std::uint32_t X = Graph.Hierarchy.Nodes[A.Basin].OceanLinkRank, Y = Graph.Hierarchy.Nodes[B.Basin].OceanLinkRank;
				// 同じ接続順でも入力順に依らない整列の返戻
				return X < Y || X == Y && A.Basin < B.Basin;
			}
		);
		// 補正の定義が変わる場合は，異なる元場の準備を再利用しない
		if(Report.LoweredSills) State->Prefix.clear();
		PreparedFlowNetwork Result;
		Result.Source = PreparedSource;
		Result.SourceHeight = PreparedHeight;
		Result.Channels.reserve(Outlets.size());
		std::map<std::uint32_t, std::size_t> Prepared;
		for(const SpillNetworkOutlet &Outlet : Outlets) {
			Report.Basin = Outlet.Basin;
			Report.OutletPassDirection = Network.Connections[Outlet.Connection].Pass.Direction.Vector();
			Report.UpstreamWaterMeters = Outlet.UpstreamWaterMeters;
			Report.DownstreamWaterMeters = Outlet.DownstreamWaterMeters;
			Report.IsPositiveDownstream = Outlet.IsPositiveDownstream;
			// 準備前の中断への返戻
			if(Stop.stop_requested()) return Reject(FlowNetworkFailure::Cancelled);
			// 同水位の湖連結を，正の勾配を持つ河川へ捏造しない返戻
			if(!(Outlet.DownstreamWaterMeters < Outlet.UpstreamWaterMeters)) return Reject(FlowNetworkFailure::EqualWaterConnection);
			std::shared_ptr<const SpillLakeShore> Receiving;
			if(Outlet.ReceivingBasin) {
				const std::map<std::uint32_t, std::size_t>::const_iterator Found = Prepared.find(Representatives[Outlet.ReceivingBasin]);
				// 受入側が未準備の循環又は欠落を拒否する返戻
				if(Found == Prepared.end()) return Reject(FlowNetworkFailure::ReceivingOrder);
				const std::shared_ptr<const FlowChannelField> Field = Result.Channels[Found->second].Channel.Field;
				// 岸を複製せず，下流の不変場の寿命と共に参照を保持
				Receiving = std::shared_ptr<const SpillLakeShore>(Field, &Field->GetWaterGeometry().Upstream);
			}
			FlowPreparationSettings ChannelSettings = Settings.Channel;
			ChannelSettings.Outlet.DownstreamWaterMeters = Outlet.DownstreamWaterMeters;
			ChannelSettings.Outlet.IsPositiveBranch = Outlet.IsPositiveDownstream;
			ChannelSettings.Outlet.ReferenceMinimumDirection = Network.Connections[Outlet.Connection].Pass.MinimumCurvatureDirection;
			// 同じ物理流出口の淵を，接続の列挙順や準備順に結び付けない
			ChannelSettings.Seed = Impl::MakeChannelSeed(Settings.Channel.Seed, Network.Connections[Outlet.Connection].Pass);
			const std::size_t PrefixIndex = Result.Channels.size();
			std::optional<PreparedFlowChannel> Channel;
			bool IsReused = false;
			if(!Report.LoweredSills && PrefixIndex < State->Prefix.size()) {
				const Impl::Entry &Cached = State->Prefix[PrefixIndex];
				const Vec3 Pass = Network.Connections[Outlet.Connection].Pass.Direction.Vector();
				if(Cached.HasSameInputs(Outlet, Pass, *ChannelSettings.Outlet.ReferenceMinimumDirection, Receiving)) {
					Channel = Cached.Channel;
					Report.Channel = Cached.Diagnostics;
					IsReused = true;
				}
			}
			if(!IsReused) {
				// 最初の不一致以降は解放し，古い異なる受入関係を保持しない
				if(PrefixIndex < State->Prefix.size()) State->Prefix.resize(PrefixIndex);
				Channel = PrepareFlowChannel(
					Network.Connections[Outlet.Connection].Pass.Direction,
					PreparedSource,
					PreparedHeight,
					ChannelSettings,
					Receiving,
					Stop,
					&Report.Channel
				);
			}
			// 下位の準備が不完全な流域を公開しない返戻
			if(!Channel) return Reject(FlowNetworkFailure::Channel);
			// 元網の実鞍点の高さと再準備した水位を照合する返戻
			if(std::abs(Channel->UpstreamWaterMeters - Outlet.UpstreamWaterMeters) > 1E-6) return Reject(FlowNetworkFailure::WaterLevel);
			const FlowWaterGeometry Geometry = Channel->Field->GetWaterGeometry();
			const std::size_t AddedSamples =
			Geometry.Upstream.GetContour().Samples.size() + Geometry.Route.GetSegments().size() + Report.Channel.Water.Knots +
			Channel->PoolCount;
			// 一つの河道の作業領域以外を，上流への追加毎に水系全体の有界な保持量で制限する
			if(AddedSamples > Settings.MaximumGeometrySamples - Report.RetainedGeometrySamples) {
				// 確認済の河道を捨てて部分的な成功を返さない返戻
				return Reject(FlowNetworkFailure::GeometryBudget);
			}
			for(const std::uint32_t Site : Members.at(Outlet.Basin)) {
				const bool Contains = Graph.Minima[Site] && Geometry.Upstream.Contains(Graph.Minima[Site]->Direction);
				// 包含した実極小を湖岸が囲まない候補への返戻
				if(!Contains) {
					Report.Membership = FlowMembershipFailure::UpstreamMinimum;
					Report.MembershipSite = Site;
					if(Graph.Minima[Site]) {
						const UnitDirection Direction = Graph.Minima[Site]->Direction;
						Report.MembershipDirection = Direction.Vector();
						Report.MembershipHeightMeters = PreparedSource(Direction).ElevationMeters;
						Report.MembershipSignedDistanceMeters = Geometry.Upstream.SignedPlanarDistanceMeters(Direction);
					}
					// 元場の実極小を除外して湖を成功にしない返戻
					return Reject(FlowNetworkFailure::Membership);
				}
			}
			const std::optional<TerrainFlowCurveSample> End = Channel->Route->Sample(Channel->Route->GetLengthMeters());
			// 受入側の実水域へ到達しない候補への返戻
			if(!End) {
				Report.Membership = FlowMembershipFailure::MissingRouteEnd;
				// 河道の末尾を欠く場合の返戻
				return Reject(FlowNetworkFailure::Membership);
			}
			if(Receiving && !Receiving->Contains(End->Direction)) {
				Report.Membership = FlowMembershipFailure::ReceivingLake;
				Report.MembershipDirection = End->Direction.Vector();
				Report.MembershipHeightMeters = PreparedSource(End->Direction).ElevationMeters;
				Report.MembershipSignedDistanceMeters = Receiving->SignedPlanarDistanceMeters(End->Direction);
				// 別の水域へ流れる末尾を，指定した受入湖へ置換えない返戻
				return Reject(FlowNetworkFailure::Membership);
			}
			if(!Outlet.ReceivingBasin) {
				const double Height = PreparedHeight ? PreparedHeight(End->Direction) : PreparedSource(End->Direction).ElevationMeters;
				// 大洋側でも実際に水没しない終端を拒否する返戻
				if(!std::isfinite(Height) || !(Height < Outlet.DownstreamWaterMeters)) {
					Report.Membership = FlowMembershipFailure::OceanHeight;
					Report.MembershipDirection = End->Direction.Vector();
					Report.MembershipHeightMeters = Height;
					// 水没していない末尾を大洋の所属にしない返戻
					return Reject(FlowNetworkFailure::Membership);
				}
				// 元の下降枝で確認した大洋所属を，新しい河道端点へも水没通路で接続
				const SpillConnectionBranch &Branch = Network.Connections[Outlet.Connection].Branches[Outlet.IsPositiveDownstream ? 1 : 0];
				// 下降枝の実終端が無い場合の返戻
				if(Branch.Path.Samples.empty()) return Reject(FlowNetworkFailure::OceanPassage);
				const Vec3 Terminal = Branch.Path.Samples.back().Direction.Vector();
				const std::vector<SpillOceanWitness>::const_iterator Witness = std::find_if(
					Network.OceanWitnesses.begin(),
					Network.OceanWitnesses.end(),
					[&](const SpillOceanWitness &Candidate) -> bool {
						const Vec3 Point = Candidate.Terminal.Vector();
						// 別の海域の証拠を流用しない返戻
						return Point.X == Terminal.X && Point.Y == Terminal.Y && Point.Z == Terminal.Z &&
						std::isfinite(Candidate.CheckedMaximumMeters) && Candidate.CheckedMaximumMeters <= Outlet.DownstreamWaterMeters;
					}
				);
				// 元場の大洋所属の証拠を失った枝への返戻
				if(Witness == Network.OceanWitnesses.end()) return Reject(FlowNetworkFailure::OceanPassage);
				SpillPassageSettings PassageSettings;
				PassageSettings.RadiusMeters = Radius;
				PassageSettings.SpacingMeters = std::min(50.0, Radius * 0.025);
				PassageSettings.HalfWidthMeters = std::min(1000.0, Radius * 0.25);
				const std::optional<SpillPassage> Passage =
				FindSpillPassage(End->Direction, Witness->Anchor, PreparedSource, PassageSettings, Stop);
				// 新しい端点が孤立した水没域に終わる場合の返戻
				if(!Passage) return Reject(FlowNetworkFailure::OceanPassage);
				const double Maximum = Passage->CrestMeters + Passage->MaximumIndependentExcessMeters;
				// 有限の標本通路でも受入海面を超える場合の返戻
				if(!std::isfinite(Maximum) || Maximum > Outlet.DownstreamWaterMeters) return Reject(FlowNetworkFailure::OceanPassage);
				++Report.CoastalChannels;
			}
			// 現在の所属・通水・保持量の全検査を通した後だけ，再利用へ保持する
			if(IsReused) ++Report.ReusedChannels;
			else {
				++Report.RebuiltChannels;
				if(!Report.LoweredSills) {
					State->Prefix.push_back(
						{
							Outlet.Connection,
							Network.Connections[Outlet.Connection].Pass.Direction.Vector(),
							Network.Connections[Outlet.Connection].Pass.MinimumCurvatureDirection,
							Outlet.UpstreamWaterMeters,
							Outlet.DownstreamWaterMeters,
							Outlet.IsPositiveDownstream,
							Receiving,
							*Channel,
							Report.Channel
						}
					);
				}
			}
			Prepared.emplace(Outlet.Basin, Result.Channels.size());
			Result.Channels.push_back({ Outlet, std::move(*Channel) });
			Report.RetainedGeometrySamples += static_cast<std::uint32_t>(AddedSamples);
			++Report.PreparedChannels;
		}
		// 大陸内だけで閉じた集合を大洋接続済として公開しない返戻
		if(!Report.CoastalChannels || Stop.stop_requested()) return Reject(FlowNetworkFailure::ReceivingOrder);
		// 消えた上流部分を蓄積せず，現在の完成した連続部分だけを保持する
		if(State->Prefix.size() > Result.Channels.size()) State->Prefix.resize(Result.Channels.size());
		Report.Basin = 0;
		// 各下流の岸を共有する，完成した有限網の返戻
		return Result;
	}

	/**
	 * 再利用を要求しない従来の単独準備を，一つの生成定義へ束縛する
	 * @param Network 元場上で大洋への接続を確認した有限網
	 * @param Source 所有する不変場を捕捉した標高・勾配
	 * @param SourceHeight 同じ元標高専用の照会
	 * @param Settings 固定する生成条件と有限な保持予算
	 * @param Stop 中断要求
	 * @param Diagnostics 完了した範囲と拒否理由
	 * @return 全流出口を準備した水系；一つでも未確認なら空
	 */
	std::optional<PreparedFlowNetwork> PrepareFlowNetwork(
		const SpillNetwork &Network,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::function<double(UnitDirection)> &SourceHeight,
		const FlowNetworkSettings &Settings,
		const std::stop_token Stop,
		FlowNetworkDiagnostics *const Diagnostics
	) {
		FlowNetworkPreparation Preparation(Source, SourceHeight, Settings);
		// 同じ検査経路へ束縛した結果の返戻
		return Preparation.Prepare(Network, Stop, Diagnostics);
	}
}
