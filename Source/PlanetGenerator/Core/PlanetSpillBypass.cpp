#include "PlanetSpillBypass.h"
#include "PlanetSpillPassage.h"
#include "PlanetSpillSampleCache.h"
#include "PlanetSpillString.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		/**
		 * 同じ元場の確認済停留点を，既存の位置・標高精度で照合する
		 * 計算量：時間・追加空間 O(1)
		 * @param A 一方の実停留点
		 * @param B 他方の実停留点
		 * @param Radius 惑星半径
		 * @return 同じ種類と既存精度内の位置・高さを持つ場合に真
		 */
		bool SameBypassCritical(const TerrainCriticalPoint &A, const TerrainCriticalPoint &B, const double Radius) {
			const Vec3 P = A.Direction.Vector(), Q = B.Direction.Vector();
			// 不同の停留点を高さだけで混同しない返戻
			return A.Kind == B.Kind && Radius * std::atan2(P.Cross(Q).Length(), P.Dot(Q)) <= 0.001 &&
			std::abs(A.ElevationMeters - B.ElevationMeters) <= 1E-6;
		}

		/**
		 * 実下降終端が入力成分又は追加済の実通路の極小に属するか調べる
		 * 計算量：時間 O(C)，追加空間 O(1)，C は既存接続数
		 * @param Network 作成中の同じ不変場の有限網
		 * @param Branch 候補の実下降枝
		 * @param Radius 惑星半径
		 * @return 元場で照合した極小が成分に含まれる場合に真
		 */
		bool IsKnownBypassMinimum(const SpillNetwork &Network, const SpillConnectionBranch &Branch, const double Radius) {
			// 高度だけの海面終端を極小として扱わない返戻
			if(!Branch.Minimum) return false;
			for(const SpillConnection &Connection : Network.Connections) {
				for(const SpillConnectionBranch &Existing : Connection.Branches) {
					// 新たに接続した極小も，物理位置と標高で照合する返戻
					if(Existing.Minimum && SameBypassCritical(*Existing.Minimum, *Branch.Minimum, Radius)) return true;
				}
			}
			// 無関係な盆地を追加しない返戻
			return false;
		}
	}

	struct SpillBypassPreparation::Impl {
		std::function<TerrainSample(UnitDirection)> Source;
		std::uint32_t Capacity = 0;
		std::optional<SpillSampleCache> Cached;
		std::vector<UnitDirection> OceanAnchors;
		std::function<bool(UnitDirection)> IsConnectedOceanAnchor;
		bool HasValidOceanInput = true;
	};

	/**
	 * 元場と所属判定を値で保持し，同じ元場の大洋標本を準備中だけ所有する
	 * 計算量：時間・追加空間 O(A)，A は大洋標本数；不正な規模では複製しない
	 * @param Source 生存期間中に変わらない元場
	 * @param Capacity 逐次補完で共有する有界な厳密標本枠
	 * @param ConnectedOceanAnchors 同じ元場の大洋標本；最大百万点
	 * @param IsConnectedOceanAnchor 標本の共通大洋所属を確認する述語；標本と共に省略可
	 */
	SpillBypassPreparation::SpillBypassPreparation(
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::uint32_t Capacity,
		const std::span<const UnitDirection> ConnectedOceanAnchors,
		const std::function<bool(UnitDirection)> &IsConnectedOceanAnchor
	) : State(std::make_unique<Impl>()) {
		State->Source = Source;
		State->Capacity = Capacity;
		State->HasValidOceanInput = ConnectedOceanAnchors.size() < 1000001 &&
		(ConnectedOceanAnchors.empty() ? !IsConnectedOceanAnchor : static_cast<bool>(IsConnectedOceanAnchor));
		if(State->HasValidOceanInput) {
			State->OceanAnchors.assign(ConnectedOceanAnchors.begin(), ConnectedOceanAnchors.end());
			State->IsConnectedOceanAnchor = IsConnectedOceanAnchor;
		}
		// 同じ元場への束縛の終了
		return;
	}

	/**
	 * 既存の入力網の大洋証跡だけを使う互換の準備を作る
	 * 計算量：時間・追加空間 O(1)，元場関数の複製費用を除く
	 * @param Source 生存期間中に変わらない元場
	 * @param Capacity 逐次補完で共有する有界な厳密標本枠
	 */
	SpillBypassPreparation::SpillBypassPreparation(
		const std::function<TerrainSample(UnitDirection)> &Source,
		const std::uint32_t Capacity
	) : SpillBypassPreparation(Source, Capacity, std::span<const UnitDirection>{}, std::function<bool(UnitDirection)>{}) {
		// 既存の呼出形式の束縛の終了
		return;
	}

	/**
	 * 公開された網へ準備標本を持越さず，所有する領域を解放する
	 */
	SpillBypassPreparation::~SpillBypassPreparation() = default;

	/**
	 * 拒否された流出口の実極小又は検証済大洋を結ぶ低い実測通路から，欠けた実鞍点を補い包含を再構築する
	 * 計算量：時間 O(E+QC³)，追加空間 O(B+C+S+W)，元場照会１回の費用を一定とする
	 * E は再利用込みの照会数，Q は網の再検査数，C は候補込みの接続数
	 * B は保持する枝標本数，S は厳密な再利用枠，W は最大の通路格子数
	 * @param Original 同じ元場で大洋所属を確認した入力網；失敗でも変更しない
	 * @param OutletConnection 湖の外岸として使えなかった実鞍点の番号
	 * @param Settings 同じ尺度・海面と，全窓・追加枝を合算した作業予算
	 * @param Stop 下位探索と元場照会へ伝播する中断要求
	 * @param Diagnostics 元の峠・実測通路と実際の追加数
	 * @return 実接続を追加して再検証した網；同じ失敗の再試行や部分的な成功は返さない
	 * @note 有限窓での位相補完であり，全球の全出口の列挙又は最低出口の証明ではない
	 */
	std::optional<SpillNetwork> SpillBypassPreparation::PrepareCandidates(
		const SpillNetwork &Original,
		const std::uint32_t OutletConnection,
		const SpillBypassSettings &Settings,
		const std::stop_token Stop,
		SpillBypassDiagnostics *const Diagnostics
	) {
		const std::function<TerrainSample(UnitDirection)> &Source = State->Source;
		SpillBypassDiagnostics Local;
		SpillBypassDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		bool HasSource = true, HasBudget = true;
		const auto Reject = [&](const SpillBypassFailure Failure) -> std::optional<SpillNetwork> {
			Report.Failure = Stop.stop_requested() ?
			SpillBypassFailure::Cancelled :
			!HasBudget ? SpillBypassFailure::Budget : !HasSource ? SpillBypassFailure::Source : Failure;
			// 不完全な修正を公開せず入力網を保持する返戻
			return {};
		};
		const bool Valid = Source && State->HasValidOceanInput && Settings.MaximumCachedSamples == State->Capacity &&
		OutletConnection < Original.Connections.size() && !Original.OceanWitnesses.empty() &&
		Original.Graph.BranchSites.size() == Original.Connections.size() &&
		Original.Graph.Hierarchy.Saturation.Nodes.size() == Original.Graph.Minima.size() && std::isfinite(Settings.RadiusMeters) &&
		!(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 && std::isfinite(Settings.SeaLevelMeters) &&
		Settings.MaximumEvaluations > 0 && Settings.MaximumEvaluations < 32000001 &&
		Settings.MaximumCachedSamples <= SpillSampleCache::MaximumCapacity && Settings.MaximumConnections > 0 &&
		Settings.MaximumConnections < 4097 && Settings.MaximumBranchSamples > 0 && Settings.MaximumBranchSamples < 1048577 &&
		std::isfinite(Settings.MaximumLocalHalfWidthMeters) &&
		Settings.MaximumLocalHalfWidthMeters >= std::min(256.0, Settings.RadiusMeters * 0.0625) &&
		Settings.MaximumLocalHalfWidthMeters < 262145;
		// 無効な入力又は開始前の中断への返戻
		if(!Valid || Stop.stop_requested()) return Reject(SpillBypassFailure::InvalidInput);
		const std::array<std::uint32_t, 2> OldSites = Original.Graph.BranchSites[OutletConnection];
		const bool HasSites = OldSites[0] < Original.Graph.Minima.size() && OldSites[1] < Original.Graph.Minima.size() &&
		Original.Graph.Hierarchy.SiteRoots.size() == Original.Graph.Minima.size();
		// 不正な地点対応を，添字参照の前に拒否する返戻
		if(!HasSites) return Reject(SpillBypassFailure::InvalidInput);
		const SpillConnection &Failed = Original.Connections[OutletConnection];
		std::array<std::optional<UnitDirection>, 2> Terminals;
		std::array<double, 2> StoredHeights {};
		bool HasTerminals = Failed.Pass.Kind == TerrainCriticalKind::Saddle && std::isfinite(Failed.Pass.ElevationMeters) &&
		Failed.Pass.ElevationMeters > Settings.SeaLevelMeters && (Failed.Branches[0].Minimum || Failed.Branches[1].Minimum);
		for(unsigned Side = 0; Side < 2; ++Side) {
			const SpillConnectionBranch &Branch = Failed.Branches[Side];
			if(
				Branch.Kind == SpillTerminalKind::Minimum && Branch.Minimum && Branch.Minimum->Kind == TerrainCriticalKind::Minimum &&
				std::isfinite(Branch.Minimum->ElevationMeters)
			) {
				Terminals[Side] = Branch.Minimum->Direction;
				StoredHeights[Side] = Branch.Minimum->ElevationMeters;
			} else if(
				Branch.Kind == SpillTerminalKind::SeaLevel && !Branch.Minimum && !OldSites[Side] && !Branch.Path.Samples.empty() &&
				Branch.Path.End == TerrainDescentEnd::HeightLimit
			) {
				const TerrainDescentSample &End = Branch.Path.Samples.back();
				const Vec3 D = End.Direction.Vector();
				const bool Verified = std::any_of(
					Original.OceanWitnesses.begin(),
					Original.OceanWitnesses.end(),
					[&](const SpillOceanWitness &Witness) -> bool {
						const Vec3 W = Witness.Terminal.Vector();
						// 水位だけで孤立した海面終端を確認済の大洋へ置換えない返戻
						return D.X == W.X && D.Y == W.Y && D.Z == W.Z && std::isfinite(Witness.CheckedMaximumMeters) &&
						Witness.CheckedMaximumMeters <= Settings.SeaLevelMeters;
					}
				);
				if(Verified && std::isfinite(End.ElevationMeters) && End.ElevationMeters <= Settings.SeaLevelMeters) {
					Terminals[Side] = End.Direction;
					StoredHeights[Side] = End.ElevationMeters;
				}
			}
			HasTerminals = HasTerminals && Terminals[Side].has_value();
		}
		// 実極小と検証済大洋以外の終端で，低い出口への修正を試みない返戻
		if(!HasTerminals) return Reject(SpillBypassFailure::InvalidInput);
		Report.OriginalPassMeters = Failed.Pass.ElevationMeters;
		std::size_t OriginalSamples = 0;
		for(const SpillConnection &Connection : Original.Connections) {
			OriginalSamples += Connection.Branches[0].Path.Samples.size() + Connection.Branches[1].Path.Samples.size();
		}
		const bool HasCapacity =
		Original.Connections.size() < Settings.MaximumConnections && OriginalSamples < Settings.MaximumBranchSamples;
		// 少なくとも一つの新しい実接続を保持出来なければ，元場の探索を開始しない返戻
		if(!HasCapacity) return Reject(SpillBypassFailure::Budget);
		// 元場と容量を束縛した準備の間だけ共有し，不正入力や中断では新規確保しない
		std::optional<SpillSampleCache> &Cached = State->Cached;
		if(!Cached) Cached = SpillSampleCache::Create(State->Capacity);
		// 無効な保持容量を参照前に拒否する返戻
		if(!Cached) return Reject(SpillBypassFailure::InvalidInput);
		Report.CachedSamples = Cached->GetCount();
		const auto Sample = [&](const UnitDirection D) -> TerrainSample {
			if(!Stop.stop_requested()) if(const std::optional<TerrainSample> Stored = Cached->Find(D)) {
				++Report.CacheHits;
				// 方向も値も丸めず，実測済の同じ元場だけを再利用する返戻
				return *Stored;
			}
			if(Stop.stop_requested() || Report.Evaluations >= Settings.MaximumEvaluations) {
				HasBudget = false;
				const double Invalid = std::numeric_limits<double>::quiet_NaN();
				// 予算後は元場を呼出さない返戻
				return { Invalid, { Invalid, Invalid, Invalid } };
			}
			++Report.Evaluations;
			const TerrainSample Value = Source(D);
			HasSource = HasSource && std::isfinite(Value.ElevationMeters) && std::isfinite(Value.TangentialGradientPerRadian.Length());
			if(HasSource) Cached->Store(D, Value);
			Report.CachedSamples = Cached->GetCount();
			// 同じ元場の評価だけを通路と再準備へ渡す返戻
			return Value;
		};
		const UnitDirection From = *Terminals[0], To = *Terminals[1];
		// 古い水系を別の元場に適用する明らかな誤用を，探索前に拒否する
		const double PassHeight = Sample(Failed.Pass.Direction).ElevationMeters, FromHeight = Sample(From).ElevationMeters;
		const double ToHeight = Sample(To).ElevationMeters;
		const bool HasSameSource = HasSource && HasBudget && std::abs(PassHeight - Failed.Pass.ElevationMeters) <= 1E-6 &&
		std::abs(FromHeight - StoredHeights[0]) <= 1E-6 && std::abs(ToHeight - StoredHeights[1]) <= 1E-6;
		// 元場の矛盾又は評価不能への返戻
		if(!HasSameSource) return Reject(SpillBypassFailure::Source);
		std::optional<SpillPassage> Bypass;
		std::vector<SpillPassage> PassageCandidates;
		std::optional<SpillString> Relaxed;
		// 元の狭窓を先行し，広い候補でも辺上の実測精度と全予算は維持
		const std::array<std::array<double, 2>, 7> Windows {
			std::array<double, 2>{ 256, 16 },
			{ 512, 16 },
			{ 1024, 16 },
			{ 2048, 16 },
			{ 4096, 32 },
			{ 8192, 64 },
			{ 16384, 128 }
		};
		const std::uint64_t PassageStart = Report.Evaluations;
		const std::uint64_t PassageLimit = std::min<std::uint64_t>(1000000, Settings.MaximumEvaluations / 4);
		for(const std::array<double, 2> Window : Windows) {
			if(!Settings.UseGridPassages) break;
			if(Report.Evaluations - PassageStart >= PassageLimit) {
				Report.PassageCandidateBudgetExhausted = true;
				break;
			}
			const double HalfWidth = std::min(Window[0], Settings.RadiusMeters * 0.25);
			if(HalfWidth == Report.LastHalfWidthMeters) continue;
			Report.LastHalfWidthMeters = HalfWidth;
			++Report.Windows;
			SpillPassageSettings PassageSettings;
			PassageSettings.RadiusMeters = Settings.RadiusMeters;
			PassageSettings.SpacingMeters = Window[1];
			Report.LastSpacingMeters = Window[1];
			PassageSettings.MaximumSampleStepMeters = 4;
			PassageSettings.UsePathEdgeSelection = true;
			PassageSettings.AdaptCandidateSpacingToBudget = true;
			PassageSettings.HalfWidthMeters = HalfWidth;
			PassageSettings.MaximumEvaluations = static_cast<std::uint32_t>(PassageLimit - (Report.Evaluations - PassageStart));
			Bypass = FindSpillPassage(From, To, Sample, PassageSettings, Stop);
			Report.PassageCandidateEvaluations = Report.Evaluations - PassageStart;
			// 元場・中断・共通予算の失敗を広い窓で隠さない返戻
			if(!HasSource || !HasBudget || Stop.stop_requested()) return Reject(SpillBypassFailure::Source);
			if(!Bypass) continue;
			++Report.MeasuredWindows;
			Report.LastSpacingMeters = Bypass->CandidateSpacingMeters;
			Report.MeasuredBypassMeters = Bypass->CrestMeters + Bypass->MaximumIndependentExcessMeters;
			if(Report.MeasuredBypassMeters < Failed.Pass.ElevationMeters - 1E-6) break;
			// 格子で最も低い候補だけが，緩和後にも低い通路になるとは限らない
			PassageCandidates.push_back(std::move(*Bypass));
			Bypass.reset();
		}
		SpillStringSettings StringSettings;
		StringSettings.RadiusMeters = Settings.RadiusMeters;
		StringSettings.SpacingMeters = 16;
		StringSettings.TimeStepMeters = 5000;
		StringSettings.MaximumMoveMeters = 8;
		StringSettings.MaximumIterations = 512;
		StringSettings.MaximumEvaluations = std::min(Settings.MaximumEvaluations, 1000000U);
		// 従来の最小候補を先行し，同率では窓の準備順を維持
		std::stable_sort(
			PassageCandidates.begin(),
			PassageCandidates.end(),
			[](const SpillPassage &A, const SpillPassage &B) -> bool {
				// 独立試料込みの格子上の通過高度を比較した返戻
				return A.CrestMeters + A.MaximumIndependentExcessMeters < B.CrestMeters + B.MaximumIndependentExcessMeters;
			}
		);
		for(SpillPassage &Candidate : PassageCandidates) {
			if(Bypass) break;
			if(Report.Evaluations - PassageStart >= PassageLimit) {
				Report.PassageCandidateBudgetExhausted = true;
				break;
			}
			StringSettings.MaximumEvaluations = static_cast<std::uint32_t>(PassageLimit - (Report.Evaluations - PassageStart));
			++Report.RelaxationAttempts;
			const std::uint64_t Before = Report.Evaluations;
			Relaxed = RelaxSpillString(Candidate.Path, Sample, StringSettings, Stop);
			// 絞込みの失敗を，予算超過又は欠落した元場へ混同しない返戻
			if(!HasSource || !HasBudget || Stop.stop_requested()) return Reject(SpillBypassFailure::Source);
			if(Relaxed) {
				SpillPassageSettings MeasureSettings;
				MeasureSettings.RadiusMeters = Settings.RadiusMeters;
				MeasureSettings.MaximumSampleStepMeters = 4;
				MeasureSettings.MaximumEvaluations = static_cast<std::uint32_t>(PassageLimit - (Report.Evaluations - PassageStart));
				std::optional<SpillPathCrest> Measured = MeasureSpillPathCrest(Relaxed->Path, Sample, MeasureSettings, Stop);
				// 頂点間を測定する段階でも，共通の全準備予算を維持する返戻
				if(!HasSource || !HasBudget || Stop.stop_requested()) return Reject(SpillBypassFailure::Source);
				if(Measured) {
					const double Crest = Measured->CrestMeters + Measured->MaximumIndependentExcessMeters;
					if(!Report.MeasuredRelaxationAttempts || Crest < Report.RelaxedCrestMeters) Report.RelaxedCrestMeters = Crest;
					++Report.MeasuredRelaxationAttempts;
					if(Crest < Failed.Pass.ElevationMeters - 1E-6) {
						Report.HasLowerRelaxedPath = true;
						Bypass = std::move(Candidate);
						Bypass->Path = std::move(Relaxed->Path);
						Bypass->PassCandidates = std::move(Measured->PassCandidates);
						Bypass->CrestMeters = Measured->CrestMeters;
						Bypass->Peak = Measured->Peak;
						Bypass->PathLengthMeters = Measured->LengthMeters;
						Bypass->MaximumIndependentExcessMeters = Measured->MaximumIndependentExcessMeters;
						// 絞込み後の通路を元格子の最小解とはしない
						Bypass->IsGraphInteriorMinimum = false;
						Report.MeasuredBypassMeters = Crest;
					}
				}
			}
			Report.RelaxationEvaluations += Report.Evaluations - Before;
			Report.PassageCandidateEvaluations = Report.Evaluations - PassageStart;
		}
		Report.PassageCandidateEvaluations = Report.Evaluations - PassageStart;
		Report.PassageCandidateBudgetExhausted = Report.PassageCandidateEvaluations >= PassageLimit;
		// 比較用の従来経路は，格子又は緩和通路の未発見で拒否する返戻
		if(!Bypass && !Settings.UseContinuousConnectionFallback) return Reject(SpillBypassFailure::NoLowerPassage);
		// 格子で未発見の場合も，実接続と両大洋出口を確認する迄は何も公開しない
		Report.UsedContinuousConnectionFallback = !Bypass;
		// 採用しない有限窓の経路は，実接続の準備前に解放
		PassageCandidates.clear();
		SpillConnectionSettings ConnectionSettings;
		ConnectionSettings.RadiusMeters = Settings.RadiusMeters;
		ConnectionSettings.SeaLevelMeters = Settings.SeaLevelMeters;
		// 全準備と同じ実下降の既定範囲を使い，頂点数と総評価量で有界にする
		std::vector<SpillConnection> Pending;
		const auto AddCandidates = [&](const std::vector<UnitDirection> &Candidates) -> bool {
			unsigned Checked = 0;
			for(const UnitDirection Candidate : Candidates) {
				if(Checked++ > 31) break;
				if(Settings.ReusePreparedConnections) {
					TerrainCriticalSettings Critical;
					Critical.RadiusMeters = Settings.RadiusMeters;
					Critical.MaximumDisplacementMeters = std::min(Critical.MaximumDisplacementMeters, Settings.RadiusMeters * 0.125);
					const std::optional<TerrainCriticalPoint> Pass = RefineTerrainCriticalPoint(Candidate, Sample, Critical, Stop);
					// 再利用の照合時も，同じ元場・全予算・中断を維持する返戻
					if(!HasSource || !HasBudget || Stop.stop_requested()) return false;
					// 後段で破棄する高い候補に，長い両下降の費用を掛けない
					if(Pass && !(Pass->ElevationMeters < Failed.Pass.ElevationMeters - 1E-6)) continue;
					if(Pass && Pass->Kind == TerrainCriticalKind::Saddle) {
						const auto Same = [&](const SpillConnection &Known) -> bool {
							// 元場で精査した同じ峠の位置・種類・標高の照合結果の返戻
							return SameBypassCritical(Known.Pass, *Pass, Settings.RadiusMeters);
						};
						if(
							std::any_of(Original.Connections.begin(), Original.Connections.end(), Same) || std::any_of(Pending.begin(), Pending.end(), Same)
						) {
							++Report.ReusedCandidates;
							continue;
						}
					}
				}
				SpillConnectionDiagnostics ConnectionReport;
				std::optional<SpillConnection> Connection =
				PrepareSpillConnection(Candidate, Sample, ConnectionSettings, Stop, &ConnectionReport);
				// 部分的な候補群を，完全な位相補完として公開しない返戻
				if(!HasSource || !HasBudget || Stop.stop_requested()) return false;
				if(!Connection) {
					++Report.RejectedCandidates;
					if(!Report.FirstRejectedCandidate) {
						Report.FirstRejectedCandidate = Candidate;
						Report.FirstRejectedConnection = ConnectionReport;
					}
					continue;
				}
				if(!(Connection->Pass.ElevationMeters < Failed.Pass.ElevationMeters - 1E-6)) continue;
				const auto Same = [&](const SpillConnection &Known) -> bool {
					// 一つの実峠を候補又は入力網へ重複しない返戻
					return SameBypassCritical(Known.Pass, Connection->Pass, Settings.RadiusMeters);
				};
				if(
					std::any_of(Original.Connections.begin(), Original.Connections.end(), Same) || std::any_of(Pending.begin(), Pending.end(), Same)
				) continue;
				Pending.push_back(std::move(*Connection));
			}
			// 候補を実峠と両終端へ渡した結果の返戻
			return true;
		};
		// 格子の経路上の局所最高点を，元場で直接確認する
		// 実接続の途中に元場又は共通予算を失った場合の返戻
		if(Bypass && !AddCandidates(Bypass->PassCandidates)) return Reject(SpillBypassFailure::Source);
		SpillNetwork Result = Original;
		std::size_t BranchSamples = 0;
		for(const SpillConnection &Connection : Result.Connections) {
			BranchSamples += Connection.Branches[0].Path.Samples.size() + Connection.Branches[1].Path.Samples.size();
		}
		std::vector<bool> Used(Pending.size(), false);
		std::optional<ContinuousSpillGraph> Graph;
		bool HasLowerEscape = false;
		const auto ValidateConnections = [&]() -> bool {
			const auto Invalidate = [&](const SpillBypassFailure Failure) -> bool {
				Report.Failure = Failure;
				// 同じ元場で構成出来ない部分網を成功にしない返戻
				return false;
			};
			Used.resize(Pending.size(), false);
			bool Changed = true;
			while(Changed) {
				Changed = false;
				for(std::size_t I = 0; I < Pending.size(); ++I) {
					if(Used[I]) continue;
					const SpillConnection &Connection = Pending[I];
					const bool Incident = IsKnownBypassMinimum(Result, Connection.Branches[0], Settings.RadiusMeters) ||
					IsKnownBypassMinimum(Result, Connection.Branches[1], Settings.RadiusMeters);
					if(!Incident) continue;
					const std::size_t Added = Connection.Branches[0].Path.Samples.size() + Connection.Branches[1].Path.Samples.size();
					if(Result.Connections.size() >= Settings.MaximumConnections || BranchSamples + Added > Settings.MaximumBranchSamples) {
						// 入力と追加分を合算した保持量を超えない返戻
						return Invalidate(SpillBypassFailure::Budget);
					}
					for(const SpillConnectionBranch &Branch : Connection.Branches) {
						if(Branch.Kind != SpillTerminalKind::SeaLevel) continue;
						const UnitDirection Terminal = Branch.Path.Samples.back().Direction;
						const auto Key = [Terminal](const SpillOceanWitness &Witness) -> double {
							const Vec3 Delta = Terminal.Vector() - Witness.Anchor.Vector();
							// 所属確認済海域の近い代表を選ぶキーの返戻
							return Delta.Dot(Delta);
						};
						const SpillOceanWitness &Nearest = *std::min_element(
							Result.OceanWitnesses.begin(),
							Result.OceanWitnesses.end(),
							[&Key](const SpillOceanWitness &A, const SpillOceanWitness &B) -> bool {
								// 同じ確認済大洋の代表に対する距離比較の返戻
								return Key(A) < Key(B);
							}
						);
						UnitDirection Anchor = Nearest.Anchor;
						if(!State->OceanAnchors.empty()) {
							std::optional<UnitDirection> VerifiedAnchor;
							double Best = std::numeric_limits<double>::infinity();
							for(const UnitDirection Candidate : State->OceanAnchors) {
								// 所属の照会中にも中断した部分網を公開しない返戻
								if(Stop.stop_requested()) return Invalidate(SpillBypassFailure::Cancelled);
								if(!State->IsConnectedOceanAnchor(Candidate)) continue;
								const Vec3 Delta = Terminal.Vector() - Candidate.Vector();
								const double Distance = Delta.Dot(Delta);
								if(Distance < Best) {
									Best = Distance;
									VerifiedAnchor = Candidate;
								}
							}
							// 水位だけ一致する未所属の標本を採用しない返戻
							if(!VerifiedAnchor) return Invalidate(SpillBypassFailure::Ocean);
							Anchor = *VerifiedAnchor;
						}
						bool HasOcean = false;
						Report.LastOceanTerminal = Terminal;
						Report.LastOceanAnchor = Anchor;
						double PreviousOceanWidth = 0;
						const Vec3 A = Terminal.Vector(), B = Anchor.Vector();
						Report.LastOceanDistanceMeters = Settings.RadiusMeters * std::atan2(A.Cross(B).Length(), A.Dot(B));
						for(const double Width : { 1000.0, 2000.0, 4000.0, 8000.0, 16000.0, 32000.0, 64000.0 }) {
							const double ActualWidth = std::min(Width, Settings.RadiusMeters * 0.25);
							if(ActualWidth == PreviousOceanWidth) continue;
							PreviousOceanWidth = ActualWidth;
							Report.LastOceanHalfWidthMeters = ActualWidth;
							SpillPassageSettings WetSettings;
							WetSettings.RadiusMeters = Settings.RadiusMeters;
							WetSettings.SpacingMeters = std::min(50.0, Settings.RadiusMeters * 0.025);
							WetSettings.HalfWidthMeters = ActualWidth;
							WetSettings.MaximumEvaluations = Settings.MaximumEvaluations;
							// 長い海域通路も同じ保持枠へ収め，選択辺の実測間隔は維持する
							WetSettings.AdaptCandidateSpacingToBudget = true;
							WetSettings.UseEndpointNeighborRing = true;
							++Report.OceanVerificationAttempts;
							const std::uint64_t BeforeOcean = Report.Evaluations;
							const std::optional<SpillPassage> Wet = FindSpillPassage(Terminal, Anchor, Sample, WetSettings, Stop);
							Report.OceanVerificationEvaluations += Report.Evaluations - BeforeOcean;
							Report.HasLastOceanPassage = Wet.has_value();
							Report.LastOceanPeak = Wet ? std::optional<UnitDirection>{ Wet->Peak } : std::optional<UnitDirection>{};
							Report.LastOceanNodes = Wet ? Wet->Nodes : 0;
							Report.LastOceanSpacingMeters = Wet ? Wet->CandidateSpacingMeters : 0;
							Report.LastOceanCrestMeters = Wet ? Wet->CrestMeters + Wet->MaximumIndependentExcessMeters : 0;
							// 元場・共通予算・中断による失敗を再試行しない返戻
							if(!HasSource || !HasBudget || Stop.stop_requested()) return Invalidate(SpillBypassFailure::Ocean);
							if(!Wet) continue;
							const double Maximum = Wet->CrestMeters + Wet->MaximumIndependentExcessMeters;
							if(Maximum <= Settings.SeaLevelMeters) {
								Result.OceanWitnesses.push_back({ Terminal, Anchor, Maximum });
								HasOcean = true;
								break;
							}
						}
						// 孤立した海面終端を所属確認済大洋へ置換えない返戻
						if(!HasOcean) return Invalidate(SpillBypassFailure::Ocean);
					}
					BranchSamples += Added;
					Result.Connections.push_back(Connection);
					Used[I] = true;
					Changed = true;
					++Report.AddedConnections;
				}
			}
			const auto IsOcean = [&Result](const UnitDirection D) -> bool {
				const Vec3 A = D.Vector();
				for(const SpillOceanWitness &Witness : Result.OceanWitnesses) {
					const Vec3 B = Witness.Terminal.Vector();
					// 同じ元場で水没通路を確認した正確な終端の返戻
					if(A.X == B.X && A.Y == B.Y && A.Z == B.Z) return true;
				}
				// 水位だけで所属を偽造しない返戻
				return false;
			};
			ContinuousSpillSettings GraphSettings;
			GraphSettings.RadiusMeters = Settings.RadiusMeters;
			GraphSettings.SeaLevelMeters = Settings.SeaLevelMeters;
			Graph = BuildContinuousSpillGraph(Result.Connections, IsOcean, GraphSettings, Stop, &Report.Graph);
			// 一部の新極小を捨てて大洋接続済にしない返戻
			if(!Graph) return Invalidate(SpillBypassFailure::Graph);
			// 次の前線探索でも扱える有界な極小数への返戻
			if(Graph->Minima.size() > Settings.MaximumConnections + 1) return Invalidate(SpillBypassFailure::Budget);
			// 実測の低い通路で結べる実終端が，旧峠を越えずに大洋へ接続したか確認する
			HasLowerEscape = true;
			unsigned BranchIndex = 0;
			for(const SpillConnectionBranch &Branch : Failed.Branches) {
				if(Branch.Kind == SpillTerminalKind::SeaLevel) {
					Report.EscapeMeters[BranchIndex++] = Settings.SeaLevelMeters;
					continue;
				}
				bool HasLowerBranch = false;
				for(std::size_t I = 0; I < Graph->Minima.size(); ++I) {
					if(!Graph->Minima[I] || !SameBypassCritical(*Graph->Minima[I], *Branch.Minimum, Settings.RadiusMeters)) continue;
					Report.EscapeMeters[BranchIndex] = Graph->Hierarchy.Saturation.Nodes[I].EscapeMeters;
					HasLowerBranch = I < Graph->Hierarchy.Saturation.Nodes.size() &&
					Graph->Hierarchy.Saturation.Nodes[I].EscapeMeters < Failed.Pass.ElevationMeters - 1E-6;
					break;
				}
				HasLowerEscape = HasLowerEscape && HasLowerBranch;
				++BranchIndex;
			}
			// 現在の全接続と大洋証拠を再検査した結果の返戻
			return true;
		};
		bool HasCheckedRelaxation = !Bypass;
		const auto AddRelaxedCandidates = [&]() -> bool {
			HasCheckedRelaxation = true;
			const std::uint64_t Before = Report.Evaluations;
			if(!Relaxed) {
				++Report.ConnectionRelaxationAttempts;
				SpillStringAcceptance Acceptance;
				if(Settings.UseConnectionCheckpoints && Settings.RelaxOnlyWhenNeeded) Acceptance = [&](const SpillString &Checkpoint) -> bool {
					++Report.ConnectionCheckpointChecks;
					// 停止して失敗を上位へ返し，不完全な候補を公開しない
					// 実条件の欠落又は元場の失敗を，変位収束と混同しない返戻
					if(!AddCandidates(Checkpoint.PassCandidates) || !ValidateConnections()) return true;
					// 低い実大洋出口と，新しい実接続の両方を確認した返戻
					return HasLowerEscape && Report.AddedConnections;
				};
				Relaxed = RelaxSpillString(Bypass->Path, Sample, StringSettings, Stop, Acceptance);
				if(Relaxed) {
					Report.ConnectionRelaxationIterations += Relaxed->Iterations;
					if(Relaxed->AcceptedCheckpoint) Report.AcceptedConnectionIteration = Relaxed->Iterations;
				}
			}
			Report.ConnectionRelaxationEvaluations += Report.Evaluations - Before;
			// 候補不足と，元場・全予算・中断の失敗を混同しない返戻
			if(!HasSource || !HasBudget || Stop.stop_requested() || Report.Failure != SpillBypassFailure::None) return false;
			// 途中で実条件を確認済の候補を二重照会せず，最終候補でも同じ実接続を確認する返戻
			return !Relaxed || Relaxed->AcceptedCheckpoint || AddCandidates(Relaxed->PassCandidates);
		};
		// 従来の緩和順序を使う比較でも，同じ実接続と予算を確認する返戻
		if(Bypass && !Settings.RelaxOnlyWhenNeeded && !AddRelaxedCandidates()) {
			// 緩和と実接続の拒否理由を保持した返戻
			return Reject(Report.Failure == SpillBypassFailure::None ? SpillBypassFailure::Source : Report.Failure);
		}
		std::vector<TerrainCriticalPoint> Examined;
		const double InitialLocalWidth = std::min(256.0, Settings.RadiusMeters * 0.0625);
		const double MaximumLocalWidth = std::min(Settings.MaximumLocalHalfWidthMeters, Settings.RadiusMeters * 0.25);
		double LocalWidth = InitialLocalWidth;
		while(true) {
			// 元場の全追加接続と大洋所属を構成出来ない場合の返戻
			if(!ValidateConnections()) return Reject(Report.Failure);
			if(HasLowerEscape) {
				// 未変更の入力を，接続を追加した成功として返さない返戻
				if(!Report.AddedConnections) return Reject(SpillBypassFailure::NoNewConnection);
				Report.HasLowerConnectionRoute = true;
				Result.Graph = std::move(*Graph);
				// 再構築後の中断への返戻
				if(Stop.stop_requested()) return Reject(SpillBypassFailure::Cancelled);
				// 入力を変更せず，実峠を追加して再検証した結果の返戻
				return Result;
			}
			if(!HasCheckedRelaxation) {
				// 格子辺の最高点で不足した実鞍点を，同じ経路の法線勾配の緩和で補う
				// 低い出口の確認に失敗した場合も，元場・全予算・中断を守る返戻
				if(!AddRelaxedCandidates()) {
					// 途中の接続確認の実拒否理由を保持した返戻
					return Reject(Report.Failure == SpillBypassFailure::None ? SpillBypassFailure::Source : Report.Failure);
				}
				continue;
			}
			// 元の受入水域へ向かう，補完中の同じ上流成分の未試行の谷を選ぶ
			const double Escape0 = Original.Graph.Hierarchy.Saturation.Nodes[OldSites[0]].EscapeMeters;
			const double Escape1 = Original.Graph.Hierarchy.Saturation.Nodes[OldSites[1]].EscapeMeters;
			const unsigned Downstream = Escape0 < Escape1 ? 0 : 1;
			const UnitDirection Target = *Terminals[Downstream];
			const std::uint32_t UpstreamSite = Graph->BranchSites[OutletConnection][1 - Downstream];
			const std::uint32_t Root = Graph->Hierarchy.SiteRoots[UpstreamSite];
			std::optional<TerrainCriticalPoint> Center;
			double Best = std::numeric_limits<double>::infinity();
			for(std::size_t I = 1; I < Graph->Minima.size(); ++I) {
				if(!Graph->Minima[I] || Graph->Hierarchy.SiteRoots[I] != Root) continue;
				const TerrainCriticalPoint &Minimum = *Graph->Minima[I];
				if(
					std::any_of(
						Examined.begin(),
						Examined.end(),
						[&](const TerrainCriticalPoint &Done) -> bool {
							// 同じ不変場の谷で，同じ有限方向を再試行しない返戻
							return SameBypassCritical(Done, Minimum, Settings.RadiusMeters);
						}
					)
				) continue;
				const Vec3 Delta = Minimum.Direction.Vector() - Target.Vector();
				const double Distance = Delta.Dot(Delta);
				if(Distance < Best) {
					Best = Distance;
					Center = Minimum;
				}
			}
			if(!Center) {
				if(LocalWidth < MaximumLocalWidth) {
					// 初めの全小窓・登坂の順を保ち，未発見の場合だけ同じ全成分を広い窓で調べ直す
					LocalWidth = std::min(LocalWidth * 2, MaximumLocalWidth);
					Examined.clear();
					continue;
				}
				// 有限な全窓を尽くしても低い大洋出口が未確認の場合の返戻
				return Reject(SpillBypassFailure::Graph);
			}
			Examined.push_back(*Center);
			// 固定方向の登坂だけで見落した出口を，同じ谷を囲む二次元候補窓でも調べる
			std::vector<TerrainCriticalPoint> RootMinima;
			for(std::size_t I = 1; I < Graph->Minima.size(); ++I) {
				if(Graph->Minima[I] && Graph->Hierarchy.SiteRoots[I] == Root) RootMinima.push_back(*Graph->Minima[I]);
			}
			// 省略してよいのは，同じ入力集合内の両極小を結ぶ準備済の実通路だけ
			std::vector<SpillConnection> Completed;
			for(std::size_t I = 0; I < Result.Connections.size(); ++I) {
				const std::array<std::uint32_t, 2> Sites = Graph->BranchSites[I];
				const SpillConnection &Connection = Result.Connections[I];
				if(
					Connection.Branches[0].Minimum && Connection.Branches[1].Minimum && Graph->Hierarchy.SiteRoots[Sites[0]] == Root &&
					Graph->Hierarchy.SiteRoots[Sites[1]] == Root
				) Completed.push_back(Connection);
			}
			SpillDiscoverySettings WindowSettings;
			WindowSettings.RadiusMeters = Settings.RadiusMeters;
			WindowSettings.MaximumKnownMinima = Settings.MaximumConnections + 1;
			WindowSettings.SeaLevelMeters = Settings.SeaLevelMeters;
			WindowSettings.MaximumPassMeters = Failed.Pass.ElevationMeters - 1E-6;
			WindowSettings.PreferredCenter = Center->Direction;
			WindowSettings.OutletTarget = Target;
			WindowSettings.InitialHalfWidthMeters = LocalWidth;
			WindowSettings.MaximumHalfWidthMeters = WindowSettings.InitialHalfWidthMeters;
			WindowSettings.CandidateSpacingMeters = 16;
			WindowSettings.IncludeInternalConnections = true;
			// 外側で窓拡大と登坂を管理し，小窓内の長い経路緩和で広域候補の予算を使切らない
			WindowSettings.InitialWindowOnly = true;
			WindowSettings.MaximumEvaluations = Settings.MaximumEvaluations;
			++Report.LocalWindows;
			const std::optional<SpillDiscoveryBatch> Batch =
			FindSpillComponentExits(RootMinima, Sample, WindowSettings, Stop, &Report.LastWindow, Completed);
			// 局所窓も同じ元場・中断・総評価量を共有する返戻
			if(!HasSource || !HasBudget || Stop.stop_requested()) return Reject(SpillBypassFailure::Source);
			// 有限窓での未発見と，誤った入力又は準備不能を区別する返戻
			if(!Batch && Report.LastWindow.Failure != SpillDiscoveryFailure::NoIncidentExit) return Reject(SpillBypassFailure::Window);
			const std::size_t BeforeWindow = Pending.size();
			if(Batch) {
				std::vector<UnitDirection> Candidates;
				for(const SpillConnection &Connection : Batch->Connections) Candidates.push_back(Connection.Pass.Direction);
				// 新しい候補を元場の同じ実鞍点と下降枝へ戻せない場合の返戻
				if(!AddCandidates(Candidates)) return Reject(SpillBypassFailure::Source);
			}
			if(Pending.size() > BeforeWindow) continue;
			// 固定方向の登坂は小窓の段階で実施済であり，拡大段階では窓だけを試す
			if(LocalWidth > InitialLocalWidth) continue;
			const Vec3 P = Center->Direction.Vector(), Aim = Target.Vector();
			const std::optional<UnitDirection> Toward = UnitDirection::FromVector(Aim - P * Aim.Dot(P));
			if(!Toward) continue;
			const Vec3 Forward = Toward->Vector(), Side = P.Cross(Forward);
			const double Quarter = std::acos(-1.0) * 0.25, Angle = std::min(16.0, Settings.RadiusMeters * 0.004) / Settings.RadiusMeters;
			const std::array<double, 8> Turns { 0, Quarter, -Quarter, 2 * Quarter, -2 * Quarter, 3 * Quarter, -3 * Quarter, 4 * Quarter };
			const std::size_t Before = Pending.size();
			for(const double Turn : Turns) {
				const Vec3 Along = Forward * std::cos(Turn) + Side * std::sin(Turn);
				const UnitDirection Tip = *UnitDirection::FromVector(P * std::cos(Angle) + Along * std::sin(Angle));
				SpillClimbSettings ClimbSettings;
				ClimbSettings.RadiusMeters = Settings.RadiusMeters;
				ClimbSettings.MaximumLengthMeters = std::min(2000.0, Settings.RadiusMeters * 0.125);
				++Report.ClimbingAttempts;
				const std::optional<SpillClimb> Climb = ClimbSpillString(*Center, Tip, Sample, ClimbSettings, Stop);
				// 候補の未収束と，元場・中断・全予算の不足を混同しない返戻
				if(!HasSource || !HasBudget || Stop.stop_requested()) return Reject(SpillBypassFailure::Source);
				if(!Climb || Climb->Path.empty()) continue;
				// 登坂の候補も，元場の実鞍点と両下降終端を確認出来ない場合の返戻
				if(!AddCandidates(std::vector<UnitDirection>{ Climb->Path.back() })) return Reject(SpillBypassFailure::Source);
				if(Pending.size() > Before) break;
			}
		}
	}

	/**
	 * 実成分の候補を先行し，未確認時だけ同じ入力網の格子候補を残りの予算で調べる
	 * 計算量：最大二回の PrepareCandidates の有界な費用；標本領域は共有し二重確保しない
	 * @param Original 同じ不変な元場の完成網
	 * @param OutletConnection 未確認の実流出口
	 * @param Settings 候補順と，双方を合算する実照会上限
	 * @param Stop 両試行へ伝播する中断要求
	 * @param Diagnostics 先行試行の拒否も保持した結果；追加数は公開する網だけ
	 * @return 実鞍点と両出口を再検証した完成網；未確認の部分網は空
	 */
	std::optional<SpillNetwork> SpillBypassPreparation::Prepare(
		const SpillNetwork &Original,
		const std::uint32_t OutletConnection,
		const SpillBypassSettings &Settings,
		const std::stop_token Stop,
		SpillBypassDiagnostics *const Diagnostics
	) {
		// 比較用と連続候補だけの既存経路の返戻
		if(!Settings.PreferContinuousCandidates || !Settings.UseGridPassages) {
			// 既存の正規経路へ条件をそのまま渡す返戻
			return PrepareCandidates(Original, OutletConnection, Settings, Stop, Diagnostics);
		}
		SpillBypassDiagnostics Local;
		SpillBypassDiagnostics &Report = Diagnostics ? *Diagnostics : Local;
		Report = {};
		if(
			!Settings.MaximumEvaluations || Settings.MaximumEvaluations > 32000000 || !Settings.ContinuousEvaluationLimit ||
			Settings.ContinuousEvaluationLimit > 4000000
		) {
			Report.Failure = Stop.stop_requested() ? SpillBypassFailure::Cancelled : SpillBypassFailure::InvalidInput;
			// 不正な先行予算では元場を調べない返戻
			return {};
		}
		SpillBypassSettings ContinuousSettings = Settings;
		ContinuousSettings.UseGridPassages = false;
		ContinuousSettings.PreferContinuousCandidates = false;
		ContinuousSettings.MaximumEvaluations = std::min(Settings.MaximumEvaluations, Settings.ContinuousEvaluationLimit);
		std::optional<SpillNetwork> Early = PrepareCandidates(Original, OutletConnection, ContinuousSettings, Stop, &Report);
		Report.PreferredContinuousAttempts = 1;
		Report.AcceptedPreferredContinuous = Early.has_value();
		Report.FirstPreferredFailure = Report.Failure;
		Report.PreferredContinuousEvaluations = Report.Evaluations;
		const bool CannotContinue = Report.Failure == SpillBypassFailure::Cancelled || Report.Failure == SpillBypassFailure::Source ||
		Report.Failure == SpillBypassFailure::InvalidInput || Report.Evaluations >= Settings.MaximumEvaluations;
		// 完成，元場欠落，中断，不正入力，又は共通予算不足への返戻
		if(Early || CannotContinue) return Early;
		const SpillBypassDiagnostics First = Report;
		SpillBypassSettings GridSettings = Settings;
		GridSettings.PreferContinuousCandidates = false;
		GridSettings.MaximumEvaluations -= static_cast<std::uint32_t>(First.Evaluations);
		std::optional<SpillNetwork> Result = PrepareCandidates(Original, OutletConnection, GridSettings, Stop, &Report);
		Report.PreferredContinuousAttempts = 1;
		Report.FirstPreferredFailure = First.Failure;
		Report.PreferredContinuousEvaluations = First.Evaluations;
		Report.PreferredGridFallbacks = 1;
		Report.Evaluations += First.Evaluations;
		Report.CacheHits += First.CacheHits;
		Report.Windows += First.Windows;
		Report.MeasuredWindows += First.MeasuredWindows;
		Report.RelaxationAttempts += First.RelaxationAttempts;
		Report.MeasuredRelaxationAttempts += First.MeasuredRelaxationAttempts;
		Report.RelaxationEvaluations += First.RelaxationEvaluations;
		Report.ConnectionRelaxationAttempts += First.ConnectionRelaxationAttempts;
		Report.ConnectionRelaxationEvaluations += First.ConnectionRelaxationEvaluations;
		Report.ConnectionRelaxationIterations += First.ConnectionRelaxationIterations;
		Report.ConnectionCheckpointChecks += First.ConnectionCheckpointChecks;
		Report.RejectedCandidates += First.RejectedCandidates;
		Report.ReusedCandidates += First.ReusedCandidates;
		Report.ClimbingAttempts += First.ClimbingAttempts;
		Report.LocalWindows += First.LocalWindows;
		const bool HadGridOceanAttempt = Report.OceanVerificationAttempts > 0;
		Report.OceanVerificationAttempts += First.OceanVerificationAttempts;
		Report.OceanVerificationEvaluations += First.OceanVerificationEvaluations;
		if(!HadGridOceanAttempt && First.OceanVerificationAttempts) {
			Report.LastOceanDistanceMeters = First.LastOceanDistanceMeters;
			Report.LastOceanCrestMeters = First.LastOceanCrestMeters;
			Report.LastOceanSpacingMeters = First.LastOceanSpacingMeters;
			Report.LastOceanHalfWidthMeters = First.LastOceanHalfWidthMeters;
			Report.LastOceanTerminal = First.LastOceanTerminal;
			Report.LastOceanAnchor = First.LastOceanAnchor;
			Report.LastOceanPeak = First.LastOceanPeak;
			Report.LastOceanNodes = First.LastOceanNodes;
			Report.HasLastOceanPassage = First.HasLastOceanPassage;
		}
		// 不成立だった先行部分網を混ぜず，同じ元網から確認した結果の返戻
		return Result;
	}

	/**
	 * 単独の補完では標本を他の元場又は次の呼出へ持越さない
	 * 計算量：Prepare と同じ有界な時間・追加空間
	 * @param Original 同じ元場の確認済網
	 * @param OutletConnection 拒否された実流出口
	 * @param Source 入力網と同じ不変な元場
	 * @param Settings 有界な探索・保持条件
	 * @param Stop 下位へ伝播する中断
	 * @param Diagnostics 実補完の結果
	 * @return 両出口を再検証した完成網；未確認又は失敗は空
	 */
	std::optional<SpillNetwork> CompleteSpillNetworkBypass(
		const SpillNetwork &Original,
		const std::uint32_t OutletConnection,
		const std::function<TerrainSample(UnitDirection)> &Source,
		const SpillBypassSettings &Settings,
		const std::stop_token Stop,
		SpillBypassDiagnostics *const Diagnostics
	) {
		SpillBypassPreparation Preparation(Source, Settings.MaximumCachedSamples);
		// 独立な準備の同じ正規経路の返戻
		return Preparation.Prepare(Original, OutletConnection, Settings, Stop, Diagnostics);
	}
}
