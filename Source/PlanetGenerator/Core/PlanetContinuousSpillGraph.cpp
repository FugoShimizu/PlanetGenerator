#include "PlanetContinuousSpillGraph.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <tuple>
#include <utility>

namespace PlanetCore {
	/**
	 * 確認済の下降枝の終端をまとめ，元の鞍点高度から包含と下流接続を構成する
	 * 未探索の峠を発見する処理ではなく，渡した接続全体の整合性だけを扱う
	 * 計算量：O(K log K + Q)，K は接続数，Q は近傍の極小照合回数；記憶量 O(K)
	 * @param Connections 同じ元場・半径で準備した接続
	 * @param IsConnectedOcean 海面終端が共通の大洋に属する事を別途確認する述語
	 * @param Settings 半径，海面と極小の位置・標高の照合精度
	 * @param Stop 中断要求
	 * @param Diagnostics 不完全な網を受理しない理由
	 * @return 大洋へ接続した有限網；欠落・競合・中断時は空
	 */
	std::optional<ContinuousSpillGraph> BuildContinuousSpillGraph(
		const std::span<const SpillConnection> Connections,
		const std::function<bool(UnitDirection)> &IsConnectedOcean,
		const ContinuousSpillSettings &Settings,
		const std::stop_token Stop,
		ContinuousSpillDiagnostics *const Diagnostics
	) {
		// 診断の初期化と不完全な網の分類
		if(Diagnostics) *Diagnostics = {};
		const auto Reject = [&](const ContinuousSpillFailure Failure) -> std::optional<ContinuousSpillGraph> {
			if(Diagnostics) Diagnostics->Failure = Stop.stop_requested() ? ContinuousSpillFailure::Cancelled : Failure;
			// 未接続の部分網を完成した排水網として公開しない返戻
			return {};
		};
		// 物理尺度と近傍照合精度の検査
		const bool HasValidSettings =
		std::isfinite(Settings.RadiusMeters) && !(Settings.RadiusMeters < 1000) && Settings.RadiusMeters <= 1E9 &&
		std::isfinite(Settings.SeaLevelMeters) && std::isfinite(Settings.MinimumMergeDistanceMeters) &&
		Settings.MinimumMergeDistanceMeters >= 0.0001 && Settings.MinimumMergeDistanceMeters <= 0.1 &&
		std::isfinite(Settings.MinimumHeightToleranceMeters) && Settings.MinimumHeightToleranceMeters > 0 &&
		Settings.MinimumHeightToleranceMeters <= 0.0001;
		if(!HasValidSettings || !IsConnectedOcean || Connections.empty() || Connections.size() > 1000000) {
			// 有効な準備条件を満たさない場合の返戻
			return Reject(ContinuousSpillFailure::InvalidInput);
		}
		// 開始前の中断時の返戻
		if(Stop.stop_requested()) return Reject(ContinuousSpillFailure::Cancelled);
		// 単一の大洋代表と枝の対応表の準備
		ContinuousSpillGraph Result;
		Result.Sites.push_back({ Settings.SeaLevelMeters, true });
		Result.Minima.emplace_back();
		Result.BranchSites.resize(Connections.size());
		// 元の列挙順に依らない鞍点の順序
		std::vector<std::uint32_t> Order(Connections.size());
		std::iota(Order.begin(), Order.end(), 0U);
		const auto Key = [&](const std::uint32_t Index) -> std::tuple<double, double, double, double> {
			const TerrainCriticalPoint &Pass = Connections[Index].Pass;
			const Vec3 D = Pass.Direction.Vector();
			// 同じ入力集合に対する固定した整列キーの返戻
			return { D.X, D.Y, D.Z, Pass.ElevationMeters };
		};
		for(const SpillConnection &Connection : Connections) {
			// 整列前に非有限な比較値を除く返戻
			if(!std::isfinite(Connection.Pass.ElevationMeters)) return Reject(ContinuousSpillFailure::InvalidConnection);
		}
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				// 完全に同じ鞍点の重複だけは元の番号で区別する返戻
				return Key(A) != Key(B) ? Key(A) < Key(B) : A < B;
			}
		);
		// 物理距離の箱を使い，全極小との総当りを避ける
		std::map<std::array<std::int64_t, 3>, std::vector<std::uint32_t>> Buckets;
		const double Scale = Settings.RadiusMeters / Settings.MinimumMergeDistanceMeters;
		bool HasOceanContact = false;
		for(const std::uint32_t Index : Order) {
			if(Diagnostics) Diagnostics->Connection = Index;
			// 途中の中断時の返戻
			if(Stop.stop_requested()) return Reject(ContinuousSpillFailure::Cancelled);
			// 現在の鞍点の確認
			const SpillConnection &Connection = Connections[Index];
			const bool HasPass =
			Connection.Pass.Kind == TerrainCriticalKind::Saddle && Connection.Pass.ElevationMeters > Settings.SeaLevelMeters;
			// 陸上の鞍点を持たない接続への返戻
			if(!HasPass) return Reject(ContinuousSpillFailure::InvalidConnection);
			for(unsigned Side = 0; Side < 2; ++Side) {
				// 終端の大洋又は極小への対応付
				const SpillConnectionBranch &Branch = Connection.Branches[Side];
				// 空の部分経路を接続として使わない返戻
				if(Branch.Path.Samples.size() < 2) return Reject(ContinuousSpillFailure::InvalidConnection);
				const TerrainDescentSample &End = Branch.Path.Samples.back();
				if(Branch.Kind == SpillTerminalKind::SeaLevel) {
					const bool IsOcean =
					std::isfinite(End.ElevationMeters) && End.ElevationMeters <= Settings.SeaLevelMeters && IsConnectedOcean(End.Direction);
					// 高度だけで孤立水域を大洋へ読み替えない返戻
					if(!IsOcean) return Reject(ContinuousSpillFailure::UnverifiedOcean);
					Result.BranchSites[Index][Side] = 0;
					HasOceanContact = true;
					continue;
				}
				const bool HasMinimum =
				Branch.Minimum && Branch.Minimum->Kind == TerrainCriticalKind::Minimum && std::isfinite(Branch.Minimum->ElevationMeters) &&
				Branch.Minimum->ElevationMeters < Connection.Pass.ElevationMeters;
				// 未確定の終端を最寄の極小へ無条件に割当てない返戻
				if(!HasMinimum) return Reject(ContinuousSpillFailure::InvalidConnection);
				// 物理座標の近傍箱による同一極小の照合
				const TerrainCriticalPoint &Minimum = *Branch.Minimum;
				const Vec3 D = Minimum.Direction.Vector();
				const std::array<std::int64_t, 3> Cell {
					static_cast<std::int64_t>(std::floor(D.X * Scale)),
					static_cast<std::int64_t>(std::floor(D.Y * Scale)),
					static_cast<std::int64_t>(std::floor(D.Z * Scale))
				};
				std::uint32_t Match = MissingSpillIndex;
				for(int X = -1; X < 2; ++X) for(int Y = -1; Y < 2; ++Y) for(int Z = -1; Z < 2; ++Z) {
					const std::map<std::array<std::int64_t, 3>, std::vector<std::uint32_t>>::const_iterator Bucket =
					Buckets.find({ Cell[0] + X, Cell[1] + Y, Cell[2] + Z });
					if(Bucket == Buckets.end()) continue;
					for(const std::uint32_t Candidate : Bucket->second) {
						const TerrainCriticalPoint &Known = *Result.Minima[Candidate];
						const Vec3 A = Known.Direction.Vector();
						const double Distance = Settings.RadiusMeters * std::atan2(A.Cross(D).Length(), A.Dot(D));
						if(Distance > Settings.MinimumMergeDistanceMeters) continue;
						if(std::abs(Known.ElevationMeters - Minimum.ElevationMeters) > Settings.MinimumHeightToleranceMeters) {
							// 両経路の元場が矛盾する場合の返戻
							return Reject(ContinuousSpillFailure::ConflictingMinimum);
						}
						Match = std::min(Match, Candidate);
					}
				}
				if(Match == MissingSpillIndex) {
					Match = static_cast<std::uint32_t>(Result.Sites.size());
					Result.Sites.push_back({ Minimum.ElevationMeters, false });
					Result.Minima.push_back(Minimum);
					Buckets[Cell].push_back(Match);
				}
				Result.BranchSites[Index][Side] = Match;
			}
			// 確認した終端間の通路の登録
			const std::array<std::uint32_t, 2> Pair = Result.BranchSites[Index];
			// 同じ窪地へ戻る峠は別水域への出口にしない
			if(Pair[0] == Pair[1]) continue;
			Result.Edges.push_back({ Pair, Connection.Pass.ElevationMeters });
			Result.EdgeConnections.push_back(Index);
		}
		// 一つも確認済の大洋終端が無い場合の返戻
		if(!HasOceanContact) return Reject(ContinuousSpillFailure::UnverifiedOcean);
		// 包含と下流受入先を分ける階層の準備
		std::optional<DepressionHierarchy> Hierarchy =
		BuildDepressionHierarchy(Result.Sites, Result.Edges, Settings.SeaLevelMeters, Stop);
		// 切離された窪地を除外して部分的な成功にしない返戻
		if(!Hierarchy) return Reject(ContinuousSpillFailure::UnconnectedNetwork);
		Result.Hierarchy = std::move(*Hierarchy);
		// 元標高・元鞍点と其の接続だけを用いた有限網の返戻
		return Result;
	}
}
