#include "PlanetDrainageSampling.h"
#include "PlanetDrainage.h"
#include "PlanetSphericalMesh.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		using Triangle = std::array<std::uint32_t, 3>;

		constexpr double AngularTolerance = 64 * std::numeric_limits<double>::epsilon();
		constexpr std::size_t MaximumSamples = 16384;

		struct LocatedTriangle {
			std::uint32_t Base = 0;
			std::uint32_t Local = 0;
			unsigned Edge = 3;
		};

		/**
		 * 球面三角形の内側と，最も近い共有辺を独立な大円平面で判定する
		 * @param Point 照会する単位方向
		 * @param Corners 外向きの三角形の単位方向
		 * @param Edge 辺上なら其の辺番号，内部なら３を返す
		 * @return 三角形の内部又は数値精度内の境界に有る場合に真
		 */
		bool ContainsSamplingPoint(const Vec3 Point, const std::array<Vec3, 3> &Corners, unsigned &Edge) {
			Edge = 3;
			double Closest = AngularTolerance;
			for(unsigned Index = 0; Index < 3; ++Index) {
				const Vec3 Normal = Corners[Index].Cross(Corners[(Index + 1) % 3]);
				const double Length = Normal.Length();
				// 退化した面を包含と誤認しない返戻
				if(!(Length > 0)) return false;
				const double Distance = Point.Dot(Normal) / Length;
				// 大円境界の外側に有る地点を含めない返戻
				if(Distance < -AngularTolerance || !std::isfinite(Distance)) return false;
				if(std::abs(Distance) <= Closest) {
					Closest = std::abs(Distance);
					Edge = Index;
				}
			}
			// 全ての辺の内側に有る照会結果の返戻
			return true;
		}

		/**
		 * 既存の球面階層を辿り，境界の両側を含む段階６の元面を集める
		 * 計算量：O(d)，d は固定階層の深さ，境界では隣接枝も辿る
		 * @param Point 照会する単位方向
		 * @return BuildUniformSphere と同じ面番号，階層が不正なら空
		 */
		std::vector<std::uint32_t> LocateSamplingFaces(const Vec3 Point) {
			std::vector<TileId> Pending;
			for(std::uint8_t Root = 0; Root < 20; ++Root) Pending.push_back({ 0, Root, 0 });
			std::vector<std::uint32_t> Result;
			while(!Pending.empty()) {
				const TileId Tile = Pending.back();
				Pending.pop_back();
				std::array<Vec3, 3> Corners;
				for(std::uint8_t Corner = 0; Corner < 3; ++Corner) {
					const std::optional<VertexKey> Key = GetVertexKey(Tile, Corner);
					// 有効な階層の頂点を取得出来ない場合の返戻
					if(!Key) return {};
					const std::optional<UnitDirection> Direction = GetDirection(*Key);
					// 階層の単位方向を取得出来ない場合の返戻
					if(!Direction) return {};
					Corners[Corner] = Direction->Vector();
				}
				unsigned Edge = 3;
				if(!ContainsSamplingPoint(Point, Corners, Edge)) continue;
				if(Tile.Level == 6) Result.push_back(static_cast<std::uint32_t>(Tile.RootFace * 4096 + Tile.Path));
				else for(std::uint8_t Child = 0; Child < 4; ++Child) {
					const std::optional<TileId> Next = Tile.Child(Child);
					// 固定深さの子を作れない場合の返戻
					if(!Next) return {};
					Pending.push_back(*Next);
				}
			}
			std::sort(Result.begin(), Result.end());
			// 境界の所属面を番号順に揃えた返戻
			return Result;
		}
	}

	/**
	 * 指定地点を含む面又は共有辺を分割し，局所的に排水の標本位置を追加する
	 * 計算量：O(n log n + s log s + (a + s) d + s k)，n は入力網，a は適応追加点，s は標本数，d は固定階層，k は局所面数
	 * @param InitialDirections 元の段階６の頂点を先頭に保持する適応節点
	 * @param InitialTriangles 段階６の元面の内側だけを分割した共有網
	 * @param Samples 任意の単位方向，入力順・重複に依存せず元網の番号を保持する
	 * @param Stop 中断要求
	 * @return 全周を共有する局所網，過大入力・不正な位相・中断では空
	 */
	std::optional<DrainageSamplingMesh> BuildDrainageSamplingMeshOnMesh(
		const std::span<const UnitDirection> InitialDirections,
		const std::span<const Triangle> InitialTriangles,
		const std::span<const UnitDirection> Samples,
		const std::stop_token Stop
	) {
		// 作業量の上限を超える入力や中断の返戻
		if(Samples.size() > MaximumSamples || !IsClosedSphericalSamplingMesh(InitialDirections, InitialTriangles, Stop)) return {};
		// 新規標本が全て別の頂点である場合でも，作業容量を越えない入力の限定
		if(Samples.size() > 65536 - InitialDirections.size()) return {};
		const std::optional<SphereMesh> Base = BuildUniformSphere(6);
		// 基準の閉じた球面を準備出来ない場合の返戻
		if(!Base || InitialDirections.size() < Base->Directions.size()) return {};
		// 元番号に対応する基準方向の確認
		for(std::size_t Index = 0; Index < Base->Directions.size(); ++Index) {
			// 元番号を別の球面方向として解釈しない返戻
			if((InitialDirections[Index].Vector() - Base->Directions[Index].Vector()).Length() > AngularTolerance) return {};
		}
		DrainageSamplingMesh Result;
		Result.Directions.assign(InitialDirections.begin(), InitialDirections.end());
		Result.SampleNodes.resize(Samples.size());
		std::vector<std::vector<Triangle>> Children(Base->Triangles.size());
		// 元頂点の所属は既知の面番号から構成し，追加地点だけを階層探索する
		std::vector<std::vector<std::uint32_t>> IncidentFaces(InitialDirections.size());
		for(std::uint32_t Face = 0; Face < Base->Triangles.size(); ++Face) {
			for(const std::uint32_t Vertex : Base->Triangles[Face]) IncidentFaces[Vertex].push_back(Face);
		}
		for(std::size_t Index = Base->Directions.size(); Index < InitialDirections.size(); ++Index) {
			// 追加地点の分類中の中断に対する返戻
			if(Stop.stop_requested()) return {};
			IncidentFaces[Index] = LocateSamplingFaces(InitialDirections[Index].Vector());
			// 元面に所属しない節点を受取らない返戻
			if(IncidentFaces[Index].empty()) return {};
		}
		// 適応済の各面の三頂点に共通する元面を求め，既存の辺を保持する
		for(const Triangle &Vertices : InitialTriangles) {
			std::optional<std::uint32_t> Owner;
			for(const std::uint32_t Face : IncidentFaces[Vertices[0]]) {
				const std::vector<std::uint32_t> &Second = IncidentFaces[Vertices[1]], &Third = IncidentFaces[Vertices[2]];
				if(!std::binary_search(Second.begin(), Second.end(), Face) || !std::binary_search(Third.begin(), Third.end(), Face)) continue;
				// 元面の境界へ潰れた面を任意の片側へ押込まない返戻
				if(Owner) return {};
				Owner = Face;
			}
			// 元面を跨ぐ入力の返戻
			if(!Owner) return {};
			Children[*Owner].push_back(Vertices);
		}
		// 一部の元面の欠落を固定網で埋合わせない返戻
		for(const std::vector<Triangle> &Local : Children) if(Local.empty()) return {};
		std::vector<std::uint32_t> Order(Samples.size());
		std::iota(Order.begin(), Order.end(), 0U);
		std::sort(
			Order.begin(),
			Order.end(),
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				const Vec3 P = Samples[A].Vector(), Q = Samples[B].Vector();
				// 同じ位置では入力番号だけを補助にする安定な幾何順序の返戻
				return std::tie(P.X, P.Y, P.Z, A) < std::tie(Q.X, Q.Y, Q.Z, B);
			}
		);
		for(const std::uint32_t Input : Order) {
			// 標本の追加を中断した場合の返戻
			if(Stop.stop_requested()) return {};
			const Vec3 Point = Samples[Input].Vector();
			const std::vector<std::uint32_t> Faces = LocateSamplingFaces(Point);
			// 元網へ所属しない点を単独の頂点にしない返戻
			if(Faces.empty()) return {};
			std::vector<LocatedTriangle> Matches;
			std::uint32_t Existing = std::numeric_limits<std::uint32_t>::max();
			for(const std::uint32_t Face : Faces) {
				// 元網の番号に対応しない階層を使用しない返戻
				if(Face >= Children.size()) return {};
				const std::span<const Triangle> Local = Children[Face];
				for(std::uint32_t Index = 0; Index < Local.size(); ++Index) {
					const Triangle &Vertices = Local[Index];
					std::array<Vec3, 3> Corners;
					for(unsigned Corner = 0; Corner < 3; ++Corner) {
						Corners[Corner] = Result.Directions[Vertices[Corner]].Vector();
						if((Corners[Corner] - Point).Length() <= AngularTolerance) Existing = std::min(Existing, Vertices[Corner]);
					}
					unsigned Edge = 3;
					if(ContainsSamplingPoint(Point, Corners, Edge)) Matches.push_back({ Face, Index, Edge });
				}
			}
			if(Existing != std::numeric_limits<std::uint32_t>::max()) {
				Result.SampleNodes[Input] = Existing;
				continue;
			}
			// 内部の一面又は共有辺の二面以外を同時に分割しない返戻
			if(Matches.empty() || Matches.size() > 2) return {};
			const bool OnEdge = Matches.size() == 2;
			std::array<std::uint32_t, 2> Shared {};
			for(const LocatedTriangle &Match : Matches) {
				// 境界の片側だけ又は無関係な二面を分割しない返戻
				if(OnEdge != Match.Edge < 3) return {};
				if(!OnEdge) continue;
				const Triangle &V = Children[Match.Base][Match.Local];
				const std::uint32_t A = V[Match.Edge], B = V[(Match.Edge + 1) % 3];
				const std::array<std::uint32_t, 2> Key { std::min(A, B), std::max(A, B) };
				if(Shared[0] == Shared[1]) Shared = Key;
				// 同じ辺を共有していない二面を接続しない返戻
				else if(Shared != Key) return {};
			}
			const std::uint32_t Added = static_cast<std::uint32_t>(Result.Directions.size());
			Result.Directions.push_back(Samples[Input]);
			Result.SampleNodes[Input] = Added;
			for(const LocatedTriangle &Match : Matches) {
				std::vector<Triangle> &Local = Children[Match.Base];
				const Triangle V = Local[Match.Local];
				if(OnEdge) {
					const unsigned Edge = Match.Edge;
					Local[Match.Local] = { V[Edge], Added, V[(Edge + 2) % 3] };
					Local.push_back({ Added, V[(Edge + 1) % 3], V[(Edge + 2) % 3] });
				} else {
					Local[Match.Local] = { V[0], V[1], Added };
					Local.push_back({ V[1], V[2], Added });
					Local.push_back({ V[2], V[0], Added });
				}
			}
		}
		Result.Triangles.reserve(Base->Triangles.size() + (Result.Directions.size() - Base->Directions.size()) * 2);
		for(const std::vector<Triangle> &Local : Children) Result.Triangles.insert(Result.Triangles.end(), Local.begin(), Local.end());
		// 指定位置と元頂点を保持した局所網の返戻
		return Result;
	}

	/**
	 * 段階６の固定球面へ，入力順・重複に依存しない標本を追加する
	 * @param Samples 挿入する単位方向
	 * @param Stop 中断要求
	 * @return 全辺を共有する網，不正又は中断では空
	 */
	std::optional<DrainageSamplingMesh> BuildDrainageSamplingMesh(
		const std::span<const UnitDirection> Samples,
		const std::stop_token Stop
	) {
		// 固定網を準備する前の規模と中断の検査
		if(Samples.size() > MaximumSamples || Stop.stop_requested()) return {};
		const std::optional<SphereMesh> Base = BuildUniformSphere(6);
		// 元網を準備出来ない場合の返戻
		if(!Base) return {};
		// 固定網も同じ位相保持の挿入経路を使う返戻
		return BuildDrainageSamplingMeshOnMesh(Base->Directions, Base->Triangles, Samples, Stop);
	}

	/**
	 * 分割辺に位置を限定せず，指定した標本を同じ地質・海岸の排水網へ追加する
	 * @param Ocean 元の海陸と地形
	 * @param Samples 必要な流出口を検出した単位方向
	 * @param Stop 中断要求
	 * @return 既存の解法で接続した排水，不正な準備又は中断では空
	 */
	std::optional<DrainageNetwork> DrainageNetwork::CreateWithSamplingSites(
		const OceanConnectivityField &Ocean,
		const std::span<const UnitDirection> Samples,
		const std::stop_token Stop
	) {
		const std::optional<DrainageSamplingMesh> Mesh = BuildDrainageSamplingMesh(Samples, Stop);
		// 位相を保持した標本網を準備出来ない場合の返戻
		if(!Mesh) return {};
		const auto Height = [&Ocean](const UnitDirection Direction) -> double {
			// 同じ元地形の標高の返戻
			return Ocean.EvaluateElevation(Direction);
		};
		// 面積・沿岸・下流解法を既存の正規経路と共有した返戻
		return CreateOnMesh(Ocean, Mesh->Directions, Mesh->Triangles, Height, true, Stop);
	}
}
