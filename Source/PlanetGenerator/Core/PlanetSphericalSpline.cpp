#include "PlanetSphericalSpline.h"
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
	 * 球面三角形の同次座標を求める双対基底を作る
	 * @param Corners 同一半球内の独立な三方向
	 * @return 退化時は空，其の他は方向との内積で座標を得る３ベクトル
	 */
	std::optional<std::array<Vec3, 3>> SphericalSplineField::DualBasis(const std::array<Vec3, 3> &Corners) {
		const Vec3 U = Corners[1] - Corners[0], V = Corners[2] - Corners[0];
		const Vec3 Normal = U.Cross(V);
		const double Determinant = Corners[0].Dot(Normal);
		// 退化した区画を広い補間へ置換えない返戻
		if(!std::isfinite(Determinant) || std::abs(Determinant) <= 1E-14) return {};
		// ほぼ平行な単位方向の積でなく，短い辺の差で逆基底を作る
		const Vec3 Second = V.Cross(Corners[0]) * (1 / Determinant);
		const Vec3 Third = Corners[0].Cross(U) * (1 / Determinant);
		const std::array<Vec3, 3> Dual { Normal * (1 / Determinant) - Second - Third, Second, Third };
		// 同次座標用の逆基底の返戻
		return Dual;
	}

	/**
	 * 共通境界では順序を固定し，丸めで最小座標が僅かに負でも被覆を保つ
	 * @param Bounds 完全に被覆する親又は子の三角形
	 * @param Point 有限な単位方向
	 * @return 最小同次座標が最大になる区画の相対番号
	 */
	std::size_t SphericalSplineField::SelectTriangle(const std::span<const std::array<Vec3, 3>> Bounds, const Vec3 Point) {
		std::size_t Best = 0;
		double Score = -std::numeric_limits<double>::infinity();
		for(std::size_t Index = 0; Index < Bounds.size(); ++Index) {
			const std::array<Vec3, 3> &Dual = Bounds[Index];
			const double Candidate = std::min({ Dual[0].Dot(Point), Dual[1].Dot(Point), Dual[2].Dot(Point) });
			if(Candidate > Score) {
				Score = Candidate;
				Best = Index;
			}
		}
		// 境界の同点では先の区画を所有者とする返戻
		return Best;
	}

	/**
	 * 準備済みの一つの同次二次式と球面勾配を評価する
	 * @param Index 有効な小区画番号
	 * @param Direction 球面方向
	 * @return 標高と単位球面の接線勾配
	 */
	TerrainSample SphericalSplineField::SamplePiece(const std::size_t Index, const UnitDirection Direction) const {
		const Piece &Part = Pieces[Index];
		const Vec3 Point = Direction.Vector();
		const double V = Part.Dual[1].Dot(Point), W = Part.Dual[2].Dot(Point), GV = Part.Quadratic[0] * V + Part.Quadratic[1] * W;
		const double GW = Part.Quadratic[1] * V + Part.Quadratic[2] * W;
		if(HasVertexGradients) {
			const std::array<double, 2> &Jet = VertexJets[Index];
			const double U = Part.Dual[0].Dot(Point), GU = Jet[0] * V + Jet[1] * W;
			const Vec3 Ambient = (Part.Dual[0] * GU + Part.Dual[1] * (GV + Jet[0] * U) + Part.Dual[2] * (GW + Jet[1] * U)) * 2;
			// 指定した頂点接線を含む同次二次式と其の球面勾配の返戻
			return { Part.ReferenceMeters + V * GV + W * GW + 2 * U * GU, Ambient - Point * Ambient.Dot(Point) };
		}
		const Vec3 Ambient = (Part.Dual[1] * GV + Part.Dual[2] * GW) * 2;
		// 原点側の頂点の一定標高を分離し，定数再現時の大きな項の相殺を避ける返戻
		return { Part.ReferenceMeters + V * GV + W * GW, Ambient - Point * Ambient.Dot(Point) };
	}

	/**
	 * 固定階層を辿り，局所分割前の元面を選ぶ
	 * @param Point 単位方向
	 * @return 最終階層内での面番号
	 */
	std::size_t SphericalSplineField::LocateBase(const Vec3 Point) const {
		std::size_t Index = SelectTriangle(std::span(Bounds).first(20), Point), Offset = 0, Count = 20;
		for(std::uint8_t Depth = 1; Depth <= Level; ++Depth) {
			const std::size_t Child = Offset + Count + 4 * (Index - Offset);
			Index = Child + SelectTriangle(std::span(Bounds).subspan(Child, 4), Point);
			Offset += Count;
			Count *= 4;
		}
		// 親階層の保存位置を除いた相対番号の返戻
		return Index - Offset;
	}

	/**
	 * 河口等の追加点を含む共有網から，内心と共有大円交点で６分割した C1 場を作る
	 * 計算量：準備 O(n log n ＋ n d)，保持 O(n)，一点評価 O(d ＋ k)
	 * n は入力面数，d は探索階層，k は照会元面内の局所面数
	 * @param InLevel ０～６の探索階層，入力面は各元面の内部に収まる必要が有る
	 * @param Directions 全ての面で共有する単位方向，最大６５５３６頂点
	 * @param Triangles 外向き且つ閉じた球面網，元階層の境界を越える面は受入れない
	 * @param ElevationsMeters Directions と同じ順序の有限な標高
	 * @param Gradients 各節点の勾配，空なら従来の零接線
	 * @param Stop 中断要求
	 * @return 入力・幾何・有限性が成立した不変場，不正又は中断では空
	 */
	std::optional<SphericalSplineField> SphericalSplineField::CreateImpl(
		const std::uint8_t InLevel,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> ElevationsMeters,
		const std::span<const Vec3> Gradients,
		const std::stop_token Stop
	) {
		const bool HasValidSize =
		InLevel < 7 && Directions.size() > 11 && Directions.size() < 65537 && ElevationsMeters.size() == Directions.size() &&
		Triangles.size() == 2 * (Directions.size() - 2) && (Gradients.empty() || Gradients.size() == Directions.size());
		// 規定した準備量と閉球面の頂点・面の関係を満たさない入力の返戻
		if(!HasValidSize || Stop.stop_requested()) return {};
		const bool HasFiniteHeights = std::all_of(
			ElevationsMeters.begin(),
			ElevationsMeters.end(),
			[](const double Height) -> bool {
				// 有限な標高入力だけを通す返戻
				return std::isfinite(Height);
			}
		);
		// 非有限な標高を受入れない返戻
		if(!HasFiniteHeights) return {};
		SphericalSplineField Result;
		for(const Vec3 Gradient : Gradients) {
			// 非有限な接線を係数へ渡さない返戻
			if(!std::isfinite(Gradient.X) || !std::isfinite(Gradient.Y) || !std::isfinite(Gradient.Z)) return {};
		}
		Result.Level = InLevel;
		Result.HasVertexGradients = !Gradients.empty();
		Result.Stats.Vertices = static_cast<std::uint32_t>(Directions.size());
		Result.Stats.Triangles = static_cast<std::uint32_t>(Triangles.size());
		// 評価時に全球の小区画を走査せず，既存 TileId の根面・４分木順を使う
		for(std::uint8_t Depth = 0; Depth <= InLevel; ++Depth) {
			const std::optional<SphereMesh> Coarse = BuildUniformSphere(Depth);
			// 探索用の網が作れない場合の返戻
			if(!Coarse || Stop.stop_requested()) return {};
			for(const std::array<std::uint32_t, 3> &Triangle : Coarse->Triangles) {
				const std::optional<std::array<Vec3, 3>> Dual = DualBasis(
					{ Coarse->Directions[Triangle[0]].Vector(), Coarse->Directions[Triangle[1]].Vector(), Coarse->Directions[Triangle[2]].Vector() }
				);
				// 被覆検査に必要な逆基底が無い場合の返戻
				if(!Dual) return {};
				Result.Bounds.push_back(*Dual);
			}
		}
		const std::size_t BaseFaces = 20 * (std::size_t{ 1 } << 2 * InLevel);
		const std::size_t BaseOffset = Result.Bounds.size() - BaseFaces;
		Result.FaceOffsets.resize(BaseFaces + 1);
		Result.FaceBounds.reserve(Triangles.size());
		std::vector<std::uint32_t> Owners;
		std::vector<bool> IsUsed(Directions.size(), false);
		Owners.reserve(Triangles.size());
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) {
			const bool HasValidIndices =
			Triangle[0] < Directions.size() && Triangle[1] < Directions.size() && Triangle[2] < Directions.size();
			// 未定義の頂点番号を参照しない返戻
			if(!HasValidIndices || Stop.stop_requested()) return {};
			const std::array<Vec3, 3> Corners {
				Directions[Triangle[0]].Vector(),
				Directions[Triangle[1]].Vector(),
				Directions[Triangle[2]].Vector()
			};
			const double Orientation = Corners[0].Dot((Corners[1] - Corners[0]).Cross(Corners[2] - Corners[0]));
			const std::optional<std::array<Vec3, 3>> Dual = DualBasis(Corners);
			const std::optional<UnitDirection> Middle = UnitDirection::FromVector(Corners[0] + Corners[1] + Corners[2]);
			// 反転・退化・非半球の面を補間へ渡さない返戻
			if(!(Orientation > 0) || !Dual || !Middle) return {};
			const std::size_t Owner = Result.LocateBase(Middle->Vector());
			for(unsigned Corner = 0; Corner < 3; ++Corner) {
				IsUsed[Triangle[Corner]] = true;
				// 別の元面へ跨ぐ三角形を一点の索引へ誤登録しない返戻
				for(const Vec3 Plane : Result.Bounds[BaseOffset + Owner]) if(Plane.Dot(Corners[Corner]) < -1E-9) return {};
			}
			Result.FaceBounds.push_back(*Dual);
			Owners.push_back(static_cast<std::uint32_t>(Owner));
			++Result.FaceOffsets[Owner + 1];
		}
		// 孤立した頂点で閉球面の節点数を偽装しない返戻
		if(std::find(IsUsed.begin(), IsUsed.end(), false) != IsUsed.end()) return {};
		for(std::size_t Face = 1; Face < Result.FaceOffsets.size(); ++Face) {
			// 元面の一部が欠けた網を公開しない返戻
			if(!Result.FaceOffsets[Face]) return {};
			Result.FaceOffsets[Face] += Result.FaceOffsets[Face - 1];
		}
		std::vector<std::uint32_t> Cursors = Result.FaceOffsets;
		Result.FaceIndices.resize(Triangles.size());
		for(std::uint32_t Face = 0; Face < Owners.size(); ++Face) Result.FaceIndices[Cursors[Owners[Face]]++] = Face;
		std::vector<Vec3> Centers;
		std::vector<SplitEdge> Edges;
		Centers.reserve(Triangles.size());
		Edges.reserve(3 * Triangles.size());
		for(std::uint32_t Face = 0; Face < Triangles.size(); ++Face) {
			// 一定間隔で中断を確認する返戻
			if(!(Face % 64) && Stop.stop_requested()) return {};
			const std::array<std::uint32_t, 3> &Indices = Triangles[Face];
			Vec3 Center;
			for(unsigned Corner = 0; Corner < 3; ++Corner) {
				const std::uint32_t A = Indices[Corner], B = Indices[(Corner + 1) % 3], C = Indices[(Corner + 2) % 3];
				Center = Center + Directions[A].Vector() * (Directions[B].Vector() - Directions[C].Vector()).Length();
				Edges.push_back({ std::min(A, B), std::max(A, B), Face, Corner, A < B });
			}
			const std::optional<UnitDirection> Normalized = UnitDirection::FromVector(Center);
			// 内心が退化した場合の返戻
			if(!Normalized) return {};
			Centers.push_back(Normalized->Vector());
		}
		std::sort(
			Edges.begin(),
			Edges.end(),
			[](const SplitEdge &A, const SplitEdge &B) -> bool {
				// 共有辺と其の所有面を決定論的に並べる返戻
				if(A.A != B.A) return A.A < B.A;
				// 辺の第２頂点に依る順序の返戻
				if(A.B != B.B) return A.B < B.B;
				// 同じ辺では面番号に依る順序の返戻
				return A.Face < B.Face;
			}
		);
		std::vector<std::array<Vec3, 3>> Splits(Triangles.size());
		for(std::size_t Index = 0; Index < Edges.size(); Index += 2) {
			const SplitEdge &A = Edges[Index], &B = Edges[Index + 1];
			const bool HasMatchingPair = A.A == B.A && A.B == B.B && A.Face != B.Face && A.IsForward != B.IsForward;
			const bool HasThirdOwner = Index + 2 < Edges.size() && Edges[Index + 2].A == A.A && Edges[Index + 2].B == A.B;
			// 閉じた共有辺の対と中断状態を確認する返戻
			if(!HasMatchingPair || HasThirdOwner || Stop.stop_requested()) return {};
			const Vec3 Start = Directions[A.A].Vector(), End = Directions[A.B].Vector();
			const std::optional<UnitDirection> Crossing =
			UnitDirection::FromVector(Start.Cross(End - Start).Cross(Centers[A.Face].Cross(Centers[B.Face] - Centers[A.Face])));
			// 内心間の大円と辺が交わらない退化時の返戻
			if(!Crossing) return {};
			Vec3 Split = Crossing->Vector();
			if(Split.Dot(Start + End) < 0) Split = Split * -1;
			Splits[A.Face][A.Side] = Split;
			Splits[B.Face][B.Side] = Split;
		}
		Result.Pieces.reserve(6 * Triangles.size());
		if(Result.HasVertexGradients) Result.VertexJets.reserve(6 * Triangles.size());
		for(std::size_t Face = 0; Face < Triangles.size(); ++Face) {
			// 準備の途中で要求が無効になった場合の返戻
			if(!(Face % 64) && Stop.stop_requested()) return {};
			const std::array<std::uint32_t, 3> &Indices = Triangles[Face];
			const std::array<Vec3, 3> Corners {
				Directions[Indices[0]].Vector(),
				Directions[Indices[1]].Vector(),
				Directions[Indices[2]].Vector()
			};
			const std::optional<std::array<Vec3, 3>> Dual = DualBasis(Corners);
			// 元区画が退化した場合の返戻
			if(!Dual) return {};
			const Vec3 Center = Centers[Face];
			std::array<Vec3, 3> Jets {};
			if(!Gradients.empty()) for(unsigned Vertex = 0; Vertex < 3; ++Vertex) {
				const Vec3 Gradient = Gradients[Indices[Vertex]], Point = Corners[Vertex];
				Jets[Vertex] = Gradient - Point * Point.Dot(Gradient);
			}
			for(unsigned Side = 0; Side < 3; ++Side) for(unsigned Half = 0; Half < 2; ++Half) {
				const unsigned Anchor = (Side + Half) % 3, Other = (Side + 1 - Half) % 3;
				const Vec3 A = Corners[Anchor], B = Corners[Other], Split = Splits[Face][Side];
				const std::optional<std::array<Vec3, 3>> LocalDual = DualBasis({ A, Split, Center });
				// 分割区画の退化を隠さない返戻
				if(!LocalDual) return {};
				const double Reference = ElevationsMeters[Indices[Anchor]];
				double CenterValue = 0;
				for(unsigned Vertex = 0; Vertex < 3; ++Vertex) {
					CenterValue += (*Dual)[Vertex].Dot(Center) *
					((ElevationsMeters[Indices[Vertex]] - Reference) * Corners[Vertex].Dot(Center) + 0.5 * Jets[Vertex].Dot(Center));
				}
				const Vec3 EdgeNormal = A.Cross(B - A);
				const double Beta = EdgeNormal.Dot(A.Cross(Split - A)) / EdgeNormal.Dot(EdgeNormal);
				const double Difference = ElevationsMeters[Indices[Other]] - Reference, Alpha = Split.Dot(A) - Beta * A.Dot(B);
				const Vec3 EdgeJet = (Jets[Anchor] * Alpha + Jets[Other] * Beta) * 0.5;
				const std::array<double, 3> Coefficients {
					Beta * Difference * B.Dot(Split) + EdgeJet.Dot(Split),
					Beta * Difference * B.Dot(Center) + EdgeJet.Dot(Center),
					CenterValue
				};
				const std::array<double, 2> VertexJet { 0.5 * Jets[Anchor].Dot(Split), 0.5 * Jets[Anchor].Dot(Center) };
				const bool HasFiniteCoefficients = std::all_of(
					Coefficients.begin(),
					Coefficients.end(),
					[](const double C) -> bool {
						// 有限な二次式の係数だけを通す返戻
						return std::isfinite(C);
					}
				);
				// 極端な標高差が係数を溢れさせた場合の返戻
				if(!HasFiniteCoefficients || !std::isfinite(VertexJet[0]) || !std::isfinite(VertexJet[1])) return {};
				Result.Pieces.push_back({ *LocalDual, Coefficients, Reference });
				if(Result.HasVertexGradients) Result.VertexJets.push_back(VertexJet);
			}
		}
		Result.Stats.Pieces = static_cast<std::uint32_t>(Result.Pieces.size());
		Result.Stats.ResidentBytes = Result.Bounds.capacity() * sizeof(Result.Bounds[0]) + Result.Pieces.capacity() * sizeof(Piece) +
		Result.VertexJets.capacity() * sizeof(Result.VertexJets[0]) + Result.FaceBounds.capacity() * sizeof(Result.FaceBounds[0]) +
		(Result.FaceOffsets.capacity() + Result.FaceIndices.capacity()) * sizeof(std::uint32_t);
		// 補間値だけを RAM に保持する比較場の返戻
		return Result;
	}

	/**
	 * 指定した節点値を従来の零接線で補間する
	 * @param InLevel 探索階層
	 * @param Directions 共有節点
	 * @param Triangles 閉じた外向き網
	 * @param ElevationsMeters 節点標高
	 * @param Stop 中断要求
	 * @return 既存経路と同じ補間，不正又は中断では空
	 */
	std::optional<SphericalSplineField> SphericalSplineField::CreateOnMesh(
		const std::uint8_t InLevel,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> ElevationsMeters,
		const std::stop_token Stop
	) {
		// 従来の零接線経路の返戻
		return CreateImpl(InLevel, Directions, Triangles, ElevationsMeters, {}, Stop);
	}

	/**
	 * 節点の値と接線成分を同じ C1 場へ補間する
	 * @param InLevel 探索階層
	 * @param Directions 共有節点
	 * @param Triangles 閉じた外向き網
	 * @param ElevationsMeters 節点標高
	 * @param Gradients 節点勾配，半径方向の成分は除外する
	 * @param Stop 中断要求
	 * @return 指定した値と接線を補間する場，不正又は中断では空
	 */
	std::optional<SphericalSplineField> SphericalSplineField::CreateOnMeshWithGradients(
		const std::uint8_t InLevel,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> ElevationsMeters,
		const std::span<const Vec3> Gradients,
		const std::stop_token Stop
	) {
		// 勾配の欠落を暗黙の零接線で代用しない返戻
		if(Gradients.size() != Directions.size()) return {};
		// 値と接線を共有する補間経路の返戻
		return CreateImpl(InLevel, Directions, Triangles, ElevationsMeters, Gradients, Stop);
	}

	/**
	 * 従来の一様球面網を同じ構成経路で補間する
	 * @param InLevel ０～６の固定全球網，最大４０９６２頂点
	 * @param ElevationsMeters BuildUniformSphere と同じ順序の有限な標高
	 * @param Stop 中断要求
	 * @return 不正な標高・階層・中断では空，其の他は不変な補間場
	 */
	std::optional<SphericalSplineField> SphericalSplineField::Create(
		const std::uint8_t InLevel,
		const std::span<const double> ElevationsMeters,
		const std::stop_token Stop
	) {
		// 過大な網を構成する前に入力を拒否する返戻
		if(InLevel > 6 || Stop.stop_requested()) return {};
		const std::size_t ExpectedVertices = 10 * (std::size_t{ 1 } << 2 * InLevel) + 2;
		// 欠落した節点値を受入れない返戻
		if(ElevationsMeters.size() != ExpectedVertices) return {};
		const std::optional<SphereMesh> Mesh = BuildUniformSphere(InLevel);
		// 一様網を構成出来ない場合の返戻
		if(!Mesh) return {};
		// 河口を含む網と共通の構成経路の返戻
		return CreateOnMesh(InLevel, Mesh->Directions, Mesh->Triangles, ElevationsMeters, Stop);
	}

	/**
	 * 根面・４分木・６小区画の順で地点を選び，同じ場の値と勾配を返す
	 * @param Direction 有限な球面方向
	 * @return 頂点値と接線を連続に補間した標本
	 */
	TerrainSample SphericalSplineField::Evaluate(const UnitDirection Direction) const {
		const Vec3 Point = Direction.Vector();
		const std::size_t Base = LocateBase(Point);
		std::size_t Selected = FaceIndices[FaceOffsets[Base]];
		double Score = -std::numeric_limits<double>::infinity();
		for(std::size_t Candidate = FaceOffsets[Base]; Candidate < FaceOffsets[Base + 1]; ++Candidate) {
			const std::uint32_t Face = FaceIndices[Candidate];
			const std::array<Vec3, 3> &Dual = FaceBounds[Face];
			const double Value = std::min({ Dual[0].Dot(Point), Dual[1].Dot(Point), Dual[2].Dot(Point) });
			if(Value > Score) {
				Score = Value;
				Selected = Face;
			}
		}
		const std::size_t First = 6 * Selected;
		std::array<std::array<Vec3, 3>, 6> Domains;
		for(unsigned Part = 0; Part < 6; ++Part) Domains[Part] = Pieces[First + Part].Dual;
		// 境界の両側が同じ値と接線へ接続した場の返戻
		return SamplePiece(First + SelectTriangle(Domains, Point), Direction);
	}

	/**
	 * 指定した小区画の同次二次式を直接評価する
	 * @param Index 小区画番号
	 * @param Direction 任意の球面方向，区画内検査は行わない
	 * @return 番号が有効なら延長した二次式の標本，其の他は空
	 */
	std::optional<TerrainSample> SphericalSplineField::EvaluatePiece(const std::size_t Index, const UnitDirection Direction) const {
		// 無効な診断番号の返戻
		if(Index >= Pieces.size()) return {};
		// 区画単独の値と接線の返戻
		return SamplePiece(Index, Direction);
	}

	/**
	 * 保持した逆基底から小区画の角を復元する
	 * @param Index 小区画番号
	 * @return 原頂点・共有辺の分割点・内心の３方向，範囲外では空
	 */
	std::optional<std::array<Vec3, 3>> SphericalSplineField::GetPieceDomain(const std::size_t Index) const {
		// 無効な診断番号の返戻
		if(Index >= Pieces.size()) return {};
		const std::array<Vec3, 3> &Dual = Pieces[Index].Dual;
		std::array<Vec3, 3> Corners;
		for(unsigned Corner = 0; Corner < 3; ++Corner) {
			const Vec3 Cross = Dual[(Corner + 1) % 3].Cross(Dual[(Corner + 2) % 3]);
			const std::optional<UnitDirection> Point = UnitDirection::FromVector(Cross);
			// 準備済みの幾何が復元出来ない場合の返戻
			if(!Point) return {};
			Corners[Corner] = Point->Vector() * (Cross.Dot(Dual[Corner]) < 0 ? -1 : 1);
		}
		// 検査用の区画形状の返戻
		return Corners;
	}

	/**
	 * 保持する区画数と容量を取得する
	 * @return 同じ不変場の保持量
	 */
	const SphericalSplineStatistics &SphericalSplineField::GetStatistics() const {
		// 準備済みの統計の返戻
		return Stats;
	}
}
