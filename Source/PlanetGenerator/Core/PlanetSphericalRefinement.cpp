#include "PlanetSphericalRefinement.h"
#include "PlanetGradientProjection.h"
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
#include <vector>

namespace PlanetCore {
	/**
	 * 準備予算と物理尺度が有限且つ範囲内に有るかを検査する
	 * @return 対応する局所細分化を実行出来る場合に真
	 */
	bool SphericalReliefSettings::IsValid() const {
		// 半径が対応範囲に無い場合の返戻
		if(!std::isfinite(RadiusMeters) || RadiusMeters < 1000 || RadiusMeters > 1E9) return false;
		// 正の勾配・誤差と有限な作業上限の返戻
		return std::isfinite(SeaLevelMeters) && std::isfinite(MinimumSlope) && MinimumSlope > 0 && std::isfinite(TargetErrorMeters) &&
		TargetErrorMeters > 0 && MaximumAddedVertices < 16385 && MaximumDepth > 0 && MaximumDepth < 9;
	}

	/**
	 * 内部４試料のうち，下降経路を持つ条件付標高と節点内挿の差が最大の点を選ぶ
	 * @param Triangle 外向きの既存三角形
	 * @param FaceIndex 待機列内で識別する面番号
	 * @param Settings 半径・海面・最小の下降勾配
	 * @param Source 元の連続地形の標高
	 * @return 有効な候補，対象外の面では誤差０，非有限な元標高では空
	 */
	std::optional<PlanetSphericalRefinement::Candidate> PlanetSphericalRefinement::SelectCandidate(
		const Face &Triangle,
		const std::uint32_t FaceIndex,
		const SphericalReliefSettings &Settings,
		const std::function<double(UnitDirection)> &Source
	) {
		Candidate Best;
		Best.FaceIndex = FaceIndex;
		// 最大深さを越えた面は追加分割しない返戻
		if(Triangle.Depth >= Settings.MaximumDepth) return Best;
		std::uint32_t Lowest = Triangle.Vertices[0];
		for(const std::uint32_t Vertex : Triangle.Vertices) {
			// 水没面と海岸を跨ぐ面には陸上の起伏を追加しない返戻
			if(Heights[Vertex] <= Settings.SeaLevelMeters) return Best;
			if(Heights[Vertex] < Heights[Lowest]) Lowest = Vertex;
		}
		constexpr std::array<std::array<double, 3>, 4> Weights {
			{ { 1.0 / 3, 1.0 / 3, 1.0 / 3 }, { 0.6, 0.2, 0.2 }, { 0.2, 0.6, 0.2 }, { 0.2, 0.2, 0.6 } }
		};
		for(const std::array<double, 3> &Weight : Weights) {
			Vec3 Sum;
			double Interpolated = 0;
			for(unsigned Corner = 0; Corner < 3; ++Corner) {
				const std::uint32_t Vertex = Triangle.Vertices[Corner];
				Sum = Sum + Directions[Vertex].Vector() * Weight[Corner];
				Interpolated += Heights[Vertex] * Weight[Corner];
			}
			const std::optional<UnitDirection> Point = UnitDirection::FromVector(Sum);
			// 内部方向を構成出来ない場合の返戻
			if(!Point) return {};
			// 長い元辺を繰返し残した細長い子面は，零接線補間に人工的な急斜面を作る
			double MinimumQuality = 1;
			for(unsigned Side = 0; Side < 3; ++Side) {
				const Vec3 First = Directions[Triangle.Vertices[Side]].Vector();
				const Vec3 Second = Directions[Triangle.Vertices[(Side + 1) % 3]].Vector();
				const Vec3 A = Second - First, B = Point->Vector() - First, C = Point->Vector() - Second;
				const double SumSquared = A.Dot(A) + B.Dot(B) + C.Dot(C), Quality = 2 * std::sqrt(3.0) * A.Cross(B).Length() / SumSquared;
				MinimumQuality = std::min(MinimumQuality, Quality);
			}
			if(MinimumQuality < MinimumChildQuality) continue;
			const double Original = Source(*Point);
			++Stats.SourceEvaluations;
			// 非有限な標高を優先度又は節点へ渡さない返戻
			if(!std::isfinite(Original)) return {};
			const double Chord = (Point->Vector() - Directions[Lowest].Vector()).Length();
			const double Distance = 2 * std::asin(std::min(1.0, Chord * 0.5)) * Settings.RadiusMeters;
			const double Floor = Heights[Lowest] + Settings.MinimumSlope * Distance;
			const double Elevation = std::max(Original, std::nextafter(Floor, std::numeric_limits<double>::infinity()));
			const double Error = std::abs(Elevation - Interpolated);
			// 有限な入力の演算でも範囲を超えた場合の返戻
			if(!std::isfinite(Elevation) || !std::isfinite(Error)) return {};
			if(Error > Best.ErrorMeters) Best = { Point->Vector(), Elevation, Original, Error, MinimumQuality, FaceIndex, Lowest };
		}
		// 既存節点へ下る辺を追加出来る内部候補の返戻
		return Best;
	}

	/**
	 * 元辺を分断せず三角形内部を３面へ分け，誤差の大きい陸上起伏を優先して保持する
	 * 計算量：O(n ＋ b log(n ＋ b))，n は入力面数，b は追加節点上限，元標高照会は O(n ＋ b) 回
	 * @param InDirections 元の共有節点，番号を保持する
	 * @param InTriangles 接続を検証済の閉じた外向き網，辺は分割も除去もしない
	 * @param ElevationsMeters 元節点の有限な調整済標高，値を変更しない
	 * @param Source 保持したい元地形の連続標高
	 * @param Settings 誤差の優先度と作業量の上限
	 * @param Stop 中断要求
	 * @return 元網と追加節点の下降辺を保持する細分化，不正値・予算超過・中断では空
	 */
	std::optional<PlanetSphericalRefinement> PlanetSphericalRefinement::Create(
		const std::span<const UnitDirection> InDirections,
		const std::span<const std::array<std::uint32_t, 3>> InTriangles,
		const std::span<const double> ElevationsMeters,
		const std::function<double(UnitDirection)> &Source,
		const SphericalReliefSettings &Settings,
		const std::stop_token Stop
	) {
		const bool HasValidSize =
		InDirections.size() > 11 && InDirections.size() < 65537 && ElevationsMeters.size() == InDirections.size() &&
		InTriangles.size() == 2 * (InDirections.size() - 2);
		const bool HasValidInput = Settings.IsValid() && Source && HasValidSize && !Stop.stop_requested() &&
		Settings.MaximumAddedVertices <= 65536 - InDirections.size();
		// 不正な入力又は過大な作業領域に対する返戻
		if(!HasValidInput) return {};
		PlanetSphericalRefinement Result;
		Result.Directions.assign(InDirections.begin(), InDirections.end());
		Result.Heights.assign(ElevationsMeters.begin(), ElevationsMeters.end());
		Result.Downstream.resize(InDirections.size());
		Result.Stats.OriginalVertices = static_cast<std::uint32_t>(InDirections.size());
		std::vector<Face> Faces;
		Faces.reserve(InTriangles.size() + 3 * Settings.MaximumAddedVertices);
		for(std::uint32_t Vertex = 0; Vertex < InDirections.size(); ++Vertex) {
			// 非有限な元標高を含む入力の返戻
			if(!std::isfinite(ElevationsMeters[Vertex])) return {};
			Result.Downstream[Vertex] = Vertex;
		}
		// 同じ誤差では既存面番号を優先する決定論的な順序
		const auto Compare = [](const Candidate &Left, const Candidate &Right) -> bool {
			// 誤差と面番号に依る優先度比較の返戻
			return Left.ErrorMeters != Right.ErrorMeters ? Left.ErrorMeters < Right.ErrorMeters : Left.FaceIndex > Right.FaceIndex;
		};
		std::priority_queue<Candidate, std::vector<Candidate>, decltype(Compare)> Pending(Compare);
		for(const std::array<std::uint32_t, 3> &Triangle : InTriangles) {
			const bool HasValidTriangle =
			Triangle[0] < InDirections.size() && Triangle[1] < InDirections.size() && Triangle[2] < InDirections.size();
			// 範囲外の頂点又は中断要求の返戻
			if(!HasValidTriangle || Stop.stop_requested()) return {};
			const Vec3 First = InDirections[Triangle[0]].Vector();
			const Vec3 Second = InDirections[Triangle[1]].Vector(), Third = InDirections[Triangle[2]].Vector();
			// 退化面又は逆向きの面を分割しない返戻
			if(First.Dot((Second - First).Cross(Third - First)) <= 1E-14) return {};
			const std::uint32_t Index = static_cast<std::uint32_t>(Faces.size());
			Faces.push_back({ Triangle, 0, true });
			if(!Settings.MaximumAddedVertices) continue;
			const std::optional<Candidate> Item = Result.SelectCandidate(Faces.back(), Index, Settings, Source);
			// 元関数の評価が不正な場合の返戻
			if(!Item) return {};
			if(Item->ErrorMeters > Settings.TargetErrorMeters) Pending.push(*Item);
		}
		while(!Pending.empty() && Result.Stats.AddedVertices < Settings.MaximumAddedVertices) {
			// 完了前の中断に対する返戻
			if(Stop.stop_requested()) return {};
			const Candidate Item = Pending.top();
			Pending.pop();
			const Face Parent = Faces[Item.FaceIndex];
			// 同じ面の二重分割を不変条件違反として返す返戻
			if(!Parent.IsActive) return {};
			const std::optional<UnitDirection> Point = UnitDirection::FromVector(Item.Direction);
			// 準備済候補の方向が不正な場合の返戻
			if(!Point) return {};
			const std::uint32_t Added = static_cast<std::uint32_t>(Result.Directions.size());
			Result.Directions.push_back(*Point);
			Result.Heights.push_back(Item.ElevationMeters);
			Result.Downstream.push_back(Item.DownstreamVertex);
			++Result.Stats.AddedVertices;
			Result.Stats.MinimumCreatedTriangleQuality = std::min(Result.Stats.MinimumCreatedTriangleQuality, Item.TriangleQuality);
			const double Raise = Item.ElevationMeters - Item.SourceMeters;
			if(Raise > Result.Stats.MaximumConditioningRaiseMeters) {
				Result.Stats.MaximumConditioningRaiseMeters = Raise;
				Result.Stats.MaximumRaiseDirection = Item.Direction;
			}
			Faces[Item.FaceIndex].IsActive = false;
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t Index = static_cast<std::uint32_t>(Faces.size());
				Faces.push_back(
					{ { Parent.Vertices[Side], Parent.Vertices[(Side + 1) % 3], Added }, static_cast<std::uint8_t>(Parent.Depth + 1), true }
				);
				const std::optional<Candidate> Next = Result.SelectCandidate(Faces.back(), Index, Settings, Source);
				// 子面の元標高が不正な場合の返戻
				if(!Next) return {};
				if(Next->ErrorMeters > Settings.TargetErrorMeters) Pending.push(*Next);
			}
		}
		if(!Pending.empty()) Result.Stats.LargestRemainingPriorityMeters = Pending.top().ErrorMeters;
		Result.Triangles.reserve(InTriangles.size() + 2 * Result.Stats.AddedVertices);
		for(const Face &Triangle : Faces) if(Triangle.IsActive) Result.Triangles.push_back(Triangle.Vertices);
		// 既存の辺と節点を保存した細分化の返戻
		return Result;
	}

	/**
	 * 指定した辺の両端微分を単調な球面二次式の範囲へ制限する
	 * 全辺比較は同方向の縮小，実下流辺は接線平面内の最近点射影を使う
	 * @param Gradients 同じ節点順の元接線勾配
	 * @param SeaLevelMeters 海岸を含む面と海底では従来の零接線を保つ基準
	 * @param RequiredDownstream 全節点の下流番号
	 * 空なら全共有辺を制限し，自分自身は選択辺を持たない
	 * @param Stop 中断要求
	 * @return 有限な制限済勾配，不正又は中断では空
	 * 面内部の極小は別途検査する
	 */
	std::optional<std::vector<Vec3>> PlanetSphericalRefinement::ConstrainGradientsImpl(
		const std::span<const Vec3> Gradients,
		const double SeaLevelMeters,
		const std::span<const std::uint32_t> RequiredDownstream,
		const std::stop_token Stop
	) const {
		// 欠落・非有限な海面・中断を黙って零接線へ置換えない返戻
		if(Gradients.size() != Directions.size() || !std::isfinite(SeaLevelMeters) || Stop.stop_requested()) return {};
		std::vector<Vec3> Result;
		std::vector<double> Factors(Directions.size(), 1);
		std::vector<std::vector<SphericalGradientConstraint>> Constraints;
		if(!RequiredDownstream.empty()) Constraints.resize(Directions.size());
		std::vector<bool> FoundRequired(Directions.size(), RequiredDownstream.empty());
		// 欠けた番号を全共有辺への制約と混同しない返戻
		if(!RequiredDownstream.empty() && RequiredDownstream.size() != Directions.size()) return {};
		Result.reserve(Directions.size());
		for(std::size_t Index = 0; Index < Directions.size(); ++Index) {
			const Vec3 Point = Directions[Index].Vector(), Gradient = Gradients[Index];
			// 非有限な元勾配を計算へ渡さない返戻
			if(!std::isfinite(Gradient.X) || !std::isfinite(Gradient.Y) || !std::isfinite(Gradient.Z)) return {};
			const Vec3 Tangential = Gradient - Point * Point.Dot(Gradient);
			// 有限成分の射影が演算範囲を超えた場合の返戻
			if(!std::isfinite(Tangential.X) || !std::isfinite(Tangential.Y) || !std::isfinite(Tangential.Z)) return {};
			Result.push_back(Tangential);
			if(!RequiredDownstream.empty()) {
				const std::uint32_t Parent = RequiredDownstream[Index];
				// 不正な番号と，下降しない指定辺を受入れない返戻
				if(Parent >= Directions.size() || Parent != Index && !(Heights[Index] > Heights[Parent])) return {};
				FoundRequired[Index] = Parent == Index;
			}
		}
		for(const std::array<std::uint32_t, 3> &Face : Triangles) {
			// 準備を中断した場合の返戻
			if(Stop.stop_requested()) return {};
			if(Heights[Face[0]] <= SeaLevelMeters || Heights[Face[1]] <= SeaLevelMeters || Heights[Face[2]] <= SeaLevelMeters) {
				for(const std::uint32_t Vertex : Face) Factors[Vertex] = 0;
			}
			for(unsigned Corner = 0; Corner < 3; ++Corner) for(unsigned Offset = 1; Offset < 3; ++Offset) {
				const std::uint32_t A = Face[Corner], B = Face[(Corner + Offset) % 3];
				if(!RequiredDownstream.empty() && RequiredDownstream[A] == B) FoundRequired[A] = true;
				const Vec3 Start = Directions[A].Vector(), End = Directions[B].Vector();
				const Vec3 Normal = Start.Cross(End - Start);
				const double Sine = Normal.Length(), Cosine = Start.Dot(End);
				// 半球を越す辺又は退化した辺へ単調性の条件を流用しない返戻
				if(!(Sine > 0 && Cosine > 0)) return {};
				const Vec3 Tangent = Normal.Cross(Start) * (1 / Sine);
				const double Derivative = Result[A].Dot(Tangent), Difference = Heights[B] - Heights[A];
				if(!RequiredDownstream.empty()) {
					if(Offset == 1) {
						if(Heights[A] > SeaLevelMeters && Heights[B] > SeaLevelMeters) {
							// 正の節点から全隣接頂点への同次係数を非負に保ち，陸面内部の海面下への行き過ぎを防ぐ
							const double Lower = -2 * (Heights[A] - SeaLevelMeters) * Cosine / Sine;
							Constraints[A].push_back({ Tangent, Result[A].Length(), Lower });
						}
						if(RequiredDownstream[A] == B || RequiredDownstream[B] == A) {
							const double Limit = 2 * std::abs(Difference) * Cosine / Sine;
							Constraints[A].push_back({ Tangent * (Difference > 0 ? 1 : -1), Limit });
						}
					}
					continue;
				}
				// 中間の演算が溢れた場合の返戻
				if(!std::isfinite(Derivative) || !std::isfinite(Difference)) return {};
				if(!Derivative) continue;
				if(!Difference || Difference > 0 != Derivative > 0) Factors[A] = 0;
				else {
					// 両端で |d| <= 2 |ΔH| cot L とすれば全区間の単調性と，元から単調な二次式を保持する
					const double Limit = 2 * std::abs(Difference) * Cosine / Sine;
					Factors[A] = std::min(Factors[A], Limit / std::abs(Derivative));
				}
			}
		}
		// 存在しない下流辺を検査せず通した事にしない返戻
		if(std::find(FoundRequired.begin(), FoundRequired.end(), false) != FoundRequired.end()) return {};
		for(std::size_t Index = 0; Index < Result.size(); ++Index) {
			if(!RequiredDownstream.empty() && Factors[Index] > 0) {
				const std::optional<Vec3> Projected = ProjectSphericalGradient(Directions[Index], Result[Index], Constraints[Index]);
				// 有効な制約射影を準備出来ない場合の返戻
				if(!Projected || Stop.stop_requested()) return {};
				Result[Index] = *Projected;
			} else Result[Index] = Result[Index] * Factors[Index];
		}
		// 全共有辺で同じ節点微分を使う為の制限済勾配の返戻
		return Result;
	}

	/**
	 * 全共有辺の単調性を制限する従来の比較経路
	 * @param Gradients 元の節点勾配
	 * @param SeaLevelMeters 海岸を保持する基準
	 * @param Stop 中断要求
	 * @return 制限済の勾配，不正又は中断では空
	 */
	std::optional<std::vector<Vec3>> PlanetSphericalRefinement::ConstrainGradients(
		const std::span<const Vec3> Gradients,
		const double SeaLevelMeters,
		const std::stop_token Stop
	) const {
		// 全共有辺を対象とした制限の返戻
		return ConstrainGradientsImpl(Gradients, SeaLevelMeters, {}, Stop);
	}

	/**
	 * 実際の下流辺だけを単調に保ち，元勾配から最短距離の接線を共有する比較経路
	 * @param Gradients 元の節点勾配
	 * @param SeaLevelMeters 海岸を保持する基準
	 * @param RequiredDownstream 全節点の下流番号，自分自身は終端を表す
	 * @param Stop 中断要求
	 * @return 制限済の勾配，存在しない辺・不正・中断では空
	 */
	std::optional<std::vector<Vec3>> PlanetSphericalRefinement::ConstrainDrainageGradients(
		const std::span<const Vec3> Gradients,
		const double SeaLevelMeters,
		const std::span<const std::uint32_t> RequiredDownstream,
		const std::stop_token Stop
	) const {
		// 下流配列の欠落を全辺の指定と混同しない返戻
		if(RequiredDownstream.size() != Directions.size()) return {};
		// 明示した下流辺を対象とした制限の返戻
		return ConstrainGradientsImpl(Gradients, SeaLevelMeters, RequiredDownstream, Stop);
	}

	/**
	 * 元節点を先頭に保持する方向列を参照する
	 * @return 元節点と追加節点の方向
	 */
	std::span<const UnitDirection> PlanetSphericalRefinement::GetDirections() const {
		return Directions;
	}

	/**
	 * 元辺を保持した分割面を参照する
	 * @return 外向きの三角形
	 */
	std::span<const std::array<std::uint32_t, 3>> PlanetSphericalRefinement::GetTriangles() const {
		return Triangles;
	}

	/**
	 * 元の調整済標高と追加点の条件付標高を参照する
	 * @return 方向列と同じ順の標高
	 */
	std::span<const double> PlanetSphericalRefinement::GetElevations() const {
		return Heights;
	}

	/**
	 * 追加節点から既存節点へ向かう下降辺を参照する
	 * @return 内部細分化では元節点は自己参照，辺細分化では指定した下流関係も分割して保持
	 */
	std::span<const std::uint32_t> PlanetSphericalRefinement::GetDownstreamVertices() const {
		return Downstream;
	}

	/**
	 * 準備量と標高制約の診断を取得する
	 * @return 準備済統計，残存優先度は最終補間誤差の上界ではない
	 */
	const SphericalReliefStatistics &PlanetSphericalRefinement::GetStatistics() const {
		return Stats;
	}
}
