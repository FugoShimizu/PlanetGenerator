#include "PlanetBasinReconstruction.h"
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
#include <vector>

namespace PlanetCore {
	namespace {
		constexpr std::uint32_t GridSide = 64;
		constexpr double ResidualLimitMeters = 1E-6;

		/**
		 * 三次元で正定値な Wendland C2 核を単位支持幅で評価する
		 * @param DistanceSquared 支持幅で正規化した二乗距離
		 * @return 支持外で０となる連続な重み
		 */
		double Kernel(const double DistanceSquared) {
			// 支持外で厳密に寄与を消す返戻
			if(DistanceSquared >= 1) return 0;
			const double R = std::sqrt(DistanceSquared), Rest = 1 - R, Square = Rest * Rest;
			// 値と二階迄の微分が支持端で零に接続する重みの返戻
			return Square * Square * (1 + 4 * R);
		}
	}

	/**
	 * 球面方向を包含する索引の格子座標を求める
	 * @param Coordinate 単位方向又は探索境界の一成分
	 * @return 有効な格子の座標
	 */
	std::uint32_t PlanetBasinReconstruction::GridCoordinate(const double Coordinate) {
		// 球の端を含む格子座標の返戻
		return static_cast<std::uint32_t>(
			std::clamp(std::floor((Coordinate + 1) * (GridSide * 0.5)), 0.0, static_cast<double>(GridSide - 1))
		);
	}

	/**
	 * 不変な補間標本を格子毎に安定な順序で登録する
	 * 計算量：O(n)，n は標本数
	 */
	void PlanetBasinReconstruction::BuildIndex() {
		Offsets.resize(GridSide * GridSide * GridSide + 1);
		for(const Site &Value : Sites) {
			const Vec3 P = Value.Direction;
			const std::uint32_t Cell = GridCoordinate(P.X) + GridSide * (GridCoordinate(P.Y) + GridSide * GridCoordinate(P.Z));
			++Offsets[Cell + 1];
		}
		for(std::size_t Index = 1; Index < Offsets.size(); ++Index) Offsets[Index] += Offsets[Index - 1];
		std::vector<std::uint32_t> Cursor = Offsets;
		Order.resize(Sites.size());
		for(std::uint32_t Index = 0; Index < Sites.size(); ++Index) {
			const Vec3 P = Sites[Index].Direction;
			const std::uint32_t Cell = GridCoordinate(P.X) + GridSide * (GridCoordinate(P.Y) + GridSide * GridCoordinate(P.Z));
			Order[Cursor[Cell]++] = Index;
		}
		// 終了
		return;
	}

	/**
	 * 一地点へ寄与する有限支持標本を準備計算用の配列へ集める
	 * @param Point 単位方向
	 * @param Candidates 書換える候補配列
	 */
	void PlanetBasinReconstruction::Gather(const Vec3 Point, std::vector<std::uint32_t> &Candidates) const {
		Candidates.clear();
		const double SupportSquared = SupportChord * SupportChord;
		const std::uint32_t MinX = GridCoordinate(Point.X - SupportChord), MaxX = GridCoordinate(Point.X + SupportChord);
		const std::uint32_t MinY = GridCoordinate(Point.Y - SupportChord), MaxY = GridCoordinate(Point.Y + SupportChord);
		const std::uint32_t MinZ = GridCoordinate(Point.Z - SupportChord), MaxZ = GridCoordinate(Point.Z + SupportChord);
		for(std::uint32_t Z = MinZ; Z <= MaxZ; ++Z) {
			for(std::uint32_t Y = MinY; Y <= MaxY; ++Y) for(std::uint32_t X = MinX; X <= MaxX; ++X) {
				const std::uint32_t Cell = X + GridSide * (Y + GridSide * Z);
				for(std::uint32_t At = Offsets[Cell]; At < Offsets[Cell + 1]; ++At) {
					const std::uint32_t Index = Order[At];
					const Vec3 Delta = Point - Sites[Index].Direction;
					if(Delta.Dot(Delta) < SupportSquared) Candidates.push_back(Index);
				}
			}
		}
		std::sort(Candidates.begin(), Candidates.end());
		// 終了
		return;
	}

	/**
	 * 有限支持の符号付き補正を，支持外で０となる形として準備する
	 * @param Directions 重複の無い方向
	 * @param CorrectionsMeters 各方向の補正
	 * @param Width 単位球上の支持弦長
	 * @param Stop 中断要求
	 * @return 有限支持補間，入力不正又は未収束・中断では空
	 */
	std::optional<PlanetBasinReconstruction> PlanetBasinReconstruction::Create(
		const std::span<const UnitDirection> Directions,
		const std::span<const double> CorrectionsMeters,
		const double Width,
		const std::stop_token Stop
	) {
		// 支持外で消える通常補間の返戻
		return CreateImpl(Directions, CorrectionsMeters, Width, false, Stop);
	}

	/**
	 * 全球を覆う固定網で重みを正規化し，定数の盛土に格子状の起伏を加えない
	 * @param Level 固定球面網の細分段階，０～６
	 * @param CorrectionsMeters BuildUniformSphere と同じ地点順の補正
	 * @param Stop 中断要求
	 * @return 定数を再現する連続補間，入力不正又は未収束・中断では空
	 */
	std::optional<PlanetBasinReconstruction> PlanetBasinReconstruction::CreateGlobal(
		const unsigned Level,
		const std::span<const double> CorrectionsMeters,
		const std::stop_token Stop
	) {
		// 準備網の上限又は中断を満たさない入力の返戻
		if(Level > 6 || Stop.stop_requested()) return {};
		const std::optional<SphereMesh> Mesh = BuildUniformSphere(static_cast<std::uint8_t>(Level));
		// 固定網の地点数と一致しない入力の返戻
		if(!Mesh || Mesh->Directions.size() != CorrectionsMeters.size()) return {};
		double Width = 0;
		for(const std::array<std::uint32_t, 3> &Triangle : Mesh->Triangles) for(std::uint32_t Corner = 0; Corner < 3; ++Corner) {
			const Vec3 A = Mesh->Directions[Triangle[Corner]].Vector(), B = Mesh->Directions[Triangle[(Corner + 1) % 3]].Vector();
			Width = std::max(Width, (A - B).Length() * 1.5);
		}
		// 各球面三角形を頂点の支持球で覆い，正規化の分母を正に保つ
		return CreateImpl(Mesh->Directions, CorrectionsMeters, Width, true, Stop);
	}

	/**
	 * 標高補正の標本を疎な正定値補間系で再構成する
	 * 計算量：O(n log n + n k log k + i m)，n は標本数，k は索引候補数，m は非零結合数，i ≤ 256
	 * @param Directions 重複の無い単位方向，最大20万地点
	 * @param CorrectionsMeters 各方向の有限な符号付き補正
	 * @param Width 単位球での有限支持の弦長，０より大きく２以下
	 * @param Normalize 固定網で全球の被覆を確認した場合に重みを正規化する
	 * @param Stop 中断要求
	 * @return 標本残差１マイクロメートル以下の連続補正，入力不正・結合予算超過・収束失敗・中断では空
	 */
	std::optional<PlanetBasinReconstruction> PlanetBasinReconstruction::CreateImpl(
		const std::span<const UnitDirection> Directions,
		const std::span<const double> CorrectionsMeters,
		const double Width,
		const bool Normalize,
		const std::stop_token Stop
	) {
		// 空，不一致，過大な入力又は不正な支持幅の返戻
		if(
			Directions.empty() || Directions.size() != CorrectionsMeters.size() || Directions.size() > 200000 || !std::isfinite(Width) ||
			Width <= 0 || Width > 2 || Width * Width < std::numeric_limits<double>::min() || Stop.stop_requested()
		) return {};
		const std::uint32_t Count = static_cast<std::uint32_t>(Directions.size());
		PlanetBasinReconstruction Result;
		Result.SupportChord = Width;
		Result.Normalized = Normalize;
		Result.Stats.Sites = Count;
		Result.Sites.reserve(Count);
		std::vector<std::uint32_t> Unique(Count);
		std::iota(Unique.begin(), Unique.end(), 0U);
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			// 非有限な標本や中断済みの入力の返戻
			if(!std::isfinite(CorrectionsMeters[Index]) || Stop.stop_requested()) return {};
			Result.Sites.push_back({ Directions[Index].Vector(), 0 });
		}
		// 完全に重複する方向は独立した拘束として扱わない
		std::sort(
			Unique.begin(),
			Unique.end(),
			[&](const std::uint32_t A, const std::uint32_t B) {
				const Vec3 P = Result.Sites[A].Direction, Q = Result.Sites[B].Direction;
				// 三成分の辞書順の返戻
				return P.X != Q.X ? P.X < Q.X : P.Y != Q.Y ? P.Y < Q.Y : P.Z < Q.Z;
			}
		);
		for(std::uint32_t Index = 1; Index < Count; ++Index) {
			const Vec3 Delta = Result.Sites[Unique[Index]].Direction - Result.Sites[Unique[Index - 1]].Direction;
			// 同じ位置へ複数の拘束を与える入力の返戻
			if(!Delta.Dot(Delta)) return {};
		}
		Result.BuildIndex();
		struct Coupling {
			std::uint32_t Index = 0;
			double Weight = 0;
		};
		std::vector<Coupling> Matrix;
		Matrix.reserve(static_cast<std::size_t>(Count) * 20);
		std::vector<std::uint32_t> Rows(Count + 1), Candidates;
		std::vector<double> NodeWeights(Count, 1), RightSide(CorrectionsMeters.begin(), CorrectionsMeters.end());
		const double SupportSquared = Width * Width;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			// 係数準備中の中断時の返戻
			if(Stop.stop_requested()) return {};
			Result.Gather(Result.Sites[Index].Direction, Candidates);
			// 一時行列の結合数を制限し，密な入力による無制限な確保を避ける返戻
			if(Matrix.size() + Candidates.size() > 8000000) return {};
			for(const std::uint32_t Other : Candidates) {
				const Vec3 Delta = Result.Sites[Index].Direction - Result.Sites[Other].Direction;
				Matrix.push_back({ Other, Kernel(Delta.Dot(Delta) / SupportSquared) });
			}
			Rows[Index + 1] = static_cast<std::uint32_t>(Matrix.size());
			if(Normalize) {
				double Weight = 0;
				for(std::uint32_t At = Rows[Index]; At < Rows[Index + 1]; ++At) Weight += Matrix[At].Weight;
				NodeWeights[Index] = Weight;
				RightSide[Index] *= Weight;
			}
		}
		Result.Stats.Couplings = Matrix.size();
		std::vector<double> Coefficients(Count), Residual = RightSide;
		std::vector<double> Search = Residual, Product(Count);
		// 係数の順序を保ち，同じ疎行列で全ての残差を照合する
		const auto Multiply = [&](const std::vector<double> &Input, std::vector<double> &Output) {
			for(std::uint32_t Row = 0; Row < Count; ++Row) {
				double Sum = 0;
				for(std::uint32_t At = Rows[Row]; At < Rows[Row + 1]; ++At) Sum += Matrix[At].Weight * Input[Matrix[At].Index];
				Output[Row] = Sum;
			}
			// 終了
			return;
		};
		double Norm = std::inner_product(Residual.begin(), Residual.end(), Residual.begin(), 0.0);
		bool Converged = false;
		for(std::uint32_t Iteration = 0; Iteration < 257; ++Iteration) {
			// 非有限な費用又は中断された計算の返戻
			if(!std::isfinite(Norm) || Stop.stop_requested()) return {};
			double Maximum = 0;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				Maximum = std::max(Maximum, std::abs(Residual[Index]) / NodeWeights[Index]);
			}
			if(Maximum <= ResidualLimitMeters || Iteration == 256) {
				// 反復式の残差だけで合格にせず，最終係数から再計算する
				Multiply(Coefficients, Product);
				Maximum = 0;
				for(std::uint32_t Index = 0; Index < Count; ++Index) {
					// 再計算した残差が非有限な場合の返戻
					if(!std::isfinite(Product[Index])) return {};
					Residual[Index] = RightSide[Index] - Product[Index];
					Maximum = std::max(Maximum, std::abs(Residual[Index]) / NodeWeights[Index]);
				}
				Result.Stats.MaximumResidualMeters = Maximum;
				if(Maximum <= ResidualLimitMeters) {
					Converged = true;
					break;
				}
				Search = Residual;
				Norm = std::inner_product(Residual.begin(), Residual.end(), Residual.begin(), 0.0);
			}
			if(Iteration == 256) break;
			Multiply(Search, Product);
			const double Curvature = std::inner_product(Search.begin(), Search.end(), Product.begin(), 0.0);
			// 正定値性又は有限性を失った系の返戻
			if(!std::isfinite(Curvature) || Curvature <= 0) return {};
			const double Step = Norm / Curvature;
			for(std::uint32_t Index = 0; Index < Count; ++Index) {
				Coefficients[Index] += Step * Search[Index];
				Residual[Index] -= Step * Product[Index];
			}
			const double Next = std::inner_product(Residual.begin(), Residual.end(), Residual.begin(), 0.0), Ratio = Next / Norm;
			for(std::uint32_t Index = 0; Index < Count; ++Index) Search[Index] = Residual[Index] + Ratio * Search[Index];
			Norm = Next;
			Result.Stats.Iterations = Iteration + 1;
		}
		// 残差の許容値を満たさない補間を公開しない返戻
		if(!Converged || Stop.stop_requested()) return {};
		double AbsoluteSum = 0;
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			// 非有限な係数の返戻
			if(!std::isfinite(Coefficients[Index])) return {};
			Result.Sites[Index].CoefficientMeters = Coefficients[Index];
			AbsoluteSum += std::abs(Coefficients[Index]);
			Result.Stats.AbsoluteBoundMeters =
			Normalize ? std::max(Result.Stats.AbsoluteBoundMeters, std::abs(Coefficients[Index])) : AbsoluteSum;
		}
		// 有限な全球上限を失った補間の返戻
		if(!std::isfinite(4000 * AbsoluteSum * Count / SupportSquared)) return {};
		// 一時行列を解放して保持する，有限支持の不変な標高補正の返戻
		return Result;
	}

	/**
	 * 正規化した正の基底で係数を増加させ，既存の下限を壊さず追加地点の下限を満たす
	 * 計算量：O(q k log k ＋ n)，記憶量：O(n ＋ k)，q は追加地点数，k は索引候補数，n は元標本数
	 * @param Directions 下限を指定する方向，入力順を生成定義の一部として固定する
	 * @param MinimumElevations 同地点で必要な最低標高
	 * @param Stop 中断要求
	 * @return 元の面を全点で下げない C2 面，全球の正規化なし・入力不正・有限性喪失・中断では空
	 */
	std::optional<PlanetBasinReconstruction> PlanetBasinReconstruction::WithLowerBounds(
		const std::span<const UnitDirection> Directions,
		const std::span<const double> MinimumElevations,
		const std::stop_token Stop
	) const {
		// 全球を覆う正の基底を持たない入力の返戻
		if(!Normalized || Directions.size() != MinimumElevations.size() || Directions.size() > 1000000 || Stop.stop_requested()) {
			return {};
		}
		PlanetBasinReconstruction Result = *this;
		std::vector<std::uint32_t> Candidates;
		std::vector<double> Weights;
		const double SupportSquared = SupportChord * SupportChord;
		for(std::size_t At = 0; At < Directions.size(); ++At) {
			const double Minimum = MinimumElevations[At];
			// 非有限な下限又は中断の返戻
			if(!std::isfinite(Minimum) || Stop.stop_requested()) return {};
			const Vec3 Point = Directions[At].Vector();
			Result.Gather(Point, Candidates);
			Weights.clear();
			double Sum = 0, SquareSum = 0, Numerator = 0;
			for(const std::uint32_t Index : Candidates) {
				const Vec3 Delta = Point - Sites[Index].Direction;
				const double Weight = Kernel(Delta.Dot(Delta) / SupportSquared);
				Weights.push_back(Weight);
				Sum += Weight;
				SquareSum += Weight * Weight;
				Numerator += Result.Sites[Index].CoefficientMeters * Weight;
			}
			// 正規化の被覆や有限性を失った場合の返戻
			if(Sum <= 0 || SquareSum <= 0 || !std::isfinite(Numerator)) return {};
			const double Current = Numerator / Sum;
			if(Current >= Minimum) continue;
			// 一つの半空間への最小二乗射影は，正の重みに依り全ての係数を非減少にする
			// 微小な丸め余裕を含め，既に満たした下限を後続の射影で下げない
			const double Change = (Minimum - Current + ResidualLimitMeters) * Sum / SquareSum;
			// 表現不能な係数変更の返戻
			if(!std::isfinite(Change)) return {};
			for(std::size_t Candidate = 0; Candidate < Candidates.size(); ++Candidate) {
				if(!Weights[Candidate]) continue;
				double &Coefficient = Result.Sites[Candidates[Candidate]].CoefficientMeters;
				Coefficient = std::nextafter(Coefficient + Change * Weights[Candidate], std::numeric_limits<double>::infinity());
				// 桁溢れした係数を公開しない返戻
				if(!std::isfinite(Coefficient)) return {};
			}
		}
		Result.Stats.LowerBounds = static_cast<std::uint32_t>(Directions.size());
		Result.Stats.MaximumLowerBoundDeficitMeters = 0;
		for(std::size_t At = 0; At < Directions.size(); ++At) {
			// 全拘束の最終照合中に中断した場合の返戻
			if(Stop.stop_requested()) return {};
			const double Actual = Result.EvaluateElevation(Directions[At]);
			// 実際の索引評価で下限を満たさない場合の返戻
			if(!std::isfinite(Actual) || MinimumElevations[At] - Actual > ResidualLimitMeters) return {};
			Result.Stats.MaximumLowerBoundDeficitMeters =
			std::max(Result.Stats.MaximumLowerBoundDeficitMeters, MinimumElevations[At] - Actual);
		}
		double AbsoluteSum = 0;
		Result.Stats.AbsoluteBoundMeters = 0;
		for(const Site &Value : Result.Sites) {
			AbsoluteSum += std::abs(Value.CoefficientMeters);
			Result.Stats.AbsoluteBoundMeters = std::max(Result.Stats.AbsoluteBoundMeters, std::abs(Value.CoefficientMeters));
		}
		// 導関数を含めた有限な評価上限を失った場合の返戻
		if(!std::isfinite(4000 * AbsoluteSum * Sites.size() / SupportSquared)) return {};
		// 等値補間の残差は初期補間の記録として残し，追加下限の残差とは区別した場の返戻
		return Result;
	}

	/**
	 * 準備した有限支持関数の値と球面接線勾配を評価する
	 * @tparam Derivatives 勾配を含める場合に真
	 * @param Direction 評価する方向
	 * @param Indexed 索引を使う場合に真，全点探索との比較では偽
	 * @return 元地形へ加える符号付き標高補正
	 */
	template<bool Derivatives> TerrainSample PlanetBasinReconstruction::EvaluateImpl(
		const UnitDirection Direction,
		const bool Indexed
	) const {
		const Vec3 Point = Direction.Vector();
		const double SupportSquared = SupportChord * SupportChord;
		TerrainSample Result;
		double WeightSum = 0;
		Vec3 WeightGradient;
		// 一標本の C2 補正を加え，半径方向の微分を除外する
		const auto Accumulate = [&](const std::uint32_t Index) {
			const Site &Value = Sites[Index];
			const Vec3 Delta = Point - Value.Direction;
			const double Q = Delta.Dot(Delta) / SupportSquared;
			// 支持外では値と勾配を厳密に消す終了
			if(Q >= 1 || !Normalized && !Value.CoefficientMeters) return;
			const double R = std::sqrt(Q), Rest = 1 - R, Square = Rest * Rest, Weight = Square * Square * (1 + 4 * R);
			Result.ElevationMeters += Value.CoefficientMeters * Weight;
			if(Normalized) WeightSum += Weight;
			if constexpr(Derivatives) {
				const Vec3 Gradient = (Delta - Point * Delta.Dot(Point)) * (-20 * Square * Rest / SupportSquared);
				Result.TangentialGradientPerRadian = Result.TangentialGradientPerRadian + Gradient * Value.CoefficientMeters;
				if(Normalized) WeightGradient = WeightGradient + Gradient;
			}
			// 終了
			return;
		};
		if(Indexed) {
			const std::uint32_t MinX = GridCoordinate(Point.X - SupportChord), MaxX = GridCoordinate(Point.X + SupportChord);
			const std::uint32_t MinY = GridCoordinate(Point.Y - SupportChord), MaxY = GridCoordinate(Point.Y + SupportChord);
			const std::uint32_t MinZ = GridCoordinate(Point.Z - SupportChord), MaxZ = GridCoordinate(Point.Z + SupportChord);
			for(std::uint32_t Z = MinZ; Z <= MaxZ; ++Z) {
				for(std::uint32_t Y = MinY; Y <= MaxY; ++Y) for(std::uint32_t X = MinX; X <= MaxX; ++X) {
					const std::uint32_t Cell = X + GridSide * (Y + GridSide * Z);
					for(std::uint32_t At = Offsets[Cell]; At < Offsets[Cell + 1]; ++At) Accumulate(Order[At]);
				}
			}
		} else for(std::uint32_t Index = 0; Index < Sites.size(); ++Index) Accumulate(Index);
		if(Normalized) {
			// 被覆の破綻を０補正として隠さない返戻
			if(WeightSum <= 0) return { std::numeric_limits<double>::quiet_NaN(), {} };
			Result.ElevationMeters /= WeightSum;
			if constexpr(Derivatives) {
				Result.TangentialGradientPerRadian =
				(Result.TangentialGradientPerRadian - WeightGradient * Result.ElevationMeters) * (1 / WeightSum);
			}
		}
		// 有限支持の連続補正と解析勾配の返戻
		return Result;
	}

	/**
	 * 地形へ加える補正と法線用の解析勾配を取得する
	 * @param Direction 評価方向
	 * @return 符号付き標高補正と接線勾配
	 */
	TerrainSample PlanetBasinReconstruction::Evaluate(const UnitDirection Direction) const {
		// 索引を使う補正の返戻
		return EvaluateImpl<true>(Direction, true);
	}

	/**
	 * 同じ式から値だけを取得する
	 * @param Direction 評価方向
	 * @return 符号付き標高補正
	 */
	double PlanetBasinReconstruction::EvaluateElevation(const UnitDirection Direction) const {
		// 微分計算を省いた同じ補正の返戻
		return EvaluateImpl<false>(Direction, true).ElevationMeters;
	}

	/**
	 * 索引の取りこぼしを全標本の独立な探索と比較する
	 * @param Direction 評価方向
	 * @return 全標本を調べた補正と接線勾配
	 */
	TerrainSample PlanetBasinReconstruction::EvaluateReference(const UnitDirection Direction) const {
		// 全点探索の補正の返戻
		return EvaluateImpl<true>(Direction, false);
	}

	/**
	 * 標本残差と準備規模を参照する
	 * @return 不変な準備統計
	 */
	const BasinReconstructionStatistics &PlanetBasinReconstruction::Statistics() const {
		// 補間の準備統計の返戻
		return Stats;
	}
}
