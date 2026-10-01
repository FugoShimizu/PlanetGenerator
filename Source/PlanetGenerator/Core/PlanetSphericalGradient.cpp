#include "PlanetSphericalGradient.h"
#include "PlanetSphericalMesh.h"
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

namespace {
	using PlanetCore::Vec3;

	using FitRow = std::array<double, 6>;

	/**
	 * 重み付き回帰を直交回転で解き，正規方程式で条件数を二乗しない
	 * @tparam Dimension 線形又は二次の係数数
	 * @param Rows 無次元化した５説明変数と右辺
	 * @return 一意な局所係数，退化又は非有限では空
	 */
	template<unsigned Dimension> std::optional<std::array<double, Dimension>> FitLocalPolynomial(
		const std::span<const FitRow> Rows
	) {
		// 上三角化した拡大行列
		std::array<std::array<double, Dimension + 1>, Dimension> Upper {};
		for(const FitRow &Input : Rows) {
			std::array<double, Dimension + 1> Row {};
			for(unsigned Column = 0; Column < Dimension; ++Column) Row[Column] = Input[Column];
			Row[Dimension] = Input[5];
			for(unsigned Column = 0; Column < Dimension; ++Column) {
				const double Length = std::hypot(Upper[Column][Column], Row[Column]);
				if(!Length) continue;
				const double Cosine = Upper[Column][Column] / Length, Sine = Row[Column] / Length;
				for(unsigned Entry = Column; Entry < Dimension + 1; ++Entry) {
					const double Previous = Upper[Column][Entry];
					Upper[Column][Entry] = Cosine * Previous + Sine * Row[Entry];
					Row[Entry] = -Sine * Previous + Cosine * Row[Entry];
				}
			}
		}
		// 相対階数の検査
		double Largest = 0, Smallest = std::numeric_limits<double>::infinity();
		for(unsigned Column = 0; Column < Dimension; ++Column) {
			const double Diagonal = std::abs(Upper[Column][Column]);
			Largest = std::max(Largest, Diagonal);
			Smallest = std::min(Smallest, Diagonal);
		}
		// 不安定な係数を見掛の勾配として公開しない返戻
		if(!std::isfinite(Largest) || !Largest || Smallest < Largest * 1E-9) return {};
		std::array<double, Dimension> Coefficients {};
		for(unsigned Reverse = Dimension; Reverse > 0; --Reverse) {
			const unsigned Row = Reverse - 1;
			double Value = Upper[Row][Dimension];
			for(unsigned Column = Row + 1; Column < Dimension; ++Column) Value -= Upper[Row][Column] * Coefficients[Column];
			Coefficients[Row] = Value / Upper[Row][Row];
			// 非有限な局所解の返戻
			if(!std::isfinite(Coefficients[Row])) return {};
		}
		// 中心値を固定した局所最小二乗解の返戻
		return Coefficients;
	}
}

namespace PlanetCore {
	/**
	 * 同じ閉球面の近傍標高から，標本間隔に応じた共有接線を推定する
	 * SSRFPACK の局所二次近似を参照するが，近傍は実メッシュの２環，重みと次数退化の扱いは独自である
	 * @param Directions 全球の節点方向
	 * @param Triangles 同じ節点の外向き閉球面
	 * @param ElevationsMeters 補間で厳密に保持する節点値
	 * @param Stop 中断要求
	 * @return 全節点の接線と次数統計，不正又は一意な接線を得られない場合は空
	 */
	std::optional<SphericalGradientEstimate> EstimateSphericalGradients(
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::span<const double> ElevationsMeters,
		const std::stop_token Stop
	) {
		// 入力寸法と閉球面の検査
		const bool HasValidMesh =
		Directions.size() == ElevationsMeters.size() && IsClosedSphericalSamplingMesh(Directions, Triangles, Stop);
		// 別の標高列や開いた網を推定へ渡さない返戻
		if(!HasValidMesh || Stop.stop_requested()) return {};
		// 非有限な元標高の返戻
		for(const double Height : ElevationsMeters) if(!std::isfinite(Height)) return {};
		// 全頂点の実隣接関係
		std::vector<std::vector<std::uint32_t>> Neighbors(Directions.size());
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint32_t A = Triangle[Side], B = Triangle[(Side + 1) % 3];
			Neighbors[A].push_back(B);
			Neighbors[B].push_back(A);
		}
		for(std::vector<std::uint32_t> &List : Neighbors) {
			std::sort(List.begin(), List.end());
			List.erase(std::unique(List.begin(), List.end()), List.end());
		}
		// 局所処理用の再利用領域と結果
		std::vector<std::uint32_t> Marked(Directions.size(), std::numeric_limits<std::uint32_t>::max()), Candidates;
		std::vector<FitRow> Rows;
		SphericalGradientEstimate Result;
		Result.Gradients.reserve(Directions.size());
		Candidates.reserve(64);
		Rows.reserve(32);
		for(std::uint32_t Center = 0; Center < Directions.size(); ++Center) {
			// 準備途中の中断時の返戻
			if(Stop.stop_requested()) return {};
			Candidates.clear();
			Rows.clear();
			const Vec3 Up = Directions[Center].Vector();
			Marked[Center] = Center;
			const auto Include = [&](const std::uint32_t Index) -> void {
				// 同じ候補又は裏半球を二重に用いない返戻
				if(Marked[Index] == Center || !(Up.Dot(Directions[Index].Vector()) > 0)) return;
				Marked[Index] = Center;
				Candidates.push_back(Index);
				// 終了
				return;
			};
			for(const std::uint32_t First : Neighbors[Center]) {
				Include(First);
				for(const std::uint32_t Second : Neighbors[First]) Include(Second);
			}
			std::sort(
				Candidates.begin(),
				Candidates.end(),
				[&](const std::uint32_t A, const std::uint32_t B) -> bool {
					const Vec3 DA = Directions[A].Vector() - Up, DB = Directions[B].Vector() - Up;
					const double X = DA.Dot(DA), Y = DB.Dot(DB);
					// 同距離を番号順に固定した候補順の返戻
					return X != Y ? X < Y : A < B;
				}
			);
			if(Candidates.size() > 32) Candidates.resize(32);
			// 二成分の微分さえ一意に求められない場合の返戻
			if(Candidates.size() < 2) return {};
			// 安定した接平面の正規直交基底と標本尺度
			const Vec3 Axis = std::abs(Up.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
			const Vec3 East = UnitDirection::FromVector(Axis.Cross(Up))->Vector(), North = Up.Cross(East);
			double SquareScale = 0;
			for(const std::uint32_t Index : Candidates) {
				const Vec3 Delta = Directions[Index].Vector() - Up * Up.Dot(Directions[Index].Vector());
				SquareScale += Delta.Dot(Delta);
			}
			SquareScale /= Candidates.size();
			const double Scale = std::sqrt(SquareScale);
			// 重複位置から微分を作らない返戻
			if(!std::isfinite(Scale) || Scale < 1E-12) return {};
			for(const std::uint32_t Index : Candidates) {
				const Vec3 Delta = Directions[Index].Vector() - Up * Up.Dot(Directions[Index].Vector());
				const double X = Delta.Dot(East) / Scale, Y = Delta.Dot(North) / Scale, Weight = 1 / (1 + X * X + Y * Y);
				Rows.push_back(
					{
						X * Weight,
						Y * Weight,
						X * X * Weight,
						X * Y * Weight,
						Y * Y * Weight,
						(ElevationsMeters[Index] - ElevationsMeters[Center]) * Weight
					}
				);
			}
			// 標本の局所二次近似と，退化時に限る線形近似
			const std::optional<std::array<double, 5>> Quadratic = FitLocalPolynomial<5>(Rows);
			Vec3 Gradient;
			if(Quadratic) {
				Gradient = (East * (*Quadratic)[0] + North * (*Quadratic)[1]) * (1 / Scale);
				++Result.QuadraticVertices;
			} else {
				const std::optional<std::array<double, 2>> Linear = FitLocalPolynomial<2>(Rows);
				// 一意な接線が存在しない入力の返戻
				if(!Linear) return {};
				Gradient = (East * (*Linear)[0] + North * (*Linear)[1]) * (1 / Scale);
				++Result.LinearVertices;
			}
			// 非有限な再構成を公開しない返戻
			if(!std::isfinite(Gradient.X) || !std::isfinite(Gradient.Y) || !std::isfinite(Gradient.Z)) return {};
			Result.Gradients.push_back(Gradient);
		}
		// 同じ節点値を保持する尺度付き接線の返戻
		return Result;
	}
}
