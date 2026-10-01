#include "PlanetSphericalContinentalDescent.h"
#include "PlanetSphericalContinentalField.h"
#include "PlanetSphericalWarp.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stop_token>
#include <utility>

namespace PlanetCore {
	/**
	 * 同じ海陸判定と立体射影で，海岸へ到達した円板経路だけを所有する
	 * @param PreparedField 生存期間を共有する不変な比較用球面地形
	 * @param Start 実惑星の開始方向
	 * @param Settings 円板内の区間長と準備予算
	 * @param Error 球面と円板の各段階の失敗理由
	 * @param Stop 中断要求
	 * @return 数値的な海岸へ接続した比較曲線，拒否又は中断時は空
	 */
	std::optional<SphericalContinentalDescent> SphericalContinentalDescent::Create(
		std::shared_ptr<const SphericalContinentalField> PreparedField,
		const UnitDirection Start,
		const ContinentalDescentSettings &Settings,
		SphericalContinentalDescentError &Error,
		const std::stop_token Stop
	) {
		Error = {};
		const auto Fail = [&](const SphericalContinentalDescentReason Reason) -> std::optional<SphericalContinentalDescent> {
			Error.Reason = Reason;
			// 未完成の球面経路を公開しない返戻
			return {};
		};
		if(Stop.stop_requested()) {
			Error.DiskReason = ContinentalDescentError::Cancelled;
			// 中断した準備の返戻
			return Fail(SphericalContinentalDescentReason::Cancelled);
		}
		const bool IsInvalidField = !PreparedField || !PreparedField->Domain || !PreparedField->Warp ||
		PreparedField->Continents.size() != PreparedField->Stats.ContinentalCount || PreparedField->Continents.empty();
		// 不変な構成が不足する準備場の返戻
		if(IsInvalidField) return Fail(SphericalContinentalDescentReason::InvalidField);
		const UnitDirection Source = PreparedField->Warp->ToSource(Start);
		// 大洋の内部を陸の開始点として仮造しない返戻
		if(PreparedField->Domain->EvaluateInSource(Source).CoastSign < 0) return Fail(SphericalContinentalDescentReason::StartInOcean);
		const std::optional<SphericalContinentalField::ContinentalCoordinates> Coordinates =
		PreparedField->FindContinentalCoordinates(Source.Vector());
		// 高さと同じ正規座標を解像出来ない開始点の返戻
		if(!Coordinates) return Fail(SphericalContinentalDescentReason::UnresolvedAtlas);
		const SphericalContinentalField::PreparedContinent &Continent = PreparedField->Continents[Coordinates->Index];
		const std::optional<ContinentalDiskWarpMapping> Shape = Continent.Shape.MapToSource(Coordinates->X, Coordinates->Y);
		// 表示と排水に共有する隆起座標を解像出来ない場合の返戻
		if(!Shape) return Fail(SphericalContinentalDescentReason::UnresolvedAtlas);
		std::optional<ContinentalDescentPath> Path =
		ContinentalDescentPath::Create(Continent.Potential, Shape->Coordinates, Settings, Error.DiskReason, Stop);
		if(!Path) {
			// 円板の中断も球面の中断として保持した返戻
			if(Error.DiskReason == ContinentalDescentError::Cancelled) return Fail(SphericalContinentalDescentReason::Cancelled);
			// 未到達又は解像不足の円板経路の返戻
			return Fail(SphericalContinentalDescentReason::DiskPathRejected);
		}
		SphericalContinentalDescent Result;
		Result.ContinentIndex = Coordinates->Index;
		Result.Field = std::move(PreparedField);
		Result.DiskPath = std::move(Path);
		Result.Hash = (Result.Field->GetDefinitionHash() ^ Result.DiskPath->GetDefinitionHash()) * UINT64_C(1099511628211);
		Result.Hash = (Result.Hash ^ static_cast<std::uint64_t>(Result.ContinentIndex)) * UINT64_C(1099511628211) ^
		SphericalContinentalDescentVersion;
		// 海岸を解像した円板経路でも，球面の接続が不足する候補の返戻
		if(!Result.EvaluateOutlet()) return Fail(SphericalContinentalDescentReason::InvalidProfile);
		// 準備完了前の中断を反映した返戻
		if(Stop.stop_requested()) return Fail(SphericalContinentalDescentReason::Cancelled);
		// 同じ不変場と生存期間を共有した曲線の返戻
		return Result;
	}

	/**
	 * 立体射影の逆と可逆写像の解析微分で，区間内の方向と標高を返す
	 * @param Coordinates 同じ円板内の座標
	 * @param CoordinateDerivative 区間の割合に対する円板座標の微分
	 * @param IsOutlet 河口なら正確な基準海面の値と零勾配を使う
	 * @return 同じ源の比較曲線，座標又は微分を解像出来ない場合は空
	 */
	std::optional<SphericalContinentalDescentSample> SphericalContinentalDescent::EvaluateCoordinates(
		std::array<double, 2> Coordinates,
		const std::array<double, 2> CoordinateDerivative,
		const bool IsOutlet
	) const {
		// 移動後の空の所有状態又は存在しない大陸の返戻
		if(!Field || !DiskPath || ContinentIndex >= Field->Continents.size()) return {};
		const SphericalContinentalField::PreparedContinent &Continent = Field->Continents[ContinentIndex];
		double Height = 0, HeightDerivative = 0;
		if(IsOutlet) {
			// 数値的な閉包を，同じ大陸の解析的な海岸の角度へ対応付ける
			const double Length = std::hypot(Coordinates[0], Coordinates[1]);
			// 海岸の方位を解像出来ない候補の返戻
			if(!(Length > 0)) return {};
			Coordinates[0] /= Length;
			Coordinates[1] /= Length;
			for(
				unsigned Correction = 0;
				Correction < 4 && Coordinates[0] * Coordinates[0] + Coordinates[1] * Coordinates[1] > 1;
				++Correction
			) {
				Coordinates[0] = std::nextafter(Coordinates[0], 0.0);
				Coordinates[1] = std::nextafter(Coordinates[1], 0.0);
			}
			// 海岸の正規化で閉円板外へ残る座標の返戻
			if(Coordinates[0] * Coordinates[0] + Coordinates[1] * Coordinates[1] > 1) return {};
		} else {
			const std::optional<ContinentalPotentialSample> Sample = Continent.Potential.Evaluate(Coordinates[0], Coordinates[1]);
			// 同じ元地形の高さと勾配を解像出来ない区間の返戻
			if(!Sample) return {};
			const double Ratio = Sample->HeightMeters / Field->ReferenceMeters, Square = Ratio * Ratio;
			Height = Sample->HeightMeters * Square;
			HeightDerivative =
			3 * Square * (Sample->GradientPerUnit[0] * CoordinateDerivative[0] + Sample->GradientPerUnit[1] * CoordinateDerivative[1]);
		}
		const std::optional<ContinentalDiskWarpMapping> Shape = Continent.Shape.MapToWorld(Coordinates[0], Coordinates[1]);
		// 同じ隆起の流路を実円板へ戻せない場合の返戻
		if(!Shape) return {};
		const std::array<double, 2> WorldCoordinates = Shape->Coordinates, WorldDerivative = Shape->Pushforward(CoordinateDerivative);
		const double Tangent = Continent.HalfAngleTangent, TangentSquare = Tangent * Tangent;
		const double CoordinateSquare =
		IsOutlet ? 1 : WorldCoordinates[0] * WorldCoordinates[0] + WorldCoordinates[1] * WorldCoordinates[1];
		const double Q = TangentSquare * CoordinateSquare, Denominator = 1 + Q;
		const Vec3 Plane = Continent.East * WorldCoordinates[0] + Continent.North * WorldCoordinates[1];
		const Vec3 Raw = (Continent.Center * (1 - Q) + Plane * (2 * Tangent)) * (1 / Denominator);
		const std::optional<UnitDirection> Source = UnitDirection::FromVector(Raw);
		// 解像出来ない立体射影の返戻
		if(!Source) return {};
		const double QDerivative =
		2 * TangentSquare * (WorldCoordinates[0] * WorldDerivative[0] + WorldCoordinates[1] * WorldDerivative[1]);
		const Vec3 PlaneDerivative = Continent.East * WorldDerivative[0] + Continent.North * WorldDerivative[1];
		const Vec3 RawDerivative = (PlaneDerivative * (2 * Tangent) - (Continent.Center + Raw) * QDerivative) * (1 / Denominator);
		const Vec3 SourceDerivative = (RawDerivative - Source->Vector() * Source->Vector().Dot(RawDerivative)) * (1 / Raw.Length());
		const SphericalWarpMapping Mapping = Field->Warp->MapToWorldWithDerivatives(*Source);
		const Vec3 Derivative = Mapping.Derivatives[0] * SourceDerivative.X + Mapping.Derivatives[1] * SourceDerivative.Y +
		Mapping.Derivatives[2] * SourceDerivative.Z;
		const bool IsFinite = std::isfinite(Height) && std::isfinite(HeightDerivative) && std::isfinite(Derivative.Length());
		// 解像出来ない高さ又は曲線微分を公開しない返戻
		if(!IsFinite) return {};
		// 同じ源の単調変換と球面の解析的な曲線の返戻
		return SphericalContinentalDescentSample{ Mapping.Direction, Derivative, Height, HeightDerivative };
	}

	/**
	 * 指定した区間を，折線の球面補間ではなく元の円板で評価する
	 * @param Segment 構成済みの区間番号
	 * @param Fraction 区間内の割合，０以上１以下
	 * @return 同じ源の方向と比較水位，区間又は割合が不正の場合は空
	 */
	std::optional<SphericalContinentalDescentSample> SphericalContinentalDescent::Evaluate(
		const std::size_t Segment,
		const double Fraction
	) const {
		// 不正な補間又は存在しない区間の返戻
		if(!Field || !DiskPath || !std::isfinite(Fraction) || Fraction < 0 || Fraction > 1 || Segment >= GetSegmentCount()) return {};
		const std::span<const ContinentalDescentPoint> Points = DiskPath->GetPoints();
		const ContinentalDescentPoint &Before = Points[Segment], &After = Points[Segment + 1];
		const std::array<double, 2> Coordinates {
			(1 - Fraction) * Before.X + Fraction * After.X,
			(1 - Fraction) * Before.Y + Fraction * After.Y
		};
		// 河口だけを同じ解析的な海岸へ閉じた評価結果の返戻
		return EvaluateCoordinates(
			Coordinates,
			{ After.X - Before.X, After.Y - Before.Y },
			Fraction == 1 && Segment + 1 == GetSegmentCount()
		);
	}

	/**
	 * 空の区間列でも同じ大陸の海岸位置と球状海面を取得する
	 * @return 河口の方向と零水位，空の所有状態の場合は空
	 */
	std::optional<SphericalContinentalDescentSample> SphericalContinentalDescent::EvaluateOutlet() const {
		// 空の所有状態又は点の無い候補の返戻
		if(!Field || !DiskPath || DiskPath->GetPoints().empty()) return {};
		const ContinentalDescentPoint &Last = DiskPath->GetPoints().back();
		// 同じ海岸の方位に対する零水位の返戻
		return EvaluateCoordinates({ Last.X, Last.Y }, { 0, 0 }, true);
	}

	/**
	 * 基盤の円板経路の準備費用と境界残差を参照する
	 * @return 不変な構成統計，最終河川の景観又は性能保証とは別
	 */
	const ContinentalDescentStatistics &SphericalContinentalDescent::GetDiskStatistics() const {
		static const ContinentalDescentStatistics Empty;
		// 基盤の構成統計又は空の所有状態の統計の返戻
		return DiskPath ? DiskPath->GetStatistics() : Empty;
	}

	/**
	 * 評価可能な構成区間の個数を取得する
	 * @return 非空の円板経路の区間数，空の所有状態では零
	 */
	std::size_t SphericalContinentalDescent::GetSegmentCount() const {
		return DiskPath ? DiskPath->GetSegments().size() : 0;
	}

	/**
	 * 共有地形と経路の再利用識別子を取得する
	 * @return 同じ不変場，円板経路と構成版の識別子
	 */
	std::uint64_t SphericalContinentalDescent::GetDefinitionHash() const {
		return Hash;
	}
}
