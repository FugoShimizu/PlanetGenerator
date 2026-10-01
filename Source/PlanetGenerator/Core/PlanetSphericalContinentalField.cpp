#include "PlanetSphericalContinentalField.h"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stop_token>
#include <utility>

namespace PlanetCore {
	namespace {
		/**
		 * 固定幅の状態から端点を含まない乱数を得る
		 * @param State 生成定義のみに使う状態
		 * @return ０より大きく１より小さい５２ビットの値
		 */
		double RecipeRandom(std::uint64_t &State) {
			State += UINT64_C(0X9E3779B97F4A7C15);
			std::uint64_t Bits = State;
			Bits = (Bits ^ Bits >> 30) * UINT64_C(0XBF58476D1CE4E5B9);
			Bits = (Bits ^ Bits >> 27) * UINT64_C(0X94D049BB133111EB);
			Bits ^= Bits >> 31;
			// 端点を避けた固定の乱数値の返戻
			return (static_cast<double>(Bits >> 12) + 0.5) / 4503599627370496.0;
		}
	}

	/**
	 * 海域の構成を検証し，各陸域の有限源と立体射影の尺度を準備する
	 * @param Settings 領域，各大陸の隆起，単調変換の尺度，基準海底深度
	 * @param Error 準備の失敗理由，成功時は None
	 * @param Stop 中断要求
	 * @return 不変な比較用の場，不正入力・数値尺度の不足・中断時は空
	 */
	std::optional<SphericalContinentalField> SphericalContinentalField::Create(
		const SphericalContinentalFieldSettings &Settings,
		SphericalContinentalFieldError &Error,
		const std::stop_token Stop
	) {
		// 入力と失敗理由の準備
		Error = SphericalContinentalFieldError::None;
		const auto Fail = [&](const SphericalContinentalFieldError Reason) -> std::optional<SphericalContinentalField> {
			Error = Reason;
			// 未完成の比較場を公開しない返戻
			return {};
		};
		// 中断された準備の返戻
		if(Stop.stop_requested()) return Fail(SphericalContinentalFieldError::Cancelled);
		const double Reference = Settings.PotentialReferenceMeters, OceanDepth = Settings.OceanDepthMeters;
		const std::size_t Count = Settings.Domain.Continents.size();
		const bool IsLayered = !Settings.UpliftLayers.empty();
		const bool IsInvalid = (
			IsLayered ?
			!Settings.Uplifts.empty() || Settings.UpliftLayers.size() != Count :
			Settings.Uplifts.size() != Count || Settings.BlendTemperatureMeters != 0
		) || !Settings.DiskShapes.empty() && Settings.DiskShapes.size() != Count || !std::isfinite(Reference) || Reference <= 0 ||
		!std::isfinite(OceanDepth) || OceanDepth <= 0 ||
		IsLayered && (!std::isfinite(Settings.BlendTemperatureMeters) || !(Settings.BlendTemperatureMeters > 0));
		// 大陸と源の対応又は有限な物理尺度を持たない入力の返戻
		if(IsInvalid) return Fail(SphericalContinentalFieldError::InvalidSettings);
		// 海域拘束と対応する写像の準備
		SphericalContinentalField Result;
		ConnectedOceanDomainError DomainError = ConnectedOceanDomainError::None;
		Result.Domain = ConnectedOceanDomain::Create(Settings.Domain, DomainError, Stop);
		// 海陸比と連結海域の前提を満たさない入力の返戻
		if(!Result.Domain) {
			// 中断と領域の拒否を区別した返戻
			return Fail(
				Stop.stop_requested() ? SphericalContinentalFieldError::Cancelled : SphericalContinentalFieldError::OceanDomainRejected
			);
		}
		Result.Warp = PlanetSphericalWarp::Create(Settings.Domain.Warp);
		// 領域と同じ写像を準備出来ない入力の返戻
		if(!Result.Warp) return Fail(SphericalContinentalFieldError::InvalidSettings);
		Result.RadiusMeters = Settings.Domain.Warp.RadiusMeters;
		Result.ReferenceMeters = Reference;
		Result.OceanDepthMeters = OceanDepth;
		// 海底を惑星中心へ近付ける過大な深度の返戻
		if(Result.OceanDepthMeters > 0.25 * Result.RadiusMeters) return Fail(SphericalContinentalFieldError::ReliefOutsideSphere);
		// 構成統計と識別子の初期化
		Result.Stats.ContinentalCount = Count;
		Result.Stats.OceanFraction = Result.Domain->Statistics().OceanFraction;
		Result.Stats.AbsoluteReliefBoundMeters = Result.OceanDepthMeters;
		Result.Hash = Result.Domain->DefinitionHash() ^ SphericalContinentalFieldVersion;
		const auto AppendHash = [&](const std::uint64_t Bits) -> void {
			Result.Hash = (Result.Hash ^ Bits) * UINT64_C(1099511628211);
			return;
		};
		AppendHash(std::bit_cast<std::uint64_t>(Result.ReferenceMeters));
		AppendHash(std::bit_cast<std::uint64_t>(Result.OceanDepthMeters));
		// 独立した大陸の隆起と正規円板座標の準備
		Result.Continents.reserve(Count);
		for(std::size_t i = 0; i < Count; ++i) {
			std::optional<ContinentalPotential> Potential = IsLayered ?
			ContinentalPotential::CreateBlend(Settings.UpliftLayers[i], Settings.BlendTemperatureMeters, Stop) :
			ContinentalPotential::Create(Settings.Uplifts[i], Stop);
			// 数値的な隆起拘束の不足又は中断された源の返戻
			if(!Potential) {
				// 未完成の源又は中断の返戻
				return Fail(Stop.stop_requested() ? SphericalContinentalFieldError::Cancelled : SphericalContinentalFieldError::UpliftRejected);
			}
			// 同じ海岸を固定した可逆な円板内変形の準備
			const ContinentalDiskWarpSettings EmptyShape;
			const ContinentalDiskWarpSettings &ShapeSettings = Settings.DiskShapes.empty() ? EmptyShape : Settings.DiskShapes[i];
			std::optional<ContinentalDiskWarp> Shape = ContinentalDiskWarp::Create(ShapeSettings);
			// 非可逆又は海岸を跨ぐ山地変形の返戻
			if(!Shape) return Fail(SphericalContinentalFieldError::ShapeRejected);
			// 大陸円板の正規座標と丸めの尺度の準備
			const ContinentalDomainCap &Cap = Settings.Domain.Continents[i];
			const Vec3 Center = Cap.Center.Vector(), Axis = std::abs(Center.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 };
			const Vec3 East = UnitDirection::FromVector(Axis.Cross(Center))->Vector();
			const Vec3 North = UnitDirection::FromVector(Center.Cross(East))->Vector();
			const double Cosine = std::cos(Cap.RadiusRadians), Tangent = std::tan(0.5 * Cap.RadiusRadians);
			const double Roundoff = 32 * std::numeric_limits<double>::epsilon() / (1 - Cosine);
			const bool HasResolvedAtlas = std::isfinite(Roundoff) && Roundoff < 1E-8 && std::isfinite(Tangent) && Tangent > 0;
			// 境界の正規座標が丸めに埋もれる大陸の返戻
			if(!HasResolvedAtlas) return Fail(SphericalContinentalFieldError::UnresolvedAtlas);
			// 単調変換の表現範囲の検査
			const double Bound = Potential->GetAbsoluteHeightBoundMeters(), Ratio = Bound / Result.ReferenceMeters;
			const double HeightBound = Bound * Ratio * Ratio;
			// 単調変換の上界を表現出来ない又は球面を著しく歪める設定の返戻
			if(!std::isfinite(HeightBound) || HeightBound <= 0 || HeightBound > 0.25 * Result.RadiusMeters) {
				// 安全な比較球面の半径を保持出来ない入力の返戻
				return Fail(SphericalContinentalFieldError::ReliefOutsideSphere);
			}
			Result.Stats.SourceCount += Potential->GetSourceCount();
			Result.Stats.ShapeTwistCount += Shape->GetTwistCount();
			Result.Stats.AbsoluteReliefBoundMeters = std::max(Result.Stats.AbsoluteReliefBoundMeters, HeightBound);
			Result.Stats.MaximumCoordinateRoundoffBound = std::max(Result.Stats.MaximumCoordinateRoundoffBound, Roundoff);
			AppendHash(Potential->GetDefinitionHash());
			AppendHash(Shape->GetDefinitionHash());
			Result.Continents.push_back({ Center, East, North, Cosine, Tangent, Roundoff, std::move(*Potential), std::move(*Shape) });
		}
		// 中断後に準備結果を公開しない返戻
		if(Stop.stop_requested()) return Fail(SphericalContinentalFieldError::Cancelled);
		// 同じ海域と解析的な陸の座標を持つ比較用の場の返戻
		return Result;
	}

	/**
	 * 面積の異なる大陸と曲がった正の隆起帯を Seed から構成する比較用レシピ
	 * @param Seed 生成定義の固定幅乱数値
	 * @param Radius 惑星の物理半径，１㎞以上
	 * @param Error 準備の失敗理由
	 * @param Stop 準備の中断要求
	 * @return 侵食前の比較用高さ場，自然さと完成水系は別に検査
	 */
	std::optional<SphericalContinentalField> SphericalContinentalField::CreateSeeded(
		const std::uint64_t Seed,
		const double Radius,
		SphericalContinentalFieldError &Error,
		const std::stop_token Stop
	) {
		// 大陸規模の可逆変形と固定の海陸比の準備
		SphericalContinentalFieldSettings Settings;
		Settings.Domain.Warp.Seed = Seed;
		Settings.Domain.Warp.RadiusMeters = Radius;
		Settings.Domain.Warp.LargestWavelengthMeters = 0.5 * std::numbers::pi_v<double> * Radius;
		Settings.Domain.Warp.DerivativeBudget = 8;
		Settings.Domain.Warp.ShouldUseContinentalWavelengths = true;
		// 非有限又は非対応の半径から乱数と幾何を生成しない返戻
		if(!Settings.Domain.Warp.IsValid()) {
			Error = SphericalContinentalFieldError::InvalidSettings;
			// 設定の拒否理由を保持した返戻
			return {};
		}
		// 中断されたレシピの返戻
		if(Stop.stop_requested()) {
			Error = SphericalContinentalFieldError::Cancelled;
			// 完成していない生成定義の返戻
			return {};
		}
		const double Scale = std::min(1.0, Radius / 1000000);
		Settings.PotentialReferenceMeters = 1850 * Scale;
		Settings.BlendTemperatureMeters = 85 * Scale;
		Settings.OceanDepthMeters = std::min(4000.0, 0.04 * Radius);
		// 大陸の向きと面積配分，照会時には使わない乱数の準備
		std::uint64_t State = Seed;
		const double Longitude = 2 * std::numbers::pi_v<double> * RecipeRandom(State), Z = 2 * RecipeRandom(State) - 1;
		const double Ring = std::sqrt(1 - Z * Z), Spin = 2 * std::numbers::pi_v<double> * RecipeRandom(State);
		const Vec3 Pole = UnitDirection::FromVector({ Ring * std::cos(Longitude), Ring * std::sin(Longitude), Z })->Vector();
		const Vec3 Reference = std::abs(Pole.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 };
		const Vec3 First = UnitDirection::FromVector(Reference.Cross(Pole))->Vector();
		const Vec3 East = First * std::cos(Spin) + Pole.Cross(First) * std::sin(Spin), North = Pole.Cross(East);
		const std::array<Vec3, 6> Centers {
			Vec3{ 1, 0, 0 },
			Vec3{ -1, 0, 0 },
			Vec3{ 0, 1, 0 },
			Vec3{ 0, -1, 0 },
			Vec3{ 0, 0, 1 },
			Vec3{ 0, 0, -1 }
		};
		constexpr std::array<double, 6> BaseAreas { 0.09, 0.07, 0.055, 0.045, 0.05, 0.04 };
		std::array<double, 6> Areas {};
		double TotalArea = 0;
		for(std::size_t i = 0; i < Areas.size(); ++i) {
			Areas[i] = BaseAreas[i] * (0.8 + 0.4 * RecipeRandom(State));
			TotalArea += Areas[i];
		}
		for(std::size_t i = 0; i < Centers.size(); ++i) {
			// 離隔の有る大陸円板を Seed の方位へ写像
			const Vec3 Basis = Centers[i], Center = East * Basis.X + North * Basis.Y + Pole * Basis.Z;
			const Vec3 Axis = std::abs(Center.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 0, 1, 0 };
			const Vec3 FirstTangent = UnitDirection::FromVector(Axis.Cross(Center))->Vector();
			const double JitterAngle = 2 * std::numbers::pi_v<double> * RecipeRandom(State), Jitter = 0.045 * RecipeRandom(State);
			const Vec3 Tangent = FirstTangent * std::cos(JitterAngle) + Center.Cross(FirstTangent) * std::sin(JitterAngle);
			const UnitDirection Direction = *UnitDirection::FromVector(Center * std::cos(Jitter) + Tangent * std::sin(Jitter));
			const double Area = 0.35 * Areas[i] / TotalArea, CapRadius = std::acos(1 - 2 * Area);
			Settings.Domain.Continents.push_back({ Direction, CapRadius });
			// 有限密度の隆起を一本の点峰に集中させず，主稜と二本の枝へ分配する
			std::vector<ContinentalPotentialSettings> Layers(2);
			ContinentalDiskWarpSettings Shape;
			Layers[0].BackgroundMeters = 25 * Scale;
			Layers[1].BackgroundMeters = 25 * Scale;
			const double MassifPhase = 2 * std::numbers::pi_v<double> * RecipeRandom(State);
			for(unsigned Massif = 0; Massif < 6; ++Massif) {
				const double Angle = MassifPhase + (Massif + 0.45 * (RecipeRandom(State) - 0.5)) * std::numbers::pi_v<double> / 3;
				const double Distance = 0.28 + 0.43 * RecipeRandom(State), X = Distance * std::cos(Angle), Y = Distance * std::sin(Angle);
				const double Phase = 2 * std::numbers::pi_v<double> * RecipeRandom(State), Cosine = std::cos(Angle), Sine = std::sin(Angle);
				for(unsigned j = 0; j < 9; ++j) {
					const unsigned BranchStep = j < 5 ? 0 : (j - 5) % 2 + 1;
					const double Along = j < 5 ? 0.014 * j : 0.028 + 0.014 * BranchStep, Side = j < 5 ? 0 : j < 7 ? 1 : -1;
					const double Across = 0.004 * std::sin(35 * Along + Phase) + Side * (0.006 + 0.008 * BranchStep);
					const double SourceX = X + Cosine * Along - Sine * Across, SourceY = Y + Sine * Along + Cosine * Across;
					const double Width = 0.009 + 0.006 * RecipeRandom(State), Weight = 35 * Scale * (0.8 + 0.4 * RecipeRandom(State));
					const ContinentalUpliftSource Source { SourceX, SourceY, Width, Weight };
					if(j < 5) {
						Layers[0].Sources.push_back(Source);
						Layers[1].Sources.push_back(Source);
					} else Layers[j < 7 ? 0 : 1].Sources.push_back(Source);
				}
				// 山稜をずらした軸の捩りで曲げ，円板の海岸と隆起の内部極値を保存する
				Shape.Twists.push_back({ X - 0.055 * std::sin(Angle), Y + 0.055 * std::cos(Angle), 0.17, 1.8 + 0.5 * std::sin(Phase) });
			}
			Settings.UpliftLayers.push_back(std::move(Layers));
			Settings.DiskShapes.push_back(std::move(Shape));
		}
		// 同じ領域・源・数値検査を通る比較場の返戻
		return Create(Settings, Error, Stop);
	}

	/**
	 * 高さと排水で同じ円板座標と丸め条件を使う
	 * @param Source 元地形の正規方向
	 * @return 対応する大陸と立体射影の座標，海側又は解像不足の場合は空
	 */
	std::optional<SphericalContinentalField::ContinentalCoordinates> SphericalContinentalField::FindContinentalCoordinates(
		const Vec3 Source
	) const {
		for(std::size_t i = 0; i < Continents.size(); ++i) {
			const PreparedContinent &Continent = Continents[i];
			const double Dot = Continent.Center.Dot(Source);
			if(Dot < Continent.CoastCosine) continue;
			const double Denominator = Continent.HalfAngleTangent * (1 + Dot);
			double X = Continent.East.Dot(Source) / Denominator, Y = Continent.North.Dot(Source) / Denominator;
			const double SquaredRadius = X * X + Y * Y;
			if(SquaredRadius > 1) {
				// 海岸の丸め上界を超える座標の返戻
				if(SquaredRadius - 1 > Continent.CoordinateRoundoffBound) return {};
				const double Scale = 1 / std::sqrt(SquaredRadius);
				X *= Scale;
				Y *= Scale;
				for(unsigned Correction = 0; Correction < 4 && X * X + Y * Y > 1; ++Correction) {
					X = std::nextafter(X, 0.0);
					Y = std::nextafter(Y, 0.0);
				}
				// 有界な補正で閉円板に収まらない座標の返戻
				if(X * X + Y * Y > 1) return {};
			}
			// 高さと排水で共有する正規座標の返戻
			return ContinentalCoordinates{ i, X, Y, Dot, Denominator };
		}
		// 対応する大陸円板が無い座標の返戻
		return {};
	}

	/**
	 * 同じ可逆写像を１回だけ使い，陸の単調変換と海の負の高さを合成する
	 * @param Direction 実惑星の正規方向
	 * @return 高さと必要な勾配，表現範囲外の計算結果は空
	 */
	template<bool HasDerivatives> std::optional<TerrainSample> SphericalContinentalField::EvaluateImpl(
		const UnitDirection Direction
	) const {
		// 共有座標と海陸符号の照会
		UnitDirection Source = Direction;
		std::optional<SphericalWarpMapping> Mapping;
		if constexpr(HasDerivatives) {
			Mapping = Warp->MapWithDerivatives(Direction);
			Source = Mapping->Direction;
		} else Source = Warp->ToSource(Direction);
		const ConnectedOceanDomainSample Coast = Domain->EvaluateInSource(Source);
		const Vec3 Point = Source.Vector();
		const double Sign = Coast.CoastSign;
		TerrainSample Result;
		Vec3 Gradient;
		if(Sign <= 0) {
			// 海岸で２階迄零になる固定海底の比較形
			const double Square = Sign * Sign;
			Result.ElevationMeters = OceanDepthMeters * Sign * Square;
			if constexpr(HasDerivatives) Gradient = Coast.TangentialGradientPerRadian * (3 * OceanDepthMeters * Square);
		} else {
			// 高さと排水で共有した唯一の陸域円板へ立体射影
			const std::optional<ContinentalCoordinates> Coordinates = FindContinentalCoordinates(Point);
			// 領域と同じ大陸を解決出来ない場合の返戻
			if(!Coordinates) return {};
			const PreparedContinent &Continent = Continents[Coordinates->Index];
			const double X = Coordinates->X, Y = Coordinates->Y, Dot = Coordinates->CenterDot, Denominator = Coordinates->Denominator;
			const std::optional<ContinentalDiskWarpMapping> Shape = Continent.Shape.MapToSource(X, Y);
			// 正規円板の内部を同じ隆起円板へ写せない場合の返戻
			if(!Shape) return {};
			// 同じ源の高さと必要な微分の評価
			double PotentialHeight = 0;
			std::optional<ContinentalPotentialSample> Potential;
			if constexpr(HasDerivatives) {
				Potential = Continent.Potential.Evaluate(Shape->Coordinates[0], Shape->Coordinates[1]);
				// 表現出来ない微分付きの源の返戻
				if(!Potential) return {};
				PotentialHeight = Potential->HeightMeters;
			} else {
				const std::optional<double> Height = Continent.Potential.EvaluateHeight(Shape->Coordinates[0], Shape->Coordinates[1]);
				// 表現出来ない高さ照会の返戻
				if(!Height) return {};
				PotentialHeight = *Height;
			}
			// 下降方向を保つ三乗変換と解析連鎖微分
			const double Ratio = PotentialHeight / ReferenceMeters, Square = Ratio * Ratio;
			Result.ElevationMeters = PotentialHeight * Square;
			if constexpr(HasDerivatives) {
				const std::array<double, 2> Pulled = Shape->Pullback(Potential->GradientPerUnit);
				const double GX = Pulled[0], GY = Pulled[1];
				const Vec3 Raw =
				(Continent.East * GX + Continent.North * GY) * (1 / Denominator) - Continent.Center * ((GX * X + GY * Y) / (1 + Dot));
				Gradient = (Raw - Point * Raw.Dot(Point)) * (3 * Square);
			}
		}
		if constexpr(HasDerivatives) Result.TangentialGradientPerRadian = Mapping->PullbackGradient(Gradient);
		const bool IsFinite = std::isfinite(Result.ElevationMeters) && std::isfinite(Result.TangentialGradientPerRadian.Length());
		// 不正な値又は微分を公開しない返戻
		if(!IsFinite) return {};
		// 同じ源の加算順序と物理尺度に依る高さ場の返戻
		return Result;
	}

	/**
	 * 解析的な高さと接線勾配を照会する
	 * @param Direction 実惑星の正規方向
	 * @return メートル単位の高さと１ラジアン当りの勾配，数値失敗時は空
	 */
	std::optional<TerrainSample> SphericalContinentalField::Evaluate(const UnitDirection Direction) const {
		return EvaluateImpl<true>(Direction);
	}

	/**
	 * 微分を計算せず，同じ高さを照会する
	 * @param Direction 実惑星の正規方向
	 * @return 微分付き照会とビット一致する高さ，数値失敗時は空
	 */
	std::optional<double> SphericalContinentalField::EvaluateElevation(const UnitDirection Direction) const {
		const std::optional<TerrainSample> Sample = EvaluateImpl<false>(Direction);
		// 公開出来ない照会結果の返戻
		if(!Sample) return {};
		// 同じ可逆写像と加算順序に依る高さの返戻
		return Sample->ElevationMeters;
	}

	/**
	 * 接続された同じ大洋の内部基準点を取得する
	 * @return 全大陸から正の離隔を持つ実惑星方向
	 */
	UnitDirection SphericalContinentalField::GetOceanAnchor() const {
		return Domain->OceanAnchor();
	}

	/**
	 * 比較構成の面積と数値上界を取得する
	 * @return 不変な構成統計，全点の河川到達の証明ではない
	 */
	const SphericalContinentalFieldStatistics &SphericalContinentalField::GetStatistics() const {
		return Stats;
	}

	/**
	 * 同じ場で用いる基準半径を取得する
	 * @return メートル単位の惑星半径
	 */
	double SphericalContinentalField::GetRadiusMeters() const {
		return RadiusMeters;
	}

	/**
	 * 全準備入力を含む再利用識別子を取得する
	 * @return 領域，有限源，物理尺度と版の識別子
	 */
	std::uint64_t SphericalContinentalField::GetDefinitionHash() const {
		return Hash;
	}
}
