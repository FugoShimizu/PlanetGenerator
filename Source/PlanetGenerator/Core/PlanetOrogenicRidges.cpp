#include "PlanetOrogenicRidges.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		constexpr double BendFraction = 0.18;

		/**
		 * 特徴番号と区間番号を独立した固定幅の乱数へ写す
		 * @param Value 入力ビット
		 * @return 混合したビット
		 */
		std::uint64_t MixBits(std::uint64_t Value) {
			Value ^= Value >> 30;
			Value *= UINT64_C(0XBF58476D1CE4E5B9);
			Value ^= Value >> 27;
			Value *= UINT64_C(0X94D049BB133111EB);
			// 固定幅の混合結果の返戻
			return Value ^ Value >> 31;
		}

		/**
		 * 空間索引の分割に使うベクトル成分を取得する
		 * @param Value 対象の座標
		 * @param Axis 軸番号
		 * @return 指定した成分
		 */
		double Coordinate(const Vec3 Value, const unsigned Axis) {
			// 軸番号に対応した成分の返戻
			return !Axis ? Value.X : Axis == 1 ? Value.Y : Value.Z;
		}

		/**
		 * 各成分の小さい方を取得する
		 * @param A 第１の境界
		 * @param B 第２の境界
		 * @return 成分毎の下限
		 */
		Vec3 Low(const Vec3 A, const Vec3 B) {
			// 成分毎の下限の返戻
			return { std::min(A.X, B.X), std::min(A.Y, B.Y), std::min(A.Z, B.Z) };
		}

		/**
		 * 各成分の大きい方を取得する
		 * @param A 第１の境界
		 * @param B 第２の境界
		 * @return 成分毎の上限
		 */
		Vec3 High(const Vec3 A, const Vec3 B) {
			// 成分毎の上限の返戻
			return { std::max(A.X, B.X), std::max(A.Y, B.Y), std::max(A.Z, B.Z) };
		}

		/**
		 * 閉じた軸平行境界に方向が含まれるか調べる
		 * @param Point 球面方向
		 * @param Min 各軸の下限
		 * @param Max 各軸の上限
		 * @return 全ての座標が境界内である場合に真
		 */
		bool Contains(const Vec3 Point, const Vec3 Min, const Vec3 Max) {
			// 境界への接触も含めた判定の返戻
			return Point.X >= Min.X && Point.Y >= Min.Y && Point.Z >= Min.Z && Point.X <= Max.X && Point.Y <= Max.Y && Point.Z <= Max.Z;
		}
	} // namespace

	/**
	 * 地質の衝突帯と火山弧を有限な分岐山稜へ写す
	 * @param Source 同じ Seed・半径で準備した地質場
	 * @param Stop 中断要求
	 * @return 表示タイルに依存しない不変な試作場
	 */
	std::optional<PlanetOrogenicRidges> PlanetOrogenicRidges::Create(const GeologyField &Source, const std::stop_token Stop) {
		// 中断済みの準備要求を拒否する返戻
		if(Stop.stop_requested()) return {};
		PlanetOrogenicRidges Result;
		const GeologySettings &Settings = Source.Settings();
		const double Radius = Settings.RadiusMeters;
		Result.SourceDefinitionHash = Settings.IdentityHash();
		const std::span<const GeologicFeature> Features = Source.Features();
		// 特徴番号が固定幅へ収まらない場合の返戻
		if(Features.size() > std::numeric_limits<std::uint32_t>::max()) return {};
		for(std::uint32_t FeatureIndex = 0; FeatureIndex < Features.size(); ++FeatureIndex) {
			// 特徴列挙中に中断された場を公開しない返戻
			if(!(FeatureIndex & 63) && Stop.stop_requested()) return {};
			const GeologicFeature &Feature = Features[FeatureIndex];
			if(
				Feature.Kind != GeologicFeatureKind::Collision && Feature.Kind != GeologicFeatureKind::VolcanicArc ||
				Feature.AmplitudeMeters <= 0
			) continue;
			const double SourceLength = std::asin(std::clamp(Feature.AlongSine, 0.0, 1.0));
			const double SourceWidth = std::asin(std::clamp(Feature.AcrossSine, 0.0, 1.0));
			if(!std::isfinite(SourceLength) || !std::isfinite(SourceWidth) || SourceLength <= 0 || SourceWidth <= 0) continue;
			const double TargetLengthMeters = std::max(90000.0, Radius * SourceLength / 16);
			const std::uint32_t Sections =
			static_cast<std::uint32_t>(std::clamp(std::ceil(Radius * SourceLength / TargetLengthMeters), 1.0, 16.0));
			const double LengthAngle = std::min(0.22, 0.95 * SourceLength / Sections + 0.012);
			const double WidthAngle = std::min({ 0.15, 0.36 * SourceWidth, 0.12 * TargetLengthMeters / Radius });
			if(WidthAngle <= 0 || LengthAngle <= WidthAngle) continue;
			++Result.Stats.SourceFeatures;
			for(std::uint32_t Section = 0; Section < Sections; ++Section) {
				const double Position = 1.6 * (Section + 0.5) / Sections - 0.8, Angle = SourceLength * Position;
				OrogenicRidgeDescriptor Ridge;
				Ridge.Center = Feature.Center * std::cos(Angle) + Feature.Along * std::sin(Angle);
				Ridge.Along = Feature.Along * std::cos(Angle) - Feature.Center * std::sin(Angle);
				Ridge.Across = Feature.Across;
				Ridge.AlongSine = std::sin(LengthAngle);
				Ridge.AcrossSine = std::sin(WidthAngle);
				Ridge.SourceFeature = FeatureIndex;
				const double RadiusScale = Radius / (Radius + 250000);
				Ridge.AmplitudeMeters = std::min(450.0, Feature.AmplitudeMeters * 0.075) * RadiusScale * (1 - 0.65 * Position * Position);
				const std::uint64_t Bits = MixBits(
					Settings.Seed ^ static_cast<std::uint64_t>(FeatureIndex) * UINT64_C(0X9E3779B97F4A7C15) ^
					static_cast<std::uint64_t>(Section) * UINT64_C(0XBF58476D1CE4E5B9)
				);
				Ridge.BendPhase = 2 * std::numbers::pi_v<double> * static_cast<double>(Bits >> 11) * (1.0 / 9007199254740992.0);
				const double PerpendicularSquared = Ridge.AlongSine * Ridge.AlongSine + std::pow((1 + BendFraction) * Ridge.AcrossSine, 2);
				// 支持円が対蹠半球へ達する不正な形状の返戻
				if(PerpendicularSquared >= 1) return {};
				Ridge.SupportCosine = std::sqrt(1 - PerpendicularSquared);
				const double ChordBound = std::sqrt(2 * (1 - Ridge.SupportCosine)) + 1E-12;
				const double Bound = std::nextafter(ChordBound, std::numeric_limits<double>::infinity());
				Ridge.BoundsMin = Ridge.Center - Vec3{ Bound, Bound, Bound };
				Ridge.BoundsMax = Ridge.Center + Vec3{ Bound, Bound, Bound };
				const double DescriptorAmplitude = Ridge.AmplitudeMeters;
				double &MaximumDescriptorAmplitude = Result.Stats.MaximumDescriptorAmplitudeMeters;
				MaximumDescriptorAmplitude = std::max(MaximumDescriptorAmplitude, DescriptorAmplitude);
				// 固定幅の索引を超える地質履歴の返戻
				if(Result.Ridges.size() >= std::numeric_limits<std::uint32_t>::max()) return {};
				Result.Ridges.push_back(Ridge);
			}
		}
		// 完成前に中断された場の返戻
		if(Stop.stop_requested()) return {};
		Result.Stats.Descriptors = static_cast<std::uint32_t>(Result.Ridges.size());
		Result.BuildIndex();
		Result.Stats.IndexNodes = static_cast<std::uint32_t>(Result.Index.size());
		// 地質方向に固定した有限支持の山稜場の返戻
		return Result;
	}

	/**
	 * 有限支持の山稜境界を空間索引へ登録する
	 */
	void PlanetOrogenicRidges::BuildIndex() {
		if(Ridges.empty()) return;
		Order.resize(Ridges.size());
		std::iota(Order.begin(), Order.end(), 0U);
		Index.reserve(Ridges.size() * 2);
		const auto Build = [&](auto &&Self, const std::uint32_t Begin, const std::uint32_t End) -> std::uint32_t {
			IndexNode Node;
			Node.Min = Ridges[Order[Begin]].BoundsMin;
			Node.Max = Ridges[Order[Begin]].BoundsMax;
			for(std::uint32_t i = Begin + 1; i < End; ++i) {
				Node.Min = Low(Node.Min, Ridges[Order[i]].BoundsMin);
				Node.Max = High(Node.Max, Ridges[Order[i]].BoundsMax);
			}
			const std::uint32_t NodeIndex = static_cast<std::uint32_t>(Index.size());
			Index.push_back(Node);
			if(End - Begin < 5) {
				Index[NodeIndex].Begin = Begin;
				Index[NodeIndex].Count = End - Begin;
				// 構築した葉の番号の返戻
				return NodeIndex;
			}
			const Vec3 Extent = Node.Max - Node.Min;
			const unsigned SplitAxis = Extent.X >= Extent.Y && Extent.X >= Extent.Z ? 0 : Extent.Y >= Extent.Z ? 1 : 2;
			const std::uint32_t Middle = Begin + (End - Begin) / 2;
			std::nth_element(
				Order.begin() + Begin,
				Order.begin() + Middle,
				Order.begin() + End,
				[&](const std::uint32_t Left, const std::uint32_t Right) {
					const double X = Coordinate(Ridges[Left].Center, SplitAxis), Y = Coordinate(Ridges[Right].Center, SplitAxis);
					// 同じ中心では特徴番号で順序を確定する返戻
					return X != Y ? X < Y : Left < Right;
				}
			);
			const std::uint32_t Left = Self(Self, Begin, Middle), Right = Self(Self, Middle, End);
			Index[NodeIndex].Left = Left;
			Index[NodeIndex].Right = Right;
			// 構築した内部節点の番号の返戻
			return NodeIndex;
		};
		Build(Build, 0, static_cast<std::uint32_t>(Order.size()));
		return;
	}

	/**
	 * 山稜と接線勾配を同じ式から評価する
	 * @tparam Derivatives 接線勾配が必要な場合に真
	 * @param Direction 惑星中心からの単位方向
	 * @param Indexed 空間索引を使用する場合に真
	 * @param Candidates 任意の候補数の出力先
	 * @return 有限支持の山稜標高と指定時の勾配
	 */
	template<bool Derivatives> TerrainSample PlanetOrogenicRidges::EvaluateImpl(
		const UnitDirection Direction,
		const bool Indexed,
		std::size_t *const Candidates
	) const {
		const Vec3 Point = Direction.Vector();
		std::array<std::uint32_t, 64> Small {};
		std::vector<std::uint32_t> Large;
		std::size_t Size = 0;
		const auto Add = [&](const std::uint32_t Candidate) {
			if(Size < Small.size()) Small[Size] = Candidate;
			else {
				if(Large.empty()) Large.assign(Small.begin(), Small.end());
				Large.push_back(Candidate);
			}
			++Size;
		};
		if(Indexed && !Index.empty()) {
			std::array<std::uint32_t, 64> Stack {};
			unsigned Count = 1;
			while(Count) {
				const IndexNode &Node = Index[Stack[--Count]];
				if(!Contains(Point, Node.Min, Node.Max)) continue;
				if(!Node.Count) {
					Stack[Count++] = Node.Right;
					Stack[Count++] = Node.Left;
					continue;
				}
				for(std::uint32_t i = Node.Begin; i < Node.Begin + Node.Count; ++i) {
					const std::uint32_t Candidate = Order[i];
					if(Contains(Point, Ridges[Candidate].BoundsMin, Ridges[Candidate].BoundsMax)) Add(Candidate);
				}
			}
		} else if(!Indexed) for(std::uint32_t i = 0; i < Ridges.size(); ++i) Add(i);
		std::uint32_t *const Identifiers = Large.empty() ? Small.data() : Large.data();
		std::sort(Identifiers, Identifiers + Size);
		if(Candidates) *Candidates = Size;
		TerrainSample Result;
		for(std::size_t i = 0; i < Size; ++i) {
			const OrogenicRidgeDescriptor &Ridge = Ridges[Identifiers[i]];
			if(Point.Dot(Ridge.Center) <= Ridge.SupportCosine) continue;
			const double U = Ridge.Along.Dot(Point) / Ridge.AlongSine;
			if(std::abs(U) >= 1) continue;
			const double Bend = BendFraction * std::sin(std::numbers::pi_v<double> * U + Ridge.BendPhase);
			const double V = Ridge.Across.Dot(Point) / Ridge.AcrossSine - Bend;
			if(std::abs(V) >= 1) continue;
			const double LongSupport = 1 - U * U, CrossSupport = 1 - V * V, AlongWeight = LongSupport * LongSupport * LongSupport;
			const double AcrossWeight = CrossSupport * CrossSupport * CrossSupport;
			Result.ElevationMeters += Ridge.AmplitudeMeters * AlongWeight * AcrossWeight;
			if constexpr(Derivatives) {
				const double AlongDerivative = -6 * U * LongSupport * LongSupport, AcrossDerivative = -6 * V * CrossSupport * CrossSupport;
				const double BendDerivative =
				BendFraction * std::numbers::pi_v<double> * std::cos(std::numbers::pi_v<double> * U + Ridge.BendPhase);
				const double AlongChange = AlongDerivative * AcrossWeight - AlongWeight * AcrossDerivative * BendDerivative;
				const double Du = Ridge.AmplitudeMeters * AlongChange, Dv = Ridge.AmplitudeMeters * AlongWeight * AcrossDerivative;
				Result.TangentialGradientPerRadian =
				Result.TangentialGradientPerRadian + Ridge.Along * (Du / Ridge.AlongSine) + Ridge.Across * (Dv / Ridge.AcrossSine);
			}
		}
		if constexpr(Derivatives) {
			const Vec3 Gradient = Result.TangentialGradientPerRadian;
			Result.TangentialGradientPerRadian = Gradient - Point * Gradient.Dot(Point);
		}
		// 同じ順序の標高と任意の解析勾配の返戻
		return Result;
	}

	/**
	 * 山稜の標高と解析接線勾配を評価する
	 * @param Direction 球面上の単位方向
	 * @return 地質方向に沿う山稜標高と勾配
	 */
	TerrainSample PlanetOrogenicRidges::Evaluate(const UnitDirection Direction) const {
		// 索引付き地点評価の返戻
		return EvaluateImpl<true>(Direction, true);
	}

	/**
	 * 勾配計算を省いて山稜標高を取得する
	 * @param Direction 球面上の単位方向
	 * @return 地質方向に沿う山稜標高
	 */
	double PlanetOrogenicRidges::EvaluateElevation(const UnitDirection Direction) const {
		// 値だけを計算した標高の返戻
		return EvaluateImpl<false>(Direction, true).ElevationMeters;
	}

	/**
	 * 全記述子を調べた参照値を取得する
	 * @param Direction 球面上の単位方向
	 * @return 索引を使わない標高と勾配
	 */
	TerrainSample PlanetOrogenicRidges::EvaluateReference(const UnitDirection Direction) const {
		// 独立した候補列挙による参照値の返戻
		return EvaluateImpl<true>(Direction, false);
	}

	/**
	 * 軸平行包囲箱で絞った山稜数を取得する
	 * @param Direction 球面上の単位方向
	 * @return 精密な支持判定前の候補数
	 */
	std::size_t PlanetOrogenicRidges::CandidateCount(const UnitDirection Direction) const {
		std::size_t Count = 0;
		EvaluateImpl<false>(Direction, true, &Count);
		// 索引が列挙した候補数の返戻
		return Count;
	}

	/**
	 * 元の特徴順で準備した山稜を参照する
	 * @return 不変な山稜記述子
	 */
	std::span<const OrogenicRidgeDescriptor> PlanetOrogenicRidges::Descriptors() const {
		// 準備済み記述子の返戻
		return Ridges;
	}

	/**
	 * 記述子と索引の準備統計を参照する
	 * @return 準備統計
	 */
	const OrogenicRidgeStatistics &PlanetOrogenicRidges::Statistics() const {
		// 準備統計の返戻
		return Stats;
	}

	/**
	 * 元の地質定義の識別子を取得する
	 * @return Seed・半径・版の定義ハッシュ
	 */
	std::uint64_t PlanetOrogenicRidges::DefinitionHash() const {
		// 元の定義ハッシュの返戻
		return SourceDefinitionHash;
	}
} // namespace PlanetCore
