#include "PlanetSeasonalMap.h"
#include "PlanetTreePalette.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace PlanetCore {
UnitDirection PlanetSeasonalMap::PixelDirection(const std::uint32_t X, const std::uint32_t Y) {
	const std::uint32_t Row = std::min(Y, Height - 1);
	// 全経度が同じ極点へ収束する返戻
	if(!Row || Row == Height - 1) return *UnitDirection::FromVector({0, 0, Row ? -1.0 : 1.0});
	const double Latitude = std::numbers::pi * Row / (Height - 1);
	const double Longitude = 2 * std::numbers::pi * ((X % Width + 0.5) / Width - 0.5);
	const double Radial = std::sin(Latitude);
	// テクセル中心の単位方向の返戻
	return *UnitDirection::FromVector({Radial * std::cos(Longitude), Radial * std::sin(Longitude), std::cos(Latitude)});
}

std::optional<PlanetSeasonalMap> PlanetSeasonalMap::Create(const PlanetClimate &Climate, const std::stop_token Stop) {
	PlanetSeasonalMap Result;
	Result.Pixels.resize(Width * Height * Layers);
	for(std::uint32_t y = 0; y < Height; ++y)
		for(std::uint32_t x = 0; x < Width; ++x) {
			// 中断した配列を公開しない返戻
			if(Stop.stop_requested()) return {};
			const std::optional<BiomeProfile> Profile = Climate.SampleBiome(PixelDirection(x, y));
			// 有効なバイオーム応答が無い場合の返戻
			if(!Profile) return {};
			const std::optional<BiomeHabitat> Habitat = Profile->Habitat({});
			// 陸上の生育応答が無い場合の返戻
			if(!Habitat) return {};
			const double Desert = Habitat->Weights[static_cast<unsigned>(LandBiome::HotDesert)] +
								  Habitat->Weights[static_cast<unsigned>(LandBiome::ColdDesert)];
			for(std::uint32_t Period = 0; Period < ClimatePeriodCount; ++Period) {
				const std::optional<SeasonalAppearance> Appearance =
					Profile->Appearance(static_cast<double>(Period) / ClimatePeriodCount, {});
				// 季節応答が無い場合の返戻
				if(!Appearance) return {};
				const std::uint32_t Index = (Period * Height + y) * Width + x;
				Result.Pixels[Index] = {static_cast<float>(Appearance->GreenGrass), static_cast<float>(Appearance->DryGrass),
										static_cast<float>(Appearance->LeafLitter), static_cast<float>(Appearance->Snow)};
				Result.Pixels[Index + ClimatePeriodCount * Height * Width] = {
					static_cast<float>(Appearance->Ice), static_cast<float>(Appearance->Wetness), static_cast<float>(Desert),
					static_cast<float>(Habitat->TreeSuitability)};
			}
			Result.Pixels[(ConiferLayer * Height + y) * Width + x] = {static_cast<float>(ConiferFraction(*Habitat)), 0, 0, 0};
		}
	// 準備済みの固定寸法配列の返戻
	return Result;
}

std::span<const std::array<float, 4>> PlanetSeasonalMap::Data() const {
	return Pixels;
}

std::array<double, 4> PlanetSeasonalMap::SampleLayer(const UnitDirection Direction, const std::uint32_t Layer) const {
	const Vec3 Point = Direction.Vector();
	const double U = std::atan2(Point.Y, Point.X) / (2 * std::numbers::pi) + 0.5;
	const double V = std::acos(std::clamp(Point.Z, -1.0, 1.0)) / std::numbers::pi;
	const double X = U * Width - 0.5, Y = V * (Height - 1);
	const double BlendX = X - std::floor(X), BlendY = Y - std::floor(Y);
	const std::int32_t Left = static_cast<std::int32_t>(std::floor(X));
	const std::uint32_t Top = static_cast<std::uint32_t>(std::floor(Y));
	std::array<double, 4> Result{};
	for(std::uint32_t y = 0; y < 2; ++y)
		for(std::uint32_t x = 0; x < 2; ++x) {
			const std::uint32_t Column = static_cast<std::uint32_t>((Left + static_cast<std::int32_t>(Width + x)) % Width);
			const std::uint32_t Row = std::min(Top + y, Height - 1);
			const double Weight = (x ? BlendX : 1 - BlendX) * (y ? BlendY : 1 - BlendY);
			const std::array<float, 4> &Pixel = Pixels[(Layer * Height + Row) * Width + Column];
			for(std::uint32_t Channel = 0; Channel < Result.size(); ++Channel) Result[Channel] += Weight * Pixel[Channel];
		}
	// 経度境界と極点を含む双一次補間の返戻
	return Result;
}

double PlanetSeasonalMap::SampleConiferFraction(const UnitDirection Direction) const {
	return SampleLayer(Direction, ConiferLayer)[0];
}

std::optional<std::array<double, 8>> PlanetSeasonalMap::Sample(const UnitDirection Direction,
															   const double SolarYearPhase) const {
	// 非有限な年周位相を拒否する返戻
	if(!std::isfinite(SolarYearPhase)) return {};
	const double Period = (SolarYearPhase - std::floor(SolarYearPhase)) * ClimatePeriodCount;
	const std::uint32_t First = static_cast<std::uint32_t>(Period) % ClimatePeriodCount;
	const std::uint32_t Second = (First + 1) % ClimatePeriodCount;
	const double Fraction = Period - std::floor(Period);
	const double Blend = Fraction * Fraction * Fraction * (10 + Fraction * (-15 + 6 * Fraction));
	std::array<double, 8> Result{};
	for(std::uint32_t Group = 0; Group < 2; ++Group) {
		const std::array<double, 4> Begin = SampleLayer(Direction, First + Group * ClimatePeriodCount);
		const std::array<double, 4> End = SampleLayer(Direction, Second + Group * ClimatePeriodCount);
		for(std::uint32_t Channel = 0; Channel < 4; ++Channel) {
			Result[(Group << 2) + Channel] = Begin[Channel] + (End[Channel] - Begin[Channel]) * Blend;
		}
	}
	// 年境界を跨ぐ季節補間の返戻
	return Result;
}
} // namespace PlanetCore
