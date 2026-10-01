#include "PlanetSurfaceCorridor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace PlanetCore {
std::optional<SurfaceCorridor> SurfaceCorridor::Create(const Vec3 StartMeters, const Vec3 EndMeters,
													   const double SphereRadiusMeters, const double TubeRadiusMeters) {
	const std::optional<UnitDirection> Start = UnitDirection::FromVector(StartMeters);
	const std::optional<UnitDirection> End = UnitDirection::FromVector(EndMeters);
	if(!Start || !End || !std::isfinite(SphereRadiusMeters) || SphereRadiusMeters < 1000 || !std::isfinite(TubeRadiusMeters) ||
	   TubeRadiusMeters < 0 || TubeRadiusMeters > 256) {
		// 不正な端点又は半径から通路を作らない．
		return {};
	}
	SurfaceCorridor Region;
	Region.StartDirection = Start->Vector();
	Region.EndDirection = End->Vector();
	Region.Span = Region.EndDirection - Region.StartDirection;
	Region.SpanSquared = Region.Span.Dot(Region.Span);
	Region.SphereRadiusMeters = SphereRadiusMeters;
	Region.TubeRadiusMeters = TubeRadiusMeters;
	if(Region.SpanSquared * SphereRadiusMeters * SphereRadiusMeters > 256.0 * 256.0) {
		// 長過ぎる弧は短い走行通路の近似として扱わない．
		return {};
	}
	Region.SagittaMeters = SphereRadiusMeters * (1 - std::sqrt(std::max(0.0, 1 - Region.SpanSquared * 0.25)));
	// 弧の弦への最大偏差を保守的な判定幅へ含める．
	return Region;
}

double SurfaceCorridor::DistanceMeters(const Vec3 Direction) const {
	const double Projection =
		SpanSquared > 0 ? std::clamp((Direction - StartDirection).Dot(Span) / SpanSquared, 0.0, 1.0) : 0.0;
	// 弦上の最近点と球面方向の距離を返す．
	return (Direction - StartDirection - Span * Projection).Length() * SphereRadiusMeters;
}

bool SurfaceCorridor::Contains(const Vec3 Direction, const double SafetyMeters) const {
	const double Tolerance = 64 * std::numeric_limits<double>::epsilon() * SphereRadiusMeters;
	// 弦の内側へ曲がる球面弧のずれを加えた包含判定を返す．
	return std::isfinite(SafetyMeters) && SafetyMeters >= 0 &&
		   DistanceMeters(Direction) + SafetyMeters <= TubeRadiusMeters + SagittaMeters + Tolerance;
}

bool SurfaceCorridor::Intersects(const Vec3 Center, const double CapMeters) const {
	if(!std::isfinite(CapMeters) || CapMeters < 0) {
		// 不正なタイル包囲半径を拒否する．
		return false;
	}
	const double Projection = SpanSquared > 0 ? std::clamp((Center - StartDirection).Dot(Span) / SpanSquared, 0.0, 1.0) : 0.0;
	const Vec3 Offset = Center - StartDirection - Span * Projection;
	const double Tolerance = 64 * std::numeric_limits<double>::epsilon() * SphereRadiusMeters;
	const double Bound = (TubeRadiusMeters + SagittaMeters + CapMeters + Tolerance) / SphereRadiusMeters;
	// 距離の平方で比較し，LOD のタイル毎に平方根を取らない．
	return Offset.Dot(Offset) <= Bound * Bound;
}
} // namespace PlanetCore
