#include "PlanetStreamingFocus.h"
#include <algorithm>
#include <cmath>

namespace PlanetCore {
	std::optional<StreamingFocus> PredictStreamingFocus(const Vec3 PositionMeters, const Vec3 VelocityMeters,
		const Vec3 ForwardMeters, const double PlanetRadiusMeters, const double HorizonSeconds,
		const double AllowedSpeedMetersPerSecond) {
		const std::optional<UnitDirection> Up = UnitDirection::FromVector(PositionMeters);
		const double Distance = PositionMeters.Length();
		if(!Up || !std::isfinite(Distance) || !std::isfinite(PlanetRadiusMeters) || PlanetRadiusMeters < 1000 ||
			!std::isfinite(HorizonSeconds) || HorizonSeconds < 0 ||
			!std::isfinite(AllowedSpeedMetersPerSecond) || AllowedSpeedMetersPerSecond < 0 ||
			AllowedSpeedMetersPerSecond > 20 ||
			!std::isfinite(VelocityMeters.X) || !std::isfinite(VelocityMeters.Y) || !std::isfinite(VelocityMeters.Z) ||
			!std::isfinite(ForwardMeters.X) || !std::isfinite(ForwardMeters.Y) || !std::isfinite(ForwardMeters.Z)) {
			// 不正な半径・速度・時間・方向を受理しない．
			return {};
		}
		const Vec3 Tangent = VelocityMeters - Up->Vector() * VelocityMeters.Dot(Up->Vector());
		const Vec3 Forward = ForwardMeters - Up->Vector() * ForwardMeters.Dot(Up->Vector());
		const double Speed = Tangent.Length();
		const double ForwardLength = Forward.Length();
		if(!std::isfinite(Speed) || !std::isfinite(ForwardLength)) {
			// 接線成分が有限でなければ進行方向を作れない．
			return {};
		}
		const double Lead = std::min(StreamingFocus::MaximumLeadMeters,
			std::max(Speed, AllowedSpeedMetersPerSecond) * std::min(HorizonSeconds, 10.0));
		if(Lead == 0 || (Speed <= 0.25 && ForwardLength == 0)) {
			// 停止して方向も無い時は身体中心の近景円を残す．
			return StreamingFocus{PositionMeters, PositionMeters};
		}
		const Vec3 Heading = Speed > 0.25 ? Tangent * (1 / Speed) : Forward * (1 / ForwardLength);
		const std::optional<UnitDirection> End = UnitDirection::FromVector(Up->Vector() + Heading * (Lead / PlanetRadiusMeters));
		if(!End) {
			// 進行方向を球面へ射影出来ない場合は要求を拒否する．
			return {};
		}
		// 高度を保つ両端と半径24 mの通路を返し，広い外接円は作らない．
		return StreamingFocus{PositionMeters, End->Vector() * Distance};
	}
}
