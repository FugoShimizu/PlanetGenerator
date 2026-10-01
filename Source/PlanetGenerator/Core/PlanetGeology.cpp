#include "PlanetGeology.h"
#include "PlanetJet.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <numeric>

namespace PlanetCore {
	namespace {
		constexpr double Pi = 3.1415926535897932384626433832795;
		constexpr double EarthRadius = 6371000.0;

		template <bool Derivatives> TerrainJet<Derivatives> Smooth(TerrainJet<Derivatives> X, double A, double B) {
			using Jet = TerrainJet<Derivatives>;
			const auto Constant = [](double V) { return Jet{V, {}}; };
			if(X.V <= A) return Constant(0);
			if(X.V >= B) return Constant(1);
			const Jet T = (X - Constant(A)) * (1 / (B - A));
			return T * T * T * (T * (T * 6 - Constant(15)) + Constant(10));
		}
		template <bool Derivatives> TerrainJet<Derivatives> Sqrt(TerrainJet<Derivatives> X) {
			const double S = std::sqrt(X.V);
			if constexpr(Derivatives) return {S, X.G * (0.5 / S)};
			else return {S, {}};
		}
		template <bool Derivatives> TerrainJet<Derivatives> Exp(TerrainJet<Derivatives> X) {
			const double E = std::exp(X.V);
			if constexpr(Derivatives) return {E, X.G * E};
			else return {E, {}};
		}

		struct Random {
			std::uint64_t State;
			double Next() {
				auto X = (State += UINT64_C(0x9e3779b97f4a7c15));
				X = (X ^ (X >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
				X = (X ^ (X >> 27)) * UINT64_C(0x94d049bb133111eb);
				X ^= X >> 31;
				return static_cast<double>(X >> 11) * 0x1.0p-53;
			}
			Vec3 Direction() {
				const double Z = 2 * Next() - 1, Phi = 2 * Pi * Next();
				const double R = std::sqrt(std::max(0.0, 1 - Z * Z));
				return {R * std::cos(Phi), R * std::sin(Phi), Z};
			}
		};
		Vec3 Unit(Vec3 V) {
			return UnitDirection::FromVector(V)->Vector();
		}
		Vec3 Rotate(Vec3 V, Vec3 Axis, double Angle) {
			const double C = std::cos(Angle), S = std::sin(Angle);
			return V * C + Axis.Cross(V) * S + Axis * (Axis.Dot(V) * (1 - C));
		}
		Vec3 ProbeDirection(std::uint32_t I, std::uint32_t Count, Vec3 Axis, double Angle) {
			const double Z = 1 - 2 * (static_cast<double>(I) + 0.5) / Count;
			const double Phi = static_cast<double>(I) * Pi * (3 - std::sqrt(5.0));
			const double R = std::sqrt(1 - Z * Z);
			return Unit(Rotate({R * std::cos(Phi), R * std::sin(Phi), Z}, Axis, Angle));
		}
		GeologicFeature Lobe(GeologicFeatureKind Kind, Vec3 Center, Vec3 Along, double Length, double Width, double Amplitude) {
			Center = Unit(Center);
			Along = Unit(Along - Center * Along.Dot(Center));
			GeologicFeature F;
			F.Kind = Kind;
			F.Center = Center;
			F.Along = Along;
			F.Across = Unit(Center.Cross(Along));
			F.AlongSine = std::sin(std::clamp(Length, 0.0001, 1.25));
			F.AcrossSine = std::sin(std::clamp(Width, 0.0001, 1.25));
			const double S = std::max(F.AlongSine, F.AcrossSine);
			F.SupportCosine = std::sqrt(1 - S * S);
			F.AmplitudeMeters = Amplitude;
			return F;
		}
		template <bool Derivatives> TerrainJet<Derivatives> Kernel(const GeologicFeature &F, Vec3 D) {
			using Jet = TerrainJet<Derivatives>;
			const auto Constant = [](double V) { return Jet{V, {}}; };
			// The cap rejects the antipodal copy of an orthographic ellipse. This
			// rejection lies outside the compact kernel, where value and derivatives
			// already vanish; it cannot introduce a discontinuous hemisphere seam.
			if(F.Center.Dot(D) <= F.SupportCosine) return Constant(0);
			const double U = F.Along.Dot(D) / F.AlongSine, V = F.Across.Dot(D) / F.AcrossSine;
			const double Q2 = U * U + V * V;
			if(Q2 >= 1) return Constant(0);
			// Wendland phi(3,1), (1-q)^4_+ (1+4q). See implementation notes.
			const double Q = std::sqrt(Q2), S = 1 - Q;
			Vec3 MetricGradient;
			if constexpr(Derivatives) MetricGradient = F.Along * (U / F.AlongSine) + F.Across * (V / F.AcrossSine);
			// The q factor cancels analytically; the center has no divide-by-zero.
			if constexpr(Derivatives) return {S * S * S * S * (1 + 4 * Q), MetricGradient * (-20 * S * S * S)};
			else return {S * S * S * S * (1 + 4 * Q), {}};
		}
		template <bool Derivatives>
		TerrainJet<Derivatives> Hermite(TerrainJet<Derivatives> X, double A, double B, TerrainJet<Derivatives> YA,
		                                TerrainJet<Derivatives> YB, double DA, double DB) {
			using Jet = TerrainJet<Derivatives>;
			const auto Constant = [](double V) { return Jet{V, {}}; };
			const Jet T = (X - Constant(A)) * (1 / (B - A));
			const Jet Delta = YB - YA;
			const double MA = DA * (B - A), MB = DB * (B - A);
			const Jet C3 = Delta * 10 - Constant(6 * MA + 4 * MB);
			const Jet C4 = Delta * -15 + Constant(8 * MA + 7 * MB);
			const Jet C5 = Delta * 6 - Constant(3 * MA + 3 * MB);
			return YA + T * MA + T * T * T * (C3 + T * (C4 + T * C5));
		}
		template <bool Derivatives>
		TerrainJet<Derivatives> ShelfProfile(TerrainJet<Derivatives> Coordinate, TerrainJet<Derivatives> DeepOcean) {
			using Jet = TerrainJet<Derivatives>;
			const auto Constant = [](double V) { return Jet{V, {}}; };
			if(Coordinate.V <= -0.30) return DeepOcean;
			if(Coordinate.V < -0.07) return Hermite(Coordinate, -0.30, -0.07, DeepOcean, Constant(-180), 0, 1500);
			if(Coordinate.V < 0) return Hermite(Coordinate, -0.07, 0, Constant(-180), Constant(0), 1500, 4500);
			if(Coordinate.V < 0.08) return Hermite(Coordinate, 0, 0.08, Constant(0), Constant(320), 4500, 2500);
			if(Coordinate.V < 0.35) return Hermite(Coordinate, 0.08, 0.35, Constant(320), Constant(1100), 2500, 0);
			return Constant(1100);
		}
		struct Plate {
			Vec3 Initial, Axis;
			double Rate = 0, Age = 0;
		};
	} // namespace

	bool GeologySettings::IsValid() const {
		return Version == GeologyGeneratorVersion && std::isfinite(RadiusMeters) && RadiusMeters >= 1000 &&
		       RadiusMeters <= 1e9 && std::isfinite(SeaLevelMeters) && std::abs(SeaLevelMeters) <= 0.05 * RadiusMeters &&
		       PlateCount >= 8 && PlateCount <= 32 && HistoryIntervals >= 1 && HistoryIntervals <= 16 &&
		       std::isfinite(HistoryMillionYears) && HistoryMillionYears >= 1 && HistoryMillionYears <= 200;
	}
	std::uint64_t GeologySettings::IdentityHash() const {
		std::uint64_t H = UINT64_C(14695981039346656037);
		const auto Add = [&H](std::uint64_t V, unsigned N) {
			for(unsigned I = 0; I < N; ++I) {
				H ^= (V >> (8 * I)) & 255;
				H *= UINT64_C(1099511628211);
			}
		};
		const auto Bits = [](double V) { return std::bit_cast<std::uint64_t>(V == 0 ? 0.0 : V); };
		Add(UINT64_C(0x47454f4c), 4);
		Add(Version, 4);
		Add(Seed, 8);
		Add(Bits(RadiusMeters), 8);
		Add(Bits(SeaLevelMeters), 8);
		Add(PlateCount, 4);
		Add(HistoryIntervals, 4);
		Add(Bits(HistoryMillionYears), 8);
		return H;
	}

	template <bool Derivatives> TerrainSample GeologyField::ContinentalScoreImpl(Vec3 D) const {
		using Jet = TerrainJet<Derivatives>;
		Jet Score;
		for(const auto &F : Crust)
			Score = Score + Kernel<Derivatives>(F, D) * F.AmplitudeMeters;
		for(unsigned I = 0; I < 6; ++I) {
			const auto &W = Waves[I];
			const double P = W.Frequency * W.Axis.Dot(D) + W.Phase;
			constexpr std::array<double, 6> Amplitudes{0.07, 0.035, 0.0175, 0.009, 0.0045, 0.00225};
			Vec3 Gradient;
			if constexpr(Derivatives) Gradient = W.Axis * (W.Frequency * std::cos(P) * Amplitudes[I]);
			Score = Score + Jet{std::sin(P) * Amplitudes[I], Gradient};
		}
		if constexpr(Derivatives) return {Score.V, Score.G - D * Score.G.Dot(D)};
		else return {Score.V, {}};
	}

	TerrainSample GeologyField::ContinentalScore(Vec3 D) const {
		return ContinentalScoreImpl<true>(D);
	}

	std::optional<GeologyField> GeologyField::Create(const GeologySettings &S, std::stop_token Stop) {
		if(!S.IsValid() || Stop.stop_requested()) return {};
		GeologyField Field;
		Field.Parameters = S;
		Random R{S.Seed};
		// A uniform axis with a uniform angle is not uniform in SO(3).
		// Uniform S3 quaternion (LaValle, Planning Algorithms, eq. 5.15).
		const double U1 = R.Next(), U2 = R.Next(), U3 = R.Next();
		const double QuaternionRadiusA = std::sqrt(1 - U1), QuaternionRadiusB = std::sqrt(U1);
		const Vec3 Imaginary{QuaternionRadiusA * std::sin(2 * Pi * U2), QuaternionRadiusA * std::cos(2 * Pi * U2),
		                     QuaternionRadiusB * std::sin(2 * Pi * U3)};
		const double Real = QuaternionRadiusB * std::cos(2 * Pi * U3);
		const auto QuaternionAxis = UnitDirection::FromVector(Imaginary);
		const Vec3 RotationAxis = QuaternionAxis ? QuaternionAxis->Vector() : Vec3{0, 0, 1};
		const double RotationAngle = 2 * std::atan2(Imaginary.Length(), Real);
		std::vector<Plate> Plates;
		for(std::uint32_t I = 0; I < S.PlateCount; ++I)
			Plates.push_back({Unit(ProbeDirection(I, S.PlateCount, RotationAxis, RotationAngle) + R.Direction() * 0.16),
			                  R.Direction(), 0.0003 + 0.0012 * R.Next(), 30 + 150 * R.Next()});
		constexpr std::array<double, 6> CoastFrequencies{3, 7, 15, 31, 63, 127};
		for(unsigned I = 0; I < Field.Waves.size(); ++I)
			Field.Waves[I] = {R.Direction(), I < 6 ? CoastFrequencies[I] : 13.0 + (I - 6) * 11, R.Next() * 2 * Pi};

		// Cratons are material markers with their own outlines, not Voronoi cells.
		const unsigned Cratons = 4 + static_cast<unsigned>(R.Next() * 3);
		std::vector<GeologicFeature> InitialCrust;
		for(unsigned I = 0; I < Cratons; ++I) {
			const auto Owner = static_cast<std::uint32_t>((I * S.PlateCount) / Cratons);
			const auto &P = Plates[Owner];
			Vec3 Center = P.Initial;
			Vec3 Along = Unit(P.Initial.Cross(R.Direction()));
			const unsigned Fragments = 5 + static_cast<unsigned>(R.Next() * 3);
			for(unsigned Fragment = 0; Fragment < Fragments; ++Fragment) {
				const double Length = 0.42 + R.Next() * 0.27, Width = 0.28 + R.Next() * 0.25, Strength = 0.65 + R.Next() * 0.55;
				auto F = Lobe(GeologicFeatureKind::ContinentalCrust, Center, Along, Length, Width, Strength);
				// Material fragments can attach to different plates along the
				// craton. Their later relative motion can open a rift or a gulf.
				std::uint32_t Host = 0;
				for(std::uint32_t J = 1; J < S.PlateCount; ++J)
					if(Center.Dot(Plates[J].Initial) > Center.Dot(Plates[Host].Initial)) Host = J;
				F.PlateA = Host;
				F.PlateB = Host;
				InitialCrust.push_back(F);
				const double Bend = (R.Next() - 0.5) * 1.8, StepLength = 0.14 + R.Next() * 0.20;
				const Vec3 T = Along * std::cos(Bend) + Center.Cross(Along) * std::sin(Bend);
				Center = Unit(Center * std::cos(StepLength) + T * std::sin(StepLength));
				Along = Unit(T - Center * T.Dot(Center));
				if(Fragment == 3 && R.Next() < 0.65) {
					Center = P.Initial;
					Along = Unit(Center.Cross(R.Direction()));
				}
			}
		}
		const auto MoveCrust = [&](double Time) {
			Field.Crust = InitialCrust;
			for(auto &F : Field.Crust) {
				const auto &P = Plates[F.PlateA];
				const double Angle = P.Rate * Time;
				F.Center = Unit(Rotate(F.Center, P.Axis, Angle));
				F.Along = Unit(Rotate(F.Along, P.Axis, Angle));
				F.Across = Unit(F.Center.Cross(F.Along));
			}
		};
		MoveCrust(S.HistoryMillionYears);
		constexpr std::uint32_t ProbeCount = 8192;
		std::vector<double> Scores;
		Scores.reserve(ProbeCount);
		for(std::uint32_t I = 0; I < ProbeCount; ++I) {
			if((I & 255) == 0 && Stop.stop_requested()) return {};
			Scores.push_back(
			    Field.ContinentalScore(ProbeDirection(I, ProbeCount, RotationAxis, RotationAngle)).ElevationMeters);
		}
		// Seed-specific initial target inside the requested interval. This initial
		// quantile is not a substitute for final ocean reconciliation after erosion.
		const double Target = 0.63 + R.Next() * 0.04;
		const auto Rank = static_cast<std::size_t>(Target * ProbeCount);
		std::nth_element(Scores.begin(), Scores.begin() + static_cast<std::ptrdiff_t>(Rank), Scores.end());
		Field.Stats.ContinentalThreshold = Scores[Rank];
		Field.Stats.CalibrationProbeCount = ProbeCount;
		Field.Stats.TargetOceanFraction = Target;
		const auto LowerRank = static_cast<std::size_t>(ProbeCount / 10);
		std::nth_element(Scores.begin(), Scores.begin() + static_cast<std::ptrdiff_t>(LowerRank), Scores.end());
		// Normalize score contrast: fragmented cratons otherwise leave almost all
		// submerged points inside the shallow shelf segment of the profile.
		Field.Stats.ContinentalScale = std::clamp((Field.Stats.ContinentalThreshold - Scores[LowerRank]) / 0.30, 0.1, 4.0);

		for(std::uint32_t Step = 0; Step <= S.HistoryIntervals; ++Step) {
			if(Stop.stop_requested()) return {};
			const double Time = S.HistoryMillionYears * Step / S.HistoryIntervals;
			const double Age = S.HistoryMillionYears - Time;
			MoveCrust(Time);
			PlateHistoryFrame Frame;
			Frame.AgeMillionYears = Age;
			for(const auto &P : Plates)
				Frame.Sites.push_back(Unit(Rotate(P.Initial, P.Axis, P.Rate * Time)));
			const auto &Sites = Frame.Sites;
			for(std::uint32_t A = 0; A < S.PlateCount; ++A) {
				if(Stop.stop_requested()) return {};
				for(std::uint32_t B = A + 1; B < S.PlateCount; ++B) {
					const auto Bisector = UnitDirection::FromVector(Sites[B] - Sites[A]);
					if(!Bisector) continue;
					const Vec3 Across = Bisector->Vector();
					std::vector<Vec3> Ends;
					for(std::uint32_t C = 0; C < S.PlateCount; ++C) {
						if(C == A || C == B) continue;
						const auto Vertex = UnitDirection::FromVector(Across.Cross(Sites[C] - Sites[A]));
						if(!Vertex) continue;
						for(const double Sign : {-1.0, 1.0}) {
							const Vec3 V = Vertex->Vector() * Sign;
							const double Score = V.Dot(Sites[A]);
							bool OnBoundary = true;
							for(const auto Site : Sites)
								if(V.Dot(Site) > Score + 1e-11) {
									OnBoundary = false;
									break;
								}
							if(OnBoundary &&
							   std::none_of(Ends.begin(), Ends.end(), [&](Vec3 E) { return (E - V).Length() < 1e-8; }))
								Ends.push_back(V);
						}
					}
					if(Ends.size() != 2 || Ends[0].Dot(Ends[1]) < -0.95) continue;
					const Vec3 Center = Unit(Ends[0] + Ends[1]), Along = Unit(Across.Cross(Center));
					const double HalfLength = 0.5 * std::acos(std::clamp(Ends[0].Dot(Ends[1]), -1.0, 1.0));
					if(HalfLength < 0.002) continue;
					const Vec3 Relative = (Plates[A].Axis * Plates[A].Rate - Plates[B].Axis * Plates[B].Rate).Cross(Center);
					const double Convergence = Relative.Dot(Across);
					const bool LandA = Field.ContinentalScore(Unit(Center - Across * 0.045)).ElevationMeters >
					                   Field.Stats.ContinentalThreshold;
					const bool LandB = Field.ContinentalScore(Unit(Center + Across * 0.045)).ElevationMeters >
					                   Field.Stats.ContinentalThreshold;
					const std::uint32_t Upper = LandA ? A : (LandB ? B : (Plates[A].Age < Plates[B].Age ? A : B));
					const double OceanSide = Upper == A ? 1.0 : -1.0;
					const double Rate = std::clamp(std::abs(Convergence) / 0.0015, 0.15, 1.0);
					const auto Add = [&](GeologicFeatureKind Kind, double OffsetMeters, double WidthMeters, double Amplitude,
					                     std::uint32_t Owner) {
						const double Offset = OffsetMeters / EarthRadius;
						Vec3 C = Center * std::cos(Offset) + Across * std::sin(Offset);
						const auto &P = Plates[Owner];
						C = Unit(Rotate(C, P.Axis, P.Rate * Age));
						const Vec3 T = Unit(Rotate(Along, P.Axis, P.Rate * Age));
						auto F =
						    Lobe(Kind, C, T, std::min(1.20, HalfLength * 1.35 + 0.03), WidthMeters / EarthRadius, Amplitude);
						F.AgeMillionYears = Age;
						F.PlateA = A;
						F.PlateB = B;
						Field.Lobes.push_back(F);
					};
					if(Convergence > 0.00012) {
						if(LandA && LandB)
							Add(GeologicFeatureKind::Collision, 0, 210000, 6500 * Rate * std::exp(-Age / 160), Upper);
						else {
							// Old subduction surfaces fade, while their advected
							// volcanic arcs can survive after the boundary moves.
							Add(GeologicFeatureKind::Trench, OceanSide * 35000, 65000, -6500 * Rate * std::exp(-Age / 28),
							    Upper);
							Add(GeologicFeatureKind::OuterRise, OceanSide * 120000, 110000, 600 * Rate * std::exp(-Age / 40),
							    Upper);
							Add(GeologicFeatureKind::Forearc, -OceanSide * 45000, 80000, 1300 * Rate * std::exp(-Age / 65),
							    Upper);
							Add(GeologicFeatureKind::VolcanicArc, -OceanSide * 180000, 140000,
							    3500 * Rate * std::exp(-Age / 120), Upper);
						}
					} else if(Convergence < -0.00012) {
						// Two sides carry the newly formed crust away from the
						// spreading axis. Age drives the broad thermal elevation;
						// this narrow term describes only the axial valley/rise.
						for(const auto Owner : {A, B})
							Add(GeologicFeatureKind::Spreading, 0, 190000, Rate < 0.55 ? -900 : 600, Owner);
					} else Add(GeologicFeatureKind::Transform, 0, 50000, 400 * std::exp(-Age / 110), Upper);
				}
			}
			Field.Frames.push_back(std::move(Frame));
		}
		MoveCrust(S.HistoryMillionYears);
		Field.Stats.FeatureCount = Field.Lobes.size();
		if(!Field.BuildFeatureIndex(Stop)) return {};
		// Fix the geological vertical datum after adding boundary features. The
		// sea radius remains exactly the requested value; no tide or sea adjustment.
		std::vector<double> Heights;
		Heights.reserve(ProbeCount);
		for(std::uint32_t I = 0; I < ProbeCount; ++I) {
			if((I & 127) == 0 && Stop.stop_requested()) return {};
			Heights.push_back(
			    Field.Evaluate(*UnitDirection::FromVector(ProbeDirection(I, ProbeCount, RotationAxis, RotationAngle)))
			        .Terrain.ElevationMeters);
		}
		std::nth_element(Heights.begin(), Heights.begin() + static_cast<std::ptrdiff_t>(Rank), Heights.end());
		Field.Stats.DatumShiftMeters = Heights[Rank] - S.SeaLevelMeters;
		// Independent quadrature: a different count and rotation from calibration.
		// This is an estimate, not a connectivity or continuous-area certificate.
		constexpr std::uint32_t ValidationCount = 16384;
		const Vec3 ValidationAxis = Unit(Vec3{1, 2, 3});
		const double ValidationAngle = 0.731;
		std::uint32_t Ocean = 0;
		for(std::uint32_t I = 0; I < ValidationCount; ++I) {
			if((I & 127) == 0 && Stop.stop_requested()) return {};
			if(Field.Evaluate(*UnitDirection::FromVector(ProbeDirection(I, ValidationCount, ValidationAxis, ValidationAngle)))
			       .Terrain.ElevationMeters < S.SeaLevelMeters)
				++Ocean;
		}
		Field.Stats.AreaProbeCount = ValidationCount;
		Field.Stats.InitialOceanFractionEstimate = static_cast<double>(Ocean) / ValidationCount;
		return Field;
	}

	bool GeologyField::BuildFeatureIndex(std::stop_token Stop) {
		FeatureOrder.resize(Lobes.size());
		std::iota(FeatureOrder.begin(), FeatureOrder.end(), 0U);
		FeatureIndex.clear();
		FeatureIndex.reserve(Lobes.size() * 2);
		if(Lobes.empty()) return !Stop.stop_requested();
		// The kernel can be nonzero only inside Center.Dot(D)>SupportCosine.
		// For unit vectors this implies |D-Center|^2 < 2*(1-SupportCosine).
		// Pad squared distance for normalization/dot rounding, especially narrow
		// caps where converting a dot error to a radius error amplifies it.
		// Retain the exact Kernel test on every candidate.
		const auto Bounds = [&](std::uint32_t I) {
			const auto &F = Lobes[I];
			const double R = std::nextafter(std::sqrt(2 * (1 - F.SupportCosine) + 64 * std::numeric_limits<double>::epsilon()),
			                                std::numeric_limits<double>::infinity());
			const Vec3 E{R, R, R};
			return std::array<Vec3, 2>{F.Center - E, F.Center + E};
		};
		const auto Axis = [](Vec3 V, unsigned A) { return A == 0 ? V.X : A == 1 ? V.Y : V.Z; };
		const auto Build = [&](auto &&Self, std::uint32_t Begin, std::uint32_t End) -> std::uint32_t {
			if(Stop.stop_requested()) return 0;
			const auto B = Bounds(FeatureOrder[Begin]);
			FeatureIndexNode N;
			N.Min = B[0];
			N.Max = B[1];
			for(auto I = Begin + 1; I < End; ++I) {
				const auto Q = Bounds(FeatureOrder[I]);
				N.Min = {std::min(N.Min.X, Q[0].X), std::min(N.Min.Y, Q[0].Y), std::min(N.Min.Z, Q[0].Z)};
				N.Max = {std::max(N.Max.X, Q[1].X), std::max(N.Max.Y, Q[1].Y), std::max(N.Max.Z, Q[1].Z)};
			}
			const auto Id = static_cast<std::uint32_t>(FeatureIndex.size());
			FeatureIndex.push_back(N);
			if(End - Begin <= 8) {
				FeatureIndex[Id].Begin = Begin;
				FeatureIndex[Id].Count = End - Begin;
				return Id;
			}
			const auto E = N.Max - N.Min;
			const unsigned A = E.X >= E.Y && E.X >= E.Z ? 0 : E.Y >= E.Z ? 1 : 2;
			const auto Mid = Begin + (End - Begin) / 2;
			std::nth_element(FeatureOrder.begin() + Begin, FeatureOrder.begin() + Mid, FeatureOrder.begin() + End,
			                 [&](auto U, auto V) {
				                 const double X = Axis(Lobes[U].Center, A), Y = Axis(Lobes[V].Center, A);
				                 return X != Y ? X < Y : U < V;
			                 });
			const auto L = Self(Self, Begin, Mid), R = Self(Self, Mid, End);
			FeatureIndex[Id].Left = L;
			FeatureIndex[Id].Right = R;
			return Id;
		};
		Build(Build, 0, static_cast<std::uint32_t>(Lobes.size()));
		return !Stop.stop_requested();
	}
	std::vector<std::uint32_t> GeologyField::QueryFeatures(Vec3 D) const {
		std::vector<std::uint32_t> IDs;
		if(FeatureIndex.empty()) return IDs;
		// Balanced median tree over a uint32 index has depth at most 32.
		std::array<std::uint32_t, 64> Stack{};
		unsigned Count = 1;
		while(Count) {
			const auto &N = FeatureIndex[Stack[--Count]];
			if(D.X < N.Min.X || D.Y < N.Min.Y || D.Z < N.Min.Z || D.X > N.Max.X || D.Y > N.Max.Y || D.Z > N.Max.Z) continue;
			if(!N.Count) {
				Stack[Count++] = N.Right;
				Stack[Count++] = N.Left;
				continue;
			}
			for(auto I = N.Begin; I < N.Begin + N.Count; ++I)
				IDs.push_back(FeatureOrder[I]);
		}
		// Preserve the original summation order, including equal-kind features.
		std::sort(IDs.begin(), IDs.end());
		return IDs;
	}
	double GeologyField::EvaluateElevation(UnitDirection D) const {
		return EvaluateImpl<false>(D, true).Terrain.ElevationMeters;
	}
	GeologySample GeologyField::Evaluate(UnitDirection D) const {
		return EvaluateImpl<true>(D, true);
	}
	GeologySample GeologyField::EvaluateReference(UnitDirection D) const {
		return EvaluateImpl<true>(D, false);
	}
	std::size_t GeologyField::FeatureCandidateCount(UnitDirection D) const {
		return QueryFeatures(D.Vector()).size();
	}

	template <bool Derivatives> GeologySample GeologyField::EvaluateImpl(UnitDirection Direction, bool Indexed) const {
		using Jet = TerrainJet<Derivatives>;
		const auto Constant = [](double V) { return Jet{V, {}}; };
		const auto D = Direction.Vector();
		const auto Score = ContinentalScoreImpl<Derivatives>(D);
		Vec3 CoordinateGradient;
		if constexpr(Derivatives) CoordinateGradient = Score.TangentialGradientPerRadian * (1 / Stats.ContinentalScale);
		const Jet Coordinate{(Score.ElevationMeters - Stats.ContinentalThreshold) / Stats.ContinentalScale, CoordinateGradient};
		std::array<Jet, 8> Weighted{}, Weights{};
		Jet AgeNumerator = Constant(120), AgeDenominator = Constant(1);
		const auto Accumulate = [&](const GeologicFeature &F) {
			const Jet K = Kernel<Derivatives>(F, D);
			if(K.V == 0) return;
			const auto Kind = static_cast<std::size_t>(F.Kind);
			Weighted[Kind] = Weighted[Kind] + K * F.AmplitudeMeters;
			Weights[Kind] = Weights[Kind] + K;
			if(F.Kind == GeologicFeatureKind::Spreading) {
				AgeNumerator = AgeNumerator + K * (20 * (F.AgeMillionYears + 0.5));
				AgeDenominator = AgeDenominator + K * 20;
			}
		};
		if(Indexed)
			for(const auto Id : QueryFeatures(D))
				Accumulate(Lobes[Id]);
		else
			for(const auto &Feature : Lobes)
				Accumulate(Feature);
		const Jet Age = AgeNumerator / AgeDenominator;
		const Jet OceanDepth = Constant(-2600) - (Constant(1) - Exp(Sqrt(Age + Constant(1)) * (-1.0 / 7))) * 2700;
		const auto Trench = static_cast<std::size_t>(GeologicFeatureKind::Trench);
		const Jet ActiveMargin = Weights[Trench] / (Constant(1) + Weights[Trench]);
		Jet Height = ShelfProfile(Coordinate * (Constant(1) + ActiveMargin * 2), OceanDepth);
		Jet Tectonics;
		for(std::size_t I = 1; I < Weighted.size(); ++I)
			Tectonics = Tectonics + Weighted[I] / (Constant(1) + Weights[I]);
		Jet Detail;
		for(unsigned I = 6; I < Waves.size(); ++I) {
			const auto &W = Waves[I];
			const double Phase = W.Frequency * W.Axis.Dot(D) + W.Phase;
			Vec3 Gradient;
			if constexpr(Derivatives) Gradient = W.Axis * (W.Frequency * std::cos(Phase) / 6);
			Detail = Detail + Jet{std::sin(Phase) / 6, Gradient};
		}
		const Jet Land = Smooth(Coordinate, 0, 0.08);
		const Jet Marine = Constant(1) - Smooth(Coordinate, -0.08, 0);
		Height = Height + Tectonics + Detail * (Land + Marine * 0.16) * 450;
		const double ReliefScale = Parameters.RadiusMeters / (Parameters.RadiusMeters + 250000);
		Height = Height * ReliefScale + Constant(Parameters.SeaLevelMeters - Stats.DatumShiftMeters);
		Vec3 Tangent;
		if constexpr(Derivatives) Tangent = Height.G - D * Height.G.Dot(D);
		return {{Height.V, Tangent}, Coordinate.V, Age.V, Tectonics.V * ReliefScale, 0.45 + 0.45 * Land.V};
	}

	double GeologyField::AbsoluteReliefBoundMeters() const {
		// Sum of bounded normalized feature families, shelf interpolation and
		// finite residual amplitudes. Deliberately loose; not an SSE error bound.
		return 40000 * Parameters.RadiusMeters / (Parameters.RadiusMeters + 250000) + std::abs(Stats.DatumShiftMeters);
	}
	Vec3 GeologyField::SurfacePosition(UnitDirection Direction) const {
		return Direction.Vector() * (Parameters.RadiusMeters + Evaluate(Direction).Terrain.ElevationMeters);
	}
	UnitDirection GeologyField::SurfaceNormal(UnitDirection Direction) const {
		const auto S = Evaluate(Direction).Terrain;
		return *UnitDirection::FromVector(Direction.Vector() * (Parameters.RadiusMeters + S.ElevationMeters) -
		                                  S.TangentialGradientPerRadian);
	}
} // namespace PlanetCore
