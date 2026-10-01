#include "PlanetGround.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace PlanetCore {
	/**
	 * 半径に依らず地表へ向かう一定の重力を返す
	 * @param Position 惑星中心からの位置
	 * @return 有効な方向の加速度，中心又は非有限入力は空
	 */
	std::optional<Vec3> SurfaceGravity(const Vec3 Position) {
		const std::optional<UnitDirection> D = UnitDirection::FromVector(Position);
		// 正規化出来ない方向への返戻
		if(!D) return {};
		// 半径に依存しない加速度の返戻
		return D->Vector() * -SurfaceGravityMetersPerSecondSquared;
	}

	/**
	 * 地表原点を持つ右手系の局所座標を準備する
	 * @param Anchor 地表原点の方向
	 * @param Radius 海面の半径
	 * @param Elevation 原点の標高
	 * @return 有効な物理尺度の座標系，無効な尺度は空
	 */
	std::optional<GroundFrame> GroundFrame::Create(const UnitDirection Anchor, const double Radius, const double Elevation) {
		// 不正な半径又は標高への返戻
		if(!std::isfinite(Radius) || Radius < 1000 || Radius > 1E9 || !std::isfinite(Elevation) || !(Radius + Elevation > 0)) return {};
		GroundFrame F;
		F.Up = Anchor.Vector();
		F.RadiusMeters = Radius;
		F.AnchorElevationMeters = Elevation;
		const Vec3 Axis = std::abs(F.Up.Z) < 0.8 ? Vec3{ 0, 0, 1 } : Vec3{ 1, 0, 0 };
		F.East = UnitDirection::FromVector(F.Up.Cross(Axis))->Vector();
		F.North = F.Up.Cross(F.East);
		// 準備した直交座標の返戻
		return F;
	}

	/**
	 * 局所接平面の位置を惑星方向へ写す
	 * @param X 東方向の接平面距離
	 * @param Y 北方向の接平面距離
	 * @return 正規化した球面方向
	 */
	UnitDirection GroundFrame::DirectionAt(const double X, const double Y) const {
		// 接平面に対応する方向の返戻
		return *UnitDirection::FromVector(Up + East * (X / RadiusMeters) + North * (Y / RadiusMeters));
	}

	/**
	 * 球面の標高を局所の倍精度位置へ変換する
	 * @param D 惑星中心からの方向
	 * @param H 同じ地点の標高
	 * @return 地表原点に対する位置
	 */
	Vec3 GroundFrame::SurfaceToLocal(const UnitDirection D, const double H) const {
		// 惑星尺度の近い半径同士を直接減算せず，局所の高さを保つ
		const Vec3 Delta = D.Vector() - Up;
		const double R = RadiusMeters + H;
		// 近接した半径の差を安定化した局所位置の返戻
		return { Delta.Dot(East) * R, Delta.Dot(North) * R, H - AnchorElevationMeters - 0.5 * R * Delta.Dot(Delta) };
	}

	/**
	 * 方向を局所基底へ変換する
	 * @param V 惑星座標での方向
	 * @return 局所基底での成分
	 */
	Vec3 GroundFrame::VectorToLocal(const Vec3 V) const {
		// 局所基底への射影の返戻
		return { V.Dot(East), V.Dot(North), V.Dot(Up) };
	}

	/**
	 * 局所位置を惑星中心からの位置へ変換する
	 * @param P 地表原点に対する位置
	 * @return 惑星中心からの倍精度位置
	 */
	Vec3 GroundFrame::LocalToPlanet(const Vec3 P) const {
		// 惑星中心を原点とする位置の返戻
		return Up * (RadiusMeters + AnchorElevationMeters + P.Z) + East * P.X + North * P.Y;
	}

	/**
	 * 局所範囲，予算及び検査精度の指定を調べる
	 * @param Radius 惑星半径
	 * @return 局所メッシュの入力条件を満たす場合に真
	 */
	bool GroundPatchSettings::IsValid(const double Radius) const {
		// 有限な範囲と有効な予算の判定結果の返戻
		return std::isfinite(HalfExtentMeters) && !(HalfExtentMeters < 1) && !(HalfExtentMeters > 256) &&
		HalfExtentMeters <= Radius * 0.05 && CellsPerSide > 3 && CellsPerSide < 129 && std::isfinite(MaximumProbeErrorMeters) &&
		MaximumProbeErrorMeters > 0 && MaximumProbeErrorMeters <= 0.02;
	}

	/**
	 * 同じ標高関数から局所の連続メッシュを誤差駆動で生成する
	 * 有限標本の合格は全点の誤差上界の証明ではない
	 * @param Anchor 局所原点の方向
	 * @param Radius 惑星半径
	 * @param S 範囲と細分化の入力設定
	 * @param Evaluate 標高と解析勾配の評価
	 * @param Stop 中断要求
	 * @param EvaluateHeight 勾配が不要な地点用の同じ標高評価，省略可
	 * @param Diagnostics 拒否理由と最後の反復の任意出力
	 * @return 検査と予算を満たす候補，失敗又は中断時は空
	 */
	std::optional<GroundPatch> BuildGroundPatch(
		const UnitDirection Anchor,
		const double Radius,
		const GroundPatchSettings &S,
		const GroundEvaluator &Evaluate,
		const std::stop_token Stop,
		const GroundHeightEvaluator &EvaluateHeight,
		GroundPatchDiagnostics *const Diagnostics
	) {
		GroundPatch Patch;
		if(Diagnostics) *Diagnostics = {};
		const auto Reject = [&](const GroundPatchFailure Failure) -> std::optional<GroundPatch> {
			if(Diagnostics) {
				Diagnostics->Failure = Failure;
				Diagnostics->Vertices = Patch.Positions.size();
				Diagnostics->Triangles = Patch.Triangles.size();
			}
			// 原因を残した未公開の候補の返戻
			return {};
		};
		// 評価関数の欠落又は不正な設定への返戻
		if(!Evaluate || !S.IsValid(Radius)) return Reject(GroundPatchFailure::InvalidInput);
		// 事前中断を入力不備と区別する返戻
		if(Stop.stop_requested()) return Reject(GroundPatchFailure::Cancelled);
		const TerrainSample Base = Evaluate(Anchor);
		++Patch.Evaluations;
		const std::optional<GroundFrame> Frame = GroundFrame::Create(Anchor, Radius, Base.ElevationMeters);
		// 局所座標を準備出来ない場合の返戻
		if(!Frame) return Reject(GroundPatchFailure::InvalidFrame);
		Patch.Frame = *Frame;
		const std::uint32_t Size = S.CellsPerSide + 1;
		Patch.Positions.reserve(Size * Size);
		Patch.Normals.reserve(Size * Size);
		Patch.Triangles.reserve(S.CellsPerSide * S.CellsPerSide * 2);
		for(std::uint32_t Y = 0; Y < Size; ++Y) {
			// 中断済の候補を公開しない返戻
			if(Stop.stop_requested()) return Reject(GroundPatchFailure::Cancelled);
			for(std::uint32_t X = 0; X < Size; ++X) {
				const UnitDirection D =
				Frame->DirectionAt(S.HalfExtentMeters * (2.0 * X / S.CellsPerSide - 1), S.HalfExtentMeters * (2.0 * Y / S.CellsPerSide - 1));
				const TerrainSample Sample = Evaluate(D);
				++Patch.Evaluations;
				const bool HasValidHeight = std::isfinite(Sample.ElevationMeters) && Radius + Sample.ElevationMeters > 0;
				// 非有限な標高又は惑星中心を越える頂点への返戻
				if(!HasValidHeight) return Reject(GroundPatchFailure::InvalidSample);
				const std::optional<UnitDirection> Normal =
				UnitDirection::FromVector(D.Vector() * (Radius + Sample.ElevationMeters) - Sample.TangentialGradientPerRadian);
				// 有効な法線を構成出来ない場合の返戻
				if(!Normal) return Reject(GroundPatchFailure::InvalidNormal);
				Patch.Positions.push_back(Frame->SurfaceToLocal(D, Sample.ElevationMeters));
				Patch.Normals.push_back(Frame->VectorToLocal(Normal->Vector()));
				if(X + 1 < Size && Y + 1 < Size) {
					const std::uint32_t A = Y * Size + X, B = A + 1, C = A + Size, DIndex = C + 1;
					Patch.Triangles.push_back({ A, B, DIndex });
					Patch.Triangles.push_back({ A, DIndex, C });
				}
			}
		}
		// 誤差の大きい面だけを細分化し，共有辺の中点を一つに保つ
		// 重心だけでは細長い面の内部極値を見逃す為，三方向の内部点も調べる
		// PlanetLod と同じ８種類の分割で，不対の内部辺を作らない
		constexpr std::size_t MaxTriangles = 65536, MaxVertices = 65536;
		// 一辺の二分では一回に全辺が半減しない為，反復回数と幾何的な分割深度を区別する
		// 頂点・三角形の上限を保った儘，従来の全辺分割８回に対して最大１６回を許す
		constexpr unsigned MaxPasses = 16;
		std::vector<bool> NeedsCheck(Patch.Triangles.size(), true);
		const auto EdgeKey = [](const std::uint32_t A, const std::uint32_t B) -> std::uint64_t {
			// 端点の順序に依らない共有辺の識別値の返戻
			return static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
		};
		// 準備中に頂点は動かない為，共有辺の誤差を同じ候補の隣接面と反復だけで再利用する
		std::unordered_map<std::uint64_t, double> EdgeErrors;
		if(S.ReuseSharedEdgeProbes) EdgeErrors.reserve(8192);
		constexpr std::size_t MaximumCachedEdges = 3 * MaxTriangles;
		bool Accepted = false;
		for(unsigned Pass = 0; Pass <= MaxPasses; ++Pass) {
			if(Diagnostics) {
				Diagnostics->RefinementPass = Pass;
				Diagnostics->MaximumErrorMeters = 0;
			}
			std::vector<std::uint64_t> SplitEdges;
			for(std::size_t I = 0; I < Patch.Triangles.size(); ++I) {
				// 細分化中に中断された候補への返戻
				if(!(I & 63) && Stop.stop_requested()) return Reject(GroundPatchFailure::Cancelled);
				if(!NeedsCheck[I]) continue;
				const std::array<std::uint32_t, 3> T = Patch.Triangles[I];
				const Vec3 A = Patch.Positions[T[0]], B = Patch.Positions[T[1]], C = Patch.Positions[T[2]];
				// 反転又は退化した三角形への返戻
				if(!((B - A).Cross(C - A).Z > 0)) return Reject(GroundPatchFailure::InvertedTriangle);
				double MaximumError = 0;
				std::array<double, 7> ProbeErrors {};
				const std::array<Vec3, 7> Probes {
					(A + B) * 0.5,
					(B + C) * 0.5,
					(C + A) * 0.5,
					(A + B + C) * (1.0 / 3),
					A * 0.5 + (B + C) * 0.25,
					B * 0.5 + (C + A) * 0.25,
					C * 0.5 + (A + B) * 0.25
				};
				for(unsigned K = 0; K < Probes.size(); ++K) {
					const std::uint64_t Key = K < 3 ? EdgeKey(T[K], T[(K + 1) % 3]) : 0;
					if(S.ReuseSharedEdgeProbes && K < 3) {
						const std::unordered_map<std::uint64_t, double>::const_iterator It = EdgeErrors.find(Key);
						if(It != EdgeErrors.end()) {
							MaximumError = std::max(MaximumError, It->second);
							ProbeErrors[K] = It->second;
							++Patch.SharedEdgeProbeHits;
							continue;
						}
					}
					const Vec3 P = Probes[K];
					const double R0 = Radius + Frame->AnchorElevationMeters, DeltaSquared = 2 * R0 * P.Z + P.Dot(P);
					const double Distance = std::sqrt(R0 * R0 + DeltaSquared);
					const double InterpolatedH = Frame->AnchorElevationMeters + DeltaSquared / (Distance + R0);
					const std::optional<UnitDirection> Direction = UnitDirection::FromVector(Frame->LocalToPlanet(P));
					// 検査地点の方向が無効な場合の返戻
					if(!Direction) return Reject(GroundPatchFailure::InvalidProbe);
					const double ExactH = EvaluateHeight ? EvaluateHeight(*Direction) : Evaluate(*Direction).ElevationMeters;
					++Patch.Evaluations;
					const double Error = std::abs(ExactH - InterpolatedH);
					// 非有限な検査値を合格としない返戻
					if(!std::isfinite(Error)) return Reject(GroundPatchFailure::InvalidProbe);
					if(S.ReuseSharedEdgeProbes && K < 3 && EdgeErrors.size() < MaximumCachedEdges) EdgeErrors.emplace(Key, Error);
					ProbeErrors[K] = Error;
					MaximumError = std::max(MaximumError, Error);
				}
				if(Diagnostics && MaximumError > Diagnostics->MaximumErrorMeters) {
					Diagnostics->MaximumErrorMeters = MaximumError;
					Diagnostics->WorstTriangle = { A, B, C };
					Diagnostics->WorstProbeErrors = ProbeErrors;
				}
				if(MaximumError > S.MaximumProbeErrorMeters) {
					// 短い合格辺まで繰返し割らず，不合格の辺だけを共有する隣接面へ伝える
					for(unsigned E = 0; E < 3; ++E) {
						if(ProbeErrors[E] > S.MaximumProbeErrorMeters) SplitEdges.push_back(EdgeKey(T[E], T[(E + 1) % 3]));
					}
					{
						// 中点だけが偶然合格した長辺も分け，細長い面の短辺増殖を避ける
						unsigned Longest = 0;
						double LongestSquared = 0;
						for(unsigned E = 0; E < 3; ++E) {
							const Vec3 Edge = Patch.Positions[T[E]] - Patch.Positions[T[(E + 1) % 3]];
							const double Squared = Edge.Dot(Edge);
							if(Squared > LongestSquared) {
								Longest = E;
								LongestSquared = Squared;
							}
						}
						SplitEdges.push_back(EdgeKey(T[Longest], T[(Longest + 1) % 3]));
					}
				} else {
					NeedsCheck[I] = false;
					Patch.MaximumProbeErrorMeters = std::max(Patch.MaximumProbeErrorMeters, MaximumError);
				}
			}
			if(SplitEdges.empty()) {
				Accepted = true;
				break;
			}
			// 反復予算内に精度へ届かない候補への返戻
			if(Pass == MaxPasses) return Reject(GroundPatchFailure::RefinementLimit);
			std::sort(SplitEdges.begin(), SplitEdges.end());
			SplitEdges.erase(std::unique(SplitEdges.begin(), SplitEdges.end()), SplitEdges.end());
			// 頂点予算を超える候補への返戻
			if(Patch.Positions.size() + SplitEdges.size() > MaxVertices) return Reject(GroundPatchFailure::VertexLimit);
			const std::uint32_t FirstMidpoint = static_cast<std::uint32_t>(Patch.Positions.size());
			for(const std::uint64_t Key : SplitEdges) {
				// 中断済の候補を公開しない返戻
				if(Stop.stop_requested()) return Reject(GroundPatchFailure::Cancelled);
				const std::uint32_t A = static_cast<std::uint32_t>(Key >> 32), B = static_cast<std::uint32_t>(Key);
				const std::optional<UnitDirection> DA = UnitDirection::FromVector(Frame->LocalToPlanet(Patch.Positions[A]));
				const std::optional<UnitDirection> DB = UnitDirection::FromVector(Frame->LocalToPlanet(Patch.Positions[B]));
				// 共有辺の端点を球面へ戻せない場合の返戻
				if(!DA || !DB) return Reject(GroundPatchFailure::InvalidProbe);
				const std::optional<UnitDirection> D = UnitDirection::FromVector(DA->Vector() + DB->Vector());
				// 正規化出来ない方向への返戻
				if(!D) return Reject(GroundPatchFailure::InvalidProbe);
				const TerrainSample Sample = Evaluate(*D);
				++Patch.Evaluations;
				const bool HasValidHeight = std::isfinite(Sample.ElevationMeters) && Radius + Sample.ElevationMeters > 0;
				// 非有限な標高又は惑星中心を越える頂点への返戻
				if(!HasValidHeight) return Reject(GroundPatchFailure::InvalidSample);
				const std::optional<UnitDirection> Normal =
				UnitDirection::FromVector(D->Vector() * (Radius + Sample.ElevationMeters) - Sample.TangentialGradientPerRadian);
				// 有効な法線を構成出来ない場合の返戻
				if(!Normal) return Reject(GroundPatchFailure::InvalidNormal);
				Patch.Positions.push_back(Frame->SurfaceToLocal(*D, Sample.ElevationMeters));
				Patch.Normals.push_back(Frame->VectorToLocal(Normal->Vector()));
			}
			std::vector<std::array<std::uint32_t, 3>> Next;
			std::vector<bool> NextCheck;
			Next.reserve(std::min(MaxTriangles, Patch.Triangles.size() + SplitEdges.size() * 2));
			const auto Emit = [&](const std::uint32_t A, const std::uint32_t B, const std::uint32_t C, const bool Check=true) -> void {
				Next.push_back({ A, B, C });
				NextCheck.push_back(Check);
			};
			for(std::size_t I = 0; I < Patch.Triangles.size(); ++I) {
				// 細分化中に中断された候補への返戻
				if(!(I & 63) && Stop.stop_requested()) return Reject(GroundPatchFailure::Cancelled);
				const std::array<std::uint32_t, 3> V = Patch.Triangles[I];
				std::array<std::uint32_t, 3> M {};
				unsigned Mask = 0;
				for(unsigned E = 0; E < 3; ++E) {
					const std::uint64_t Key = EdgeKey(V[(E + 1) % 3], V[(E + 2) % 3]);
					const std::vector<std::uint64_t>::const_iterator It = std::lower_bound(SplitEdges.begin(), SplitEdges.end(), Key);
					if(It != SplitEdges.end() && *It == Key) {
						Mask |= 1U << E;
						M[E] = FirstMidpoint + static_cast<std::uint32_t>(It - SplitEdges.begin());
					}
				}
				if(!Mask) Emit(V[0], V[1], V[2], NeedsCheck[I]);
				else if(Mask == 7) {
					Emit(V[0], M[2], M[1]);
					Emit(M[2], V[1], M[0]);
					Emit(M[1], M[0], V[2]);
					Emit(M[2], M[0], M[1]);
				} else if(Mask == 1 || Mask == 2 || Mask == 4) {
					const unsigned E = Mask == 1 ? 0 : Mask == 2 ? 1 : 2;
					Emit(V[E], V[(E + 1) % 3], M[E]);
					Emit(V[E], M[E], V[(E + 2) % 3]);
				} else {
					const unsigned E = Mask == 6 ? 0 : Mask == 5 ? 1 : 2;
					const std::uint32_t AB = M[(E + 2) % 3], CA = M[(E + 1) % 3];
					Emit(V[E], AB, CA);
					Emit(AB, V[(E + 1) % 3], V[(E + 2) % 3]);
					Emit(AB, V[(E + 2) % 3], CA);
				}
				// 三角形予算を超える候補への返戻
				if(Next.size() > MaxTriangles) return Reject(GroundPatchFailure::TriangleLimit);
			}
			Patch.Triangles = std::move(Next);
			NeedsCheck = std::move(NextCheck);
		}
		// 精度の受理が確定しない候補への返戻
		if(!Accepted) return Reject(GroundPatchFailure::RefinementLimit);
		// 中断済の候補を公開しない返戻
		if(Stop.stop_requested()) return Reject(GroundPatchFailure::Cancelled);
		if(Diagnostics) {
			Diagnostics->Vertices = Patch.Positions.size();
			Diagnostics->Triangles = Patch.Triangles.size();
		}
		// 位置，法線及び共有された位相の返戻
		return Patch;
	}
}
