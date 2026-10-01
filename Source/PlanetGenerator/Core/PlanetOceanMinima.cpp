#include "PlanetOcean.h"
#include "PlanetCriticalPoint.h"
#include "PlanetLandRelief.h"
#include "PlanetOceanMinimaDiagnostics.h"
#include "PlanetSphericalMesh.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

namespace PlanetCore {
	namespace {
		#if WITH_DEV_AUTOMATION_TESTS

		thread_local OceanMinimaPreparationDiagnostics LastOceanMinimaPreparation;

		#endif

		/**
		 * 一つの球面三角形で，線形に補間した二成分勾配の零点候補を求める
		 * @param Directions 三角形の方向
		 * @param Samples 同じ方向で実測した標高と解析勾配
		 * @return 内部の孤立零点候補，退化又は外部では空；元場の停留点ではない
		 */
		std::optional<UnitDirection> TriangleCriticalCandidate(
			const std::array<UnitDirection, 3> &Directions,
			const std::array<TerrainSample, 3> &Samples
		) {
			const Vec3 A = Directions[0].Vector(), B = Directions[1].Vector(), C = Directions[2].Vector();
			const UnitDirection Center = *UnitDirection::FromVector(A + B + C);
			const Vec3 E = UnitDirection::FromVector(B - A - Center.Vector() * (B - A).Dot(Center.Vector()))->Vector();
			const Vec3 F = Center.Vector().Cross(E);
			std::array<std::array<double, 2>, 3> G;
			double Scale = 0;
			for(unsigned I = 0; I < 3; ++I) {
				G[I] = { Samples[I].TangentialGradientPerRadian.Dot(E), Samples[I].TangentialGradientPerRadian.Dot(F) };
				Scale = std::max({ Scale, std::abs(G[I][0]), std::abs(G[I][1]) });
			}
			// 零勾配面又は非有限な入力を孤立候補へ読み替えない返戻
			if(!std::isfinite(Scale) || !Scale) return {};
			for(std::array<double, 2> &V : G) for(double &X : V) X /= Scale;
			const double UX = G[1][0] - G[0][0], UY = G[1][1] - G[0][1], VX = G[2][0] - G[0][0], VY = G[2][1] - G[0][1];
			const double Det = UX * VY - UY * VX;
			// 可逆でない線形勾配から候補を生成しない返戻
			if(!std::isfinite(Det) || std::abs(Det) < 1E-12) return {};
			const double U = (G[0][1] * VX - G[0][0] * VY) / Det, V = (G[0][0] * UY - G[0][1] * UX) / Det;
			// 三角形の外側の候補を内部水域としない返戻
			if(U < 0 || V < 0 || U + V > 1) return {};
			// 線形勾配の候補方向の返戻；受理には元場の実水没を要する
			return UnitDirection::FromVector(A * (1 - U - V) + B * U + C * V);
		}
	}

	#if WITH_DEV_AUTOMATION_TESTS

	/**
	 * 同じ検査スレッドの最後の準備段階を取得する
	 * @return 未公開の場を返さず，拒否段階と有限な計測値だけを返す
	 */
	OceanMinimaPreparationDiagnostics InspectOceanMinimaPreparation() {
		return LastOceanMinimaPreparation;
	}

	#endif

	/**
	 * 節点の間の実水没点を元場で確認してから，大洋を接続する
	 * 符号を保存する起伏では地質だけから候補を求め，実証拠は元場で測る
	 * 有限候補の追加であり，全ての連続停留点の列挙保証ではない
	 * @param Field 接続前の不変な元場，成功時は全実標本の接続海峡を保持する
	 * @param Directions 呼出側の元の球面方向
	 * @param Triangles 元方向の閉じた球面三角形
	 * @param Stop 中断要求
	 * @return 拡張した同じ網の全水没標本を大洋へ接続出来た場合に真
	 */
	bool OceanConnectivityField::ConnectOnMesh(
		OceanConnectivityField &Field,
		const std::span<const UnitDirection> Directions,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const std::stop_token Stop
	) {
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation = {};
		#endif
		// 不正又は中断済みの元網の返戻
		if(Directions.empty() || Triangles.empty() || Stop.stop_requested()) return false;
		const double Radius = Field.Base().Settings().RadiusMeters, Sea = Field.Base().Settings().SeaLevelMeters;
		// 最悪の細部振幅でも，海岸接続の五次 Bernstein 制御値が正になる条件
		// 此の定数を変える場合，地質だけによる海域候補の妥当性も再検証する
		static_assert(PredrainageLandReliefStartMeters > 0);
		static_assert(PredrainageLandReliefFullMeters > PredrainageLandReliefStartMeters);
		static_assert(
			LandReliefAmplitudeMeters <
			PredrainageLandReliefStartMeters + 0.6 * (PredrainageLandReliefFullMeters - PredrainageLandReliefStartMeters)
		);
		constexpr std::uint64_t MaximumEvaluations = 1000000;
		constexpr std::size_t MaximumAddedNodes = 4096;
		std::uint64_t Evaluations = 0;
		std::uint32_t CriticalCandidates = 0;
		bool InvalidSource = false;
		// 候補探索と実水没判定に，同じ準備全体の照会上限を共有する
		const auto Read = [&](const UnitDirection Direction, const bool GeologyOnly) {
			// 準備全体の候補照会上限を越えた場合の非有限値の返戻
			if(Stop.stop_requested() || !(Evaluations < MaximumEvaluations)) {
				InvalidSource = true;
				#if WITH_DEV_AUTOMATION_TESTS
				LastOceanMinimaPreparation.SourceRejected = true;
				#endif
				// 中断又は予算切れを水没点に変換しない返戻
				return TerrainSample{ std::numeric_limits<double>::quiet_NaN(), {} };
			}
			++Evaluations;
			#if WITH_DEV_AUTOMATION_TESTS
			LastOceanMinimaPreparation.Evaluations = Evaluations;
			#endif
			const TerrainSample Value = GeologyOnly ? Field.Base().Evaluate(Direction).Terrain : Field.Evaluate(Direction);
			if(!std::isfinite(Value.ElevationMeters) || !std::isfinite(Value.TangentialGradientPerRadian.Length())) InvalidSource = true;
			#if WITH_DEV_AUTOMATION_TESTS
			LastOceanMinimaPreparation.SourceRejected = InvalidSource;
			#endif
			// 同じ元場で実測した標高と勾配の返戻
			return Value;
		};
		// 地質の水没極小の候補探索
		// 造山と中規模起伏は非負
		// ５ m の細部は海面上２～８ m の接続で符号を保存する
		// 最悪差分の五次 Bernstein 係数は 2，3.2，4.4，0.6，1.8，3 m で全て正
		// 海峡前の水没集合は地質と同じなので，高地の細部停留点を探索しない
		const std::function<TerrainSample(UnitDirection)> Proposal = [&](const UnitDirection D) {
			return Read(D, true);
		};
		// 実証拠は既存海峡も含む同じ元場で再評価し，地質候補だけでは受理しない
		const std::function<TerrainSample(UnitDirection)> Source = [&](const UnitDirection D) {
			return Read(D, false);
		};
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::GeologySamples;
		#endif
		std::vector<TerrainSample> Samples;
		Samples.reserve(Directions.size());
		for(const UnitDirection D : Directions) {
			Samples.push_back(Proposal(D));
			// 非有限な元場又は予算不足の返戻
			if(InvalidSource) return false;
		}
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::OriginalWetSamples;
		#endif
		std::vector<bool> OriginalWet;
		OriginalWet.reserve(Directions.size());
		for(std::size_t I = 0; I < Directions.size(); ++I) {
			const double Height = Field.Channels.empty() ? Samples[I].ElevationMeters : Source(Directions[I]).ElevationMeters;
			// 既存海峡を含む入力の水没状態を評価出来ない場合の返戻
			if(InvalidSource) return false;
			OriginalWet.push_back(Height < Sea);
		}
		std::vector<UnitDirection> WetDirections;
		const auto AddWet = [&](const UnitDirection Direction) {
			// 同じ実水没証拠を繰返し追加しない返戻
			for(const UnitDirection Previous : WetDirections) if((Previous.Vector() - Direction.Vector()).Length() < 1E-9) return true;
			// 候補保持量の不足で部分網を公開しない返戻
			if(!(WetDirections.size() < MaximumAddedNodes)) return false;
			WetDirections.push_back(Direction);
			#if WITH_DEV_AUTOMATION_TESTS
			LastOceanMinimaPreparation.WetWitnesses = WetDirections.size();
			#endif
			// 有界な実水没点を保持した結果の返戻
			return true;
		};
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::CandidateDiscovery;
		#endif
		for(const std::array<std::uint32_t, 3> &Triangle : Triangles) {
			// 元の頂点で既に水没域を含む三角形を内部水域の追加対象から除く
			if(OriginalWet[Triangle[0]] || OriginalWet[Triangle[1]] || OriginalWet[Triangle[2]]) continue;
			const UnitDirection A = Directions[Triangle[0]], B = Directions[Triangle[1]], C = Directions[Triangle[2]];
			const UnitDirection AB = *UnitDirection::FromVector(A.Vector() + B.Vector());
			const UnitDirection BC = *UnitDirection::FromVector(B.Vector() + C.Vector());
			const UnitDirection CA = *UnitDirection::FromVector(C.Vector() + A.Vector());
			const std::array<UnitDirection, 6> D { A, B, C, AB, BC, CA };
			const std::array<TerrainSample, 6> V {
				Samples[Triangle[0]],
				Samples[Triangle[1]],
				Samples[Triangle[2]],
				Proposal(AB),
				Proposal(BC),
				Proposal(CA)
			};
			// 中断・元場不正・予算不足を候補の不在へ読み替えない返戻
			if(InvalidSource) return false;
			// 辺内で実際に水没した証拠を落とさない返戻
			for(unsigned I = 3; I < 6; ++I) if(V[I].ElevationMeters < Sea) {
				const TerrainSample Actual = Source(D[I]);
				// 地質候補を実水没として受理出来ない場合の返戻
				if(InvalidSource || !(Actual.ElevationMeters < Sea) || !AddWet(D[I])) return false;
			}
			const std::array<std::array<unsigned, 3>, 4> Children { { { 0, 3, 5 }, { 3, 1, 4 }, { 5, 4, 2 }, { 3, 4, 5 } } };
			for(const std::array<unsigned, 3> &Child : Children) {
				const std::array<UnitDirection, 3> Corners { D[Child[0]], D[Child[1]], D[Child[2]] };
				const std::array<TerrainSample, 3> Values { V[Child[0]], V[Child[1]], V[Child[2]] };
				const std::optional<UnitDirection> Candidate = TriangleCriticalCandidate(Corners, Values);
				if(!Candidate) continue;
				++CriticalCandidates;
				#if WITH_DEV_AUTOMATION_TESTS
				LastOceanMinimaPreparation.CriticalCandidates = CriticalCandidates;
				#endif
				const TerrainSample Value = Proposal(*Candidate);
				// 有限な候補を測定出来ない場合の返戻
				if(InvalidSource) return false;
				if(Value.ElevationMeters < Sea) {
					// 実水没点の保持量を越えた場合の返戻
					const TerrainSample Actual = Source(*Candidate);
					// 候補方向を同じ元場の水没点として受理出来ない場合の返戻
					if(InvalidSource || !(Actual.ElevationMeters < Sea) || !AddWet(*Candidate)) return false;
					continue;
				}
				TerrainCriticalSettings Settings;
				Settings.RadiusMeters = Radius;
				Settings.DifferenceStepMeters = std::clamp(Radius * 5E-7, 0.0005, 500.0);
				Settings.MaximumDisplacementMeters = Radius * 0.05;
				Settings.MaximumStepMeters = Radius * 0.005;
				const std::optional<TerrainCriticalPoint> Critical = RefineTerrainCriticalPoint(*Candidate, Proposal, Settings, Stop);
				// 予算切れ又は中断後の一部成功を公開しない返戻
				if(InvalidSource) return false;
				// 補間の候補ではなく元場の実水没極小だけを追加する
				if(Critical && Critical->Kind == TerrainCriticalKind::Minimum && Critical->ElevationMeters < Sea) {
					// 追加点の保持不足の返戻
					const TerrainSample Actual = Source(Critical->Direction);
					// 地質極小を実水没として受理出来ない場合の返戻
					if(InvalidSource || !(Actual.ElevationMeters < Sea) || !AddWet(Critical->Direction)) return false;
				}
			}
		}
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::WitnessInsertion;
		#endif
		std::vector<UnitDirection> ExpandedDirections(Directions.begin(), Directions.end());
		std::vector<std::array<std::uint32_t, 3>> ExpandedTriangles(Triangles.begin(), Triangles.end());
		for(const UnitDirection Witness : WetDirections) {
			bool Added = false;
			for(std::size_t I = 0; I < ExpandedTriangles.size(); ++I) {
				const std::array<std::uint32_t, 3> T = ExpandedTriangles[I];
				const Vec3 A = ExpandedDirections[T[0]].Vector(), B = ExpandedDirections[T[1]].Vector(), C = ExpandedDirections[T[2]].Vector();
				const Vec3 P = Witness.Vector();
				if(A.Cross(B).Dot(P) < -1E-14 || B.Cross(C).Dot(P) < -1E-14 || C.Cross(A).Dot(P) < -1E-14) continue;
				// 辺上の証拠も元の場で水没を確認した微小な内側位置へ置き，共有辺を壊さず三分割する
				const Vec3 Center = UnitDirection::FromVector(A + B + C)->Vector();
				const UnitDirection Interior = *UnitDirection::FromVector(P * (1 - 1E-7) + Center * 1E-7);
				const TerrainSample Value = Source(Interior);
				// 新しい方向を元場で評価出来ない場合の返戻
				if(InvalidSource) return false;
				if(!(Value.ElevationMeters < Sea)) continue;
				const std::uint32_t Node = static_cast<std::uint32_t>(ExpandedDirections.size());
				ExpandedDirections.push_back(Interior);
				ExpandedTriangles[I] = { T[0], T[1], Node };
				ExpandedTriangles.push_back({ T[1], T[2], Node });
				ExpandedTriangles.push_back({ T[2], T[0], Node });
				Added = true;
				break;
			}
			// 実証拠を有効な同じ網へ含められない場合の返戻
			if(!Added) return false;
		}
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::MeshValidation;
		#endif
		// 元の固定高密度網と追加点を受理し，外部適応入力の既定上限は変更しない
		const bool ClosedMesh =
		IsClosedSphericalSamplingMesh(ExpandedDirections, ExpandedTriangles, Stop, Directions.size() + MaximumAddedNodes);
		// 全共有辺・向き・全球被覆を確認出来ない場合の返戻
		if(!ClosedMesh) return false;
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::GraphConnection;
		#endif
		// 共有辺と閉包を確認した拡張網の接続を公開する
		if(!ConnectPreparedMesh(Field, ExpandedDirections, ExpandedTriangles, Stop)) return false;
		Field.Stats.InputGraphNodes = static_cast<std::uint32_t>(Directions.size());
		Field.Stats.SubcellCriticalCandidates = CriticalCandidates;
		Field.Stats.SubcellOceanNodes = static_cast<std::uint32_t>(WetDirections.size());
		Field.Stats.SubcellEvaluations = Evaluations;
		#if WITH_DEV_AUTOMATION_TESTS
		LastOceanMinimaPreparation.Stage = OceanMinimaPreparationStage::Complete;
		LastOceanMinimaPreparation.Completed = true;
		#endif
		// 元場で確認した全水没標本を接続した結果の返戻
		return true;
	}
}
