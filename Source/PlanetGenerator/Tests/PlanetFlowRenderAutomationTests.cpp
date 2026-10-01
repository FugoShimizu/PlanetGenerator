#include "Misc/AutomationTest.h"
#include "Core/PlanetGround.h"
#include "PlanetSphericalSurfacePreview.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "ProceduralMeshComponent.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FPlanetFlowRenderContracts,
	"PlanetGenerator.FlowChannel.RenderMesh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter
)

/**
 * 実際に公開する局所メッシュを，生成器と異なる内部点及び float 精度で検査する
 * @param Parameters 未使用の実行引数
 * @return 解析場との２センチメートル比較と後始末を満たす場合に真
 */
bool FPlanetFlowRenderContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	UWorld *World = nullptr;
	if(GEngine) for(const FWorldContext &Context : GEngine->GetWorldContexts()) {
		if(Context.WorldType != EWorldType::Editor) continue;
		World = Context.World();
		break;
	}
	// エディタの正規の World が無い場合の返戻
	if(!TestNotNull(TEXT("検証する World が存在"), World)) return false;
	// 仮水位を使った既存経路と，実際の排水網から決める経路の独立比較
	for(const bool Connected : { false, true }) {
		FActorSpawnParameters Spawn;
		Spawn.ObjectFlags |= RF_Transient;
		const auto Destroy = [](APlanetSphericalSurfacePreview *const Actor) -> void {
			if(Actor) Actor->Destroy();
		};
		const std::unique_ptr<APlanetSphericalSurfacePreview, decltype(Destroy)> Actor(
			World->SpawnActor<APlanetSphericalSurfacePreview>(FVector::ZeroVector, FRotator::ZeroRotator, Spawn),
			Destroy
		);
		// 一時アクターを生成出来ない場合の返戻
		if(!TestNotNull(TEXT("一時的な地形を生成"), Actor.get())) return false;
		const FVector Pass(0.54180127453542282, 0.81192128360203597, -0.21733662403243018);
		constexpr double Radius = 1E6;
		const bool HasSource = Actor->BuildPreview(42, Radius, Pass, 200, 16, false, false, false, true);
		// 比較元が準備出来ない結果への返戻
		if(!TestTrue(TEXT("変更しない元地形を準備"), HasSource)) return false;
		// 同じ場の上流側と受入湖側を検査し，水系の再準備も確認
		for(const bool ReceivingView : { false, true }) {
			if(ReceivingView && !Connected) continue;
			const bool HasMesh = Connected ?
			Actor->BuildConnectedFlowChannelPreview(Pass, ReceivingView ? Actor->DiagnosticRouteEndMeters - 20 : -100, 256, 96) :
			Actor->BuildFlowChannelPreview(Pass, -100, 681.98759, 256, 96);
			AddInfo(Actor->LastBuildMessage);
			// 予算又は精度の拒否を無視しない返戻
			if(!TestTrue(TEXT("湖と流出口を含む広い局所面を公開"), HasMesh)) return false;
			if(Connected) {
				TestTrue(TEXT("受入湖の実際の岸を表示と共有"), Actor->DiagnosticReceivingShoreVertices > 2048);
				if(ReceivingView) TestTrue(TEXT("視点変更で同じ水系を再利用"), Actor->LastBuildMessage.Contains(TEXT("reused=1")));
				TestEqual(TEXT("大洋を含む確認済の水系を使用"), Actor->DiagnosticConnectedSites, 43);
				TestTrue(TEXT("受入水位を実際の下流側流出口から決定"), std::abs(Actor->DiagnosticReceivingWaterMeters - 552.9523376) < 1E-6);
				TestTrue(TEXT("上流の流出口から下流へ水位が下降"), Actor->DiagnosticOutletWaterMeters > Actor->DiagnosticReceivingWaterMeters);
			}
			const FVector Anchor = Actor->GetDiagnosticAnchorDirection();
			const PlanetCore::UnitDirection Direction = *PlanetCore::UnitDirection::FromVector({ Anchor.X, Anchor.Y, Anchor.Z });
			const PlanetCore::GroundFrame Frame = *PlanetCore::GroundFrame::Create(Direction, Radius, Actor->AnchorElevationMeters);
			TArray<UProceduralMeshComponent *> Components;
			Actor->GetComponents(Components);
			std::uint32_t Meshes = 0;
			const std::array<std::array<double, 3>, 7> Weights {
				{
					{ 1.0 / 3, 1.0 / 3, 1.0 / 3 },
					{ 0.5, 0.25, 0.25 },
					{ 0.25, 0.5, 0.25 },
					{ 0.25, 0.25, 0.5 },
					{ 0.6, 0.3, 0.1 },
					{ 0.1, 0.6, 0.3 },
					{ 0.3, 0.1, 0.6 }
				}
			};
			for(UProceduralMeshComponent *const Component : Components) {
				const bool IsWater = Component->GetFName() == TEXT("ContourWaterDiagnostic");
				if(!IsWater && Component->GetFName() != TEXT("SphericalSurfaceDiagnostic")) continue;
				FProcMeshSection *const Section = Component->GetProcMeshSection(0);
				const bool HasVertices = Section && Section->ProcVertexBuffer.Num() && Section->ProcIndexBuffer.Num();
				// 空の描画面を成功としない返戻
				if(!TestTrue(TEXT("公開面に実頂点と三角形が存在"), HasVertices)) return false;
				std::uint64_t Samples = 0;
				double MaximumError = 0, MaximumDryDepth = 0, MaximumOutsideSupport = 0;
				std::vector<std::vector<std::uint32_t>> Adjacency(static_cast<std::size_t>(Section->ProcVertexBuffer.Num()));
				for(int32 i = 0; i < Section->ProcIndexBuffer.Num(); i += 3) {
					if(IsWater) for(unsigned Edge = 0; Edge < 3; ++Edge) {
						const std::uint32_t A = Section->ProcIndexBuffer[i + Edge], B = Section->ProcIndexBuffer[i + (Edge + 1) % 3];
						Adjacency[A].push_back(B);
						Adjacency[B].push_back(A);
					}
					std::array<PlanetCore::Vec3, 3> Points;
					for(unsigned j = 0; j < 3; ++j) {
						const FVector P = Section->ProcVertexBuffer[Section->ProcIndexBuffer[i + j]].Position;
						Points[j] = {
							static_cast<double>(static_cast<float>(P.X)) * 0.01,
							static_cast<double>(static_cast<float>(P.Y)) * 0.01,
							static_cast<double>(static_cast<float>(P.Z)) * 0.01
						};
					}
					for(const std::array<double, 3> &Weight : Weights) {
						const PlanetCore::Vec3 P = Points[0] * Weight[0] + Points[1] * Weight[1] + Points[2] * Weight[2];
						const PlanetCore::UnitDirection Query = *PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(P));
						double Terrain = 0, Water = 0, Coverage = 0, Original = 0;
						const PlanetCore::Vec3 Q = Query.Vector();
						const bool HasSample = Actor->EvaluateFlowDirection(FVector(Q.X, Q.Y, Q.Z), Terrain, Water, Coverage, Original);
						// 内部標本の欠落を乾いた点として除外しない返戻
						if(!TestTrue(TEXT("三角形内部でも同じ場を評価出来る"), HasSample)) return false;
						const double Height = Frame.LocalToPlanet(P).Length() - Radius;
						MaximumError = std::max(MaximumError, std::abs(Height - (IsWater ? Water : Terrain)));
						if(IsWater) {
							double Support = 0;
							// 公開水面の所属を同じ独立標本で検査する
							if(!TestTrue(TEXT("水面の所属を照会出来る"), Actor->EvaluateFlowSupport(FVector(Q.X, Q.Y, Q.Z), Support))) return false;
							MaximumDryDepth = std::max(MaximumDryDepth, Terrain - Water);
							MaximumOutsideSupport = std::max(MaximumOutsideSupport, -Support);
						}
						++Samples;
					}
				}
				AddInfo(
					FString::Printf(
						TEXT("Flow render mesh: water=%d vertices=%d triangles=%d samples=%llu maximumError=%.12g meshMs=%.9g"),
						static_cast<int>(IsWater),
						Section->ProcVertexBuffer.Num(),
						Section->ProcIndexBuffer.Num() / 3,
						static_cast<unsigned long long>(Samples),
						MaximumError,
						Actor->MeshMilliseconds
					)
				);
				TestTrue(TEXT("独立な内部点と描画精度で２センチメートル未満"), MaximumError < 0.02);
				if(IsWater) {
					// 描画へ渡す法線を，式を共有しない２方向の高さ差分で確認
					double MaximumNormalAngle = 0;
					unsigned NormalSamples = 0;
					const int32 Stride = std::max(1, Section->ProcVertexBuffer.Num() / 256);
					for(int32 Vertex = 0; Vertex < Section->ProcVertexBuffer.Num(); Vertex += Stride) {
						const FProcMeshVertex &V = Section->ProcVertexBuffer[Vertex];
						const PlanetCore::Vec3 P { V.Position.X * 0.01, V.Position.Y * 0.01, V.Position.Z * 0.01 };
						const PlanetCore::UnitDirection Q = *PlanetCore::UnitDirection::FromVector(Frame.LocalToPlanet(P));
						const PlanetCore::Vec3 N = Q.Vector();
						const PlanetCore::Vec3 East = PlanetCore::UnitDirection::FromVector(Frame.East - N * Frame.East.Dot(N))->Vector();
						const PlanetCore::Vec3 North = N.Cross(East);
						PlanetCore::Vec3 Gradient {};
						for(const PlanetCore::Vec3 Tangent : { East, North }) {
							std::array<double, 2> Differences {};
							for(unsigned Scale = 0; Scale < 2; ++Scale) {
								const double Step = Scale ? 0.01 : 0.02, Angle = Step / Radius;
								std::array<double, 2> Heights {};
								for(unsigned Side = 0; Side < 2; ++Side) {
									const PlanetCore::Vec3 D = N * std::cos(Angle) + Tangent * (std::sin(Angle) * (Side ? 1.0 : -1.0));
									double Terrain = 0, Coverage = 0, Original = 0;
									const bool HasSample = Actor->EvaluateFlowDirection(FVector(D.X, D.Y, D.Z), Terrain, Heights[Side], Coverage, Original);
									// 微分比較の片側欠落を無視しない返戻
									if(!TestTrue(TEXT("水面の両側差分を評価出来る"), HasSample)) return false;
								}
								Differences[Scale] = (Heights[1] - Heights[0]) / (2 * Angle);
							}
							Gradient = Gradient + Tangent * ((4 * Differences[1] - Differences[0]) / 3);
						}
						const double Height = Frame.LocalToPlanet(P).Length() - Radius;
						const PlanetCore::Vec3 Expected =
						Frame.VectorToLocal(PlanetCore::UnitDirection::FromVector(N - Gradient * (1 / (Radius + Height)))->Vector());
						const PlanetCore::Vec3 Rendered = PlanetCore::UnitDirection::FromVector({ V.Normal.X, V.Normal.Y, V.Normal.Z })->Vector();
						MaximumNormalAngle = std::max(MaximumNormalAngle, std::atan2(Expected.Cross(Rendered).Length(), Expected.Dot(Rendered)));
						++NormalSamples;
					}
					TestTrue(TEXT("水面の解析法線と独立な２方向差分が一致"), MaximumNormalAngle < 1E-4);
					AddInfo(FString::Printf(TEXT("Water normals: samples=%u maximumAngle=%.12g rad"), NormalSamples, MaximumNormalAngle));
					std::vector<bool> Seen(Adjacency.size());
					unsigned ComponentsCount = 0;
					for(std::size_t Start = 0; Start < Adjacency.size(); ++Start) {
						if(Seen[Start] || Adjacency[Start].empty()) continue;
						++ComponentsCount;
						std::vector<std::uint32_t> Pending { static_cast<std::uint32_t>(Start) };
						Seen[Start] = true;
						while(!Pending.empty()) {
							const std::uint32_t Vertex = Pending.back();
							Pending.pop_back();
							for(const std::uint32_t Next : Adjacency[Vertex]) if(!Seen[Next]) {
								Seen[Next] = true;
								Pending.push_back(Next);
							}
						}
					}
					TestEqual(TEXT("表示範囲内の湖と流出口は一つの水面"), ComponentsCount, 1U);
					TestTrue(TEXT("陸上への漏れを高さ精度内に制限"), MaximumDryDepth < 0.02);
					TestTrue(TEXT("水面の所属も岸の５センチメートル精度内"), MaximumOutsideSupport < 0.05);
					AddInfo(
						FString::Printf(
							TEXT("Water domain: components=%u maximumDryDepth=%.12g maximumOutsideSupport=%.12g"),
							ComponentsCount,
							MaximumDryDepth,
							MaximumOutsideSupport
						)
					);
				}
				++Meshes;
			}
			TestEqual(TEXT("地面と水面を共に検査"), Meshes, 2U);
		}
	}
	// 所有する一時 Actor は全ての返戻経路で破棄し，検査結果を返す
	return !HasAnyErrors();
}

#endif
