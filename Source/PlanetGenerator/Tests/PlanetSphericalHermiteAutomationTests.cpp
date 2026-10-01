#include "Core/PlanetSphericalSpline.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetSphericalHermiteContracts, "PlanetGenerator.SphericalSpline.Hermite",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 指定接線の節点補間，全接合と独立な同次二次式の再現を検査する
 * @param Parameters 未使用の UE 引数
 * @return 補間契約が成立した場合に真，単調性・排水・景観の検査は別途必要
 */
bool FPlanetSphericalHermiteContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::SphericalSplineField;
	using PlanetCore::TerrainSample;
	using PlanetCore::UnitDirection;
	using PlanetCore::Vec3;
	using Triangle = std::array<std::uint32_t, 3>;
	using Edge = std::pair<std::uint32_t, std::uint32_t>;
	using Owner = std::pair<std::size_t, unsigned>;
	double MaximumValueJoin = 0, MaximumGradientJoin = 0, MaximumNodalValue = 0, MaximumNodalGradient = 0, MaximumQuadratic = 0;
	std::uint64_t Joins = 0, Nodes = 0;
	for(const unsigned Kind : { 0U, 1U }) {
		for(const std::uint8_t Level : { std::uint8_t{ 0 }, std::uint8_t{ 2 }, std::uint8_t{ 4 }, std::uint8_t{ 6 } }) {
			const std::optional<PlanetCore::SphereMesh> Mesh = PlanetCore::BuildUniformSphere(Level);
			// 正規の検査網を準備出来ない場合の返戻
			if(!TestTrue(TEXT("検査網を準備"), Mesh.has_value())) return false;
			const auto Source = [Kind](const UnitDirection Direction) -> TerrainSample {
				const Vec3 P = Direction.Vector();
				double Height;
				Vec3 Gradient;
				if(!Kind) {
					Height = 300 + 100 * P.X * P.X + 70 * P.X * P.Y - 30 * P.Y * P.Y + 40 * P.Y * P.Z + 200 * P.Z * P.Z;
					Gradient = { 200 * P.X + 70 * P.Y, 70 * P.X - 60 * P.Y + 40 * P.Z, 40 * P.Y + 400 * P.Z };
				} else {
					const double A = 1.7 * P.X + 2.3 * P.Y, B = 2 * P.Z - P.X;
					Height = 300 + 100 * std::sin(A) + 80 * std::sin(B);
					Gradient = { 170 * std::cos(A) - 80 * std::cos(B), 230 * std::cos(A), 160 * std::cos(B) };
				}
				// 別に記述した解析的な節点値と接線の返戻
				return { Height, Gradient - P * P.Dot(Gradient) };
			};
			std::vector<double> Heights;
			std::vector<Vec3> Gradients;
			for(const UnitDirection Direction : Mesh->Directions) {
				const TerrainSample Sample = Source(Direction);
				Heights.push_back(Sample.ElevationMeters);
				Gradients.push_back(Sample.TangentialGradientPerRadian);
			}
			const std::optional<SphericalSplineField> Field =
			SphericalSplineField::CreateOnMeshWithGradients(Level, Mesh->Directions, Mesh->Triangles, Heights, Gradients);
			// 指定接線の正規入力を受理出来ない場合の返戻
			if(!TestTrue(TEXT("指定接線の場を準備"), Field.has_value())) return false;
			for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
				const TerrainSample Sample = Field->Evaluate(Mesh->Directions[Index]);
				MaximumNodalValue = std::max(MaximumNodalValue, std::abs(Sample.ElevationMeters - Heights[Index]));
				MaximumNodalGradient = std::max(MaximumNodalGradient, (Sample.TangentialGradientPerRadian - Gradients[Index]).Length());
				++Nodes;
			}
			for(unsigned Index = 0; Index < 4096; ++Index) {
				const double Z = 1 - 2 * (Index + 0.5) / 4096, Angle = Index * 2.399963229728653, Ring = std::sqrt(1 - Z * Z);
				const UnitDirection Point = *UnitDirection::FromVector({ Ring * std::cos(Angle), Ring * std::sin(Angle), Z });
				if(!Kind) {
					MaximumQuadratic = std::max(MaximumQuadratic, std::abs(Field->Evaluate(Point).ElevationMeters - Source(Point).ElevationMeters));
				}
			}
			const auto Compare = [&](const std::size_t Left, const std::size_t Right, const Vec3 A, const Vec3 B) -> void {
				for(const double Fraction : { 0.23, 0.57, 0.89 }) {
					const UnitDirection Point = *UnitDirection::FromVector(A * (1 - Fraction) + B * Fraction);
					const TerrainSample First = *Field->EvaluatePiece(Left, Point), Second = *Field->EvaluatePiece(Right, Point);
					MaximumValueJoin = std::max(MaximumValueJoin, std::abs(First.ElevationMeters - Second.ElevationMeters));
					MaximumGradientJoin =
					std::max(MaximumGradientJoin, (First.TangentialGradientPerRadian - Second.TangentialGradientPerRadian).Length());
					++Joins;
				}
				// 終了
				return;
			};
			std::map<Edge, Owner> Owners;
			for(std::size_t Face = 0; Face < Mesh->Triangles.size(); ++Face) {
				const Triangle &Corners = Mesh->Triangles[Face];
				for(unsigned Side = 0; Side < 3; ++Side) {
					const std::size_t Part = 6 * Face + 2 * Side;
					const std::array<Vec3, 3> Domain = *Field->GetPieceDomain(Part);
					Compare(Part, Part + 1, Domain[1], Domain[2]);
					Compare(Part, 6 * Face + 2 * ((Side + 2) % 3) + 1, Domain[0], Domain[2]);
					const std::uint32_t A = Corners[Side], B = Corners[(Side + 1) % 3];
					const Edge Key { std::min(A, B), std::max(A, B) };
					const std::map<Edge, Owner>::const_iterator Found = Owners.find(Key);
					if(Found == Owners.end()) {
						Owners.emplace(Key, Owner{ Face, Side });
						continue;
					}
					for(unsigned Half = 0; Half < 2; ++Half) {
						const std::size_t CurrentPart = Part + Half;
						const std::uint32_t Anchor = Corners[(Side + Half) % 3];
						const unsigned OtherHalf = Mesh->Triangles[Found->second.first][Found->second.second] == Anchor ? 0 : 1;
						const std::array<Vec3, 3> Current = *Field->GetPieceDomain(CurrentPart);
						Compare(CurrentPart, 6 * Found->second.first + 2 * Found->second.second + OtherHalf, Current[0], Current[1]);
					}
					Owners.erase(Found);
				}
			}
			TestTrue(TEXT("未接続の境界を残さない"), Owners.empty());
			if(!Level && !Kind) {
				TestFalse(
					TEXT("指定接線の欠落を拒否"),
					SphericalSplineField::CreateOnMeshWithGradients(Level, Mesh->Directions, Mesh->Triangles, Heights, {}).has_value()
				);
				std::stop_source Cancelled;
				Cancelled.request_stop();
				TestFalse(
					TEXT("指定接線の準備中断を受理"),
					SphericalSplineField::CreateOnMeshWithGradients(
						Level,
						Mesh->Directions,
						Mesh->Triangles,
						Heights,
						Gradients,
						Cancelled.get_token()
					).has_value()
				);
				Gradients[0].X = std::numeric_limits<double>::quiet_NaN();
				TestFalse(
					TEXT("非有限な指定接線を拒否"),
					SphericalSplineField::CreateOnMeshWithGradients(Level, Mesh->Directions, Mesh->Triangles, Heights, Gradients).has_value()
				);
			}
		}
	}
	TestTrue(TEXT("元節点の値を補間"), MaximumNodalValue < 1E-6);
	TestTrue(TEXT("元節点の接線を補間"), MaximumNodalGradient < 1E-6);
	TestTrue(TEXT("全内部辺と元辺の値を接続"), MaximumValueJoin < 1E-6);
	TestTrue(TEXT("全内部辺と元辺の接線を接続"), MaximumGradientJoin < 1E-6);
	TestTrue(TEXT("独立な同次二次関数を再現"), MaximumQuadratic < 1E-6);
	AddInfo(
		FString::Printf(
			TEXT("Spherical Hermite: nodes=%llu joins=%llu nodalValue=%.12g nodalGradient=%.12g ")
			TEXT("valueJoin=%.12g gradientJoin=%.12g quadratic=%.12g"),
			static_cast<unsigned long long>(Nodes),
			static_cast<unsigned long long>(Joins),
			MaximumNodalValue,
			MaximumNodalGradient,
			MaximumValueJoin,
			MaximumGradientJoin,
			MaximumQuadratic
		)
	);
	// 指定接線・接続・独立二次式の検査結果の返戻
	return !HasAnyErrors();
}

#endif
