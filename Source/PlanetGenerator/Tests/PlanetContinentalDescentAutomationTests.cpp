#include "Core/PlanetContinentalDescent.h"
#include "Core/PlanetContinentalPotential.h"
#include "Misc/AutomationTest.h"
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stop_token>
#include <vector>

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlanetContinentalDescentContracts, "PlanetGenerator.ContinentalDescent.Contracts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

/**
 * 近傍微分の上界と，山頂・鞍点も含む円板内の下降経路を検査する
 * @param Parameters 未使用の UE 引数
 * @return 区間の下降と数値的海岸が成立した場合に真，最終の河川景観とは別
 */
bool FPlanetContinentalDescentContracts::RunTest(const FString &Parameters) {
	static_cast<void>(Parameters);
	using PlanetCore::ContinentalDescentControl;
	using PlanetCore::ContinentalDescentError;
	using PlanetCore::ContinentalDescentPath;
	using PlanetCore::ContinentalDescentPoint;
	using PlanetCore::ContinentalDescentSegment;
	using PlanetCore::ContinentalDescentSettings;
	using PlanetCore::ContinentalPotential;
	using PlanetCore::ContinentalPotentialDerivativeBounds;
	using PlanetCore::ContinentalPotentialSample;
	using PlanetCore::ContinentalPotentialSettings;
	const std::chrono::steady_clock::time_point Started = std::chrono::steady_clock::now();
	const double Epsilon = std::numeric_limits<double>::epsilon();
	std::uint64_t Paths = 0, Segments = 0, InteriorSamples = 0, BoundsSamples = 0, CurvatureEscapes = 0, BoundsEvaluations = 0;
	double MaximumEndpointResidual = 0, MaximumEndpointHeight = 0, MaximumDerivativeViolation = 0, MaximumCurvatureViolation = 0;
	double MaximumHessianRatio = 0, MaximumHessianVariationRatio = 0;
	for(const double Scale : { 0.001, 1.0, 1000.0 }) for(unsigned Model = 0; Model < 4; ++Model) {
		// 解析的な山頂，対称な鞍点，非対称な細かい隆起を別に用意
		ContinentalPotentialSettings Settings;
		Settings.BackgroundMeters = 25 * Scale;
		if(Model == 1) Settings.Sources.push_back({ 0, 0, 0.07, 100 * Scale });
		if(Model == 2) {
			Settings.Sources.push_back({ -0.35, 0, 0.05, 100 * Scale });
			Settings.Sources.push_back({ 0.35, 0, 0.05, 100 * Scale });
		}
		if(Model == 3) {
			Settings.Sources.push_back({ 0.45, 0.10, 0.003, 140 * Scale });
			Settings.Sources.push_back({ -0.35, 0.25, 0.006, 100 * Scale });
			Settings.Sources.push_back({ 0.05, -0.45, 0.005, 90 * Scale });
			for(unsigned i = 0; i < 32; ++i) {
				const double Radius = 0.78 * std::sqrt((i + 0.5) / 32), Angle = i * 2.399963229728653, Width = i % 2 ? 0.008 : 0.003;
				const double Weight = (2.0 + i % 7) * Scale;
				Settings.Sources.push_back({ Radius * std::cos(Angle), Radius * std::sin(Angle), Width, Weight });
			}
		}
		const std::optional<ContinentalPotential> Potential = ContinentalPotential::Create(Settings);
		// 正規な検査地形を準備出来ない場合の返戻
		if(!TestTrue(TEXT("異なる尺度の正規地形を受理"), Potential.has_value())) return false;
		const double HeightBound = Potential->GetAbsoluteHeightBoundMeters(), MachineMargin = 1024 * Epsilon * HeightBound;
		if(Model == 2) {
			const std::optional<ContinentalPotentialSample> Saddle = Potential->Evaluate(0, 0);
			TestTrue(
				TEXT("制御地形の原点は解像された鞍点"),
				Saddle && std::hypot(Saddle->GradientPerUnit[0], Saddle->GradientPerUnit[1]) < 1E-12 * Scale &&
				Saddle->HessianPerUnitSquared[0] > 0 && Saddle->HessianPerUnitSquared[2] < 0
			);
		}
		// 上界式と異なる位置で，Hessian の作用素ノルムと変化を直接比較
		for(unsigned i = 0; i < 96; ++i) {
			const double Radius = 0.95 * std::sqrt((i + 0.5) / 96), Angle = i * 2.399963229728653, X = Radius * std::cos(Angle);
			const double Y = Radius * std::sin(Angle);
			constexpr double Neighborhood = 0.02;
			const std::optional<ContinentalPotentialSample> Center = Potential->Evaluate(X, Y);
			const std::optional<ContinentalPotentialDerivativeBounds> Bounds = Potential->GetDerivativeBounds(X, Y, Neighborhood);
			// 上界又は独立位置の微分を解像出来ない場合の返戻
			if(!TestTrue(TEXT("近傍の有限な微分上界を受理"), Center && Bounds)) return false;
			for(unsigned j = 0; j < 8; ++j) {
				const double OffsetAngle = j * 0.7853981633974483, DX = Neighborhood * std::cos(OffsetAngle);
				const double DY = Neighborhood * std::sin(OffsetAngle);
				const std::optional<ContinentalPotentialSample> Neighbor = Potential->Evaluate(X + DX, Y + DY);
				if(!Neighbor) continue;
				const auto Norm = [](const std::array<double, 3> Matrix) -> double {
					const double Middle = 0.5 * (Matrix[0] + Matrix[2]), Spread = std::hypot(0.5 * (Matrix[0] - Matrix[2]), Matrix[1]);
					// 対称２行列の両固有値から得た作用素ノルムの返戻
					return std::max(std::abs(Middle - Spread), std::abs(Middle + Spread));
				};
				const std::array<double, 3> Change {
					Neighbor->HessianPerUnitSquared[0] - Center->HessianPerUnitSquared[0],
					Neighbor->HessianPerUnitSquared[1] - Center->HessianPerUnitSquared[1],
					Neighbor->HessianPerUnitSquared[2] - Center->HessianPerUnitSquared[2]
				};
				const double HessianNorm = Norm(Neighbor->HessianPerUnitSquared), Variation = Norm(Change);
				MaximumHessianRatio = std::max(MaximumHessianRatio, HessianNorm / Bounds->HessianNormPerUnitSquared);
				MaximumHessianVariationRatio = std::max(
					MaximumHessianVariationRatio,
					Variation / std::max(std::numeric_limits<double>::min(), Bounds->ThirdDerivativeNormPerUnitCubed * Neighborhood)
				);
				TestTrue(TEXT("独立な近傍の Hessian が上界内"), HessianNorm <= Bounds->HessianNormPerUnitSquared);
				TestTrue(TEXT("近傍の Hessian 変化が３階の上界内"), Variation <= Bounds->ThirdDerivativeNormPerUnitCubed * Neighborhood + MachineMargin);
				++BoundsSamples;
			}
		}
		for(unsigned i = 0; i < 20; ++i) {
			std::array<double, 2> Start {};
			if(i == 1) Start = { 1E-12, -1E-13 };
			if(i > 1) {
				const double Radius = 0.93 * std::sqrt((i - 1.5) / 18), Angle = i * 2.399963229728653;
				Start = { Radius * std::cos(Angle), Radius * std::sin(Angle) };
			}
			ContinentalDescentSettings Controls;
			ContinentalDescentError Error = ContinentalDescentError::None;
			const std::optional<ContinentalDescentPath> Path = ContinentalDescentPath::Create(*Potential, Start, Controls, Error);
			const bool HasReachedCoast = TestTrue(
				FString::Printf(
					TEXT("下降経路が海岸へ到達 Model=%u Scale=%.6g start=(%.12g,%.12g) reason=%u"),
					Model,
					Scale,
					Start[0],
					Start[1],
					static_cast<unsigned>(Error)
				),
				Path.has_value()
			);
			// 未到達の失敗と其の入力を保存した検査結果の返戻
			if(!HasReachedCoast) return false;
			const std::span<const ContinentalDescentPoint> Points = Path->GetPoints();
			const std::span<const ContinentalDescentSegment> Edges = Path->GetSegments();
			TestTrue(TEXT("完成経路の点と区間の個数が一致"), !Points.empty() && Points.size() == Edges.size() + 1);
			TestTrue(TEXT("経路の資源上限を保持"), Edges.size() <= Controls.MaximumSegments);
			if(i < 2 && Model < 3) TestTrue(TEXT("零又は極小勾配の開始点では負曲率を使う"), Path->GetStatistics().CurvatureEscapes > 0);
			for(std::size_t j = 0; j < Edges.size(); ++j) {
				const ContinentalDescentPoint &Before = Points[j], &After = Points[j + 1];
				const ContinentalDescentSegment &Edge = Edges[j];
				const double Length = std::hypot(After.X - Before.X, After.Y - Before.Y), HX = (After.X - Before.X) / Length;
				const double HY = (After.Y - Before.Y) / Length, DerivativeUpper = Edge.DirectionalDerivativeUpperBound;
				TestTrue(TEXT("全終点で元のポテンシャルが厳密に減少"), After.PotentialHeightMeters < Before.PotentialHeightMeters);
				TestTrue(TEXT("区間の長さを同じ座標から再構成"), Length == Edge.LengthPerUnit);
				TestTrue(TEXT("区間全体の微分上界が非正"), DerivativeUpper <= 0);
				if(Edge.Control == ContinentalDescentControl::Gradient) {
					TestTrue(TEXT("勾配区間は全体で厳密な下降"), Edge.DirectionalDerivativeUpperBound < 0);
				} else TestTrue(TEXT("曲率区間は全体で厳密に下向きの曲率"), Edge.DirectionalCurvatureUpperBound < 0);
				double PreviousHeight = Before.PotentialHeightMeters;
				for(unsigned k = 0; k < 17; ++k) {
					const double Fraction = k / 16.0, X = (1 - Fraction) * Before.X + Fraction * After.X;
					const double Y = (1 - Fraction) * Before.Y + Fraction * After.Y;
					const std::optional<ContinentalPotentialSample> Sample = Potential->Evaluate(X, Y);
					// 節点だけの成功で区間内の照会失敗を隠さない返戻
					if(!TestTrue(TEXT("区間内の独立な照会が有限"), Sample.has_value())) return false;
					const double Derivative = Sample->GradientPerUnit[0] * HX + Sample->GradientPerUnit[1] * HY;
					const double Curvature = Sample->HessianPerUnitSquared[0] * HX * HX + 2 * Sample->HessianPerUnitSquared[1] * HX * HY +
					Sample->HessianPerUnitSquared[2] * HY * HY;
					MaximumDerivativeViolation = std::max(MaximumDerivativeViolation, Derivative - Edge.DirectionalDerivativeUpperBound);
					TestTrue(TEXT("節点外の微分も下降証明値の範囲内"), Derivative <= Edge.DirectionalDerivativeUpperBound + MachineMargin);
					if(Edge.Control == ContinentalDescentControl::NegativeCurvature) {
						MaximumCurvatureViolation = std::max(MaximumCurvatureViolation, Curvature - Edge.DirectionalCurvatureUpperBound);
						TestTrue(TEXT("負曲率の区間も節点外で上界内"), Curvature <= Edge.DirectionalCurvatureUpperBound + MachineMargin);
					}
					if(k) TestTrue(TEXT("独立な区間内標本の高さも非増加"), Sample->HeightMeters <= PreviousHeight);
					PreviousHeight = Sample->HeightMeters;
					++InteriorSamples;
				}
			}
			const std::optional<ContinentalDescentPath> Again = ContinentalDescentPath::Create(*Potential, Start, Controls, Error);
			// 再準備の正規経路に失敗した場合の返戻
			if(!TestTrue(TEXT("同じ経路を独立に再準備"), Again.has_value())) return false;
			TestTrue(
				TEXT("経路の識別子と点の個数が再現"),
				Path->GetDefinitionHash() == Again->GetDefinitionHash() && Points.size() == Again->GetPoints().size()
			);
			for(std::size_t j = 0; j < Points.size(); ++j) {
				TestTrue(
					TEXT("同じ開始点の全座標と高さがビット一致"),
					std::bit_cast<std::uint64_t>(Points[j].X) == std::bit_cast<std::uint64_t>(Again->GetPoints()[j].X) &&
					std::bit_cast<std::uint64_t>(Points[j].Y) == std::bit_cast<std::uint64_t>(Again->GetPoints()[j].Y) &&
					std::bit_cast<std::uint64_t>(Points[j].PotentialHeightMeters) ==
					std::bit_cast<std::uint64_t>(Again->GetPoints()[j].PotentialHeightMeters)
				);
			}
			MaximumEndpointResidual = std::max(MaximumEndpointResidual, Path->GetStatistics().EndBoundaryResidual);
			MaximumEndpointHeight = std::max(MaximumEndpointHeight, Path->GetStatistics().EndPotentialHeightMeters / Scale);
			CurvatureEscapes += Path->GetStatistics().CurvatureEscapes;
			BoundsEvaluations += Path->GetStatistics().BoundsEvaluations;
			Segments += Edges.size();
			++Paths;
		}
	}
	// 拒否経路だけでなく，既に海岸に在る正規経路も検査
	ContinentalPotentialSettings Settings;
	Settings.BackgroundMeters = 25;
	const std::optional<ContinentalPotential> Potential = ContinentalPotential::Create(Settings);
	// 最後の入力比較を準備出来ない場合の返戻
	if(!TestTrue(TEXT("入力比較の地形を受理"), Potential.has_value())) return false;
	ContinentalDescentSettings Controls;
	ContinentalDescentError Error = ContinentalDescentError::None;
	const std::optional<ContinentalDescentPath> Coast = ContinentalDescentPath::Create(*Potential, { 1, 0 }, Controls, Error);
	TestTrue(TEXT("海岸の開始点は空でない完成経路"), Coast && Coast->GetPoints().size() == 1 && Coast->GetSegments().empty());
	TestFalse(TEXT("円板外の開始点を拒否"), ContinentalDescentPath::Create(*Potential, { 1.001, 0 }, Controls, Error).has_value());
	TestTrue(TEXT("円板外の拒否理由を保持"), Error == ContinentalDescentError::InvalidStart);
	TestFalse(
		TEXT("非有限な開始点を拒否"),
		ContinentalDescentPath::Create(*Potential, { std::numeric_limits<double>::quiet_NaN(), 0 }, Controls, Error).has_value()
	);
	Controls.MaximumSegments = 1;
	TestFalse(TEXT("未到達の経路を上限成功として扱わない"), ContinentalDescentPath::Create(*Potential, { 0, 0 }, Controls, Error).has_value());
	TestTrue(TEXT("資源上限の拒否理由を保持"), Error == ContinentalDescentError::SegmentBudget);
	Controls.MaximumSegments = 0;
	TestFalse(TEXT("区間上限０を拒否"), ContinentalDescentPath::Create(*Potential, { 0, 0 }, Controls, Error).has_value());
	Controls.MaximumSegments = 4096;
	Controls.MaximumSegmentLength = 0;
	TestFalse(TEXT("区間長０を拒否"), ContinentalDescentPath::Create(*Potential, { 0, 0 }, Controls, Error).has_value());
	Controls.MaximumSegmentLength = 0.04;
	std::stop_source Cancellation;
	Cancellation.request_stop();
	TestFalse(
		TEXT("中断した経路を公開しない"),
		ContinentalDescentPath::Create(*Potential, { 0, 0 }, Controls, Error, Cancellation.get_token()).has_value()
	);
	TestTrue(TEXT("中断の理由を保持"), Error == ContinentalDescentError::Cancelled);
	TestFalse(TEXT("近傍半径が負の上界を拒否"), Potential->GetDerivativeBounds(0, 0, -0.001).has_value());
	TestFalse(TEXT("非有限な近傍上界を拒否"), Potential->GetDerivativeBounds(0, 0, std::numeric_limits<double>::quiet_NaN()).has_value());
	TestFalse(TEXT("円板外の微分上界を拒否"), Potential->GetDerivativeBounds(1.001, 0, 0.04).has_value());
	AddInfo(
		FString::Printf(
			TEXT("Continental descent: paths=%llu segments=%llu interior=%llu bounds=%llu escapes=%llu boundEvaluations=%llu ")
			TEXT("coastResidual=%.12g scaledCoastHeight=%.12g derivativeViolation=%.12g curvatureViolation=%.12g ")
			TEXT("hessianRatio=%.12g variationRatio=%.12g seconds=%.7f. Disk paths only; sphere, river surfaces and scenery are pending."),
			static_cast<unsigned long long>(Paths),
			static_cast<unsigned long long>(Segments),
			static_cast<unsigned long long>(InteriorSamples),
			static_cast<unsigned long long>(BoundsSamples),
			static_cast<unsigned long long>(CurvatureEscapes),
			static_cast<unsigned long long>(BoundsEvaluations),
			MaximumEndpointResidual,
			MaximumEndpointHeight,
			MaximumDerivativeViolation,
			MaximumCurvatureViolation,
			MaximumHessianRatio,
			MaximumHessianVariationRatio,
			std::chrono::duration<double>(std::chrono::steady_clock::now() - Started).count()
		)
	);
	// 下降の上界，数値境界，再現性，許可・拒否経路の検査結果の返戻
	return !HasAnyErrors();
}

#endif
