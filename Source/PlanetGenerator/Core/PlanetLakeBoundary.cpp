#include "PlanetLakeBoundary.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <stop_token>
#include <tuple>
#include <utility>
#include <vector>

namespace PlanetCore {
	/**
	 * 同水位の鞍点の同じ側へ着く岸の弧を組合せ，個別に単純な閉輪郭へする
	 * @param Passes 弧の終点番号と同じ配列の確認済鞍点
	 * @param Arcs TraceSpillLakeArc が返した，開始鞍点を含む有向弧
	 * @param Settings 半径・高さ精度と全入力頂点の保持予算；水位は鞍点の値を用いる
	 * @param Stop 有界な走査と輪郭検査間の中断要求
	 * @return 指定弧を一度ずつ含む閉輪郭；欠落・重複・交差・投影外・予算超過時は空
	 * @note 各輪郭の内外関係，島の完全列挙，零水深の鞍点の有限幅通水は別の契約
	 */
	std::optional<std::vector<LakeBoundaryLoop>> AssembleLakeBoundary(
		const std::span<const TerrainCriticalPoint> Passes,
		const std::span<const TerrainContour> Arcs,
		const TerrainContourSettings &Settings,
		const std::stop_token Stop
	) {
		const bool Valid =
		!Passes.empty() && Passes.size() < 1025 && !Arcs.empty() && Arcs.size() <= 2 * Passes.size() && Settings.IsValid();
		// 無効な入口又は中断への返戻
		if(!Valid || Stop.stop_requested()) return {};
		using Position = std::tuple<double, double, double>;
		std::map<Position, std::uint32_t> PassIndices;
		std::vector<Vec3> Axes;
		constexpr std::uint32_t Missing = std::numeric_limits<std::uint32_t>::max();
		const double Water = Passes[0].ElevationMeters;
		for(std::uint32_t Index = 0; Index < Passes.size(); ++Index) {
			const TerrainCriticalPoint &Pass = Passes[Index];
			const Vec3 D = Pass.Direction.Vector();
			const std::optional<UnitDirection> Axis =
			UnitDirection::FromVector(Pass.MinimumCurvatureDirection - D * Pass.MinimumCurvatureDirection.Dot(D));
			const bool HasPass =
			Pass.Kind == TerrainCriticalKind::Saddle && Axis && std::isfinite(Pass.ElevationMeters) && Pass.ElevationMeters == Water &&
			PassIndices.emplace(Position{ D.X, D.Y, D.Z }, Index).second;
			// 別水位・重複・主方向を失った鞍点への返戻
			if(!HasPass || Stop.stop_requested()) return {};
			Axes.push_back(Axis->Vector());
		}
		std::vector<std::uint32_t> Outgoing(Passes.size() * 2, Missing), EndKeys;
		std::size_t TotalVertices = 0;
		for(std::uint32_t Index = 0; Index < Arcs.size(); ++Index) {
			const TerrainContour &Arc = Arcs[Index];
			const bool HasArc =
			Arc.End == TerrainContourEnd::Terminal && Arc.TerminalIndex && *Arc.TerminalIndex < Passes.size() && Arc.Samples.size() > 3;
			// 部分経路又は未定義の到達点を，短い線分で閉じない返戻
			if(!HasArc || Stop.stop_requested()) return {};
			// 準備済の弧であっても全保持量を越える場合の返戻
			if(Arc.Samples.size() > Settings.MaximumVertices - TotalVertices) return {};
			TotalVertices += Arc.Samples.size();
			const Vec3 First = Arc.Samples.front().Direction.Vector(), Last = Arc.Samples.back().Direction.Vector();
			const std::map<Position, std::uint32_t>::const_iterator Start = PassIndices.find({ First.X, First.Y, First.Z });
			const std::uint32_t End = *Arc.TerminalIndex;
			// 実際の端点と鞍点番号が一致しない入力への返戻
			if(Start == PassIndices.end() || (Last - Passes[End].Direction.Vector()).Length()) return {};
			const double StartSide = (Arc.Samples[1].Direction.Vector() - First).Dot(Axes[Start->second]);
			const double EndSide = (Arc.Samples[Arc.Samples.size() - 2].Direction.Vector() - Last).Dot(Axes[End]);
			// 入出枝の側を識別出来ない入力への返戻
			if(!std::isfinite(StartSide) || !std::isfinite(EndSide) || !StartSide || !EndSide) return {};
			const std::uint32_t StartKey = 2 * Start->second + (StartSide > 0 ? 1U : 0U), EndKey = 2 * End + (EndSide > 0 ? 1U : 0U);
			// 同じ側から出る二つの弧を重複した岸として返さない返戻
			if(Outgoing[StartKey] != Missing || Arc.Samples.front().DistanceMeters) return {};
			Outgoing[StartKey] = Index;
			EndKeys.push_back(EndKey);
			double Previous = -1;
			for(const TerrainContourSample &Sample : Arc.Samples) {
				const bool HasSample =
				std::isfinite(Sample.DistanceMeters) && Sample.DistanceMeters > Previous && std::isfinite(Sample.ElevationMeters) &&
				std::abs(Sample.ElevationMeters - Water) <= Settings.HeightToleranceMeters;
				// 不正な弧長又は別水位の標本への返戻
				if(!HasSample || Stop.stop_requested()) return {};
				Previous = Sample.DistanceMeters;
			}
		}
		std::vector<std::uint32_t> Next, InDegree(Arcs.size(), 0);
		for(const std::uint32_t Key : EndKeys) {
			const std::uint32_t Target = Outgoing[Key];
			// 欠落又は合流して閉輪郭にならない岸を拒否する返戻
			if(Target == Missing || ++InDegree[Target] != 1 || Stop.stop_requested()) return {};
			Next.push_back(Target);
		}
		std::vector<LakeBoundaryLoop> Result;
		std::vector<bool> Used(Arcs.size(), false);
		double TotalLength = 0;
		for(std::uint32_t Start = 0; Start < Arcs.size(); ++Start) {
			if(Used[Start]) continue;
			LakeBoundaryLoop Loop;
			Loop.Contour.End = TerrainContourEnd::Closed;
			std::uint32_t Index = Start;
			std::uint64_t Evaluations = 0, RejectedSteps = 0;
			while(!Used[Index]) {
				// 所有する弧だけを順に繋ぎ，鞍点の頂点を二重に置かない
				Used[Index] = true;
				Loop.Arcs.push_back(Index);
				const TerrainContour &Arc = Arcs[Index];
				const double Offset = Loop.Contour.Samples.empty() ? 0 : Loop.Contour.Samples.back().DistanceMeters;
				const std::size_t Begin = Loop.Contour.Samples.empty() ? 0 : 1;
				for(std::size_t Point = Begin; Point < Arc.Samples.size(); ++Point) {
					TerrainContourSample Value = Arc.Samples[Point];
					Value.DistanceMeters += Offset;
					Loop.Contour.Samples.push_back(Value);
				}
				Evaluations += Arc.Evaluations;
				RejectedSteps += Arc.RejectedSteps;
				Loop.Contour.MaximumHeightResidualMeters = std::max(Loop.Contour.MaximumHeightResidualMeters, Arc.MaximumHeightResidualMeters);
				Loop.Contour.MaximumChordCorrectionMeters =
				std::max(Loop.Contour.MaximumChordCorrectionMeters, Arc.MaximumChordCorrectionMeters);
				Index = Next[Index];
				// 連結中の中断を完成した輪郭として返さない返戻
				if(Stop.stop_requested()) return {};
			}
			// 別の既存輪郭へ入る列，又は診断値の溢れへの返戻
			if(Index != Start || Evaluations > Missing || RejectedSteps > Missing) return {};
			Loop.Contour.Evaluations = static_cast<std::uint32_t>(Evaluations);
			Loop.Contour.RejectedSteps = static_cast<std::uint32_t>(RejectedSteps);
			TotalLength += Loop.Contour.Samples.back().DistanceMeters;
			// 全ての弧の長さを同じ準備予算へ含める返戻
			if(TotalLength > Settings.MaximumLengthMeters) return {};
			Vec3 Sum;
			for(std::size_t Point = 0; Point + 1 < Loop.Contour.Samples.size(); ++Point) {
				Sum = Sum + Loop.Contour.Samples[Point].Direction.Vector();
			}
			const std::optional<UnitDirection> Center = UnitDirection::FromVector(Sum);
			// 一つの半球に収まる候補中心が無い輪郭への返戻
			if(!Center) return {};
			const Vec3 Origin = Center->Vector(), Axis = std::abs(Origin.X) < 0.8 ? Vec3{ 1, 0, 0 } : Vec3{ 0, 1, 0 };
			const Vec3 East = UnitDirection::FromVector(Origin.Cross(Axis))->Vector(), North = Origin.Cross(East);
			std::vector<std::array<double, 2>> Planar;
			for(std::size_t Point = 0; Point + 1 < Loop.Contour.Samples.size(); ++Point) {
				const Vec3 D = Loop.Contour.Samples[Point].Direction.Vector();
				const double Z = D.Dot(Origin);
				// 射影の反転又は中断への返戻
				if(!(Z > 0) || Stop.stop_requested()) return {};
				Planar.push_back({ Settings.RadiusMeters * D.Dot(East) / Z, Settings.RadiusMeters * D.Dot(North) / Z });
			}
			const std::optional<double> Area = MeasureTerrainContourArea(Loop.Contour, Settings.RadiusMeters);
			// 自交差・零面積・開いた輪郭を水際として返さない返戻
			if(!Area || !std::isfinite(*Area) || !*Area || !IsSimplePlanarContour(Planar) || Stop.stop_requested()) return {};
			Loop.SignedAreaMetersSquared = *Area;
			Result.push_back(std::move(Loop));
		}
		// 指定した弧を使い尽した個別の単純閉輪郭の返戻
		return Result;
	}
}
