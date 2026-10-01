#include "PlanetSphericalRefinement.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace PlanetCore {
	struct PlanetSphericalRefinement::EdgeWorkspace {
	private:

		struct Edge {
			std::array<std::uint32_t, 2> Owners {};
			std::uint64_t Revision = 0;
			double OriginalMeters = std::numeric_limits<double>::quiet_NaN();
			unsigned Count = 0;
		};

		struct Proposal {
			std::uint64_t Key = 0;
			std::uint64_t Revision = 0;
			std::optional<UnitDirection> Direction;
			double Height = 0;
			double Original = 0;
			double Error = 0;
			double Quality = 1;
			std::uint32_t Parent = 0;
			std::uint32_t Upstream = std::numeric_limits<std::uint32_t>::max();
			std::uint8_t Depth = 0;
		};

		struct Priority {
			bool operator()(const Proposal &A, const Proposal &B) const;
		};

		static constexpr std::uint32_t NoUpstream = std::numeric_limits<std::uint32_t>::max();
		PlanetSphericalRefinement &Target;
		const SphericalReliefSettings &Settings;
		const std::function<double(UnitDirection)> &Source;
		std::stop_token Stop;
		const bool IsSamplingOnly;
		std::vector<Face> Faces;
		std::vector<double> SourceHeights;
		std::map<std::uint64_t, Edge> Edges;
		std::priority_queue<Proposal, std::vector<Proposal>, Priority> Pending;
		std::uint64_t Revision = 0;
		static std::uint64_t Key(const std::uint32_t A, const std::uint32_t B);
		static double Quality(const Vec3 A, const Vec3 B, const Vec3 C);
		bool AddFace(const Face &Triangle, std::vector<std::uint64_t> *const Touched);
		bool RemoveFace(const std::uint32_t Index, std::vector<std::uint64_t> &Touched);
		std::optional<Proposal> Select(const std::uint64_t Key, Edge &Owners);
		bool IsCurrent(const Proposal &Item) const;
		bool Split(const Proposal &Item);
		bool RelaxHeights();

	public:

		bool Run();

		EdgeWorkspace(
			PlanetSphericalRefinement &Target,
			const SphericalReliefSettings &Settings,
			const std::function<double(UnitDirection)> &Source,
			const std::stop_token Stop,
			const bool ShouldOnlySample=false
		);
	};

	/**
	 * 同じ誤差では共有辺番号を優先する
	 * @param A 第１候補
	 * @param B 第２候補
	 * @return 待機列で A の優先度が低い場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::Priority::operator()(const Proposal &A, const Proposal &B) const {
		return A.Error != B.Error ? A.Error < B.Error : A.Key > B.Key;
	}

	/**
	 * 両方向で共通の辺番号を作る
	 * @param A 一方の頂点
	 * @param B 他方の頂点
	 * @return 頂点の組に固有な番号
	 */
	std::uint64_t PlanetSphericalRefinement::EdgeWorkspace::Key(const std::uint32_t A, const std::uint32_t B) {
		return static_cast<std::uint64_t>(std::min(A, B)) << 32 | std::max(A, B);
	}

	/**
	 * 面積と辺長から細長さを評価する
	 * @param A 第１頂点
	 * @param B 第２頂点
	 * @param C 第３頂点
	 * @return 正三角形で１，退化時に０となる品質
	 */
	double PlanetSphericalRefinement::EdgeWorkspace::Quality(const Vec3 A, const Vec3 B, const Vec3 C) {
		const Vec3 X = B - A, Y = C - A, Z = C - B;
		// 面積と辺長の比に依る形状品質の返戻
		return 2 * std::sqrt(3.0) * X.Cross(Y).Length() / (X.Dot(X) + Y.Dot(Y) + Z.Dot(Z));
	}

	/**
	 * 両側が共有する辺へ所有面を登録する
	 * @param Triangle 外向きの面
	 * @param Touched 再評価する辺，初期構築時は空のポインタ
	 * @return 各辺の所有数が２を超えない場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::AddFace(const Face &Triangle, std::vector<std::uint64_t> *const Touched) {
		const std::uint32_t Index = static_cast<std::uint32_t>(Faces.size());
		Faces.push_back(Triangle);
		for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint64_t Id = Key(Triangle.Vertices[Side], Triangle.Vertices[(Side + 1) % 3]);
			Edge &Owners = Edges[Id];
			// 同じ辺へ３面を接続しない返戻
			if(Owners.Count > 1) return false;
			Owners.Owners[Owners.Count++] = Index;
			Owners.Revision = ++Revision;
			if(Touched) Touched->push_back(Id);
		}
		// 共有辺への登録結果の返戻
		return true;
	}

	/**
	 * 分割される旧面の所有関係を外す
	 * @param Index 旧面の番号
	 * @param Touched 接続更新した辺
	 * @return 旧面が全３辺を所有していた場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::RemoveFace(const std::uint32_t Index, std::vector<std::uint64_t> &Touched) {
		Face &Triangle = Faces[Index];
		// 無効化した面を二重に除去しない返戻
		if(!Triangle.IsActive) return false;
		Triangle.IsActive = false;
		for(unsigned Side = 0; Side < 3; ++Side) {
			const std::uint64_t Id = Key(Triangle.Vertices[Side], Triangle.Vertices[(Side + 1) % 3]);
			const std::map<std::uint64_t, Edge>::iterator Found = Edges.find(Id);
			// 欠けた共有関係を黙って作り直さない返戻
			if(Found == Edges.end()) return false;
			Edge &Owners = Found->second;
			unsigned Position = 0;
			while(Position < Owners.Count && Owners.Owners[Position] != Index) ++Position;
			// 異なる面の所有を削除しない返戻
			if(Position == Owners.Count) return false;
			Owners.Owners[Position] = Owners.Owners[--Owners.Count];
			Owners.Revision = ++Revision;
			Touched.push_back(Id);
		}
		// 所有関係を除去した結果の返戻
		return true;
	}

	/**
	 * 辺の中点で元起伏を照会し，下降経路と形状品質を保つ候補を作る
	 * @param Id 辺の共有番号
	 * @param Owners ２所有面と元標高の一時キャッシュ
	 * @return 有効な候補，対象外は優先度０，不正又は中断では空
	 */
	std::optional<PlanetSphericalRefinement::EdgeWorkspace::Proposal> PlanetSphericalRefinement::EdgeWorkspace::Select(
		const std::uint64_t Id,
		Edge &Owners
	) {
		// 開いた辺又は中断を受入れない返戻
		if(Owners.Count != 2 || Stop.stop_requested()) return {};
		Proposal Item;
		Item.Key = Id;
		Item.Revision = Owners.Revision;
		const std::uint32_t A = static_cast<std::uint32_t>(Id >> 32), B = static_cast<std::uint32_t>(Id);
		std::uint32_t Lowest = A;
		for(const std::uint32_t Owner : Owners.Owners) {
			const Face &Triangle = Faces[Owner];
			// 上限深さに達した面を更に分割しない返戻
			if(Triangle.Depth >= Settings.MaximumDepth) return Item;
			Item.Depth = std::max(Item.Depth, static_cast<std::uint8_t>(Triangle.Depth + 1));
			for(const std::uint32_t Vertex : Triangle.Vertices) {
				// 混合海岸面と海底は元の形を保持する返戻
				if(Target.Heights[Vertex] <= Settings.SeaLevelMeters) return Item;
				if(Target.Heights[Vertex] < Target.Heights[Lowest]) Lowest = Vertex;
			}
		}
		const Vec3 First = Target.Directions[A].Vector(), Second = Target.Directions[B].Vector();
		const std::optional<UnitDirection> Point = UnitDirection::FromVector(First + Second);
		// 対向する方向で中点を作れない場合の返戻
		if(!Point) return {};
		Item.Direction = Point;
		const Vec3 Midpoint = Point->Vector();
		for(const std::uint32_t Owner : Owners.Owners) for(const std::uint32_t Vertex : Faces[Owner].Vertices) {
			if(Vertex == A || Vertex == B) continue;
			const Vec3 Third = Target.Directions[Vertex].Vector();
			Item.Quality = std::min({ Item.Quality, Quality(First, Midpoint, Third), Quality(Midpoint, Second, Third) });
		}
		// 人工的な急斜面を生む細長い子面を追加しない返戻
		if(Item.Quality < MinimumChildQuality) return Item;
		Item.Parent = Lowest;
		if(!IsSamplingOnly && Target.Downstream[A] == B) {
			Item.Upstream = A;
			Item.Parent = B;
		} else if(!IsSamplingOnly && Target.Downstream[B] == A) {
			Item.Upstream = B;
			Item.Parent = A;
		}
		const auto Distance = [&](const std::uint32_t Vertex) -> double {
			const double Chord = (Midpoint - Target.Directions[Vertex].Vector()).Length();
			// 中点から指定節点迄の球面距離の返戻
			return 2 * std::asin(std::min(1.0, 0.5 * Chord)) * Settings.RadiusMeters;
		};
		const double Lower = IsSamplingOnly ? -std::numeric_limits<double>::max() : std::nextafter(
			Target.Heights[Item.Parent] + Settings.MinimumSlope * Distance(Item.Parent),
			std::numeric_limits<double>::infinity()
		);
		double Upper = std::numeric_limits<double>::max();
		if(Item.Upstream != NoUpstream) {
			Upper = std::nextafter(
				Target.Heights[Item.Upstream] - Settings.MinimumSlope * Distance(Item.Upstream),
				-std::numeric_limits<double>::infinity()
			);
		}
		// 元の微小な落差を無理に分割しない返戻
		if(Lower > Upper) return Item;
		// 有限な入力の中間演算が範囲を超えた場合の返戻
		if(!std::isfinite(Lower) || !std::isfinite(Upper)) return {};
		if(!std::isfinite(Owners.OriginalMeters)) {
			Owners.OriginalMeters = Source(*Point);
			++Target.Stats.SourceEvaluations;
			// 非有限な元標高を節点へ渡さない返戻
			if(!std::isfinite(Owners.OriginalMeters)) return {};
		}
		Item.Original = Owners.OriginalMeters;
		Item.Height = IsSamplingOnly ? Item.Original : std::clamp(Item.Original, Lower, Upper);
		Item.Error = std::abs(Item.Height - (0.5 * Target.Heights[A] + 0.5 * Target.Heights[B]));
		// 有限な条件付標高と優先度だけを返す返戻
		if(!std::isfinite(Item.Error)) return {};
		// ２面に同時に追加出来る共有中点の返戻
		return Item;
	}

	/**
	 * 候補の作成後に所有面が変わっていないか確認する
	 * @param Item 待機列の候補
	 * @return 現在の共有辺へ適用出来る場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::IsCurrent(const Proposal &Item) const {
		const std::map<std::uint64_t, Edge>::const_iterator Found = Edges.find(Item.Key);
		// 所有数と更新番号の一致の返戻
		return Found != Edges.end() && Found->second.Count == 2 && Found->second.Revision == Item.Revision;
	}

	/**
	 * ２面を同時に分割し，下流辺を２本の下降辺へ置換える
	 * @param Item 現在の所有関係に対する有効な候補
	 * @return 共有接続と下降先を更新出来た場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::Split(const Proposal &Item) {
		const Edge Owners = Edges.at(Item.Key);
		const std::array<Face, 2> Original { Faces[Owners.Owners[0]], Faces[Owners.Owners[1]] };
		const std::uint32_t Added = static_cast<std::uint32_t>(Target.Directions.size());
		const std::optional<UnitDirection> Point = Item.Direction;
		// 準備済方向が不正な場合の返戻
		if(!Point) return false;
		Target.Directions.push_back(*Point);
		Target.Heights.push_back(Item.Height);
		SourceHeights.push_back(Item.Original);
		Target.Downstream.push_back(IsSamplingOnly ? Added : Item.Parent);
		if(Item.Upstream != NoUpstream) Target.Downstream[Item.Upstream] = Added;
		std::vector<std::uint64_t> Touched;
		// 古い２面を安全に無効化出来ない場合の返戻
		for(const std::uint32_t Owner : Owners.Owners) if(!RemoveFace(Owner, Touched)) return false;
		for(const Face &Triangle : Original) {
			bool HasSplit = false;
			for(unsigned Side = 0; Side < 3; ++Side) {
				const std::uint32_t First = Triangle.Vertices[Side], Second = Triangle.Vertices[(Side + 1) % 3];
				if(Key(First, Second) != Item.Key) continue;
				const std::uint32_t Third = Triangle.Vertices[(Side + 2) % 3];
				const bool HasAddedChildren = AddFace({ { First, Added, Third }, Item.Depth, true }, &Touched) &&
				AddFace({ { Added, Second, Third }, Item.Depth, true }, &Touched);
				// 外向きの２子面を共有網へ登録出来ない場合の返戻
				if(!HasAddedChildren) return false;
				HasSplit = true;
			}
			// 対象辺を所有しない面を交換しない返戻
			if(!HasSplit) return false;
		}
		std::sort(Touched.begin(), Touched.end());
		Touched.erase(std::unique(Touched.begin(), Touched.end()), Touched.end());
		for(const std::uint64_t Id : Touched) {
			const std::map<std::uint64_t, Edge>::iterator Found = Edges.find(Id);
			// 更新対象の共有記録が無い場合の返戻
			if(Found == Edges.end()) return false;
			if(!Found->second.Count) {
				Edges.erase(Found);
				continue;
			}
			const std::optional<Proposal> Next = Select(Id, Found->second);
			// 新しい接続を評価出来ない場合の返戻
			if(!Next) return false;
			if(Next->Error > Settings.TargetErrorMeters) Pending.push(*Next);
		}
		++Target.Stats.AddedVertices;
		Target.Stats.MinimumCreatedTriangleQuality = std::min(Target.Stats.MinimumCreatedTriangleQuality, Item.Quality);
		const double Raise = Item.Height - Item.Original, Cut = -Raise;
		if(Raise > Target.Stats.MaximumConditioningRaiseMeters) {
			Target.Stats.MaximumConditioningRaiseMeters = Raise;
			Target.Stats.MaximumRaiseDirection = Item.Direction->Vector();
		}
		if(Cut > Target.Stats.MaximumConditioningCutMeters) {
			Target.Stats.MaximumConditioningCutMeters = Cut;
			Target.Stats.MaximumCutDirection = Item.Direction->Vector();
		}
		// 共有中点と下降経路の更新結果の返戻
		return true;
	}

	/**
	 * 細分化後の隣接関係で，不要な盛土だけを最小限の高さへ下げる
	 * @return 元河道を固定した下降経路を保持する場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::RelaxHeights() {
		// 追加点が無い場合は補正作業を不要にする返戻
		if(!Target.Stats.AddedVertices) return true;
		const std::uint32_t OriginalCount = Target.Stats.OriginalVertices;
		const std::size_t Count = Target.Directions.size();
		std::vector<bool> Fixed(Count, false);
		for(std::uint32_t Index = 0; Index < OriginalCount; ++Index) Fixed[Index] = true;
		// 元河道と既に開削した支流を固定し，盛土だけの点へ対象を絞る
		for(std::uint32_t Index = 0; Index < Count; ++Index) {
			if(Index >= OriginalCount && Target.Heights[Index] >= SourceHeights[Index]) continue;
			std::uint32_t Current = Index < OriginalCount ? Target.Downstream[Index] : Index;
			while(!Fixed[Current]) {
				Fixed[Current] = true;
				++Target.Stats.ProtectedAddedVertices;
				Current = Target.Downstream[Current];
			}
		}
		std::vector<std::vector<std::uint32_t>> Neighbors(Count);
		for(const std::pair<const std::uint64_t, Edge> &Entry : Edges) {
			const std::uint32_t A = static_cast<std::uint32_t>(Entry.first >> 32), B = static_cast<std::uint32_t>(Entry.first);
			Neighbors[A].push_back(B);
			Neighbors[B].push_back(A);
		}
		using HeightEntry = std::pair<double, std::uint32_t>;
		std::priority_queue<HeightEntry, std::vector<HeightEntry>, std::greater<HeightEntry>> Open;
		const std::vector<double> PreviousHeights = Target.Heights;
		// 全地点の既知の有効解を上限にし，より低い出口が見つかった点だけを更新する
		for(std::uint32_t Index = 0; Index < Count; ++Index) Open.emplace(Target.Heights[Index], Index);
		while(!Open.empty()) {
			const HeightEntry Item = Open.top();
			Open.pop();
			// 長い補正中も中断を受理する返戻
			if(Stop.stop_requested()) return false;
			const std::uint32_t Parent = Item.second;
			if(Item.first != Target.Heights[Parent]) continue;
			for(const std::uint32_t Vertex : Neighbors[Parent]) {
				if(Fixed[Vertex]) continue;
				const double Chord = (Target.Directions[Vertex].Vector() - Target.Directions[Parent].Vector()).Length();
				const double Distance = 2 * std::asin(std::min(1.0, 0.5 * Chord)) * Settings.RadiusMeters;
				const double Lower =
				std::nextafter(Target.Heights[Parent] + Settings.MinimumSlope * Distance, std::numeric_limits<double>::infinity());
				const double Height = std::max(SourceHeights[Vertex], Lower);
				if(Height >= Target.Heights[Vertex]) continue;
				Target.Heights[Vertex] = Height;
				Target.Downstream[Vertex] = Parent;
				Open.emplace(Height, Vertex);
			}
		}
		Target.Stats.MaximumConditioningRaiseMeters = 0;
		Target.Stats.MaximumConditioningCutMeters = 0;
		Target.Stats.MaximumRaiseDirection = {};
		Target.Stats.MaximumCutDirection = {};
		for(std::uint32_t Index = OriginalCount; Index < Count; ++Index) {
			const double Lowering = PreviousHeights[Index] - Target.Heights[Index];
			if(Lowering > 0) ++Target.Stats.RelaxedVertices;
			Target.Stats.MaximumRelaxationMeters = std::max(Target.Stats.MaximumRelaxationMeters, Lowering);
			const double Raise = Target.Heights[Index] - SourceHeights[Index], Cut = -Raise;
			if(Raise > Target.Stats.MaximumConditioningRaiseMeters) {
				Target.Stats.MaximumConditioningRaiseMeters = Raise;
				Target.Stats.MaximumRaiseDirection = Target.Directions[Index].Vector();
			}
			if(Cut > Target.Stats.MaximumConditioningCutMeters) {
				Target.Stats.MaximumConditioningCutMeters = Cut;
				Target.Stats.MaximumCutDirection = Target.Directions[Index].Vector();
			}
		}
		// 盛土を増やさない再経路付の返戻
		return true;
	}

	/**
	 * 共有辺の優先度順に予算内で細分化する
	 * @return 閉球面と指定下流辺を保持した場合に真
	 */
	bool PlanetSphericalRefinement::EdgeWorkspace::Run() {
		Faces.reserve(Target.Triangles.size() + 4 * Settings.MaximumAddedVertices);
		for(const std::array<std::uint32_t, 3> &Triangle : Target.Triangles) {
			// 共有網の初期構築に失敗した場合の返戻
			if(Stop.stop_requested() || !AddFace({ Triangle, 0, true }, nullptr)) return false;
		}
		for(std::uint32_t Vertex = 0; Vertex < Target.Downstream.size(); ++Vertex) {
			const std::uint32_t Parent = Target.Downstream[Vertex];
			// 存在しない下流番号を参照しない返戻
			if(Parent >= Target.Directions.size()) return false;
			if(Parent == Vertex) continue;
			const bool HasValidParent = Target.Heights[Parent] < Target.Heights[Vertex] && Edges.contains(Key(Vertex, Parent));
			// 上り返し又は非隣接の下流を受入れない返戻
			if(!HasValidParent) return false;
		}
		for(std::pair<const std::uint64_t, Edge> &Entry : Edges) {
			// 追加予算０でも共有辺の欠落と中断を受入れない返戻
			if(Entry.second.Count != 2 || Stop.stop_requested()) return false;
			if(!Settings.MaximumAddedVertices) continue;
			const std::optional<Proposal> Item = Select(Entry.first, Entry.second);
			// 初期辺の候補を準備出来ない場合の返戻
			if(!Item) return false;
			if(Item->Error > Settings.TargetErrorMeters) Pending.push(*Item);
		}
		while(!Pending.empty() && Target.Stats.AddedVertices < Settings.MaximumAddedVertices) {
			const Proposal Item = Pending.top();
			Pending.pop();
			if(!IsCurrent(Item)) continue;
			// 中断又は共有辺の更新失敗を受理する返戻
			if(Stop.stop_requested() || !Split(Item)) return false;
		}
		while(!Pending.empty() && !IsCurrent(Pending.top())) Pending.pop();
		if(!Pending.empty()) Target.Stats.LargestRemainingPriorityMeters = Pending.top().Error;
		// 新しい隣接関係でも不要な盛土を残さない返戻
		if(!IsSamplingOnly && Settings.ShouldRelaxAddedHeights && !RelaxHeights()) return false;
		Target.Triangles.clear();
		Target.Triangles.reserve(2 * (Target.Directions.size() - 2));
		for(const Face &Triangle : Faces) if(Triangle.IsActive) Target.Triangles.push_back(Triangle.Vertices);
		// 共有中点を両側から参照する閉球面の返戻
		return Target.Triangles.size() == 2 * (Target.Directions.size() - 2);
	}

	/**
	 * 生成準備中だけ存在する共有辺の作業領域を作る
	 * @param InTarget 元節点・標高・下降先
	 * @param InSettings 作業量と品質の制限
	 * @param InSource 元地形
	 * @param InStop 中断要求
	 * @param ShouldOnlySample 排水前の標本位置だけを準備する場合に真
	 */
	PlanetSphericalRefinement::EdgeWorkspace::EdgeWorkspace(
		PlanetSphericalRefinement &InTarget,
		const SphericalReliefSettings &InSettings,
		const std::function<double(UnitDirection)> &InSource,
		const std::stop_token InStop,
		const bool ShouldOnlySample
	) : Target(InTarget),
	Settings(InSettings),
	Source(InSource),
	Stop(InStop),
	IsSamplingOnly(ShouldOnlySample),
	SourceHeights(InTarget.Heights) {
		// 終了
		return;
	}

	/**
	 * 河道を決める前に，標高を改変せず陸上起伏を適応標本化する
	 * @param InDirections 元の節点方向
	 * @param InTriangles 閉じた外向きの網
	 * @param ElevationsMeters 元節点の標高
	 * @param Source 追加位置で評価する同じ元地形
	 * @param Settings 作業予算・形状品質，排水制限の設定は適用しない
	 * @param Stop 中断要求
	 * @return 排水未解決の共有網，不正又は中断では空
	 */
	std::optional<SphericalReliefSamplingMesh> PlanetSphericalRefinement::CreateSamplingMesh(
		const std::span<const UnitDirection> InDirections,
		const std::span<const std::array<std::uint32_t, 3>> InTriangles,
		const std::span<const double> ElevationsMeters,
		const std::function<double(UnitDirection)> &Source,
		const SphericalReliefSettings &Settings,
		const std::stop_token Stop
	) {
		SphericalReliefSettings InitialSettings = Settings;
		InitialSettings.MaximumAddedVertices = 0;
		std::optional<PlanetSphericalRefinement> Prepared =
		Create(InDirections, InTriangles, ElevationsMeters, Source, InitialSettings, Stop);
		// 元網又は実際の作業予算が不正な場合の返戻
		if(!Prepared || !Settings.IsValid() || Settings.MaximumAddedVertices > 65536 - InDirections.size()) return {};
		EdgeWorkspace Work(*Prepared, Settings, Source, Stop, true);
		// 盛土を行わない標本化が成立しない場合の返戻
		if(!Work.Run()) return {};
		SphericalReliefSamplingMesh Result;
		Result.Directions = std::move(Prepared->Directions);
		Result.Triangles = std::move(Prepared->Triangles);
		Result.ElevationsMeters = std::move(Prepared->Heights);
		Result.Statistics = Prepared->Stats;
		// 排水済と誤用出来ない別の型で返す元地形の標本網の返戻
		return Result;
	}

	/**
	 * 元起伏を共有辺の内部でも拾い，下流辺の下降を保つ同時２面細分化を準備する
	 * @param InDirections 元の節点方向
	 * @param InTriangles 閉じた外向きの網
	 * @param ElevationsMeters 元節点の標高
	 * @param RequiredDownstream 全節点の下流，自分自身は終端
	 * @param Source 保持する元地形
	 * @param Settings 細分化予算・最小勾配・形状品質
	 * @param Stop 中断要求
	 * @return 下降経路を分割辺へ継承した共有網，不正又は中断では空
	 */
	std::optional<PlanetSphericalRefinement> PlanetSphericalRefinement::CreateWithRefinedEdges(
		const std::span<const UnitDirection> InDirections,
		const std::span<const std::array<std::uint32_t, 3>> InTriangles,
		const std::span<const double> ElevationsMeters,
		const std::span<const std::uint32_t> RequiredDownstream,
		const std::function<double(UnitDirection)> &Source,
		const SphericalReliefSettings &Settings,
		const std::stop_token Stop
	) {
		// 下流配列の欠落を自己参照で代用しない返戻
		if(RequiredDownstream.size() != InDirections.size()) return {};
		SphericalReliefSettings InitialSettings = Settings;
		InitialSettings.MaximumAddedVertices = 0;
		std::optional<PlanetSphericalRefinement> Result =
		Create(InDirections, InTriangles, ElevationsMeters, Source, InitialSettings, Stop);
		// 元網の準備又は実際の作業予算の確認に失敗した場合の返戻
		if(!Result || !Settings.IsValid() || Settings.MaximumAddedVertices > 65536 - InDirections.size()) return {};
		Result->Downstream.assign(RequiredDownstream.begin(), RequiredDownstream.end());
		EdgeWorkspace Work(*Result, Settings, Source, Stop);
		// 共有辺と下降経路を保った細分化が成立しない場合の返戻
		if(!Work.Run()) return {};
		// 生成後の地形を保存せず，不変な準備場へ渡す網の返戻
		return Result;
	}
}
