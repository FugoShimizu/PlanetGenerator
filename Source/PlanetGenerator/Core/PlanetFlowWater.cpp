#include "PlanetFlowWater.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		/** 唯一の生成ワーカー内の処理費用を，早期返戻も含めて集約する */
		class WaterStageTimer {
		private:

			// 呼出元の累積値と，単調時計の開始時刻
			const std::chrono::steady_clock::time_point Started;
			double &Total;

			WaterStageTimer &operator=(const WaterStageTimer &) = delete;
			WaterStageTimer(const WaterStageTimer &) = delete;

		public:

			explicit WaterStageTimer(double &Total);
			~WaterStageTimer();
		};

		struct BankProbePoint {
			// 視点と独立した実三角形内部の位置と補間残差
			Vec3 Position;
			double HeightErrorMeters = 0;
			double NormalDotError = 0;
		};

		struct WaterFieldPoint {
			// 局所座標を含まない元場の値；水面法線だけの照会では所属を遅延評価
			FlowChannelSection Section;
			std::optional<double> Support;
		};

		struct RetainedWaterPartition {
			// 新しい要求へ引継ぐ球面方向と所有葉；局所座標や以前の合否は保持しない
			AdaptiveSphereMesh Sphere;
			LodTree Tree;
		};

		/** 不変な河岸だけに束縛する，上限付きの最近使用順の再利用領域 */
		template<class Key, class Value, std::size_t Capacity> class BankReuseCache {
		private:

			struct Entry {
				// 値と，其の鍵が属する最近使用順の位置
				Value Data;
				typename std::list<Key>::iterator Recent;
			};

			// 同じ場の値を保持する順序付き探索と，追出す順序
			std::map<Key, Entry> Values;
			std::list<Key> Recent;
			std::uint64_t Evicted = 0;

			BankReuseCache &operator=(const BankReuseCache &) = delete;
			BankReuseCache(const BankReuseCache &) = delete;

		public:

			/**
			 * 現在の保持量を取得する
			 * @return 保持上限以内の項目数
			 */
			std::size_t Size() const {
				return Values.size();
			}

			/**
			 * 上限に達して解放した項目の累積数を読む
			 * @return 同じ場の保持領域で起きた追出し回数
			 */
			std::uint64_t Evictions() const {
				return Evicted;
			}

			/**
			 * 完全一致する鍵の値を取得し，最近の利用位置へ移す
			 * @param Id 同じ不変場の方向又は三頂点
			 * @return 保持済の値；未保持は空ポインター
			 * @note 保持数 N に対して O(log N)
			 */
			const Value *Find(const Key &Id) {
				// 元の照会定義と一致する項目の探索
				const typename std::map<Key, Entry>::iterator Found = Values.find(Id);
				// 未保持なら呼出側で同じ元場を再評価する返戻
				if(Found == Values.end()) return nullptr;
				Recent.splice(Recent.begin(), Recent, Found->second.Recent);
				// 使用順だけを変更した値の返戻
				return &Found->second.Data;
			}

			/**
			 * 完成した値を保持し，上限時は最も長く使われていない項目を追出す
			 * @param Id 値を識別する正確な鍵
			 * @param Data 同じ元場から計算済の値
			 * @note 保持数 N に対して O(log N)，値の変更は地形定義に影響しない
			 */
			void Store(const Key &Id, const Value &Data) {
				// 既存の鍵は重複した使用順を作らずに更新
				if(const typename std::map<Key, Entry>::iterator Found = Values.find(Id); Found != Values.end()) {
					Found->second.Data = Data;
					Recent.splice(Recent.begin(), Recent, Found->second.Recent);
					// 同じ保持数で更新を終える返戻
					return;
				}
				// 最初の地点だけで保持領域が固定されない様，古い項目を解放
				if(Values.size() == Capacity) {
					Values.erase(Recent.back());
					Recent.pop_back();
					++Evicted;
				}
				Recent.push_front(Id);
				Values.emplace(Id, Entry{ Data, Recent.begin() });
				// 有界な保持の終了
				return;
			}

			/** 空の保持領域を準備する */
			BankReuseCache() = default;
		};

		// 方向と三頂点で識別する有界な保持；河岸面は保持分割と同じ１３１，０７２件迄
		using BankSampleCache = BankReuseCache<std::array<double, 3>, TerrainSample, 262144>;
		using BankCheckCache = BankReuseCache<std::array<double, 9>, std::array<BankProbePoint, 4>, 131072>;
		using WaterFieldCache = BankReuseCache<std::array<double, 3>, WaterFieldPoint, 262144>;

		/**
		 * 完了又は拒否迄に掛かった時間を加える
		 */
		WaterStageTimer::~WaterStageTimer() {
			Total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Started).count();
			// 終了
			return;
		}

		/**
		 * 同じワーカーが持つ累積値へ計測を結び付ける
		 * @param Total 実測時間の加算先，単位はミリ秒
		 */
		WaterStageTimer::WaterStageTimer(double &Total) : Started(std::chrono::steady_clock::now()), Total(Total) {
			return;
		}

		/**
		 * 前回の完全分割に，新しい足元・視点の必須葉を加える
		 * @param Previous 前回の精度検査を通過した共有分割
		 * @param Required 現在の視点と身体から選択した完全分割
		 * @param Stop 中断要求
		 * @return 新しい必要条件以上の完全分割；予算超過・中断・不整合は空
		 * @note 葉数 N と追加分割数 K に対して O((N+K) log N)，上限は既存の LOD 予算
		 */
		std::optional<RetainedWaterPartition> ExtendWaterPartition(
			const FlowTerrainMesh &Previous,
			const LodTree &Required,
			const std::stop_token Stop
		) {
			// 古い公開候補に触れず，同じ木の作業用複製を準備
			LodTree Tree = Previous.Tree;
			for(const TileId Target : Required.Leaves()) while(true) {
				// 中断時は未完成の分割を渡さない返戻
				if(Stop.stop_requested()) return {};
				const std::optional<TileId> Cover = Tree.CoveringLeaf(Target);
				if(!Cover || *Cover == Target) break;
				// 追加予算を超えた場合は呼出側の通常生成へ戻す返戻
				if(Tree.Refine(*Cover, LodTree::MaximumLeaves).Status != LodEditStatus::Applied) return {};
			}
			std::optional<AdaptiveSphereMesh> Sphere = UpdateAdaptiveSphere(Tree, Previous.Sphere, Stop);
			// 所有葉を復元出来ない候補への返戻
			if(!Sphere || Stop.stop_requested()) return {};
			// 水位と現在の視点条件の再検査へ渡す完全分割の返戻
			return RetainedWaterPartition{ std::move(*Sphere), std::move(Tree) };
		}

		/**
		 * 索引分割に使う成分を読む
		 * @param V 単位球上の値
		 * @param Axis 成分番号
		 * @return 指定成分
		 */
		double Component(const Vec3 V, const unsigned Axis) {
			// 同じ座標軸の値の返戻
			return !Axis ? V.X : Axis == 1 ? V.Y : V.Z;
		}

		/**
		 * 同じ河岸の頂点・辺中点・内部点を評価し，視点と独立した補間残差を作る
		 * @param Field 同じ不変な河床
		 * @param Triangle 開削支持域に接する面
		 * @param Cache 完全一致する方向だけの再利用
		 * @return 四つの検査点の補間残差，不正な標本は空
		 * @note 保持数 N に対して O(log N)，元場の照会費用を除く；各面７点以下
		 */
		std::optional<std::array<BankProbePoint, 4>> BuildBankProbes(
			const FlowSurfaceField &Field,
			const std::array<UnitDirection, 3> &Triangle,
			BankSampleCache &Cache
		) {
			// 同じ場の方向を量子化せずに再利用
			const auto Sample = [&](const UnitDirection Direction) -> std::optional<TerrainSample> {
				const Vec3 V = Direction.Vector();
				const std::array<double, 3> Key { V.X, V.Y, V.Z };
				// 同じ定義の既知標本への返戻
				if(const TerrainSample *const Found = Cache.Find(Key)) return *Found;
				const std::optional<FlowChannelSection> Value = Field.Evaluate(Direction, false);
				// 欠落を乾いた岸へ変換しない返戻
				if(!Value) return {};
				Cache.Store(Key, Value->Terrain);
				// 保持予算を使切っても同じ計算値の返戻
				return Value->Terrain;
			};
			// 現在の三角形の位置と解析法線の準備
			const double Radius = Field.GetRadiusMeters();
			std::array<Vec3, 3> Positions, Normals;
			for(unsigned I = 0; I < 3; ++I) {
				const std::optional<TerrainSample> Value = Sample(Triangle[I]);
				// 定義されない頂点への返戻
				if(!Value) return {};
				Positions[I] = Triangle[I].Vector() * (Radius + Value->ElevationMeters);
				const std::optional<UnitDirection> Normal = UnitDirection::FromVector(Positions[I] - Value->TangentialGradientPerRadian);
				// 不正な解析法線への返戻
				if(!Normal) return {};
				Normals[I] = Normal->Vector();
			}
			// 辺中点と内部点で，位置と補間法線を同時に検査
			std::array<BankProbePoint, 4> Result;
			std::size_t Index = 0;
			for(
				const std::array<double, 3> Weight :
				std::array<std::array<double, 3>, 4>{ { { 0.5, 0.5, 0 }, { 0, 0.5, 0.5 }, { 0.5, 0, 0.5 }, { 1.0 / 3, 1.0 / 3, 1.0 / 3 } } }
			) {
				const Vec3 P = Positions[0] * Weight[0] + Positions[1] * Weight[1] + Positions[2] * Weight[2];
				const UnitDirection D = *UnitDirection::FromVector(P);
				const std::optional<TerrainSample> Value = Sample(D);
				// 不正な内部標本への返戻
				if(!Value) return {};
				const std::optional<UnitDirection> Normal =
				UnitDirection::FromVector(Normals[0] * Weight[0] + Normals[1] * Weight[1] + Normals[2] * Weight[2]);
				const std::optional<UnitDirection> Exact =
				UnitDirection::FromVector(D.Vector() * (Radius + Value->ElevationMeters) - Value->TangentialGradientPerRadian);
				// 定義出来ない法線差への返戻
				if(!Normal || !Exact) return {};
				Result[Index++] = { P, std::abs(P.Length() - Radius - Value->ElevationMeters), 1 - Normal->Vector().Dot(Exact->Vector()) };
			}
			// 原点・視点・表示通番に依存しない補間残差の返戻
			return Result;
		}

		/**
		 * 既知の補間残差を現在の視点で検査し，未達面の次の辺長を決める
		 * @param Points 同じ不変な場と三頂点で求めた残差
		 * @param Triangle 残差に対応する実三角形の方向
		 * @param View 現在の観察条件
		 * @param HalfWidthMeters 基準となる川幅の半分
		 * @return 達成は正の無限大，未達は現在の最大辺の半分
		 */
		double BankInterpolationEdge(
			const std::array<BankProbePoint, 4> &Points,
			const std::array<UnitDirection, 3> &Triangle,
			const LodView &View,
			const double HalfWidthMeters
		) {
			// 表示距離に依存する条件だけを現在の観察点で評価
			for(const BankProbePoint &Point : Points) {
				const double Distance = std::max(0.01, (Point.Position - View.ObserverMeters).Length());
				const bool BadHeight = Point.HeightErrorMeters > std::max(0.005, 0.25 * Distance / View.FocalLengthPixels); // 下限５ミリメートル，四分の一画素
				const bool VisibleBank = HalfWidthMeters * View.FocalLengthPixels > Distance; // 川幅が２画素を超える場合
				const bool BadNormal = VisibleBank && Point.NormalDotError > 0.005; // 約０．１ラジアン
				if(BadHeight || BadNormal) {
					// 同じ精度未達の面を分割する返戻
					return View.RadiusMeters * 0.5 * std::max(
						{
							(Triangle[0].Vector() - Triangle[1].Vector()).Length(),
							(Triangle[1].Vector() - Triangle[2].Vector()).Length(),
							(Triangle[2].Vector() - Triangle[0].Vector()).Length()
						}
					);
				}
			}
			// 補間精度を満たす面の返戻
			return std::numeric_limits<double>::infinity();
		}
	}

	struct FlowWaterView::ReuseState {
		// 同じ不変場の正確な方向と三頂点だけで識別する，有界な再利用領域
		std::optional<FlowTerrainMesh> Partition;
		std::uint64_t PartitionReuses = 0;
		BankSampleCache Samples;
		BankCheckCache Checks;
		std::uint64_t Hits = 0;
		std::uint64_t Evaluations = 0;
		WaterFieldCache WaterSamples;
		std::uint64_t WaterHits = 0;
		std::uint64_t WaterEvaluations = 0;
		std::uint64_t SupportEvaluations = 0;
		std::array<double, 5> StageMilliseconds {};
	};

	/**
	 * 地点評価の所有権を保持する
	 * @param Input 不変な河床・水位・所属
	 */
	FlowWaterView::FlowWaterView(std::shared_ptr<const FlowSurfaceField> Input) : Reuse(std::make_unique<ReuseState>()),
	Field(std::move(Input)) {
		// 所有権を移した構築の終了
		return;
	}

	/**
	 * 所有する場と有界な再利用領域を解放する
	 */
	FlowWaterView::~FlowWaterView() = default;

	/**
	 * 唯一のワーカーから再利用の累積回数と保持量を取得する
	 * @return 評価・再利用・保持量の実値
	 */
	FlowWaterReuseStatistics FlowWaterView::GetReuseStatistics() const {
		// 有界な保持領域と再利用回数の返戻
		return {
			Reuse->Hits,
			Reuse->Evaluations,
			Reuse->Samples.Size(),
			Reuse->Checks.Size(),
			Reuse->PartitionReuses,
			Reuse->Partition ? Reuse->Partition->Tree.Leaves().size() : 0,
			Reuse->WaterSamples.Size(),
			Reuse->WaterHits,
			Reuse->WaterEvaluations,
			Reuse->SupportEvaluations,
			Reuse->StageMilliseconds,
			Reuse->Checks.Evictions()
		};
	}

	/**
	 * 短い弧を包む単位球上の球を追加する
	 * @param Start 始点
	 * @param End 終点
	 * @param ElevationMeters 岸の既知の水位
	 * @param IsRiver 河道中心線の場合に真，偽は準備済の岸
	 */
	void FlowWaterView::AddGuide(
		const UnitDirection Start,
		const UnitDirection End,
		const double ElevationMeters,
		const bool IsRiver
	) {
		// 弧の端点と短い区間の確認
		const Vec3 A = Start.Vector(), B = End.Vector();
		// 重複した閉鎖点を退化した弧へ変換しない終了
		if((A - B).Length() < 1E-15) return;
		// 短い大円弧全体を包む球の準備
		const Vec3 Center = UnitDirection::FromVector(A + B)->Vector();
		Guides.push_back(
			{ Start, End, Center, std::max((A - Center).Length(), (B - Center).Length()) + 1E-14, ElevationMeters, IsRiver }
		);
		// 追加の終了
		return;
	}

	/**
	 * 弧の包囲球を含む平衡な矩形索引を作る
	 * @param Begin 順序配列内の開始位置
	 * @param Count 弧の数
	 * @return 作った節点番号
	 * @note 弧数 N に対して平均 O(N log N)，保持量 O(N)
	 */
	std::uint32_t FlowWaterView::BuildIndex(const std::uint32_t Begin, const std::uint32_t Count) {
		// 弧群の包囲箱の集約
		Vec3 Low { 3, 3, 3 }, High { -3, -3, -3 };
		for(std::uint32_t I = Begin; I < Begin + Count; ++I) {
			const Guide &Arc = Guides[Order[I]];
			const Vec3 Padding { Arc.Cap, Arc.Cap, Arc.Cap }, A = Arc.Center - Padding, B = Arc.Center + Padding;
			Low = { std::min(Low.X, A.X), std::min(Low.Y, A.Y), std::min(Low.Z, A.Z) };
			High = { std::max(High.X, B.X), std::max(High.Y, B.Y), std::max(High.Z, B.Z) };
		}
		// 葉又は内部節点の予約
		const std::uint32_t Id = static_cast<std::uint32_t>(Index.size());
		Index.push_back({ Low, High, Begin, Count, 0, 0 });
		// 少数の弧を保持する葉への返戻
		if(Count < 9) return Id;
		// 最長軸に依る二分と子の構築
		const Vec3 Span = High - Low;
		const unsigned Axis = Span.X >= Span.Y && Span.X >= Span.Z ? 0 : Span.Y >= Span.Z ? 1 : 2;
		const std::uint32_t Half = Count / 2;
		std::nth_element(
			Order.begin() + Begin,
			Order.begin() + Begin + Half,
			Order.begin() + Begin + Count,
			[&](const std::uint32_t A, const std::uint32_t B) -> bool {
				const double X = Component(Guides[A].Center, Axis), Y = Component(Guides[B].Center, Axis);
				// 同位置でも弧番号で固定する比較の返戻
				return X < Y || X == Y && A < B;
			}
		);
		const std::uint32_t Left = BuildIndex(Begin, Half), Right = BuildIndex(Begin + Half, Count - Half);
		Index[Id].Count = 0;
		Index[Id].Left = Left;
		Index[Id].Right = Right;
		// 完成した内部節点の返戻
		return Id;
	}

	/**
	 * 準備済の岸と河道の短い近似弧を作り，見落し防止の索引を構築する
	 * @param Field 同じ元場を所有する不変な定義
	 * @param Stop 準備途中の中断要求
	 * @return 完成した索引；不正・中断・発見用近似の予算超過は空
	 * @note 計算量は発見用弧数 N に対して平均 O(N log N)，保持量 O(N)
	 * @note 河道近似の誤差は有限標本で検査し，水位・河床の定義には使わない
	 */
	std::unique_ptr<FlowWaterView> FlowWaterView::Create(
		std::shared_ptr<const FlowSurfaceField> Field,
		const std::stop_token Stop
	) {
		// 無効な場又は中断済の準備への返戻
		if(!Field || Stop.stop_requested()) return {};
		// 所有する場と発見用の尺度の準備
		std::unique_ptr<FlowWaterView> Result(new FlowWaterView(std::move(Field)));
		const std::vector<FlowWaterGeometry> Geometries = Result->Field->GetWaterGeometries();
		// 水域を失った定義を乾いた成功として扱わない返戻
		if(Geometries.empty()) return {};
		const double Radius = Result->Field->GetRadiusMeters();
		Result->HalfWidthMeters = std::numeric_limits<double>::infinity();
		for(const FlowWaterGeometry &Geometry : Geometries) {
			Result->HalfWidthMeters = std::min(Result->HalfWidthMeters, Geometry.ChannelHalfWidthMeters);
			Result->BankSupportChord = std::max(Result->BankSupportChord, (Geometry.MaximumChannelHalfWidthMeters + 0.001) / Radius);
		}
		std::set<const SpillLakeShore *> Shores;
		for(const FlowWaterGeometry &Geometry : Geometries) {
			// 元の所属判定と同じ岸を共有ポインターで重複排除
			for(const SpillLakeShore *Shore : { &Geometry.Upstream, Geometry.Receiving }) {
				if(!Shore || !Shores.insert(Shore).second) continue;
				const std::vector<TerrainContourSample> &Points = Shore->GetContour().Samples;
				for(std::size_t I = 1; I < Points.size(); ++I) {
					// 岸も全河道の保持予算へ含め，中断と共に部分公開を拒否する返戻
					if(Stop.stop_requested() || Result->Guides.size() > 196607) return {};
					Result->AddGuide(Points[I - 1].Direction, Points[I].Direction, Points[I].ElevationMeters, false);
				}
			}
			// 元の連続曲線を短い弧へ適応近似し，四分点も含めて一ミリメートルを検査
			for(const TerrainFlowSegment &Segment : Geometry.Route.GetSegments()) {
				std::vector<std::array<double, 2>> Pending { { 0, 1 } };
				while(!Pending.empty()) {
					// 元場の照会へ進む前に中断と保持予算を確認する返戻
					if(Stop.stop_requested() || Result->Guides.size() > 196607) return {};
					const std::array<double, 2> Interval = Pending.back();
					Pending.pop_back();
					const double A = Interval[0], B = Interval[1], Span = B - A;
					const UnitDirection First = Segment.Evaluate(A)->Direction, Last = Segment.Evaluate(B)->Direction;
					double Error = 0;
					for(const double T : { 0.25, 0.5, 0.75 }) {
						const Vec3 Point = Segment.Evaluate(A + Span * T)->Direction.Vector();
						const Vec3 Linear = UnitDirection::FromVector(First.Vector() * (1 - T) + Last.Vector() * T)->Vector();
						Error = std::max(Error, Radius * (Point - Linear).Length());
					}
					if(Error > 0.001 || Segment.GetParameterLengthMeters() * Span > Result->HalfWidthMeters * 0.5) {
						// 有限の分割深さで精度を達成しない曲線への返戻
						if(Span < 1E-6) return {};
						Pending.push_back({ A + Span * 0.5, B });
						Pending.push_back({ A, A + Span * 0.5 });
						continue;
					}
					Result->MaximumGuideErrorMeters = std::max(Result->MaximumGuideErrorMeters, Error);
					// 河岸の投影寸法を，海面ではなく実際の河床標高から計算する
					const UnitDirection Middle = *UnitDirection::FromVector(First.Vector() + Last.Vector());
					const std::optional<FlowChannelSection> Sample = Result->Field->Evaluate(Middle, false, false);
					// 未定義の岸高を零で代用しない返戻
					if(!Sample) return {};
					Result->AddGuide(First, Last, Sample->Terrain.ElevationMeters, true);
				}
			}
		}
		// 空又は中断した索引を公開しない返戻
		if(Result->Guides.empty() || Stop.stop_requested()) return {};
		// 描画視点と独立した索引の構築
		Result->Order.resize(Result->Guides.size());
		std::iota(Result->Order.begin(), Result->Order.end(), 0U);
		Result->Index.reserve(Result->Guides.size());
		Result->BuildIndex(0, static_cast<std::uint32_t>(Result->Guides.size()));
		// 索引構築中の中断を保持しない返戻
		if(Stop.stop_requested()) return {};
		// 元場を所有する読取専用の索引の返戻
		return Result;
	}

	/**
	 * 頂点標本に現れない河道・岸と，開削を含む周辺面の必要な辺長を求める
	 * @param Center 三角形の球面中心
	 * @param Cap 頂点を包む弦半径
	 * @param Triangle 球面三角形
	 * @param Depths 共通球面へ射影した符号付き水深
	 * @param View 同じ視点条件
	 * @param TouchesBank 開削支持域に接する面なら真を返す
	 * @return 必要な最大辺長；追加条件が無い場合は正の無限大
	 * @note 最悪 O(N)，N は保持する発見用弧の数
	 */
	double FlowWaterView::RequiredEdge(
		const UnitDirection Center,
		const double Cap,
		const std::array<UnitDirection, 3> &Triangle,
		const std::array<double, 3> &Depths,
		const LodView &View,
		bool &TouchesBank
	) const {
		// 照会面の包囲箱と平衡木の探索状態
		TouchesBank = false;
		double Limit = std::numeric_limits<double>::infinity();
		const double SearchCap = Cap + BankSupportChord;
		const Vec3 C = Center.Vector(), Padding { SearchCap, SearchCap, SearchCap }, Low = C - Padding, High = C + Padding;
		std::array<std::uint32_t, 64> Stack {};
		std::uint32_t Pending = 1;
		while(Pending) {
			const IndexNode &Node = Index[Stack[--Pending]];
			if(
				Node.High.X < Low.X || Node.High.Y < Low.Y || Node.High.Z < Low.Z || Node.Low.X > High.X || Node.Low.Y > High.Y ||
				Node.Low.Z > High.Z
			) continue;
			if(!Node.Count) {
				Stack[Pending++] = Node.Left;
				Stack[Pending++] = Node.Right;
				continue;
			}
			// 包囲箱を通過した弧と現在の被覆の照合
			for(std::uint32_t I = Node.Begin; I < Node.Begin + Node.Count; ++I) {
				const Guide &Arc = Guides[Order[I]];
				const double Separation = (C - Arc.Center).Length();
				if(Separation > Cap + Arc.Cap + (Arc.IsRiver ? BankSupportChord : 0)) continue;
				const double Distance = std::max(
					0.01,
					(Arc.Center * (View.RadiusMeters + Arc.ElevationMeters) - View.ObserverMeters).Length() -
					(Cap + Arc.Cap + BankSupportChord) * (View.RadiusMeters + std::abs(Arc.ElevationMeters))
				);
				// 水面が既に被覆済でも，開削の支持幅全体に河岸の描画寸法を確保
				if(Arc.IsRiver) {
					TouchesBank = true;
					// 狭い岸を標本で発見する初期寸法；其の後は実際の形状差と法線差で分割
					Limit = std::min(Limit, std::max(HalfWidthMeters * 0.5, 8 * Distance / View.FocalLengthPixels));
				}
				if(Separation > Cap + Arc.Cap || !ArcIntersectsSphericalTriangle(Arc.Start, Arc.End, Triangle)) continue;
				if(Arc.IsRiver && ArcCoveredByLinearWater(Arc.Start, Arc.End, Triangle, Depths)) continue;
				if(!Arc.IsRiver && Depths[0] > 0 && Depths[1] > 0 && Depths[2] > 0) continue;
				const double Edge =
				Arc.IsRiver ? HalfWidthMeters * 0.5 : std::max(HalfWidthMeters * 0.5, 8 * Distance / View.FocalLengthPixels);
				Limit = std::min(Limit, Edge);
			}
		}
		// 同じ不変な幾何から求めた条件の返戻
		return Limit;
	}

	/**
	 * 全球地形と同じ初期分割から，所属する川・湖の水面だけを精度検査して切り出す
	 * @param Terrain 同じ場の描画候補；成功時だけ水面と共通の分割へ交換
	 * @param View 身体の近景と描画視点
	 * @param Diagnostics 拒否時も残す誤差と費用
	 * @param Stop 生成中断
	 * @param GroundDiagnostics 共通分割へ載せ直した地面の精度と拒否理由，省略可
	 * @return 有効な水面；海面・他の水系は此の結果に含めない
	 */
	std::optional<FlowWaterMesh> FlowWaterView::Build(
		FlowTerrainMesh &Terrain,
		const LodView &View,
		WaterRefinementDiagnostics &Diagnostics,
		const std::stop_token Stop,
		FlowViewDiagnostics *const GroundDiagnostics
	) {
		// 入力定義の照合と診断の初期化
		Diagnostics = {};
		// 異なる尺度又は不正な視点の水面を生成しない返戻
		const bool Invalid = Terrain.SourceField != Field || !View.IsValid() || View.RadiusMeters != Field->GetRadiusMeters() ||
		Terrain.Frame.RadiusMeters != Field->GetRadiusMeters() || Stop.stop_requested();
		// 不一致又は中断した候補への返戻
		if(Invalid) return {};
		// 同じ座標基準の水位と符号付き所属の照会
		const GroundFrame &Frame = Terrain.Frame;
		const UnitDirection Anchor = *UnitDirection::FromVector(Frame.Up);
		const auto SampleWater = [&](const UnitDirection Direction, const bool NeedsSupport) -> std::optional<WaterFieldPoint> {
			// 同じ定義・同じ水位照会の方向だけを，丸めずに再利用
			const Vec3 D = Direction.Vector();
			const std::array<double, 3> Key { D.X, D.Y, D.Z };
			const WaterFieldPoint *const Found = Reuse->WaterSamples.Find(Key);
			WaterFieldPoint Point;
			if(Found) {
				++Reuse->WaterHits;
				// 必要な属性が揃っている標本は再格納せず返す
				if(!NeedsSupport || Found->Support) return *Found;
				Point = *Found;
			} else {
				++Reuse->WaterEvaluations;
				if(NeedsSupport) {
					const std::optional<FlowWaterSample> Sample = Field->EvaluateWithSupport(Direction, false);
					// 欠落した組を乾いた水域へ変換しない返戻
					if(!Sample) return {};
					Point = { Sample->Section, Sample->SupportMeters };
					++Reuse->SupportEvaluations;
				} else {
					const std::optional<FlowChannelSection> Sample = Field->Evaluate(Direction, true, false);
					// 水面法線だけの照会で不要な所属を求めない返戻
					if(!Sample) return {};
					Point.Section = *Sample;
				}
				// 未定義の水位を再利用領域へ入れない返戻
				if(!std::isfinite(Point.Section.Terrain.ElevationMeters)) return {};
				const bool InvalidWater = Point.Section.HasWater && (
					!std::isfinite(Point.Section.Water.ElevationMeters) || !std::isfinite(Point.Section.Water.TangentialGradientPerRadian.Length())
				);
				// 非有限な水面勾配を有効な標本として保持しない返戻
				if(InvalidWater) return {};
			}
			// 法線だけの照会で，使用しない所属を追加計算しない
			if(NeedsSupport && !Point.Support) {
				++Reuse->SupportEvaluations;
				Point.Support = Field->WaterSupport(Direction);
			}
			if(NeedsSupport && (!Point.Support || !std::isfinite(*Point.Support) || *Point.Support > 0 && !Point.Section.HasWater)) {
				// 一括照会と遅延照会を同じ所属条件で確認する返戻
				return {};
			}
			Reuse->WaterSamples.Store(Key, Point);
			// 現在の座標基準で組立てる前の不変値の返戻
			return Point;
		};
		const auto Evaluate = [&](const UnitDirection Direction) -> WaterCoverageSample {
			const WaterStageTimer Timer(Reuse->StageMilliseconds[0]);
			const std::optional<WaterFieldPoint> Point = SampleWater(Direction, true);
			// 未定義を乾いた領域へ置換えない返戻
			if(!Point) return { {}, std::numeric_limits<double>::quiet_NaN() };
			const FlowChannelSection &Sample = Point->Section;
			const double Height = Sample.HasWater ? Sample.Water.ElevationMeters : Sample.Terrain.ElevationMeters;
			// 座標変換と水深を現在の要求から組立てる返戻
			return { Frame.SurfaceToLocal(Direction, Height), std::min(Height - Sample.Terrain.ElevationMeters, *Point->Support) };
		};
		// 発見用の幾何条件と画面上の岸精度の準備
		const WaterDetailEvaluator Detail = [&](
			const UnitDirection Center,
			const double Cap,
			const std::array<UnitDirection, 3> &Triangle,
			const std::array<double, 3> &Depths
		) -> double {
			bool TouchesBank = false;
			const double Limit = [&] {
				const WaterStageTimer Timer(Reuse->StageMilliseconds[1]);
				// 索引探索と河道発見の費用を分離する返戻
				return RequiredEdge(Center, Cap, Triangle, Depths, View, TouchesBank);
			}();
			const double Edge = View.RadiusMeters * std::max(
				{
					(Triangle[0].Vector() - Triangle[1].Vector()).Length(),
					(Triangle[1].Vector() - Triangle[2].Vector()).Length(),
					(Triangle[2].Vector() - Triangle[0].Vector()).Length()
				}
			);
			// 既に細分化が必要な面と，河岸の外では追加照会をしない返戻
			if(!TouchesBank || Edge > Limit) return Limit;
			const WaterStageTimer Timer(Reuse->StageMilliseconds[2]);
			// 身体の原点が移っても，同じ三頂点の補間残差を再利用
			std::array<double, 9> Key;
			for(unsigned I = 0; I < 3; ++I) {
				const Vec3 P = Triangle[I].Vector();
				Key[I * 3] = P.X;
				Key[I * 3 + 1] = P.Y;
				Key[I * 3 + 2] = P.Z;
			}
			const std::array<BankProbePoint, 4> *const Found = Reuse->Checks.Find(Key);
			std::optional<std::array<BankProbePoint, 4>> Points;
			if(Found) {
				++Reuse->Hits;
				Points = *Found;
			} else {
				++Reuse->Evaluations;
				Points = BuildBankProbes(*Field, Triangle, Reuse->Samples);
				if(Points) Reuse->Checks.Store(Key, *Points);
			}
			// 欠落した補間残差を精度達成として扱わない返戻
			if(!Points) return std::numeric_limits<double>::quiet_NaN();
			// 再利用した残差も現在の投影条件で確認する返戻
			return std::min(Limit, BankInterpolationEdge(*Points, Triangle, View, HalfWidthMeters));
		};
		// 前回の河道の分割を土台にし，新しい足元の必須葉を保持したまま追加
		RetainedWaterPartition Initial { Terrain.Sphere, Terrain.Tree };
		if(Reuse->Partition) {
			std::optional<RetainedWaterPartition> Extended = ExtendWaterPartition(*Reuse->Partition, Terrain.Tree, Stop);
			if(Extended) {
				Initial = std::move(*Extended);
				++Reuse->PartitionReuses;
			}
		}
		const WaterCoverageView WaterView { View.ObserverMeters, View.FocalLengthPixels, 8, 0 };
		std::optional<AdaptiveWaterCoverage> Coverage = BuildAdaptiveWaterCoverage(
			std::move(Initial.Sphere.Mesh),
			Frame,
			Anchor,
			View.SurfaceRadiusMeters,
			Evaluate,
			Diagnostics,
			Stop,
			&WaterView,
			Detail,
			SphereRefinementHierarchy{ std::move(Initial.Tree), std::move(Initial.Sphere.TriangleOwners), Initial.Sphere.StitchMaskCounts }
		);
		// 精度未達・予算超過又は中断した水面を公開しない返戻
		if(!Coverage) return {};
		// 完成した被覆と同じ場から求める法線の構築
		FlowWaterMesh Result;
		Result.Frame = Frame;
		Result.TextureOrigin = Terrain.TextureOrigin;
		Result.Surface = std::move(Coverage->Wet);
		Result.Normals.reserve(Result.Surface.Vertices.size());
		Result.PlanetNormals.reserve(Result.Surface.Vertices.size());
		{
			const WaterStageTimer Timer(Reuse->StageMilliseconds[3]);
			// 水際の同じ辺・同じ補間率も，原点の往復変換を挟まず惑星固定の方向へ対応
			const auto NormalDirection = [&](const WaterCoverageVertex &Vertex) -> std::optional<UnitDirection> {
				const UnitDirection A = Coverage->Partition.Directions[Vertex.SourceA];
				// 元頂点の正規方向をそのまま使う返戻
				if(Vertex.SourceA == Vertex.SourceB) return A;
				const UnitDirection B = Coverage->Partition.Directions[Vertex.SourceB];
				const std::optional<WaterFieldPoint> First = SampleWater(A, false), Last = SampleWater(B, false);
				// 元の水面位置を復元出来ない交点の返戻
				if(!First || !Last) return {};
				const FlowChannelSection &P = First->Section, &Q = Last->Section;
				const double HeightA = P.HasWater ? P.Water.ElevationMeters : P.Terrain.ElevationMeters;
				const double HeightB = Q.HasWater ? Q.Water.ElevationMeters : Q.Terrain.ElevationMeters;
				const Vec3 Start = A.Vector() * (1 + HeightA / View.RadiusMeters);
				const Vec3 End = B.Vector() * (1 + HeightB / View.RadiusMeters);
				// 共通の半径で尺度を揃え，実際の辺と同じ補間率を使う返戻
				return UnitDirection::FromVector(
					{
						std::lerp(Start.X, End.X, Vertex.Fraction),
						std::lerp(Start.Y, End.Y, Vertex.Fraction),
						std::lerp(Start.Z, End.Z, Vertex.Fraction)
					}
				);
			};
			for(const WaterCoverageVertex &Vertex : Result.Surface.Vertices) {
				// 法線準備中の中断への返戻
				if(Stop.stop_requested()) return {};
				const std::optional<UnitDirection> Direction = NormalDirection(Vertex);
				// 非有限又は退化した水面方向の返戻
				if(!Direction) return {};
				const std::optional<WaterFieldPoint> Point = SampleWater(*Direction, false);
				// 切出した頂点に定義されない水位への返戻
				if(!Point || !Point->Section.HasWater) return {};
				const TerrainSample &Water = Point->Section.Water;
				const std::optional<UnitDirection> Normal = UnitDirection::FromVector(
					Direction->Vector() * (View.RadiusMeters + Water.ElevationMeters) - Water.TangentialGradientPerRadian
				);
				// 不正な水面法線への返戻
				if(!Normal) return {};
				Result.PlanetNormals.push_back(Normal->Vector());
				Result.Normals.push_back(Frame.VectorToLocal(Normal->Vector()));
			}
		}
		// 所有葉を持たない候補への返戻
		if(!Coverage->Hierarchy) return {};
		// 水面の発見と岸の細分化を，地面にも同じ所有葉で反映
		FlowViewDiagnostics LocalGround;
		FlowViewDiagnostics &GroundReport = GroundDiagnostics ? *GroundDiagnostics : LocalGround;
		GroundReport = {};
		AdaptiveSphereMesh SharedSphere;
		SharedSphere.Mesh = std::move(Coverage->Partition);
		SharedSphere.TriangleOwners = std::move(Coverage->Hierarchy->TriangleOwners);
		SharedSphere.StitchMaskCounts = Coverage->Hierarchy->StitchMaskCounts;
		std::optional<FlowTerrainMesh> SharedGround = [&] {
			const WaterStageTimer Timer(Reuse->StageMilliseconds[4]);
			// 共通分割の地面を作り直す費用を分離する返戻
			return BuildFlowTerrainPartition(
				Field,
				View,
				std::move(SharedSphere),
				std::move(Coverage->Hierarchy->Tree),
				Stop,
				GroundReport,
				Reuse->Partition ? &*Reuse->Partition : nullptr
			);
		}();
		// 地面が同じ分割と精度を満たさない組を公開しない返戻
		if(!SharedGround) return {};
		// 以前の視点の高密度分割を無制限に蓄積せず，保持枠を超えたら次回は通常生成
		if(SharedGround->Tree.Leaves().size() < 131073) Reuse->Partition = *SharedGround; // 同時に保持する分割の葉数上限１３１，０７２
		else Reuse->Partition.reset();
		Terrain = std::move(*SharedGround);
		// 同じ場の水位と所属から作った候補の返戻
		return Result;
	}

	/**
	 * 発見用の保持規模を取得する
	 * @return 河道と岸の短い弧の総数
	 */
	std::size_t FlowWaterView::GuideCount() const {
		return Guides.size();
	}

	/**
	 * 元の河道から発見用の弧への有限標本の誤差を取得する
	 * @return 四分点・中点で測った最大差，単位はメートル
	 */
	double FlowWaterView::GetMaximumGuideErrorMeters() const {
		return MaximumGuideErrorMeters;
	}
}
