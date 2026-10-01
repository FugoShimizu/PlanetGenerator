#include "PlanetRiverTerrain.h"
#include "PlanetBasinFill.h"
#include "PlanetBasinPreparation.h"
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

namespace PlanetCore {
	namespace {
		/** 正規化方向の全ビットと元標高だけを持つ，一準備限りの直接写像枠 */
		struct BasinHeightEntry {
			std::uint64_t X = std::numeric_limits<std::uint64_t>::max();
			std::uint64_t Y = 0;
			std::uint64_t Z = 0;
			double Height = 0;
		};

		/**
		 * 既存の地点評価キャッシュと同じ整数混合で方向のビットを分散する
		 * @param Value 浮動小数点数の格納ビット
		 * @return 枠選択用の混合値，一致の判定には元の３成分を全て使う
		 */
		std::uint64_t MixBasinDirection(std::uint64_t Value) {
			Value ^= Value >> 30;
			Value *= 0XBF58476D1CE4E5B9ULL;
			Value ^= Value >> 27;
			Value *= 0X94D049BB133111EBULL;
			// 丸めや量子化を行わない枠選択用の値の返戻
			return Value ^ Value >> 31;
		}
	}

	/**
	 * 共有縦断の盆地盛土と河道開削を同じ地点評価へ結合する比較用の準備経路
	 * 計算量：O(n log² n ＋ q k log k ＋ b n)，n は排水網，q は河道標本数，k は補間近傍数，b は下限の分割数
	 * 下限標本の作業配列は一部分だけ保持し，一度に扱う個数上限を変更しない
	 * @param Ocean 元の海底と接続済みの海域
	 * @param Drainage 同じ生成定義の排水，開削・盛土の共有縦断なら再準備を省く
	 * @param Stop 中断要求
	 * @param Preparation 任意の準備計測，時刻は生成値へ使用しない
	 * @param HeightCacheCapacity 一準備限りの元標高保持数，０で無効，２の冪且つ2097152以下
	 * @return 盛土と河床が一体化した場，不正な生成定義・準備失敗・中断では空
	 */
	std::optional<RiverTerrainField> RiverTerrainField::CreateWithBasins(
		const std::shared_ptr<const OceanConnectivityField> &Ocean,
		const std::shared_ptr<const DrainageNetwork> &Drainage,
		const std::stop_token Stop,
		BasinPreparationStatistics *Preparation,
		const std::size_t HeightCacheCapacity
	) {
		if(Preparation) *Preparation = {};
		using Clock = std::chrono::steady_clock;
		const Clock::time_point Started = Preparation ? Clock::now() : Clock::time_point{};
		Clock::time_point Previous = Started;
		// 同じスレッドの段階境界だけで時刻を読み，地形の入力には使わない
		const auto RecordPhase = [&](const BasinPreparationPhase Phase) {
			// 計測しない正規経路では時計も読まない返戻
			if(!Preparation) return;
			const Clock::time_point Now = Clock::now();
			Preparation->PhaseMilliseconds[static_cast<std::size_t>(Phase)] +=
			std::chrono::duration<double, std::milli>(Now - Previous).count();
			Previous = Now;
		};
		// 別の生成定義や中断された入力の返戻
		if(!Ocean || !Drainage || Ocean->DefinitionHash() != Drainage->SourceDefinitionHash() || Stop.stop_requested()) return {};
		// 不正な容量を大きな割当てや別の生成方式へ黙って変換しない返戻
		if(HeightCacheCapacity > 2097152 || HeightCacheCapacity && !std::has_single_bit(HeightCacheCapacity)) return {};
		std::vector<BasinHeightEntry> HeightCache(HeightCacheCapacity);
		if(Preparation) Preparation->HeightCacheBytes = HeightCache.size() * sizeof(BasinHeightEntry);
		// 準備と最終評価に使う同じ元地形
		const auto OriginalHeight = [&Ocean, &HeightCache, Preparation](const UnitDirection Direction) -> double {
			const Vec3 Position = Direction.Vector();
			const std::uint64_t X = std::bit_cast<std::uint64_t>(Position.X), Y = std::bit_cast<std::uint64_t>(Position.Y);
			const std::uint64_t Z = std::bit_cast<std::uint64_t>(Position.Z);
			BasinHeightEntry *Entry = nullptr;
			if(!HeightCache.empty()) {
				const std::uint64_t Hash = MixBasinDirection(X) ^ std::rotl(MixBasinDirection(Y), 21) ^ std::rotl(MixBasinDirection(Z), 42);
				Entry = &HeightCache[static_cast<std::size_t>(Hash) & HeightCache.size() - 1];
				if(Entry->X == X && Entry->Y == Y && Entry->Z == Z) {
					if(Preparation) ++Preparation->OriginalHeightHits;
					// 同じ不変場，同じ方向の全ビットが一致した標高だけの返戻
					return Entry->Height;
				}
			}
			const double Height = Ocean->EvaluateElevation(Direction);
			if(Preparation) ++Preparation->OriginalHeightEvaluations;
			if(Entry) *Entry = { X, Y, Z, Height };
			// 衝突時も実地形を再評価し，置換え順序に依らない標高の返戻
			return Height;
		};
		std::shared_ptr<const DrainageNetwork> Balanced = Drainage;
		if(!Balanced->RequiresBalancedTerrain()) {
			std::optional<DrainageNetwork> Prepared = Drainage->WithBalancedTerrainProfiles(OriginalHeight, Stop);
			// 共有縦断を作れない場合の返戻
			if(!Prepared) return {};
			Balanced = std::make_shared<const DrainageNetwork>(std::move(*Prepared));
		}
		RecordPhase(BasinPreparationPhase::BalancedProfiles);
		constexpr double CoastBand = 1, RelativeCoastalLimit = 4;
		const double Sea = Balanced->Settings().SeaLevelMeters, Freeboard = Balanced->Settings().BankFreeboardMeters;
		// 最終面だけを切詰めず，同じ容量の線形部分に河川水位と河岸を先に収める
		const auto CoastalHeight = [&](const UnitDirection Direction) -> double {
			const double Height = OriginalHeight(Direction);
			// 余裕を持つ容量内の地面を，既存の共有縦断の上限へ渡す返戻
			return Height + 0.4 * PlanetBasinFill::CoastalRaiseCapacity(Height - Sea, CoastBand, RelativeCoastalLimit);
		};
		std::optional<DrainageNetwork> CoastalProfiles = Balanced->WithTerrainProfiles(CoastalHeight, Stop);
		// 海岸容量と共有水位を一緒に準備出来ない場合の返戻
		if(!CoastalProfiles) return {};
		Balanced = std::make_shared<const DrainageNetwork>(std::move(*CoastalProfiles));
		RecordPhase(BasinPreparationPhase::CoastalProfiles);
		constexpr unsigned Level = 6, Segments = RiverLongitudinalProfile::Segments;
		const std::optional<SphereMesh> Mesh = BuildUniformSphere(Level);
		// 固定網と元の排水標本の対応を持たない入力の返戻
		if(!Mesh || Balanced->Nodes().size() < Mesh->Directions.size()) return {};
		// 河口で一定海面へ近付く氾濫原の最低標高
		const auto Floodplain = [Sea, Freeboard](const double Water) -> double {
			const double Head = std::max(0.0, Water - Sea);
			// 余裕高０を含めた氾濫原標高の返戻
			return Water + (Head > 0 ? Freeboard * Head / (Head + Freeboard) : 0);
		};
		std::vector<double> Original, Minimum;
		for(std::size_t Index = 0; Index < Mesh->Directions.size(); ++Index) {
			const DrainageNode &Node = Balanced->Nodes()[Index];
			// 対応して居ない網の順序や中断の返戻
			if((Node.Direction - Mesh->Directions[Index].Vector()).Length() > 1E-12 || Stop.stop_requested()) return {};
			const double Height = OriginalHeight(Mesh->Directions[Index]);
			Original.push_back(Height);
			Minimum.push_back(Node.Ocean ? Height : Floodplain(Node.WaterMeters));
		}
		RecordPhase(BasinPreparationPhase::FloorSamples);
		std::optional<PlanetBasinFill> Fill =
		PlanetBasinFill::CreateWithCoastalLimit(Level, Original, Minimum, Sea, CoastBand, RelativeCoastalLimit, Stop);
		// 盆地底の初期補間に失敗した場合の返戻
		if(!Fill) return {};
		RecordPhase(BasinPreparationPhase::InitialFill);
		constexpr std::size_t ConstraintBatchSize = 16384;
		std::vector<UnitDirection> Locations;
		Locations.reserve(ConstraintBatchSize);
		Original.clear();
		Minimum.clear();
		/**
		 * 元の順序を保って下限を適用し，作業中の標本だけを解放する
		 * @return 全ての下限を満たす場合に真，中断又は準備失敗では偽
		 */
		const auto ApplyConstraints = [&]() -> bool {
			// 中断された準備を公開しない返戻
			if(Stop.stop_requested()) return false;
			// 空の末尾に不要な再準備を加えない返戻
			if(Locations.empty()) return true;
			RecordPhase(BasinPreparationPhase::ConstraintAssembly);
			Fill = Fill->WithMinimumElevations(Locations, Original, Minimum, Stop);
			RecordPhase(BasinPreparationPhase::ConstraintApplication);
			// 下限の適用に失敗した場合の返戻
			if(!Fill) return false;
			Locations.clear();
			Original.clear();
			Minimum.clear();
			// 後続の正の係数更新でも以前の下限を保持する部分適用の返戻
			return true;
		};
		for(std::size_t Reach = 0; Reach < Balanced->Reaches().size(); ++Reach) {
			const std::optional<RiverCrossSectionWidths> Widths = Balanced->ChannelWidths(Reach);
			// 不正な河岸幅又は中断の返戻
			if(!Widths || Stop.stop_requested()) return {};
			const double Angle = Widths->InnerHalfWidthMeters / Balanced->Settings().RadiusMeters;
			for(unsigned Knot = 0; Knot < Segments + 1; ++Knot) {
				const std::optional<RiverProfileSample> Sample = Balanced->SampleReach(Reach, static_cast<double>(Knot) / Segments);
				// 河道内部の標本を作れない場合の返戻
				if(!Sample) return {};
				const Vec3 Across = UnitDirection::FromVector(Sample->Direction.Cross(Sample->DownstreamTangent))->Vector();
				for(const double Side : { -1.0, 0.0, 1.0 }) {
					const UnitDirection Direction =
					*UnitDirection::FromVector(Sample->Direction * std::cos(Side * Angle) + Across * std::sin(Side * Angle));
					const double Height = OriginalHeight(Direction);
					if(Height <= Sea) continue;
					Locations.push_back(Direction);
					Original.push_back(Height);
					Minimum.push_back(Floodplain(Sample->WaterMeters));
					// 全河川の標本を蓄積せず，有限個の同じ順序の下限を適用する返戻
					if(Locations.size() == ConstraintBatchSize && !ApplyConstraints()) return {};
				}
			}
		}
		// 最後の部分に有る下限又は中断を処理出来ない場合の返戻
		if(!ApplyConstraints()) return {};
		RecordPhase(BasinPreparationPhase::ConstraintAssembly);
		const std::shared_ptr<const PlanetBasinFill> Basins = std::make_shared<const PlanetBasinFill>(std::move(*Fill));
		// 最終の面に合わせ，河岸の準備標本間も調べて水位の上限を再構成する
		const auto FilledHeight = [&Basins, &OriginalHeight](const UnitDirection Direction) -> double {
			// 同じ盛土を適用した地点標高の返戻
			return Basins->ApplyElevation(Direction, OriginalHeight(Direction));
		};
		std::optional<DrainageNetwork> Refitted = Balanced->WithTerrainProfiles(FilledHeight, Stop);
		// 盛土と整合する開削側の縦断を作れない場合の返戻
		if(!Refitted) return {};
		RecordPhase(BasinPreparationPhase::FinalProfiles);
		std::optional<RiverTerrainField> Result = Create(Ocean, std::make_shared<const DrainageNetwork>(std::move(*Refitted)), Stop);
		// 共通の河道合成に失敗した場合の返戻
		if(!Result) return {};
		Result->Basins = Basins;
		Result->ReliefBoundMeters += Basins->Statistics().AbsoluteBoundMeters + PlanetBasinFill::SmoothingOvershootBoundMeters();
		RecordPhase(BasinPreparationPhase::FieldAssembly);
		if(Preparation) {
			Preparation->TotalMilliseconds = std::chrono::duration<double, std::milli>(Previous - Started).count();
			Preparation->Completed = true;
		}
		// 描画と衝突へ同じ式を渡せる盛土・開削場の返戻
		return Result;
	}
}
