#include "PlanetSpillSampleCache.h"
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace PlanetCore {
	/**
	 * 球面方向の倍精度３成分を，丸めない照合キーへ変換する
	 * @param Direction 元場を照会した単位方向
	 * @return 同じ方向を完全一致で照合する３成分のビット列
	 */
	std::array<std::uint64_t, 3> SpillSampleCache::MakeKey(const UnitDirection Direction) {
		const Vec3 D = Direction.Vector();
		// 丸めない３成分の返戻
		return { std::bit_cast<std::uint64_t>(D.X), std::bit_cast<std::uint64_t>(D.Y), std::bit_cast<std::uint64_t>(D.Z) };
	}

	/**
	 * 完全なキーを確認する前に，固定容量の保持枠を選ぶ
	 * @param Key 丸めない３成分のキー
	 * @param Count 正の保持枠数
	 * @return 衝突した場合に別の値を流用しない，候補枠の添字
	 */
	std::size_t SpillSampleCache::FindSlot(const std::array<std::uint64_t, 3> &Key, const std::size_t Count) {
		// 符号無の固定幅演算で，候補枠のみに使う整数を混合
		const auto Mix = [](std::uint64_t Value) -> std::uint64_t {
			Value ^= Value >> 30;
			Value *= UINT64_C(0XBF58476D1CE4E5B9);
			Value ^= Value >> 27;
			Value *= UINT64_C(0X94D049BB133111EB);
			// 候補枠へ使う混合整数の返戻
			return Value ^ Value >> 31;
		};
		// 完全一致の確認を省かない候補枠の返戻
		return static_cast<std::size_t>((Mix(Key[0]) ^ std::rotl(Mix(Key[1]), 21) ^ std::rotl(Mix(Key[2]), 42)) % Count);
	}

	/**
	 * 検査済の固定容量だけを確保する
	 * 計算量：O(Capacity)
	 * @param Capacity 最大容量以下の保持枠数，０では再利用無し
	 */
	SpillSampleCache::SpillSampleCache(const std::uint32_t Capacity) : Entries(Capacity) {
		// 終了
		return;
	}

	/**
	 * 無制限な容量を確保せず，一つの元場の準備用の保持枠を構築する
	 * 計算量：O(Capacity)
	 * @param Capacity ０から MaximumCapacity 迄の保持枠数
	 * @return 有界な再利用枠；上限超過は空
	 */
	std::optional<SpillSampleCache> SpillSampleCache::Create(const std::uint32_t Capacity) {
		// 過大な容量への返戻
		if(Capacity > MaximumCapacity) return {};
		SpillSampleCache Cache(Capacity);
		// 実確保容量も同じバイト上限に収まる場合だけ返す
		if(Cache.GetMemoryBytes() > MaximumByteCapacity) return {};
		// 有界な保持枠の返戻
		return Cache;
	}

	/**
	 * 同じ元場で保存した完全一致の方向だけを照会する
	 * 計算量：O(1)（固定容量の直接照合）
	 * @param Direction 元場の照会方向
	 * @return 保存済の標高と全勾配，未保持・衝突・無効化では空
	 */
	std::optional<TerrainSample> SpillSampleCache::Find(const UnitDirection Direction) const {
		// 無効化された保持枠への返戻
		if(Entries.empty()) return {};
		const std::array<std::uint64_t, 3> Key = MakeKey(Direction);
		const Entry &Stored = Entries[FindSlot(Key, Entries.size())];
		// 未評価又は異なる方向の枠への返戻
		if(!Stored.IsOccupied || Stored.Key != Key) return {};
		// 元場で評価済の同じ標高と全勾配の返戻
		return Stored.Value;
	}

	/**
	 * 非有限値を保持せず，衝突した枠だけを現在の同じ元場の標本で置換える
	 * 計算量：O(1)（候補枠一つの置換）
	 * @param Direction 評価した方向
	 * @param Value 同じ不変な元場の有限な標高と全勾配
	 */
	void SpillSampleCache::Store(const UnitDirection Direction, const TerrainSample &Value) {
		// 無効な値又は無効化された枠への返戻
		if(Entries.empty() || !std::isfinite(Value.ElevationMeters) || !std::isfinite(Value.TangentialGradientPerRadian.Length())) {
			return;
		}
		const std::array<std::uint64_t, 3> Key = MakeKey(Direction);
		Entry &Stored = Entries[FindSlot(Key, Entries.size())];
		if(!Stored.IsOccupied) ++Used;
		Stored = { Key, Value, true };
		// 終了
		return;
	}

	/**
	 * 保持領域の実容量を，標本数とは区別して取得する
	 * 計算量：時間・追加空間 O(1)
	 * @return 同じ準備が所有する標本領域のバイト数
	 */
	std::size_t SpillSampleCache::GetMemoryBytes() const {
		// allocator の容量と要素の実寸から求めた値の返戻
		return Entries.capacity() * sizeof(Entry);
	}

	/**
	 * 衝突で置換えた値を重複して数えず，使用中の有界な保持量を得る
	 * @return 使用中の枠数
	 */
	std::uint32_t SpillSampleCache::GetCount() const {
		return Used;
	}
}
