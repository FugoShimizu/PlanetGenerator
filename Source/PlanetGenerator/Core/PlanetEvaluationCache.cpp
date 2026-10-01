#include "PlanetEvaluationCache.h"
#include <bit>
#include <list>
#include <unordered_map>
namespace PlanetCore {
	namespace {
		struct DirectionBits {
			std::uint64_t X, Y, Z;
			bool operator==(const DirectionBits &) const = default;
			explicit DirectionBits(UnitDirection D) {
				const auto V = D.Vector();
				X = std::bit_cast<std::uint64_t>(V.X);
				Y = std::bit_cast<std::uint64_t>(V.Y);
				Z = std::bit_cast<std::uint64_t>(V.Z);
			}
		};
		struct DirectionHash {
			std::size_t operator()(const DirectionBits &K) const {
				const auto Mix = [](std::uint64_t X) {
					X ^= X >> 30;
					X *= 0xbf58476d1ce4e5b9ULL;
					X ^= X >> 27;
					X *= 0x94d049bb133111ebULL;
					return X ^ (X >> 31);
				};
				return static_cast<std::size_t>(Mix(K.X) ^ std::rotl(Mix(K.Y), 21) ^ std::rotl(Mix(K.Z), 42));
			}
		};
		template <class Value> class ExactLru {
			struct Entry {
				Value Data;
				typename std::list<DirectionBits>::iterator Recent;
			};
			std::size_t Capacity;
			std::list<DirectionBits> Recent;
			std::unordered_map<DirectionBits, Entry, DirectionHash> Entries;

		public:
			explicit ExactLru(std::size_t C) : Capacity(C) {
				Entries.reserve(C);
			}
			std::size_t Size() const {
				return Entries.size();
			}
			std::size_t Limit() const {
				return Capacity;
			}
			void Grow(std::size_t C) {
				if(C <= Capacity) return;
				Entries.reserve(C);
				Capacity = C;
				return;
			}
			template <class Evaluator>
			Value Get(UnitDirection D, Evaluator &&Evaluate, std::uint64_t &Misses, std::uint64_t &Hits) {
				const DirectionBits Key(D);
				if(const auto It = Entries.find(Key); It != Entries.end()) {
					Recent.splice(Recent.begin(), Recent, It->second.Recent);
					++Hits;
					return It->second.Data;
				}
				const Value Data = Evaluate(D);
				++Misses;
				if(Capacity == 0) return Data;
				if(Entries.size() >= Capacity) {
					Entries.erase(Recent.back());
					Recent.pop_back();
				}
				Recent.push_front(Key);
				Entries.emplace(Key, Entry{Data, Recent.begin()});
				return Data;
			}
		};
	}
	struct TerrainEvaluationCache::Impl {
		std::shared_ptr<const RiverTerrainField> Field;
		ExactLru<RiverSurfaceSample> Samples;
		ExactLru<double> Heights;
		ExactLru<RiverWaterHeights> WaterSamples{0};
		EvaluationCacheStats Stats;
		Impl(std::shared_ptr<const RiverTerrainField> F, std::size_t S, std::size_t H)
			: Field(std::move(F)), Samples(std::min(S, MaximumSamples)), Heights(std::min(H, MaximumHeights)) {
		}
	};
	TerrainEvaluationCache::TerrainEvaluationCache(std::shared_ptr<const RiverTerrainField> Field, std::size_t S, std::size_t H)
		: State(std::make_unique<Impl>(std::move(Field), S, H)) {
	}
	TerrainEvaluationCache::~TerrainEvaluationCache() = default;
	TerrainSample TerrainEvaluationCache::Evaluate(UnitDirection D) {
		return EvaluateSurface(D).Terrain;
	}
	RiverSurfaceSample TerrainEvaluationCache::EvaluateSurface(UnitDirection D) {
		return State->Samples.Get(
			D, [&](UnitDirection P) { return State->Field->EvaluateSurface(P); }, State->Stats.Samples,
			State->Stats.SampleHits);
	}
	RiverWaterHeights TerrainEvaluationCache::EvaluateWater(const UnitDirection D) {
		// 水面だけの値を独立した有界枠から返す
		return State->WaterSamples.Get(
			D, [&](const UnitDirection P) { return State->Field->EvaluateWater(P); }, State->Stats.WaterSamples,
			State->Stats.WaterHits);
	}
	void TerrainEvaluationCache::GrowWaterSamples(const std::size_t Capacity) {
		State->WaterSamples.Grow(std::min(Capacity, MaximumSamples));
		// 終了
		return;
	}
	std::size_t TerrainEvaluationCache::WaterSampleCount() const {
		// 現在保持する水面標本数の返戻
		return State->WaterSamples.Size();
	}
	std::size_t TerrainEvaluationCache::WaterSampleCapacity() const {
		// 水面標本の保持上限の返戻
		return State->WaterSamples.Limit();
	}
	double TerrainEvaluationCache::EvaluateElevation(UnitDirection D) {
		return State->Heights.Get(
			D, [&](UnitDirection P) { return State->Field->EvaluateElevation(P); }, State->Stats.Heights,
			State->Stats.HeightHits);
	}
	EvaluationCacheStats TerrainEvaluationCache::Statistics() const {
		return State->Stats;
	}
	std::size_t TerrainEvaluationCache::SampleCount() const {
		return State->Samples.Size();
	}
	std::size_t TerrainEvaluationCache::SampleCapacity() const {
		return State->Samples.Limit();
	}
	void TerrainEvaluationCache::GrowSamples(std::size_t Capacity) {
		State->Samples.Grow(std::min(Capacity, MaximumSamples));
		return;
	}
	std::size_t TerrainEvaluationCache::HeightCount() const {
		return State->Heights.Size();
	}
	std::optional<GroundFrame> SelectRenderFrame(
		UnitDirection Anchor, double Radius, double Elevation, const std::optional<GroundFrame> &Previous) {
		const auto Fresh = GroundFrame::Create(Anchor, Radius, Elevation);
		if(!Fresh) return {};
		if(Previous && Previous->RadiusMeters == Radius && Previous->SurfaceToLocal(Anchor, Elevation).Length() <= 128)
			return Previous;
		return Fresh;
	}
}
