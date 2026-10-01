#include "PlanetRealtimeMeshSection.h"
#include "Core/RealtimeMeshDataTypes.h"
#include "DynamicMeshBuilder.h"
#include "ProceduralMeshComponent.h"
#include <utility>

namespace {
	using PackedTangents = RealtimeMesh::TRealtimeMeshTangents<FPackedNormal>;
	using TextureCoordinates = RealtimeMesh::TRealtimeMeshTexCoords<FVector2DHalf, 4>;

	/**
	 * 完成形の要素数を一度に確保し，頂点毎の追加通知と既定値の書込みを省く
	 * @param Streams 配列を所有する未公開の集合
	 * @param Key 属性の識別子
	 * @param Count 完成時の要素数
	 * @return 集合が所有する書込み範囲；全要素を初期化してから公開する事
	 */
	template<class T> TArrayView<T> AllocateStream(
		RealtimeMesh::FRealtimeMeshStreamSet &Streams,
		const FRealtimeMeshStreamKey &Key,
		const int32 Count
	) {
		RealtimeMesh::FRealtimeMeshStream &Stream = Streams.AddStream<T>(Key);
		Stream.SetNumUninitialized(Count);
		// 新しく確保した範囲の返戻
		return Stream.GetArrayView<T>();
	}
}

/**
 * 既存描画の局所座標・量子化形式・四組の模様座標を保持して変換する
 * @param Section 衝突を含まない完全な描画区画
 * @param Stop 所有ワーカーの中断要求
 * @return 公開前の転送配列；不正な属性・接続又は中断では値無し
 * @note UObject を使わず，頂点数と接続数に比例する時間・空間を使う
 */
std::optional<RealtimeMesh::FRealtimeMeshStreamSet> PreparePlanetRealtimeSection(
	const FProcMeshSection &Section,
	const std::stop_token Stop
) {
	// 衝突の黙示的な脱落と部分的な三角形を拒否
	if(Section.bEnableCollision || Section.ProcIndexBuffer.Num() % 3 || Stop.stop_requested()) return {};
	RealtimeMesh::FRealtimeMeshStreamSet Streams;
	const int32 VertexCount = Section.ProcVertexBuffer.Num();
	const TArrayView<FVector3f> Positions =
	AllocateStream<FVector3f>(Streams, RealtimeMesh::FRealtimeMeshStreams::Position, VertexCount);
	const TArrayView<PackedTangents> Tangents =
	AllocateStream<PackedTangents>(Streams, RealtimeMesh::FRealtimeMeshStreams::Tangents, VertexCount);
	const TArrayView<TextureCoordinates> UVs =
	AllocateStream<TextureCoordinates>(Streams, RealtimeMesh::FRealtimeMeshStreams::TexCoords, VertexCount);
	const TArrayView<FColor> Colors = AllocateStream<FColor>(Streams, RealtimeMesh::FRealtimeMeshStreams::Color, VertexCount);
	const TArrayView<RealtimeMesh::TIndex3<uint32>> Triangles = AllocateStream<RealtimeMesh::TIndex3<uint32>>(
		Streams,
		RealtimeMesh::FRealtimeMeshStreams::Triangles,
		Section.ProcIndexBuffer.Num() / 3
	);
	for(int32 I = 0; I < Section.ProcVertexBuffer.Num(); ++I) {
		// 中断後の部分配列を公開しない返戻
		if(!(I & 1023) && Stop.stop_requested()) return {};
		const FProcMeshVertex &Vertex = Section.ProcVertexBuffer[I];
		const FVector3f Position(Vertex.Position), Normal(Vertex.Normal), Tangent(Vertex.Tangent.TangentX);
		const FVector2f UV0(Vertex.UV0), UV1(Vertex.UV1), UV2(Vertex.UV2), UV3(Vertex.UV3);
		const bool Invalid =
		Position.ContainsNaN() || Normal.ContainsNaN() || Tangent.ContainsNaN() || UV0.ContainsNaN() || UV1.ContainsNaN() ||
		UV2.ContainsNaN() || UV3.ContainsNaN();
		// 単精度への変換時の桁溢れも含め，非有限な属性を拒否
		if(Invalid) return {};
		const FVector2DHalf Half0(UV0), Half1(UV1), Half2(UV2), Half3(UV3);
		const bool HalfInvalid = static_cast<FVector2f>(Half0).ContainsNaN() || static_cast<FVector2f>(Half1).ContainsNaN() ||
		static_cast<FVector2f>(Half2).ContainsNaN() || static_cast<FVector2f>(Half3).ContainsNaN();
		// 最終的な半精度で非有限に為る模様座標の返戻
		if(HalfInvalid) return {};
		Positions[I] = Position;
		Colors[I] = Vertex.Color;
		UVs[I] = TextureCoordinates(Half0, Half1, Half2, Half3);
		// 標準部品の二段階量子化と基底の符号再計算を維持
		FDynamicMeshVertex Packed;
		Packed.TangentX = Vertex.Tangent.TangentX;
		Packed.TangentZ = Vertex.Normal;
		Packed.TangentZ.Vector.W = Vertex.Tangent.bFlipTangentY ? -127 : 127;
		Tangents[I] = PackedTangents(Packed.TangentZ.ToFVector3f(), Packed.GetTangentY(), Packed.TangentX.ToFVector3f());
	}
	for(int32 I = 0; I < Section.ProcIndexBuffer.Num(); I += 3) {
		// 中断済の返戻
		if(!(I & 1023) && Stop.stop_requested()) return {};
		const uint32 A = Section.ProcIndexBuffer[I], B = Section.ProcIndexBuffer[I + 1], C = Section.ProcIndexBuffer[I + 2];
		const uint32 Count = static_cast<uint32>(Section.ProcVertexBuffer.Num());
		// 元の表裏を変えず，不正な接続を拒否
		if(A >= Count || B >= Count || C >= Count || A == B || A == C || B == C) return {};
		Triangles[I / 3] = RealtimeMesh::TIndex3<uint32>(A, B, C);
	}
	// 最後の配列構築後に届いた中断の返戻
	if(Stop.stop_requested()) return {};
	// 複製せずに所有権を呼出側へ渡す返戻
	return std::move(Streams);
}
