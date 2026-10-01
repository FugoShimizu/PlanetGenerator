#include "PlanetFlowRenderSection.h"
#include "Core/PlanetFlowView.h"
#include "Core/PlanetFlowWater.h"
#include "Core/PlanetSurfaceMaterial.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stop_token>

namespace {
	/**
	 * 元の倍精度成分をエンジンの座標へ渡す
	 * @param Value 変換する座標
	 * @return 同じ三成分
	 */
	FVector ToVector(const PlanetCore::Vec3 Value) {
		return { Value.X, Value.Y, Value.Z };
	}

	/**
	 * UObject に触れずに描画頂点・接続・包囲箱を構築する
	 * @param Count 頂点数
	 * @param Triangles 元の右手系の接続
	 * @param At 頂点属性を同じ標本から構成する操作
	 * @param Stop 所有ワーカーの中断要求
	 * @return 完全な部品用配列；不正な接続又は中断では値無し
	 * @note 頂点数 V と三角形数 T に対して時間・空間共に O(V+T)
	 */
	template<class VertexAt> std::optional<FProcMeshSection> PrepareSection(
		const std::size_t Count,
		const std::span<const std::array<std::uint32_t, 3>> Triangles,
		const VertexAt &At,
		const std::stop_token Stop
	) {
		// エンジンの配列長を越す候補と中断済の返戻
		if(Count > MAX_int32 || Triangles.size() > MAX_int32 / 3 || Stop.stop_requested()) return {};
		FProcMeshSection Result;
		Result.ProcVertexBuffer.Reserve(static_cast<int32>(Count));
		Result.ProcIndexBuffer.Reserve(static_cast<int32>(3 * Triangles.size()));
		for(std::size_t I = 0; I < Count; ++I) {
			// 中断後の部分配列を公開しない返戻
			if(!(I & 1023) && Stop.stop_requested()) return {}; // 中断照会の間隔１，０２４要素
			const std::optional<FProcMeshVertex> Vertex = At(I);
			// 非有限な包囲箱と無効な接線を部品へ渡さない返戻
			if(!Vertex || Vertex->Position.ContainsNaN() || Vertex->Normal.ContainsNaN()) return {};
			Result.ProcVertexBuffer.Add(*Vertex);
			Result.SectionLocalBox += Vertex->Position;
		}
		for(std::size_t I = 0; I < Triangles.size(); ++I) {
			// 中断済の返戻
			if(!(I & 1023) && Stop.stop_requested()) return {};
			const std::array<std::uint32_t, 3> Triangle = Triangles[I];
			const std::uint32_t A = Triangle[0], B = Triangle[1], C = Triangle[2];
			// 範囲外又は重複する接続を丸めて別の形状にしない返戻
			if(A >= Count || B >= Count || C >= Count || A == B || A == C || B == C) return {};
			Result.ProcIndexBuffer.Append({ A, C, B });
		}
		// 最後の接続後に中断された候補の返戻
		if(Stop.stop_requested()) return {};
		// 全配列と包囲箱を揃えた候補の返戻
		return Result;
	}
}

/**
 * 地面の解析属性を，従来の標準部品と同じ形式へ構成する
 * @param Mesh 同じ元場から準備した地面
 * @param Stop 所有ワーカーの中断要求
 * @return センチメートル座標・量子化色を持つ未公開の配列
 */
std::optional<FProcMeshSection> PreparePlanetFlowSection(const PlanetCore::FlowTerrainMesh &Mesh, const std::stop_token Stop) {
	const std::size_t Count = Mesh.Positions.size();
	const bool Complete = Mesh.Normals.size() == Count && Mesh.PlanetNormals.size() == Count && Mesh.Materials.size() == Count &&
	Mesh.Heights.size() == Count && Mesh.WaterCoordinates.size() == Count;
	// 属性が欠落した候補の返戻
	if(!Complete) return {};
	// UObject を参照しない属性変換と接続の返戻
	return PrepareSection(
		Count,
		Mesh.Sphere.Mesh.Triangles,
		[&Mesh](const std::size_t I) -> std::optional<FProcMeshVertex> {
			const PlanetCore::Vec3 PN = Mesh.PlanetNormals[I];
			const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(PN);
			// 惑星固定の接線を構成出来ない候補の返戻
			if(!Direction) return {};
			const PlanetCore::SurfaceMaterialWeights &Weights = Mesh.Materials[I];
			const double Height = Mesh.Heights[I], Depth = Mesh.WaterCoordinates[I][0], Support = Mesh.WaterCoordinates[I][1];
			// 半精度へ変換する前も，非有限な属性を飽和で隠さない返戻
			if(!std::isfinite(Height) || !std::isfinite(Depth) || !std::isfinite(Support)) return {};
			FProcMeshVertex Result;
			Result.Position = ToVector(Mesh.Positions[I]) * 100;
			Result.Normal = ToVector(Mesh.Normals[I]);
			Result.Tangent = FProcMeshTangent(ToVector(Mesh.Frame.VectorToLocal(PlanetCore::SurfaceMaterialTangent(*Direction))), false);
			// 半精度で有限な搬送範囲；地形と水位そのものは変更しない
			Result.UV0 = FVector2D(std::clamp(Height, -32768.0, 32768.0), std::clamp(Depth, -32768.0, 32768.0));
			Result.UV1 = FVector2D(std::clamp(Support, -32768.0, 32768.0), 0);
			Result.UV2 = FVector2D(PN.X, PN.Y);
			Result.UV3 = FVector2D(PN.Z, 0);
			Result.Color = FLinearColor(
				static_cast<float>(Weights.Rock),
				static_cast<float>(Weights.Sand),
				static_cast<float>(Weights.Soil),
				static_cast<float>(Weights.Wetness)
			).ToFColor(false);
			// 従来の量子化色と，新しい被覆座標を保持した返戻
			return Result;
		},
		Stop
	);
}

/**
 * 水面の解析属性を，地面と同じ局所座標の描画配列へ構成する
 * @param Mesh 同じ元場から準備した水面
 * @param Stop 所有ワーカーの中断要求
 * @return 白い頂点色と惑星固定の模様座標を持つ未公開の配列
 */
std::optional<FProcMeshSection> PreparePlanetFlowSection(const PlanetCore::FlowWaterMesh &Mesh, const std::stop_token Stop) {
	const std::size_t Count = Mesh.Surface.Vertices.size();
	// 属性が欠落した候補の返戻
	if(Mesh.Normals.size() != Count || Mesh.PlanetNormals.size() != Count) return {};
	// 完全な水面又は有効な空の水面の返戻
	return PrepareSection(
		Count,
		Mesh.Surface.Triangles,
		[&Mesh](const std::size_t I) -> std::optional<FProcMeshVertex> {
			const PlanetCore::Vec3 PN = Mesh.PlanetNormals[I];
			const std::optional<PlanetCore::UnitDirection> Direction = PlanetCore::UnitDirection::FromVector(PN);
			// 接線を構成出来ない候補の返戻
			if(!Direction) return {};
			FProcMeshVertex Result;
			Result.Position = ToVector(Mesh.Surface.Vertices[I].Position) * 100;
			Result.Normal = ToVector(Mesh.Normals[I]);
			Result.Tangent = FProcMeshTangent(ToVector(Mesh.Frame.VectorToLocal(PlanetCore::SurfaceMaterialTangent(*Direction))), false);
			Result.UV2 = FVector2D(PN.X, PN.Y);
			Result.UV3 = FVector2D(PN.Z, 0);
			// 既定の白色・UV0・UV1 を保持した返戻
			return Result;
		},
		Stop
	);
}
