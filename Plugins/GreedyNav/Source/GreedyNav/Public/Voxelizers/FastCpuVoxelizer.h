#pragma once

#include "CoreMinimal.h"
#include "VolumetricTypes.h"

class FFastCpuVoxelizer
{
public:
    // Helper signatures
    static bool IsPointInPrimitive(const FVector& LocalPoint, const FKShapeElem& Shape, int32 ShapeType);
    static float GetTriangleZ(const FVector& V0, const FVector& V1, const FVector& V2, float X, float Y);
    static bool IsPointInTriangle2D(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C);

    // UPDATED SIGNATURE: Now accepts 'OutBlockedIndices'
    static void Voxelize(const UWorld* WorldContext, FVector Center, FVector Extents, float VoxelResolution,
        FSparseVolumetricGrid& OutGrid, TArray<int64>& OutBlockedIndices);
};