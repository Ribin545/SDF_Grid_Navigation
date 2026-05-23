#pragma once

#include "CoreMinimal.h"
#include "VolumetricTypes.h"

class FSparseCollisionVoxelizer
{
public:
    // UPDATED SIGNATURE: Now accepts 'OutIndices'
    static void Voxelize(const UWorld* World, FVector Center, FVector Extents, float Resolution,
        FSparseVolumetricGrid& OutGrid, TArray<int64>& OutIndices);

};