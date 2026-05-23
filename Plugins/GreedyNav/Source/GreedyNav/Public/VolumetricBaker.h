#pragma once

#include "CoreMinimal.h"
#include "VolumetricTypes.h" // Ensures FCompactESDFNode is visible

class GREEDYNAV_API FVolumetricBaker
{
public:
    /**
     * Bakes the Distance Field using a Parallel Wavefront algorithm.
     * Uses "Shell Culling" to only store nodes near obstacles (saving RAM).
     * * @param InputGrid  The sparse grid containing obstacle data.
     * @param OutNodes   The output array of compressed nodes (Shell only).
     */
    static void BakeSVO(FSparseVolumetricGrid& Grid, const TArray<int64>& InitialBlockedIndices);
};