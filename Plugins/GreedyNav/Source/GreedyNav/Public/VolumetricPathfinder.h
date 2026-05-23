#pragma once

#include "CoreMinimal.h"
#include "VolumetricTypes.h"

class FVolumetricPathfinder
{
public:
    // --- MAIN PATHFINDING ---
    static bool FindPath(const FVector& StartWorld, const FVector& EndWorld,
        const FSparseVolumetricGrid& Grid,
        TArray<FVector>& OutPath);

    // --- HELPER FUNCTIONS ---
    static FVector GetGradient(const FVector& WorldPos, const FSparseVolumetricGrid& Grid);
    static bool IsLineSafe(const FVector& Start, const FVector& End, const FSparseVolumetricGrid& Grid);
    static void SmoothPath(const TArray<FVector>& RawPath, const FSparseVolumetricGrid& Grid, TArray<FVector>& OutSmoothPath);

    // --- NEW: SAFE SNAP HELPER ---
    // Searches outward from QueryPos to find the nearest non-blocked voxel.
    // Returns true if a safe spot was found within MaxRadius.
    static bool FindNearestSafeVoxel(const FVector& QueryPos, const FSparseVolumetricGrid& Grid, float MaxRadius, FVector& OutSafePos);

    // Helper to get distance (Made public for Subsystem checks if needed)
    static float GetDistance(int64 Index, const FSparseVolumetricGrid& Grid);
};