#pragma once
#include "CoreMinimal.h"
#include "VolumetricTypes.generated.h"

#define SDF_BRICK_SIZE 16
#define SDF_BRICK_SHIFT 4
#define SDF_BRICK_MASK 15

// SPECIAL POINTER TAGS
// We use '1' for SOLID because valid pointers are always aligned to 8 bytes.
#define BRICK_TAG_SOLID ((FSDFBrick*)0x1)

// --- 4KB DATA CHUNK ---
struct FSDFBrick
{
    int8 Distances[4096];
    FSDFBrick() { FMemory::Memset(Distances, 127, sizeof(Distances)); }

    // Helper: Check if all voxels in this brick are identical
    bool IsUniform(int8& OutValue) const
    {
        int8 First = Distances[0];
        // Optimization: Check blocks if needed, but linear scan is fine for bake time
        for (int32 i = 1; i < 4096; i++)
        {
            if (Distances[i] != First) return false;
        }
        OutValue = First;
        return true;
    }
};

USTRUCT()
struct FSparseVolumetricGrid
{
    GENERATED_BODY()

    // --- THE PAGE TABLE ---
    // A linear array of pointers.
    // nullptr = Empty Air (Safe)
    // 0x1     = Solid Wall (Blocked)
    // Valid   = Detailed Voxel Data
    TArray<FSDFBrick*> FlatBricks;

    ~FSparseVolumetricGrid()
    {
        for (FSDFBrick* Ptr : FlatBricks)
        {
            if (Ptr && Ptr != BRICK_TAG_SOLID)
            {
                delete Ptr;
            }
        }
        FlatBricks.Empty();
    }

    FSparseVolumetricGrid() = default;

    // Deep Copy Constructor
    FSparseVolumetricGrid(const FSparseVolumetricGrid& Other)
        : Origin(Other.Origin)
        , Dimensions(Other.Dimensions)
        , BrickDimensions(Other.BrickDimensions)
        , VoxelSize(Other.VoxelSize)
    {
        FlatBricks.SetNumZeroed(Other.FlatBricks.Num());
        for (int32 i = 0; i < Other.FlatBricks.Num(); ++i)
        {
            FSDFBrick* OtherBrick = Other.FlatBricks[i];
            if (OtherBrick && OtherBrick != BRICK_TAG_SOLID)
            {
                FlatBricks[i] = new FSDFBrick(*OtherBrick);
            }
            else
            {
                FlatBricks[i] = OtherBrick;
            }
        }
    }

    // Deep Copy Assignment
    FSparseVolumetricGrid& operator=(const FSparseVolumetricGrid& Other)
    {
        if (this != &Other)
        {
            for (FSDFBrick* Ptr : FlatBricks)
            {
                if (Ptr && Ptr != BRICK_TAG_SOLID)
                {
                    delete Ptr;
                }
            }
            Origin = Other.Origin;
            Dimensions = Other.Dimensions;
            BrickDimensions = Other.BrickDimensions;
            VoxelSize = Other.VoxelSize;

            FlatBricks.Empty();
            FlatBricks.SetNumZeroed(Other.FlatBricks.Num());
            for (int32 i = 0; i < Other.FlatBricks.Num(); ++i)
            {
                FSDFBrick* OtherBrick = Other.FlatBricks[i];
                if (OtherBrick && OtherBrick != BRICK_TAG_SOLID)
                {
                    FlatBricks[i] = new FSDFBrick(*OtherBrick);
                }
                else
                {
                    FlatBricks[i] = OtherBrick;
                }
            }
        }
        return *this;
    }

    // Move Constructor
    FSparseVolumetricGrid(FSparseVolumetricGrid&& Other) noexcept
        : FlatBricks(MoveTemp(Other.FlatBricks))
        , Origin(Other.Origin)
        , Dimensions(Other.Dimensions)
        , BrickDimensions(Other.BrickDimensions)
        , VoxelSize(Other.VoxelSize)
    {
        Other.Origin = FVector::ZeroVector;
        Other.Dimensions = FIntVector::ZeroValue;
        Other.BrickDimensions = FIntVector::ZeroValue;
        Other.VoxelSize = 0.0f;
    }

    // Move Assignment
    FSparseVolumetricGrid& operator=(FSparseVolumetricGrid&& Other) noexcept
    {
        if (this != &Other)
        {
            for (FSDFBrick* Ptr : FlatBricks)
            {
                if (Ptr && Ptr != BRICK_TAG_SOLID)
                {
                    delete Ptr;
                }
            }
            FlatBricks = MoveTemp(Other.FlatBricks);
            Origin = Other.Origin;
            Dimensions = Other.Dimensions;
            BrickDimensions = Other.BrickDimensions;
            VoxelSize = Other.VoxelSize;

            Other.Origin = FVector::ZeroVector;
            Other.Dimensions = FIntVector::ZeroValue;
            Other.BrickDimensions = FIntVector::ZeroValue;
            Other.VoxelSize = 0.0f;
        }
        return *this;
    }

    FVector Origin;
    FIntVector Dimensions;      // Total Voxels
    FIntVector BrickDimensions; // Total Bricks along axes
    float VoxelSize;

    // --- SETUP ---
    void Init(FVector InCenter, FVector InExtents, float InVoxelSize)
    {
        Origin = InCenter - InExtents;
        VoxelSize = InVoxelSize;

        Dimensions.X = FMath::CeilToInt((InExtents.X * 2) / VoxelSize);
        Dimensions.Y = FMath::CeilToInt((InExtents.Y * 2) / VoxelSize);
        Dimensions.Z = FMath::CeilToInt((InExtents.Z * 2) / VoxelSize);

        BrickDimensions.X = (Dimensions.X + SDF_BRICK_SIZE - 1) >> SDF_BRICK_SHIFT;
        BrickDimensions.Y = (Dimensions.Y + SDF_BRICK_SIZE - 1) >> SDF_BRICK_SHIFT;
        BrickDimensions.Z = (Dimensions.Z + SDF_BRICK_SIZE - 1) >> SDF_BRICK_SHIFT;

        int64 TotalBricks = (int64)BrickDimensions.X * (int64)BrickDimensions.Y * (int64)BrickDimensions.Z;

        // Cleanup old memory safely
        for (FSDFBrick* Ptr : FlatBricks) {
            if (Ptr && Ptr != BRICK_TAG_SOLID) delete Ptr;
        }

        FlatBricks.Empty();
        FlatBricks.SetNumZeroed(TotalBricks); // Init all to nullptr (Air)
    }

    // --- COORDINATE HELPERS (Restored!) ---

    FORCEINLINE int64 GetIndex(int64 X, int64 Y, int64 Z) const
    {
        if (X < 0 || X >= Dimensions.X || Y < 0 || Y >= Dimensions.Y || Z < 0 || Z >= Dimensions.Z) return -1;
        return X + (Y * (int64)Dimensions.X) + (Z * (int64)Dimensions.X * (int64)Dimensions.Y);
    }

    FORCEINLINE void GetCoords(int64 Index, int64& X, int64& Y, int64& Z) const
    {
        int64 DXY = (int64)Dimensions.X * (int64)Dimensions.Y;
        Z = Index / DXY;
        int64 Rem = Index % DXY;
        Y = Rem / Dimensions.X;
        X = Rem % Dimensions.X;
    }

    FORCEINLINE int64 GetIndexFromWorld(FVector WorldPos) const
    {
        FVector Local = WorldPos - Origin;
        int64 X = FMath::FloorToInt(Local.X / VoxelSize);
        int64 Y = FMath::FloorToInt(Local.Y / VoxelSize);
        int64 Z = FMath::FloorToInt(Local.Z / VoxelSize);
        return GetIndex(X, Y, Z);
    }

    FORCEINLINE FVector GetWorldPos(int64 Index) const
    {
        int64 X, Y, Z;
        GetCoords(Index, X, Y, Z);
        return Origin + FVector((X + 0.5f) * VoxelSize, (Y + 0.5f) * VoxelSize, (Z + 0.5f) * VoxelSize);
    }

    // --- ULTRA-FAST LOOKUP (No Hashing) ---
    FORCEINLINE float GetDistanceValues(int64 X, int64 Y, int64 Z) const
    {
        // 1. Bounds Check
        if (X < 0 || X >= Dimensions.X || Y < 0 || Y >= Dimensions.Y || Z < 0 || Z >= Dimensions.Z)
            return 10000.0f;

        // 2. Compute Linear Page Index
        int64 BX = X >> SDF_BRICK_SHIFT;
        int64 BY = Y >> SDF_BRICK_SHIFT;
        int64 BZ = Z >> SDF_BRICK_SHIFT;
        int64 BrickIndex = BX + (BY * BrickDimensions.X) + (BZ * BrickDimensions.X * BrickDimensions.Y);

        FSDFBrick* Brick = FlatBricks[BrickIndex];

        // 3. TAG CHECKS
        if (!Brick) return 10000.0f;             // Air
        if (Brick == BRICK_TAG_SOLID) return 0.0f; // Solid

        // 4. Memory Fetch
        int32 LocalIdx = (X & SDF_BRICK_MASK) + ((Y & SDF_BRICK_MASK) << 4) + ((Z & SDF_BRICK_MASK) << 8);
        return (float)Brick->Distances[LocalIdx] * VoxelSize;
    }

    // --- MEMORY OPTIMIZER ---
    void OptimizeBrick(int64 BrickIndex)
    {
        FSDFBrick* Brick = FlatBricks[BrickIndex];
        if (!Brick || Brick == BRICK_TAG_SOLID) return;

        int8 Val;
        if (Brick->IsUniform(Val))
        {
            delete Brick; // Release RAM

            if (Val <= 0) FlatBricks[BrickIndex] = BRICK_TAG_SOLID;
            else          FlatBricks[BrickIndex] = nullptr; // Air
        }
    }

    // Helper for Baker
    FSDFBrick* AllocateBrick(int64 X, int64 Y, int64 Z)
    {
        int64 BX = X >> SDF_BRICK_SHIFT;
        int64 BY = Y >> SDF_BRICK_SHIFT;
        int64 BZ = Z >> SDF_BRICK_SHIFT;
        int64 BrickIndex = BX + (BY * BrickDimensions.X) + (BZ * BrickDimensions.X * BrickDimensions.Y);

        if (BrickIndex < 0 || BrickIndex >= FlatBricks.Num()) return nullptr;

        if (FlatBricks[BrickIndex] == BRICK_TAG_SOLID)
        {
            FlatBricks[BrickIndex] = new FSDFBrick();
            FMemory::Memset(FlatBricks[BrickIndex]->Distances, 0, 4096);
        }
        else if (!FlatBricks[BrickIndex])
        {
            FlatBricks[BrickIndex] = new FSDFBrick();
        }
        return FlatBricks[BrickIndex];
    }
};