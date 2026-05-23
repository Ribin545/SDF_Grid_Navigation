#include "VolumetricBaker.h"
#include "Async/ParallelFor.h"
#include "Misc/ScopeLock.h"
#include "Algo/Sort.h"

// --- HELPER STRUCTS ---
struct FMassiveBitArray
{
    TArray<uint64> Words;
    void Init(int64 TotalBits) { Words.SetNumZeroed((TotalBits / 64) + 1); }
    FORCEINLINE void AtomicSet(int64 Index) {
        FPlatformAtomics::InterlockedOr((volatile int64*)&Words[Index / 64], (int64)(1ULL << (Index % 64)));
    }
    FORCEINLINE bool IsSet(int64 Index) const {
        return (Words[Index / 64] & (1ULL << (Index % 64))) != 0;
    }
};

struct FFrontierShard {
    FCriticalSection Mutex;
    TArray<int64> NewNodes;
    uint8 Pad[64];
};

void FVolumetricBaker::BakeSVO(FSparseVolumetricGrid& Grid, const TArray<int64>& InitialBlockedIndices)
{
    double StartTime = FPlatformTime::Seconds();

    // NOTE: Grid.Init() MUST be called before this to size the FlatBricks array correctly.
    // (The Subsystem handles this in ExecuteBakeMesh)

    int64 TotalVolume = (int64)Grid.Dimensions.X * (int64)Grid.Dimensions.Y * (int64)Grid.Dimensions.Z;
    FMassiveBitArray VisitedMask;
    VisitedMask.Init(TotalVolume);

    TArray<int64> CurrentFrontier;
    CurrentFrontier.Reserve(InitialBlockedIndices.Num());

    // --- BATCH WRITER HELPER (Optimized for Flat Array) ---
    auto BatchWriteToGrid = [&](const TArray<int64>& Indices, int8 DistanceVal)
        {
            if (Indices.Num() == 0) return;

            // 1. ALLOCATE STEP (Main Thread - Safe)
            // We must ensure the pointers exist before threads try to write to them.
            // Doing this on the main thread prevents race conditions on 'new FSDFBrick'.
            for (int64 GlobalIdx : Indices)
            {
                int64 X, Y, Z;
                Grid.GetCoords(GlobalIdx, X, Y, Z);
                Grid.AllocateBrick(X, Y, Z); // Helper in VolumetricTypes.h
            }

            // 2. WRITE STEP (Parallel - Fast)
            // Since pointers are guaranteed valid, threads can write bytes safely.
            ParallelFor(Indices.Num(), [&](int32 i)
                {
                    int64 GlobalIdx = Indices[i];
                    int64 X, Y, Z;
                    Grid.GetCoords(GlobalIdx, X, Y, Z);

                    // Re-calculate Linear Brick Index
                    int64 BX = X >> SDF_BRICK_SHIFT;
                    int64 BY = Y >> SDF_BRICK_SHIFT;
                    int64 BZ = Z >> SDF_BRICK_SHIFT;
                    int64 BrickIndex = BX + (BY * Grid.BrickDimensions.X) + (BZ * Grid.BrickDimensions.X * Grid.BrickDimensions.Y);

                    FSDFBrick* Brick = Grid.FlatBricks[BrickIndex];

                    // Check if it's a valid writable brick (not a solid tag)
                    if (Brick && Brick != BRICK_TAG_SOLID)
                    {
                        int32 LocalIdx = (X & SDF_BRICK_MASK) + ((Y & SDF_BRICK_MASK) << 4) + ((Z & SDF_BRICK_MASK) << 8);
                        Brick->Distances[LocalIdx] = DistanceVal;
                    }
                });
        };

    // --- 1. SEED WALLS ---
    for (int64 WallIdx : InitialBlockedIndices) {
        if (WallIdx >= 0 && WallIdx < TotalVolume) {
            VisitedMask.AtomicSet(WallIdx);
            CurrentFrontier.Add(WallIdx);
        }
    }

    // Batch Write Distance 0
    BatchWriteToGrid(CurrentFrontier, 0);

    // --- 2. SHELL PROPAGATION ---
    const float MaxShellDist = 250.0f;
    const int32 MaxSteps = FMath::CeilToInt(MaxShellDist / Grid.VoxelSize);

    const int32 Offsets[6][3] = { {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1} };
    FFrontierShard Shards[64];
    for (int i = 0; i < 64; i++) Shards[i].NewNodes.Reserve(10000);

    for (int32 Level = 1; Level <= MaxSteps; Level++)
    {
        int8 DistByte = (int8)FMath::Clamp(Level, 1, 127);

        // A. Parallel Expand
        ParallelFor(CurrentFrontier.Num(), [&](int32 Index)
            {
                int64 CIdx = CurrentFrontier[Index];
                int64 X, Y, Z;
                Grid.GetCoords(CIdx, X, Y, Z);

                for (int i = 0; i < 6; i++) {
                    int64 NX = X + Offsets[i][0];
                    int64 NY = Y + Offsets[i][1];
                    int64 NZ = Z + Offsets[i][2];
                    int64 NIdx = Grid.GetIndex(NX, NY, NZ);

                    if (NIdx != -1 && !VisitedMask.IsSet(NIdx)) {
                        VisitedMask.AtomicSet(NIdx);
                        int32 Shard = Index % 64;
                        FScopeLock Lock(&Shards[Shard].Mutex);
                        Shards[Shard].NewNodes.Add(NIdx);
                    }
                }
            });

        // B. Merge
        CurrentFrontier.Reset();
        int64 TotalNew = 0;
        for (auto& S : Shards) TotalNew += S.NewNodes.Num();
        if (TotalNew == 0) break;

        CurrentFrontier.Reserve(TotalNew);
        for (auto& S : Shards) {
            CurrentFrontier.Append(S.NewNodes);
            S.NewNodes.Reset();
        }

        // C. Batch Write
        BatchWriteToGrid(CurrentFrontier, DistByte);
    }

    // --- 3. COMPRESSION PASS (The "Native" Optimization) ---
    UE_LOG(LogTemp, Warning, TEXT("[Baker] Propagation Done. Compressing Memory..."));

    // Scan every allocated brick to identify uniform ones. We do this in parallel, but defer the actual
    // memory deletion to a serial pass to prevent platform-specific concurrent delete issues.
    TArray<int8> UniformValues;
    UniformValues.SetNum(Grid.FlatBricks.Num());

    ParallelFor(Grid.FlatBricks.Num(), [&](int32 i)
        {
            FSDFBrick* Brick = Grid.FlatBricks[i];
            if (Brick && Brick != BRICK_TAG_SOLID)
            {
                int8 Val;
                if (Brick->IsUniform(Val))
                {
                    UniformValues[i] = Val;
                }
                else
                {
                    UniformValues[i] = -128; // Sentinel: not uniform
                }
            }
            else
            {
                UniformValues[i] = -128; // Sentinel: already air or solid tag
            }
        });

    // Serial cleanup and tag assignment
    for (int64 i = 0; i < Grid.FlatBricks.Num(); ++i)
    {
        int8 Val = UniformValues[i];
        if (Val != -128)
        {
            FSDFBrick* Brick = Grid.FlatBricks[i];
            delete Brick;

            if (Val <= 0)
            {
                Grid.FlatBricks[i] = BRICK_TAG_SOLID;
            }
            else
            {
                Grid.FlatBricks[i] = nullptr;
            }
        }
    }

    // --- METRICS ---
    int32 RealBricks = 0;
    int32 SolidTags = 0;
    for (auto* Ptr : Grid.FlatBricks) {
        if (Ptr == BRICK_TAG_SOLID) SolidTags++;
        else if (Ptr) RealBricks++;
    }

    // RAM Calculation: Array Pointers + Real Bricks
    int64 PtrMem = Grid.FlatBricks.Num() * sizeof(FSDFBrick*);
    int64 DataMem = RealBricks * sizeof(FSDFBrick);
    float TotalMB = (PtrMem + DataMem) / 1024.0f / 1024.0f;

    double Dur = (FPlatformTime::Seconds() - StartTime) * 1000.0;
    UE_LOG(LogTemp, Warning, TEXT("[Baker] Done in %.2f ms."), Dur);
    UE_LOG(LogTemp, Warning, TEXT("   Final RAM:  %.2f MB"), TotalMB);
    UE_LOG(LogTemp, Warning, TEXT("   Real Bricks: %d"), RealBricks);
    UE_LOG(LogTemp, Warning, TEXT("   Solid Tags:  %d"), SolidTags);
}