#include "VolumetricPathfinder.h"
#include "Algo/Reverse.h" 
#include "DrawDebugHelpers.h" // Needed for Raycast visualization

const float HEURISTIC_WEIGHT = 3.0f;
const int32 MAX_PATH_STEPS = 2000000; // 2 Millio

struct FPathNode
{
    int64 ID;
    float F;
    FPathNode() : ID(-1), F(0.f) {}
    FPathNode(int64 InID, float InF) : ID(InID), F(InF) {}
    bool operator<(const FPathNode& Other) const { return F < Other.F; }
};

struct FNodeContext
{
    float G;
    int64 ParentID;
    bool bClosed;
};

// --- HELPER: Fast O(1) Distance Lookup ---
float FVolumetricPathfinder::GetDistance(int64 Index, const FSparseVolumetricGrid& Grid)
{
    if (Index == -1) return 0.0f; // Treat world boundary as wall
    int64 X, Y, Z;
    Grid.GetCoords(Index, X, Y, Z);
    return Grid.GetDistanceValues(X, Y, Z);
}

FVector FVolumetricPathfinder::GetGradient(const FVector& WorldPos, const FSparseVolumetricGrid& Grid)
{
    if (Grid.Dimensions.X == 0) return FVector::ZeroVector;

    float VoxelSize = Grid.VoxelSize;

    int64 Index = Grid.GetIndexFromWorld(WorldPos);
    if (Index == -1) return FVector::ZeroVector;

    int64 X, Y, Z;
    Grid.GetCoords(Index, X, Y, Z);

    float DistXPlus  = Grid.GetDistanceValues(X + 1, Y, Z);
    float DistXMinus = Grid.GetDistanceValues(X - 1, Y, Z);
    float DistYPlus  = Grid.GetDistanceValues(X, Y + 1, Z);
    float DistYMinus = Grid.GetDistanceValues(X, Y - 1, Z);
    float DistZPlus  = Grid.GetDistanceValues(X, Y, Z + 1);
    float DistZMinus = Grid.GetDistanceValues(X, Y, Z - 1);

    FVector Gradient(
        (DistXPlus - DistXMinus) / (2.0f * VoxelSize),
        (DistYPlus - DistYMinus) / (2.0f * VoxelSize),
        (DistZPlus - DistZMinus) / (2.0f * VoxelSize)
    );

    if (Gradient.IsNearlyZero())
    {
        return FVector::ZeroVector;
    }

    return Gradient.GetSafeNormal();
}

bool FVolumetricPathfinder::FindNearestSafeVoxel(const FVector& QueryPos, const FSparseVolumetricGrid& Grid, float MaxRadius, FVector& OutSafePos)
{
    int64 StartIdx = Grid.GetIndexFromWorld(QueryPos);

    // 1. If start is already safe, return it immediately
    if (GetDistance(StartIdx, Grid) > 0.0f)
    {
        OutSafePos = QueryPos;
        return true;
    }

    // 2. BFS Search Configuration
    int32 MaxSteps = FMath::CeilToInt(MaxRadius / Grid.VoxelSize);
    int64 X, Y, Z;
    Grid.GetCoords(StartIdx, X, Y, Z);

    // Optimization: Check layers outwards (Manhattan Distance expansion)
    // Radius 1, then Radius 2, etc.
    for (int32 R = 1; R <= MaxSteps; R++)
    {
        for (int32 dz = -R; dz <= R; dz++)
        {
            for (int32 dy = -R; dy <= R; dy++)
            {
                for (int32 dx = -R; dx <= R; dx++)
                {
                    // Only check the "shell" (outermost layer) of this radius
                    if (FMath::Abs(dx) != R && FMath::Abs(dy) != R && FMath::Abs(dz) != R) continue;

                    int64 NeighborIdx = Grid.GetIndex(X + dx, Y + dy, Z + dz);

                    if (GetDistance(NeighborIdx, Grid) > 0.0f)
                    {
                        // Found Safe Spot!
                        OutSafePos = Grid.GetWorldPos(NeighborIdx);
                        return true;
                    }
                }
            }
        }
    }

    return false; // No safe spot found within radius
}

// --- RAYCAST HELPER (With Debugging) ---
bool FVolumetricPathfinder::IsLineSafe(const FVector& Start, const FVector& End, const FSparseVolumetricGrid& Grid)
{
    FVector Diff = End - Start;
    float Dist = Diff.Size();
    if (Dist < Grid.VoxelSize) return true;

    // FIX 1: Reduced step size to 0.5 to prevent "tunneling" through thin walls
    int32 Steps = FMath::CeilToInt(Dist / (Grid.VoxelSize * 0.5f));
    FVector StepVec = Diff / (float)Steps;
    FVector Current = Start;

    // Safety limit for very long rays
    if (Steps > 1000) Steps = 1000;

    for (int32 i = 0; i < Steps; i++)
    {
        Current += StepVec;

        int64 Idx = Grid.GetIndexFromWorld(Current);
        float Distance = GetDistance(Idx, Grid);

        // DEBUG: Draw every check point (Purple = Check, Red = Hit)
        // Uncomment this to see the smoothing raycast visually!
        // DrawDebugPoint(Grid.BrickMap.Num() > 0 ? GWorld : nullptr, Current, 2.0f, (Distance < 10.0f) ? FColor::Red : FColor::Purple, false, 0.1f);

        if (Distance < 10.0f)
        {
            return false; // Hit Wall
        }
    }
    return true;
}

void FVolumetricPathfinder::SmoothPath(const TArray<FVector>& RawPath, const FSparseVolumetricGrid& Grid, TArray<FVector>& OutSmoothPath)
{
    if (RawPath.Num() < 3)
    {
        OutSmoothPath = RawPath;
        return;
    }

    OutSmoothPath.Reset();
    OutSmoothPath.Add(RawPath[0]);

    int32 CheckIdx = 0;
    int32 CurrentIdx = 1;

    while (CurrentIdx < RawPath.Num() - 1)
    {
        if (IsLineSafe(RawPath[CheckIdx], RawPath[CurrentIdx + 1], Grid))
        {
            CurrentIdx++; // Can skip next node
        }
        else
        {
            // Cannot skip, add the last valid node
            OutSmoothPath.Add(RawPath[CurrentIdx]);
            CheckIdx = CurrentIdx;
            CurrentIdx++;
        }
    }
    OutSmoothPath.Add(RawPath.Last());
}

// --- MAIN PATHFINDING FUNCTION ---
bool FVolumetricPathfinder::FindPath(const FVector& StartWorld, const FVector& EndWorld,
    const FSparseVolumetricGrid& Grid, TArray<FVector>& OutPath)
{
    OutPath.Reset();
    if (Grid.Dimensions.X == 0) return false;

    // DEBUG: Draw Start/End
    // DrawDebugBox(Grid.BrickMap.Num() > 0 ? GWorld : nullptr, StartWorld, FVector(10), FColor::Green, false, 5.0f);
    // DrawDebugBox(Grid.BrickMap.Num() > 0 ? GWorld : nullptr, EndWorld, FVector(10), FColor::Red, false, 5.0f);

    int64 StartID = Grid.GetIndexFromWorld(StartWorld);
    int64 EndID = Grid.GetIndexFromWorld(EndWorld);

    if (StartID == -1 || EndID == -1)
    {
        UE_LOG(LogTemp, Error, TEXT("FindPath: Start or End is Out of Bounds"));
        return false;
    }

    TMap<int64, FNodeContext> Contexts;
    Contexts.Reserve(10000);
    TArray<FPathNode> OpenSet;
    OpenSet.Reserve(2000);

    float StartH = FVector::Dist(StartWorld, EndWorld) * HEURISTIC_WEIGHT;
    Contexts.Add(StartID, { 0.0f, -1, false });
    OpenSet.HeapPush(FPathNode(StartID, StartH));

    const int32 Offsets[6][3] = { {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1} };
    int64 CurrentID, NeighborID;
    int64 X, Y, Z;
    bool bFound = false;
    int32 Steps = 0;

    while (OpenSet.Num() > 0)
    {
        if (++Steps > MAX_PATH_STEPS)
        {
            UE_LOG(LogTemp, Error, TEXT("PATH FAILED: Hit Iteration Limit (%d)"), MAX_PATH_STEPS);
            UE_LOG(LogTemp, Error, TEXT("   Nodes Visited: %d"), Contexts.Num());
            UE_LOG(LogTemp, Error, TEXT("   Heuristic Weight: %.1f (Try increasing this)"), HEURISTIC_WEIGHT);
            break;
        }

        FPathNode Top;
        OpenSet.HeapPop(Top);
        CurrentID = Top.ID;

        if (CurrentID == EndID) { bFound = true; break; }

        if (Contexts[CurrentID].bClosed) continue;
        Contexts[CurrentID].bClosed = true;

        float CurrentG = Contexts[CurrentID].G;
        Grid.GetCoords(CurrentID, X, Y, Z);

        // OPTIMIZATION: Check current safety
        // If we are deep in the sky (Distance > 500), we don't need to check every single neighbor.
        // We can just check neighbors, but we skip the heavy safety math for them.
        float MySafety = GetDistance(CurrentID, Grid);
        bool bInOpenAir = (MySafety > 400.0f);


        for (int i = 0; i < 6; i++)
        {
            NeighborID = Grid.GetIndex(X + Offsets[i][0], Y + Offsets[i][1], Z + Offsets[i][2]);
            if (NeighborID == -1) continue;

            // COST CALCULATION
            float NewG;
            if (bInOpenAir)
            {
                // Fast Path: We know it's safe, skip lookup
                NewG = CurrentG + Grid.VoxelSize;
            }
            else
            {
                // Slow Path: We are near walls, check safety carefully
                float SafetyDist = GetDistance(NeighborID, Grid);
                if (SafetyDist <= 0.0f) continue; // Blocked

                float SafetyPenalty = FMath::Max(0.0f, 200.0f - SafetyDist) * 2.0f;
                NewG = CurrentG + Grid.VoxelSize + SafetyPenalty;
            }

            FNodeContext* NeighborCtx = Contexts.Find(NeighborID);
            if (!NeighborCtx || NewG < NeighborCtx->G)
            {
                // Tie-Breaker H
                float H = FVector::Dist(Grid.GetWorldPos(NeighborID), EndWorld) * HEURISTIC_WEIGHT;
                H *= 1.001f;

                FNodeContext NewInfo = { NewG, CurrentID, false };
                if (NeighborCtx) *NeighborCtx = NewInfo;
                else Contexts.Add(NeighborID, NewInfo);

                OpenSet.HeapPush(FPathNode(NeighborID, NewG + H));
            }
        }
    }

    if (!bFound) return false;

    // Reconstruct Raw Path
    TArray<FVector> RawPath;
    int64 Trace = EndID;
    while (Trace != -1)
    {
        RawPath.Add(Grid.GetWorldPos(Trace));
        if (!Contexts.Contains(Trace)) break;
        Trace = Contexts[Trace].ParentID;
    }
    Algo::Reverse(RawPath);

    // DEBUG: Draw Raw Path (BLUE) before smoothing
    // This helps you verify if A* actually saw the wall
    for (int i = 0; i < RawPath.Num() - 1; i++)
    {
        // Uncomment to debug raw A* path
        // DrawDebugLine(Grid.BrickMap.Num() > 0 ? GWorld : nullptr, RawPath[i], RawPath[i+1], FColor::Blue, true, 10.0f, 0, 1.0f);
    }

    UE_LOG(LogTemp, Warning, TEXT("Raw Path Steps: %d"), RawPath.Num());

    // Run Smoothing
    SmoothPath(RawPath, Grid, OutPath);

    return true;
}