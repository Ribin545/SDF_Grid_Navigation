#include "VolumetricSubsystem.h"
#include "VolumetricBaker.h"
#include "VolumetricPathfinder.h" 
#include "Voxelizers/FastCpuVoxelizer.h"
#include "DrawDebugHelpers.h"     
#include "Engine/World.h"
#include "Misc/CString.h"
#include "GameFramework/PlayerController.h"
#include "Algo/Sort.h"
#include "Async/Async.h"

void UVolumetricSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    SettingsCmd = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Nav.BakeSettings"), TEXT("Sets bake area: ExtentsX ExtentsY ExtentsZ Resolution"),
        FConsoleCommandWithArgsDelegate::CreateUObject(this, &UVolumetricSubsystem::ExecuteBakeSettings), ECVF_Default);

    BakeMeshCmd = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Nav.Bake"), TEXT("Runs Voxelizer and Baker"),
        FConsoleCommandWithArgsDelegate::CreateUObject(this, &UVolumetricSubsystem::ExecuteBakeMesh), ECVF_Default);

    DebugCmd = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Nav.Debug"), TEXT("Visualizes Bricks"),
        FConsoleCommandWithArgsDelegate::CreateUObject(this, &UVolumetricSubsystem::ExecuteDebugVoxels), ECVF_Default);

    TestCmd = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Nav.Test"), TEXT("Tests Pathfinding. Usage: Nav.Test path [Sx Sy Sz] [Ex Ey Ez]"),
        FConsoleCommandWithArgsDelegate::CreateUObject(this, &UVolumetricSubsystem::ExecuteTest), ECVF_Default);

    ProbeCmd = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Nav.Probe"), TEXT("Inspects coordinate. Usage: Nav.Probe X Y Z"),
        FConsoleCommandWithArgsDelegate::CreateUObject(this, &UVolumetricSubsystem::ExecuteProbe), ECVF_Default);

    SliceCmd = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Nav.Slice"), TEXT("Draws 2D slice. Usage: Nav.Slice Z"),
        FConsoleCommandWithArgsDelegate::CreateUObject(this, &UVolumetricSubsystem::ExecuteDebugSlice), ECVF_Default);
}

void UVolumetricSubsystem::Deinitialize()
{
    Super::Deinitialize();

    IConsoleManager& CM = IConsoleManager::Get();
    if (SettingsCmd)  { CM.UnregisterConsoleObject(SettingsCmd);  SettingsCmd  = nullptr; }
    if (BakeMeshCmd)  { CM.UnregisterConsoleObject(BakeMeshCmd);  BakeMeshCmd  = nullptr; }
    if (DebugCmd)     { CM.UnregisterConsoleObject(DebugCmd);     DebugCmd     = nullptr; }
    if (TestCmd)      { CM.UnregisterConsoleObject(TestCmd);      TestCmd      = nullptr; }
    if (ProbeCmd)     { CM.UnregisterConsoleObject(ProbeCmd);     ProbeCmd     = nullptr; }
    if (SliceCmd)     { CM.UnregisterConsoleObject(SliceCmd);     SliceCmd     = nullptr; }
}

void UVolumetricSubsystem::ExecuteBakeSettings(const TArray<FString>& Args)
{
    if (Args.Num() >= 4) {
        BakeExtents = FVector(FCString::Atof(*Args[0]), FCString::Atof(*Args[1]), FCString::Atof(*Args[2]));
        BakeResolution = FCString::Atof(*Args[3]);
        UE_LOG(LogTemp, Warning, TEXT("Bake Settings Updated: Extents=%s, Res=%.2f"), *BakeExtents.ToString(), BakeResolution);
    }
}

void UVolumetricSubsystem::ExecuteBakeMesh(const TArray<FString>& Args)
{
    if (!GetWorld()) return;
    FVector Center = FVector::ZeroVector;

    UE_LOG(LogTemp, Warning, TEXT("=== STARTING BAKE (Native + Compressed) ==="));
    UE_LOG(LogTemp, Warning, TEXT("Config: Extents %s | Resolution %.2f"), *BakeExtents.ToString(), BakeResolution);

    double StartTime = FPlatformTime::Seconds();

    // 1. VOXELIZE (Fills temporary array)
    LastKnownBlockedIndices.Reset();
    FFastCpuVoxelizer::Voxelize(GetWorld(), Center, BakeExtents, BakeResolution, RawSparseGrid, LastKnownBlockedIndices);
    // Note: RawSparseGrid.Init() is called inside Voxelizer, allocating the Flat Array.

    int32 RawVoxelCount = LastKnownBlockedIndices.Num();
    double VoxelTime = (FPlatformTime::Seconds() - StartTime) * 1000.0;

    // 2. BAKE (Writes to Flat Array & Compresses)
    FVolumetricBaker::BakeSVO(RawSparseGrid, LastKnownBlockedIndices);

    double TotalTime = (FPlatformTime::Seconds() - StartTime) * 1000.0;

    // Memory stats are logged by the Baker now
    UE_LOG(LogTemp, Warning, TEXT("=== BAKE COMPLETE ==="));
    UE_LOG(LogTemp, Warning, TEXT("Time: %.2f ms (Voxelizer: %.2f ms)"), TotalTime, VoxelTime);
    UE_LOG(LogTemp, Warning, TEXT("Raw Voxels: %d"), RawVoxelCount);
}

void UVolumetricSubsystem::ExecuteDebugVoxels(const TArray<FString>& Args)
{
    if (!GetWorld()) return;
    FlushPersistentDebugLines(GetWorld());

    FVector CenterPos = FVector::ZeroVector;
    if (APlayerController* PC = GetWorld()->GetFirstPlayerController()) if (PC->GetPawn()) CenterPos = PC->GetPawn()->GetActorLocation();

    float VisibleRadSq = FMath::Square(5000.0f);
    int32 DrawnCount = 0;

    UE_LOG(LogTemp, Warning, TEXT("Visualizing Native Flat Grid within 50m..."));

    // ITERATE FLAT ARRAY
    int64 TotalBricks = RawSparseGrid.FlatBricks.Num();

    for (int64 i = 0; i < TotalBricks; i++)
    {
        FSDFBrick* Brick = RawSparseGrid.FlatBricks[i];

        // 1. Skip Air
        if (!Brick) continue;

        // 2. Handle Solid Tags (Optional: Draw center point?)
        if (Brick == BRICK_TAG_SOLID)
        {
            // For debug clarity, we usually skip drawing solid walls because they are underground/inside.
            // If you WANT to see them, uncomment below:
            /*
            FVector Pos = RawSparseGrid.GetWorldPos(RawSparseGrid.GetIndexFromBrickIndex(i));
            if (FVector::DistSquared(Pos, CenterPos) < VisibleRadSq)
                 DrawDebugPoint(GetWorld(), Pos, 10.0f, FColor::Red, true);
            */
            continue;
        }

        // 3. Draw Detailed Brick
        // Reconstruct Coordinates from Index 'i'
        int64 AreaXY = (int64)RawSparseGrid.BrickDimensions.X * (int64)RawSparseGrid.BrickDimensions.Y;
        int64 BZ = i / AreaXY;
        int64 Rem = i % AreaXY;
        int64 BY = Rem / RawSparseGrid.BrickDimensions.X;
        int64 BX = Rem % RawSparseGrid.BrickDimensions.X;

        int64 BaseX = BX * 16;
        int64 BaseY = BY * 16;
        int64 BaseZ = BZ * 16;

        for (int32 v = 0; v < 4096; v++)
        {
            if (Brick->Distances[v] >= 127) continue; // Skip Safe Air

            int32 LZ = v >> 8;           // v / 256
            int32 RemV = v & 255;        // v % 256
            int32 LY = RemV >> 4;        // RemV / 16
            int32 LX = RemV & 15;        // RemV % 16

            FVector Pos = RawSparseGrid.GetWorldPos(RawSparseGrid.GetIndex(BaseX + LX, BaseY + LY, BaseZ + LZ));

            if (FVector::DistSquared(Pos, CenterPos) > VisibleRadSq) continue;

            int8 Dist = Brick->Distances[v];
            FColor Color = (Dist == 0) ? FColor::Red : FColor::MakeRedToGreenColorFromScalar((float)Dist / 127.0f);

            DrawDebugPoint(GetWorld(), Pos, 5.0f, Color, true);

            if (++DrawnCount > 100000) goto DrawDone;
        }
    }

DrawDone:
    UE_LOG(LogTemp, Warning, TEXT("Debug Draw Complete: %d points visible."), DrawnCount);
}

void UVolumetricSubsystem::ExecuteDebugSlice(const TArray<FString>& Args)
{
    if (Args.Num() == 0 || !GetWorld()) return;
    FlushPersistentDebugLines(GetWorld());
    float SliceZ = FCString::Atof(*Args[0]);

    // Fast Slice using Voxelizer Output (Still safest/fastest method)
    int32 Drawn = 0;
    for (int64 Idx : LastKnownBlockedIndices)
    {
        FVector Pos = RawSparseGrid.GetWorldPos(Idx);
        if (FMath::Abs(Pos.Z - SliceZ) < RawSparseGrid.VoxelSize)
        {
            DrawDebugBox(GetWorld(), Pos, FVector(RawSparseGrid.VoxelSize * 0.45f), FColor::Red, true);
            Drawn++;
        }
    }
    UE_LOG(LogTemp, Warning, TEXT("Slice Z=%.1f : Drawn %d obstacles."), SliceZ, Drawn);
}

void UVolumetricSubsystem::ExecuteProbe(const TArray<FString>& Args)
{
    if (Args.Num() < 3 || !GetWorld()) return;
    FVector Point(FCString::Atof(*Args[0]), FCString::Atof(*Args[1]), FCString::Atof(*Args[2]));

    int64 Index = RawSparseGrid.GetIndexFromWorld(Point);
    if (Index == -1)
    {
        UE_LOG(LogTemp, Error, TEXT("Probe Point Out of Bounds"));
        return;
    }

    int64 X, Y, Z;
    RawSparseGrid.GetCoords(Index, X, Y, Z);

    // New Native Lookup
    float Dist = RawSparseGrid.GetDistanceValues(X, Y, Z);

    UE_LOG(LogTemp, Warning, TEXT("=== PROBE ==="));
    UE_LOG(LogTemp, Warning, TEXT("Point: %s"), *Point.ToString());
    UE_LOG(LogTemp, Warning, TEXT("Index: %lld"), Index);

    FString Status;
    if (Dist >= 1000.0f) Status = TEXT("(Open Air - Nullptr)");
    else if (Dist <= 0.0f) Status = TEXT("(Solid Wall - Tag)");
    else Status = TEXT("(Gradient Surface)");

    UE_LOG(LogTemp, Warning, TEXT("Distance: %.2f cm %s"), Dist, *Status);

    DrawDebugBox(GetWorld(), RawSparseGrid.GetWorldPos(Index), FVector(10), FColor::Purple, true, 5.0f, 0, 2.0f);
}

void UVolumetricSubsystem::ExecuteTest(const TArray<FString>& Args)
{
    if (Args.Num() == 0 || !GetWorld()) return;
    FString Mode = Args[0].ToLower();
    FVector StartPos = (Args.Num() >= 4) ? FVector(FCString::Atof(*Args[1]), FCString::Atof(*Args[2]), FCString::Atof(*Args[3])) : FVector::ZeroVector;

    if (Mode == "path")
    {
        FlushPersistentDebugLines(GetWorld());

        FVector EndPos = StartPos + FVector(5000, 0, 0);
        if (Args.Num() >= 7) EndPos = FVector(FCString::Atof(*Args[4]), FCString::Atof(*Args[5]), FCString::Atof(*Args[6]));

        // --- AUTO-SNAP LOGIC ---
        FVector SafeStart = StartPos;
        FVector SafeEnd = EndPos;

        // 1. Check & Snap Start
        if (FVolumetricPathfinder::GetDistance(RawSparseGrid.GetIndexFromWorld(StartPos), RawSparseGrid) <= 0.0f)
        {
            if (FVolumetricPathfinder::FindNearestSafeVoxel(StartPos, RawSparseGrid, 500.0f, SafeStart))
            {
                UE_LOG(LogTemp, Warning, TEXT("Start blocked. Snapped to: %s"), *SafeStart.ToString());
                DrawDebugLine(GetWorld(), StartPos, SafeStart, FColor::Yellow, true, 10.0f, 0, 2.0f);
            }
            else { UE_LOG(LogTemp, Error, TEXT("Start trapped in wall.")); return; }
        }

        // 2. Check & Snap End
        if (FVolumetricPathfinder::GetDistance(RawSparseGrid.GetIndexFromWorld(EndPos), RawSparseGrid) <= 0.0f)
        {
            if (FVolumetricPathfinder::FindNearestSafeVoxel(EndPos, RawSparseGrid, 500.0f, SafeEnd))
            {
                UE_LOG(LogTemp, Warning, TEXT("End blocked. Snapped to: %s"), *SafeEnd.ToString());
                DrawDebugLine(GetWorld(), EndPos, SafeEnd, FColor::Yellow, true, 10.0f, 0, 2.0f);
            }
            else { UE_LOG(LogTemp, Error, TEXT("End trapped in wall.")); return; }
        }

        TArray<FVector> Path;
        double Start = FPlatformTime::Seconds();

        bool bFound = FVolumetricPathfinder::FindPath(SafeStart, SafeEnd, RawSparseGrid, Path);
        double TimeMs = (FPlatformTime::Seconds() - Start) * 1000.0;

        if (bFound) {
            for (int i = 0; i < Path.Num() - 1; i++)
            {
                DrawDebugLine(GetWorld(), Path[i], Path[i + 1], FColor::Red, true, 10.0f, 0, 12.0f);
                if (i % 2 == 0) DrawDebugPoint(GetWorld(), Path[i], 10.0f, FColor::Red, true, 10.0f);
            }
            DrawDebugBox(GetWorld(), SafeStart, FVector(25), FColor::Red, true, 10.0f);
            DrawDebugBox(GetWorld(), SafeEnd, FVector(25), FColor::Red, true, 10.0f);

            UE_LOG(LogTemp, Warning, TEXT("--- PATH SUCCESS ---"));
            UE_LOG(LogTemp, Warning, TEXT("Steps: %d"), Path.Num());
            UE_LOG(LogTemp, Warning, TEXT("Time:  %.4f ms"), TimeMs);
        }
        else {
            DrawDebugLine(GetWorld(), SafeStart, SafeEnd, FColor::Red, true, 10.0f, 0, 2.0f);
            UE_LOG(LogTemp, Error, TEXT("--- PATH FAILED ---"));
            UE_LOG(LogTemp, Error, TEXT("Time:  %.4f ms"), TimeMs);
        }
    }
}