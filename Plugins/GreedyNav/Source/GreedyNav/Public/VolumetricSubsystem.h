#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "VolumetricTypes.h"
#include "VolumetricSubsystem.generated.h"

UCLASS()
class GREEDYNAV_API UVolumetricSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    // Commands
    void ExecuteBakeSettings(const TArray<FString>& Args);
    void ExecuteBakeMesh(const TArray<FString>& Args);
    void ExecuteDebugVoxels(const TArray<FString>& Args);
    void ExecuteTest(const TArray<FString>& Args);
    void ExecuteProbe(const TArray<FString>& Args);
    void ExecuteDebugSlice(const TArray<FString>& Args);

    // Data
    FSparseVolumetricGrid RawSparseGrid; // Holds the Map/Bricks

    // We keep this purely for debug visualizations (Slice/Probe) 
    // The Pathfinding itself DOES NOT use this.
    TArray<int64> LastKnownBlockedIndices;

    FVector BakeExtents = FVector(5000, 5000, 2000);
    float BakeResolution = 50.0f;

    // Helper
    IConsoleCommand* SettingsCmd;
    IConsoleCommand* BakeMeshCmd;
    IConsoleCommand* DebugCmd;
    IConsoleCommand* TestCmd;
    IConsoleCommand* ProbeCmd;
    IConsoleCommand* SliceCmd;
};