#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FGreedyNavModule : public IModuleInterface
{
public:
    /** IModuleInterface implementation */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
    // We declare the command handler here
    void HandleBakeCommand(const TArray<FString>& Args);

    // Pointer to the console command to clean it up later
    struct IConsoleCommand* BakeCommand;
};