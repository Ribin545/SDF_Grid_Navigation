#include "GreedyNav.h"
#include "VolumetricSubsystem.h" 

#if WITH_EDITOR
#include "Editor.h"
#include "Editor/EditorEngine.h"
#endif

#define LOCTEXT_NAMESPACE "FGreedyNavModule"

void FGreedyNavModule::StartupModule()
{
    // Register the command
    BakeCommand = IConsoleManager::Get().RegisterConsoleCommand(
        TEXT("Greedy.Bake"),
        TEXT("Bakes the fixed 500m area. Usage: Greedy.Bake"),
        FConsoleCommandWithArgsDelegate::CreateRaw(this, &FGreedyNavModule::HandleBakeCommand)
    );
}

void FGreedyNavModule::ShutdownModule()
{
    if (BakeCommand)
    {
        IConsoleManager::Get().UnregisterConsoleObject(BakeCommand);
    }
}

void FGreedyNavModule::HandleBakeCommand(const TArray<FString>& Args)
{
    UWorld* World = nullptr;

#if WITH_EDITOR
    if (GEditor)
    {
        World = GEditor->GetEditorWorldContext().World();
    }
#endif

    if (!World && GEngine)
    {
        World = GEngine->GetWorldContexts()[0].World();
    }

    if (!World) return;

    UVolumetricSubsystem* Subsystem = World->GetSubsystem<UVolumetricSubsystem>();
    if (Subsystem)
    {
        Subsystem->ExecuteBakeMesh(TArray<FString>());
    }
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FGreedyNavModule, GreedyNav)