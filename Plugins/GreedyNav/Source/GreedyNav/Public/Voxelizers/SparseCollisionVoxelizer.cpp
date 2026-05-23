#include "Voxelizers/SparseCollisionVoxelizer.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "Async/ParallelFor.h"
// --- MISSING INCLUDES ADDED HERE ---
#include "Algo/Sort.h"
#include "Algo/Unique.h"
// -----------------------------------


struct FVoxelizeTask
{
    enum EType { Box, Sphere, Capsule, Convex };
    EType Type;
    FTransform Transform;
    FVector Data;
};

// STATIC HELPER
static void DoTaskMath(const FVoxelizeTask& Task, const FSparseVolumetricGrid& Grid, TArray<int64>& OutIndices)
{
    float VoxelRadius = Grid.VoxelSize * 0.75f;
    FVector Scale = Task.Transform.GetScale3D().GetAbs();
    Scale.X = FMath::Max(Scale.X, KINDA_SMALL_NUMBER);
    Scale.Y = FMath::Max(Scale.Y, KINDA_SMALL_NUMBER);
    Scale.Z = FMath::Max(Scale.Z, KINDA_SMALL_NUMBER);

    FVector LocalPadding = FVector(VoxelRadius) / Scale;
    FVector EffectiveSize;
    if (Task.Type == FVoxelizeTask::Box || Task.Type == FVoxelizeTask::Convex) EffectiveSize = Task.Data + LocalPadding;
    else {
        float R = Task.Data.X + (VoxelRadius / FMath::Max(Scale.X, Scale.Y));
        EffectiveSize = FVector(R, R, R);
    }

    FVector ScaledExtent = Scale * EffectiveSize;
    float MaxRad = ScaledExtent.Size();
    FVector Center = Task.Transform.GetLocation();

    int64 MinX = FMath::Max(0LL, (int64)((Center.X - MaxRad - Grid.Origin.X) / Grid.VoxelSize));
    int64 MaxX = FMath::Min((int64)Grid.Dimensions.X - 1, (int64)((Center.X + MaxRad - Grid.Origin.X) / Grid.VoxelSize));
    int64 MinY = FMath::Max(0LL, (int64)((Center.Y - MaxRad - Grid.Origin.Y) / Grid.VoxelSize));
    int64 MaxY = FMath::Min((int64)Grid.Dimensions.Y - 1, (int64)((Center.Y + MaxRad - Grid.Origin.Y) / Grid.VoxelSize));
    int64 MinZ = FMath::Max(0LL, (int64)((Center.Z - MaxRad - Grid.Origin.Z) / Grid.VoxelSize));
    int64 MaxZ = FMath::Min((int64)Grid.Dimensions.Z - 1, (int64)((Center.Z + MaxRad - Grid.Origin.Z) / Grid.VoxelSize));

    bool bHitAny = false;
    for (int64 z = MinZ; z <= MaxZ; ++z) {
        for (int64 y = MinY; y <= MaxY; ++y) {
            for (int64 x = MinX; x <= MaxX; ++x) {
                FVector VoxelCenter = Grid.Origin + FVector(x + 0.5f, y + 0.5f, z + 0.5f) * Grid.VoxelSize;
                bool bHit = false;
                if (Task.Type == FVoxelizeTask::Box || Task.Type == FVoxelizeTask::Convex) {
                    FVector Local = Task.Transform.InverseTransformPosition(VoxelCenter);
                    if (FMath::Abs(Local.X) <= EffectiveSize.X && FMath::Abs(Local.Y) <= EffectiveSize.Y && FMath::Abs(Local.Z) <= EffectiveSize.Z) bHit = true;
                }
                else {
                    if (FVector::DistSquared(VoxelCenter, Center) <= (MaxRad * MaxRad)) bHit = true;
                }

                if (bHit) {
                    OutIndices.Add(Grid.GetIndex(x, y, z));
                    bHitAny = true;
                }
            }
        }
    }
    if (!bHitAny) {
        int64 CX = (int64)((Center.X - Grid.Origin.X) / Grid.VoxelSize);
        int64 CY = (int64)((Center.Y - Grid.Origin.Y) / Grid.VoxelSize);
        int64 CZ = (int64)((Center.Z - Grid.Origin.Z) / Grid.VoxelSize);
        if (CX >= 0 && CX < Grid.Dimensions.X && CY >= 0 && CY < Grid.Dimensions.Y && CZ >= 0 && CZ < Grid.Dimensions.Z) {
            OutIndices.Add(Grid.GetIndex(CX, CY, CZ));
        }
    }
}

// MAIN FUNCTION
void FSparseCollisionVoxelizer::Voxelize(const UWorld* World, FVector Center, FVector Extents, float Resolution, FSparseVolumetricGrid& GridInfo, TArray<int64>& OutBlockedIndices)
{
    GridInfo.Init(Center, Extents, Resolution);
    FBox GridBounds(GridInfo.Origin, GridInfo.Origin + FVector(GridInfo.Dimensions.X, GridInfo.Dimensions.Y, GridInfo.Dimensions.Z) * Resolution);
    FBox SearchBounds = GridBounds.ExpandBy(Resolution * 4.0f);

    TArray<FVoxelizeTask> Tasks;
    Tasks.Reserve(100000);

    for (TObjectIterator<UStaticMeshComponent> It; It; ++It)
    {
        UStaticMeshComponent* MeshComp = *It;
        if (!MeshComp || !MeshComp->GetWorld() || MeshComp->GetWorld() != World) continue;
        if (MeshComp->GetCollisionEnabled() == ECollisionEnabled::NoCollision) continue;

        UStaticMesh* Mesh = MeshComp->GetStaticMesh();
        if (!Mesh || !Mesh->GetBodySetup()) continue;

        UInstancedStaticMeshComponent* ISM = Cast<UInstancedStaticMeshComponent>(MeshComp);
        int32 InstanceCount = ISM ? ISM->GetInstanceCount() : 1;
        FKAggregateGeom& AggGeom = Mesh->GetBodySetup()->AggGeom;

        for (int32 i = 0; i < InstanceCount; ++i)
        {
            FTransform InstanceTrans;
            if (ISM) ISM->GetInstanceTransform(i, InstanceTrans, true);
            else InstanceTrans = MeshComp->GetComponentTransform();

            if (!SearchBounds.Intersect(Mesh->GetBoundingBox().TransformBy(InstanceTrans))) continue;

            FVector Scale = InstanceTrans.GetScale3D().GetAbs();
            float MaxScale = InstanceTrans.GetMaximumAxisScale();

            for (const FKBoxElem& Box : AggGeom.BoxElems) {
                FVoxelizeTask Task; Task.Type = FVoxelizeTask::Box; Task.Transform = Box.GetTransform() * InstanceTrans; Task.Data = FVector(Box.X, Box.Y, Box.Z) * 0.5f; Tasks.Add(Task);
            }
            for (const FKSphereElem& Sphere : AggGeom.SphereElems) {
                FVoxelizeTask Task; Task.Type = FVoxelizeTask::Sphere; Task.Transform = FTransform(InstanceTrans.TransformPosition(Sphere.Center)); Task.Data.X = Sphere.Radius * MaxScale; Tasks.Add(Task);
            }
            for (const FKSphylElem& Cap : AggGeom.SphylElems) {
                FVoxelizeTask Task; Task.Type = FVoxelizeTask::Capsule; Task.Transform = Cap.GetTransform() * InstanceTrans; Task.Data.X = Cap.Radius * FMath::Max(Scale.X, Scale.Y); Task.Data.Y = Cap.Length * 0.5f * Scale.Z; Tasks.Add(Task);
            }
            for (const FKConvexElem& Convex : AggGeom.ConvexElems) {
                FVoxelizeTask Task; Task.Type = FVoxelizeTask::Convex; FTransform ShapeTrans = Convex.GetTransform() * InstanceTrans;
                ShapeTrans.AddToTranslation(ShapeTrans.TransformVector(Convex.ElemBox.GetCenter()));
                Task.Transform = ShapeTrans; Task.Data = Convex.ElemBox.GetExtent(); Tasks.Add(Task);
            }
        }
    }

    FCriticalSection Mutex;
    ParallelFor(Tasks.Num(), [&](int32 Index)
        {
            TArray<int64> LocalHits;
            DoTaskMath(Tasks[Index], GridInfo, LocalHits);
            if (LocalHits.Num() > 0)
            {
                FScopeLock Lock(&Mutex);
                OutBlockedIndices.Append(LocalHits);
            }
        });

    if (OutBlockedIndices.Num() > 0) {
        Algo::Sort(OutBlockedIndices);
        int64 UniqueCount = Algo::Unique(OutBlockedIndices);
        OutBlockedIndices.SetNum(UniqueCount);
    }
}