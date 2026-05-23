#include "Voxelizers/FastCpuVoxelizer.h"
#include "Engine/StaticMeshActor.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "StaticMeshResources.h"
#include "Async/ParallelFor.h"
#include "Algo/Unique.h"
#include "Algo/Sort.h"
#include "LandscapeHeightfieldCollisionComponent.h"

// --- 1. HELPER IMPLEMENTATIONS ---

bool FFastCpuVoxelizer::IsPointInPrimitive(const FVector& LocalPoint, const FKShapeElem& Shape, int32 ShapeType)
{
    if (ShapeType == 0) // Box
    {
        const FKBoxElem* Box = static_cast<const FKBoxElem*>(&Shape);
        return FMath::Abs(LocalPoint.X) <= (Box->X * 0.5f) &&
            FMath::Abs(LocalPoint.Y) <= (Box->Y * 0.5f) &&
            FMath::Abs(LocalPoint.Z) <= (Box->Z * 0.5f);
    }
    else if (ShapeType == 1) // Sphere
    {
        const FKSphereElem* Sphere = static_cast<const FKSphereElem*>(&Shape);
        return LocalPoint.SizeSquared() <= (Sphere->Radius * Sphere->Radius);
    }
    else if (ShapeType == 2) // Capsule
    {
        const FKSphylElem* Cap = static_cast<const FKSphylElem*>(&Shape);
        float HalfHeight = Cap->Length * 0.5f;
        float DistZ = FMath::Clamp(LocalPoint.Z, -HalfHeight, HalfHeight);
        FVector ClosestPointOnAxis(0, 0, DistZ);
        return FVector::DistSquared(LocalPoint, ClosestPointOnAxis) <= (Cap->Radius * Cap->Radius);
    }
    return false;
}

float FFastCpuVoxelizer::GetTriangleZ(const FVector& V0, const FVector& V1, const FVector& V2, float X, float Y)
{
    FVector Normal = (V1 - V0) ^ (V2 - V0);
    if (FMath::IsNearlyZero(Normal.Z)) return V0.Z;
    float D = -FVector::DotProduct(Normal, V0);
    return -(Normal.X * X + Normal.Y * Y + D) / Normal.Z;
}

bool FFastCpuVoxelizer::IsPointInTriangle2D(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C)
{
    float w1 = (A.X * (C.Y - A.Y) + (P.Y - A.Y) * (C.X - A.X) - P.X * (C.Y - A.Y)) /
        ((B.Y - A.Y) * (C.X - A.X) - (B.X - A.X) * (C.Y - A.Y));
    float w2 = (P.Y - A.Y - w1 * (B.Y - A.Y)) / (C.Y - A.Y);
    return (w1 >= 0.0f) && (w2 >= 0.0f) && ((w1 + w2) <= 1.0f);
}

// --- 2. MAIN VOXELIZE FUNCTION ---

// UPDATED SIGNATURE: Accepts OutBlockedIndices
void FFastCpuVoxelizer::Voxelize(const UWorld* WorldContext, FVector Center, FVector Extents, float VoxelResolution, FSparseVolumetricGrid& GridInfo, TArray<int64>& OutBlockedIndices)
{
    double StartTime = FPlatformTime::Seconds();

    // 1. Init Grid (Calculates dimensions)
    GridInfo.Init(Center, Extents, VoxelResolution);

    // Cache for lambda capture
    FVector Origin = GridInfo.Origin;
    int32 SizeX = GridInfo.Dimensions.X;
    int32 SizeY = GridInfo.Dimensions.Y;
    int32 SizeZ = GridInfo.Dimensions.Z;
    FBox GridBounds(Origin, Origin + FVector(SizeX, SizeY, SizeZ) * VoxelResolution);

    // 2. GATHER WORK ITEMS
    struct FWorkItem { UStaticMeshComponent* Comp; bool bIsInstanced; };
    TArray<FWorkItem> WorkItems;
    WorkItems.Reserve(10000);

    for (TObjectIterator<UStaticMeshComponent> It; It; ++It)
    {
        if (It->GetWorld() == WorldContext && It->GetCollisionEnabled() != ECollisionEnabled::NoCollision)
        {
            if (GridBounds.Intersect(It->Bounds.GetBox()))
            {
                WorkItems.Add({ *It, It->IsA<UInstancedStaticMeshComponent>() });
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("[Voxelizer] Processing %d components..."), WorkItems.Num());

    // 3. PARALLEL EXECUTION
    TArray<TArray<int64>> TaskResults;
    TaskResults.SetNum(WorkItems.Num());

    ParallelFor(WorkItems.Num(), [&](int32 Index)
        {
            TArray<int64>& LocalBuffer = TaskResults[Index];
            FWorkItem& Item = WorkItems[Index];
            UStaticMeshComponent* Comp = Item.Comp;
            UStaticMesh* Mesh = Comp->GetStaticMesh();

            if (!Mesh || !Mesh->GetBodySetup()) return;

            UBodySetup* BodySetup = Mesh->GetBodySetup();
            FKAggregateGeom& AggGeom = BodySetup->AggGeom;

            bool bComplex = (BodySetup->CollisionTraceFlag == CTF_UseComplexAsSimple);
            if (AggGeom.BoxElems.Num() + AggGeom.SphereElems.Num() + AggGeom.SphylElems.Num() + AggGeom.ConvexElems.Num() == 0) bComplex = true;

            UInstancedStaticMeshComponent* ISM = Item.bIsInstanced ? Cast<UInstancedStaticMeshComponent>(Comp) : nullptr;
            int32 InstCount = ISM ? ISM->GetInstanceCount() : 1;

            for (int32 i = 0; i < InstCount; ++i)
            {
                FTransform InstanceTrans;
                if (ISM) ISM->GetInstanceTransform(i, InstanceTrans, true);
                else InstanceTrans = Comp->GetComponentTransform();

                if (!GridBounds.Intersect(Mesh->GetBoundingBox().TransformBy(InstanceTrans))) continue;

                auto Mark = [&](int32 x, int32 y, int32 z) {
                    if (x >= 0 && x < SizeX && y >= 0 && y < SizeY && z >= 0 && z < SizeZ)
                        LocalBuffer.Add(GridInfo.GetIndex(x, y, z)); // Use GridInfo for math, not storage
                    };

                if (bComplex && Mesh->GetRenderData() && (Mesh->bAllowCPUAccess || GIsEditor))
                {
                    const auto& LOD = Mesh->GetRenderData()->LODResources[0];
                    const auto& Verts = LOD.VertexBuffers.PositionVertexBuffer;
                    const auto& Indices = LOD.IndexBuffer;
                    int32 NumTris = Indices.GetNumIndices() / 3;

                    for (int32 t = 0; t < NumTris; t++) {
                        FVector V0 = InstanceTrans.TransformPosition((FVector)Verts.VertexPosition(Indices.GetIndex(t * 3)));
                        FVector V1 = InstanceTrans.TransformPosition((FVector)Verts.VertexPosition(Indices.GetIndex(t * 3 + 1)));
                        FVector V2 = InstanceTrans.TransformPosition((FVector)Verts.VertexPosition(Indices.GetIndex(t * 3 + 2)));

                        FVector G0 = (V0 - Origin) / VoxelResolution;
                        FVector G1 = (V1 - Origin) / VoxelResolution;
                        FVector G2 = (V2 - Origin) / VoxelResolution;

                        int32 MinX = FMath::Clamp(FMath::FloorToInt(FMath::Min3(G0.X, G1.X, G2.X)), 0, SizeX - 1);
                        int32 MaxX = FMath::Clamp(FMath::CeilToInt(FMath::Max3(G0.X, G1.X, G2.X)), 0, SizeX - 1);
                        int32 MinY = FMath::Clamp(FMath::FloorToInt(FMath::Min3(G0.Y, G1.Y, G2.Y)), 0, SizeY - 1);
                        int32 MaxY = FMath::Clamp(FMath::CeilToInt(FMath::Max3(G0.Y, G1.Y, G2.Y)), 0, SizeY - 1);

                        for (int32 y = MinY; y <= MaxY; y++)
                            for (int32 x = MinX; x <= MaxX; x++) {
                                FVector2D P(x + 0.5f, y + 0.5f);
                                if (IsPointInTriangle2D(P, FVector2D(G0), FVector2D(G1), FVector2D(G2)))
                                    Mark(x, y, FMath::RoundToInt(GetTriangleZ(G0, G1, G2, P.X, P.Y)));
                            }
                    }
                }
                else
                {
                    auto Rasterize = [&](const auto& Shape, const FTransform& LT, int32 T) {
                        FTransform WT = LT * InstanceTrans;
                        FBox B = Shape.CalcAABB(LT, 1.f).TransformBy(InstanceTrans);
                        int32 mx = FMath::Max(0, FMath::FloorToInt((B.Min.X - Origin.X) / VoxelResolution));
                        int32 Mx = FMath::Min(SizeX - 1, FMath::CeilToInt((B.Max.X - Origin.X) / VoxelResolution));
                        int32 my = FMath::Max(0, FMath::FloorToInt((B.Min.Y - Origin.Y) / VoxelResolution));
                        int32 My = FMath::Min(SizeY - 1, FMath::CeilToInt((B.Max.Y - Origin.Y) / VoxelResolution));
                        int32 mz = FMath::Max(0, FMath::FloorToInt((B.Min.Z - Origin.Z) / VoxelResolution));
                        int32 Mz = FMath::Min(SizeZ - 1, FMath::CeilToInt((B.Max.Z - Origin.Z) / VoxelResolution));

                        for (int32 z = mz; z <= Mz; z++)
                            for (int32 y = my; y <= My; y++)
                                for (int32 x = mx; x <= Mx; x++) {
                                    FVector C = Origin + FVector(x + 0.5f, y + 0.5f, z + 0.5f) * VoxelResolution;
                                    if (IsPointInPrimitive(WT.InverseTransformPosition(C), Shape, T)) Mark(x, y, z);
                                }
                        };
                    for (const auto& E : AggGeom.BoxElems) Rasterize(E, E.GetTransform(), 0);
                    for (const auto& E : AggGeom.SphereElems) Rasterize(E, E.GetTransform(), 1);
                    for (const auto& E : AggGeom.SphylElems) Rasterize(E, E.GetTransform(), 2);
                    for (const auto& E : AggGeom.ConvexElems) {
                        FBox B = E.ElemBox.TransformBy(E.GetTransform() * InstanceTrans);
                        int32 mx = FMath::Max(0, FMath::FloorToInt((B.Min.X - Origin.X) / VoxelResolution));
                        int32 Mx = FMath::Min(SizeX - 1, FMath::CeilToInt((B.Max.X - Origin.X) / VoxelResolution));
                        int32 my = FMath::Max(0, FMath::FloorToInt((B.Min.Y - Origin.Y) / VoxelResolution));
                        int32 My = FMath::Min(SizeY - 1, FMath::CeilToInt((B.Max.Y - Origin.Y) / VoxelResolution));
                        int32 mz = FMath::Max(0, FMath::FloorToInt((B.Min.Z - Origin.Z) / VoxelResolution));
                        int32 Mz = FMath::Min(SizeZ - 1, FMath::CeilToInt((B.Max.Z - Origin.Z) / VoxelResolution));
                        for (int32 z = mz; z <= Mz; z++) for (int32 y = my; y <= My; y++) for (int32 x = mx; x <= Mx; x++) Mark(x, y, z);
                    }
                }
            }
        });

    // 4. LANDSCAPE PASS (Serial — BulkData lock is not thread-safe)
    // For each ULandscapeHeightfieldCollisionComponent intersecting the grid,
    // bilinearly sample the uint16 heightfield for every XY voxel column and
    // mark the surface voxel (+ one below) as blocked.
    TArray<int64> LandscapeBuffer;
    int32 LandscapeCompCount = 0;

    for (TObjectIterator<ULandscapeHeightfieldCollisionComponent> LcIt; LcIt; ++LcIt)
    {
        ULandscapeHeightfieldCollisionComponent* LC = *LcIt;
        if (LC->GetWorld() != WorldContext) continue;
        if (!GridBounds.Intersect(LC->Bounds.GetBox())) continue;

        const int32   NumQuads = LC->CollisionSizeQuads;
        const float   LcScale  = LC->CollisionScale;
        const FTransform& LcXf = LC->GetComponentTransform();

        // Lock the raw height data (uint16 per sample, row-major, (NumQuads+1)^2 entries)
        const uint16* HeightData = reinterpret_cast<const uint16*>(LC->CollisionHeightData.Lock(LOCK_READ_ONLY));
        if (!HeightData)
        {
            LC->CollisionHeightData.Unlock();
            continue;
        }

        ++LandscapeCompCount;
        const int32 Stride = NumQuads + 1;

        // Determine the grid-column range that overlaps this component
        FBox LcBox = LC->Bounds.GetBox().Overlap(GridBounds);
        int32 ColMinX = FMath::Max(0,        FMath::FloorToInt((LcBox.Min.X - Origin.X) / VoxelResolution));
        int32 ColMaxX = FMath::Min(SizeX - 1, FMath::CeilToInt ((LcBox.Max.X - Origin.X) / VoxelResolution));
        int32 ColMinY = FMath::Max(0,        FMath::FloorToInt((LcBox.Min.Y - Origin.Y) / VoxelResolution));
        int32 ColMaxY = FMath::Min(SizeY - 1, FMath::CeilToInt ((LcBox.Max.Y - Origin.Y) / VoxelResolution));

        for (int32 gy = ColMinY; gy <= ColMaxY; ++gy)
        {
            for (int32 gx = ColMinX; gx <= ColMaxX; ++gx)
            {
                // World-space centre of this voxel column
                float wx = Origin.X + (gx + 0.5f) * VoxelResolution;
                float wy = Origin.Y + (gy + 0.5f) * VoxelResolution;

                // Transform to landscape component local space (XY only)
                FVector LocalPos = LcXf.InverseTransformPosition(FVector(wx, wy, 0.0f));

                // Convert to heightfield sample index space
                float fx = LocalPos.X / LcScale;
                float fy = LocalPos.Y / LcScale;

                if (fx < 0.f || fx > (float)NumQuads || fy < 0.f || fy > (float)NumQuads) continue;

                // Bilinear interpolation of the four surrounding height samples
                int32 x0 = FMath::FloorToInt(fx);  int32 x1 = FMath::Min(x0 + 1, NumQuads);
                int32 y0 = FMath::FloorToInt(fy);  int32 y1 = FMath::Min(y0 + 1, NumQuads);
                float tx = fx - x0,  ty = fy - y0;

                float h00 = (float)HeightData[y0 * Stride + x0] - 32768.f;
                float h10 = (float)HeightData[y0 * Stride + x1] - 32768.f;
                float h01 = (float)HeightData[y1 * Stride + x0] - 32768.f;
                float h11 = (float)HeightData[y1 * Stride + x1] - 32768.f;
                float LocalH = (h00*(1.f-tx)*(1.f-ty) + h10*tx*(1.f-ty)
                              + h01*(1.f-tx)*ty        + h11*tx*ty) / 128.f;

                // Reconstruct world Z via the component transform
                float WorldZ = LcXf.TransformPosition(FVector(LocalPos.X, LocalPos.Y, LocalH)).Z;

                int32 gz = FMath::FloorToInt((WorldZ - Origin.Z) / VoxelResolution);

                // Mark surface voxel and one below it for a solid floor
                if (gz >= 0 && gz < SizeZ)     LandscapeBuffer.Add(GridInfo.GetIndex(gx, gy, gz));
                if (gz - 1 >= 0 && gz - 1 < SizeZ) LandscapeBuffer.Add(GridInfo.GetIndex(gx, gy, gz - 1));
            }
        }

        LC->CollisionHeightData.Unlock();
    }

    if (LandscapeCompCount > 0)
        UE_LOG(LogTemp, Warning, TEXT("[Voxelizer] Landscape: %d components sampled, %d voxels marked."), LandscapeCompCount, LandscapeBuffer.Num());

    // 5. MERGE & COMPRESS
    int64 TotalSize = LandscapeBuffer.Num();
    for (const auto& Res : TaskResults) TotalSize += Res.Num();
    OutBlockedIndices.Empty(TotalSize);

    OutBlockedIndices.Append(LandscapeBuffer);
    for (const auto& Res : TaskResults) OutBlockedIndices.Append(Res);

    if (OutBlockedIndices.Num() > 0)
    {
        Algo::Sort(OutBlockedIndices);
        int64 UniqueCount = Algo::Unique(OutBlockedIndices);
        OutBlockedIndices.SetNum(UniqueCount);
    }

    UE_LOG(LogTemp, Warning, TEXT("[Voxelizer] Done. Unique Blocked Voxels: %d"), OutBlockedIndices.Num());
}