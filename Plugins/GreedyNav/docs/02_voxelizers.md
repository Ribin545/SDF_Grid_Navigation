# Voxelizer Systems

The voxelization layer is responsible for scanning world geometry and producing a flat list of
`int64` grid indices that represent **blocked voxels**. Three strategies exist; only one is
production-ready.

---

## 1. `FFastCpuVoxelizer` ✅ Active / Default

**File:** `Public/Voxelizers/FastCpuVoxelizer.h/.cpp`

### Strategy
Pure CPU math — no physics engine involvement. Iterates every `UStaticMeshComponent` in the
world, checks intersection with the bake bounds, then rasterises each component's collision
geometry (simple or complex) into voxel grid cells.

### Execution Flow

```
1. GridInfo.Init()           — sets Origin, Dimensions, BrickDimensions
2. Gather UStaticMeshComponents that overlap GridBounds
3. ParallelFor each component:
     a. If CollisionTraceFlag == CTF_UseComplexAsSimple OR no simple shapes:
          → Triangle rasterization from RenderData LOD0
     b. Otherwise:
          → Box / Sphere / Capsule primitive rasterization
          → Convex: bounding-box approximation (not exact — see bugs)
4. Merge all per-task result arrays
5. Sort + Algo::Unique → deduplicated index list
```

### Triangle Rasterisation Detail

For each triangle in LOD0:
- Projects to 2-D XY grid
- Tests each candidate grid column with `IsPointInTriangle2D()`
- Interpolates Z via `GetTriangleZ()` (planar interpolation)
- Marks a **single voxel** per XY column — does **not** fill solid interiors

> ⚠️ **Incomplete:** Only the surface shell is marked for complex meshes. A solid building
> will not have its interior marked as blocked.

### CPU Access Gate

```cpp
if (bComplex && Mesh->GetRenderData() && (Mesh->bAllowCPUAccess || GIsEditor))
```

Complex-mesh rasterization only works if:
- The StaticMesh has **"Allow CPU Access"** enabled in the Asset settings, **OR**
- Running in the Editor.

In a packaged game with CPU Access disabled, the code silently falls back to bounding-box
approximation even for `CTF_UseComplexAsSimple` meshes.

### Helper Functions

| Function | Description |
|----------|-------------|
| `IsPointInPrimitive(LocalPoint, Shape, Type)` | Point-in-shape test for Box / Sphere / Capsule |
| `GetTriangleZ(V0,V1,V2, X,Y)` | Planar Z interpolation for XY point inside triangle |
| `IsPointInTriangle2D(P, A, B, C)` | Barycentric 2-D containment test |

---

## 2. `FSparseCollisionVoxelizer` ⚠️ Alternate / Incomplete

**File:** `Public/Voxelizers/SparseCollisionVoxelizer.h/.cpp`

### Strategy
Also CPU math, but geometry-first: gathers all collision primitives from every
`UStaticMeshComponent` into a flat `TArray<FVoxelizeTask>`, then processes them in parallel.

### Key Differences from FastCpuVoxelizer

| | FastCpuVoxelizer | SparseCollisionVoxelizer |
|--|--|--|
| Unit of work | Per-component | Per-primitive |
| Complex mesh | Triangle scan (if CPU access) | Falls back to bounding box |
| Convex hulls | Bounding-box only | Bounding-box only |
| Locking strategy | No lock (per-task local buffer) | Global `FCriticalSection Mutex` per-task write |

> serialising writes and eliminating most parallelism benefit. See [Performance & Memory](./06_performance_and_memory.md).

### `MarkBox` / `MarkSphere` / `MarkCapsule` — Removed

These empty stubs were removed from both `SparseCollisionVoxelizer.h` and `SparseCollisionVoxelizer.cpp` to clean up technical debt and eliminate dead code.

### Not Wired Into the Subsystem

`FSparseCollisionVoxelizer::Voxelize` is **never called** from `VolumetricSubsystem.cpp`.
`ExecuteBakeMesh` calls `FFastCpuVoxelizer::Voxelize` exclusively. This class cannot be
selected at runtime.

---

## 3. `FPhysicsVoxelizer` [REMOVED]

The files `PhysicsVoxelizer.h` and `PhysicsVoxelizer.cpp` have been deleted. This class was an empty, non-functional stub (no-op) and was removed to clean up the codebase.

---

## Voxelizer Selection

Currently **hardcoded** in `ExecuteBakeMesh()`:

```cpp
FFastCpuVoxelizer::Voxelize(GetWorld(), Center, BakeExtents, BakeResolution,
    RawSparseGrid, LastKnownBlockedIndices);
```

There is no runtime switch or Blueprint-configurable voxelizer strategy.
