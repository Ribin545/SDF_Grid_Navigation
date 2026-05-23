# Architecture Overview — GreedyNav Plugin

## Summary

GreedyNav is a **volumetric, 3-D navigation plugin** for Unreal Engine 5. It replaces the
standard 2-D `NavMesh` with a **Signed Distance Field (SDF)** stored inside a sparse, brick-
based data structure. The plugin supports full 6-DOF pathfinding for flying/swimming AI that
need to navigate through complex 3-D spaces.

---

## Pipeline at a Glance

```
     World Geometry
           │
           ▼
  ┌─────────────────────┐
  │  Voxelizer Layer    │  (Three strategies – see docs/02)
  │  FFastCpuVoxelizer  │  ◄── Active / Default
  │  FSparseCollision.  │  ◄── Alternative (physics primitives)
  │  FPhysicsVoxelizer  │  ◄── Stub / Not Implemented
  └──────────┬──────────┘
             │ TArray<int64> BlockedIndices
             ▼
  ┌─────────────────────┐
  │  FVolumetricBaker   │  Parallel wavefront BFS + SDF propagation
  │  (BakeSVO)          │  Shell culling → brick compression
  └──────────┬──────────┘
             │ FSparseVolumetricGrid (in-memory)
             ▼
  ┌─────────────────────┐
  │ FVolumetricPathfinder│  A* with SDF-guided safety cost
  │  FindPath()         │  + Greedy line-of-sight smoothing
  └──────────┬──────────┘
             │ TArray<FVector> World-space waypoints
             ▼
         AI / Caller
```

---

## Module Layout

| Path | Role |
|------|------|
| `Public/VolumetricTypes.h` | Core data structures: `FSDFBrick`, `FSparseVolumetricGrid` |
| `Public/VolumetricBaker.h/.cpp` | Distance-field baker |
| `Public/VolumetricPathfinder.h/.cpp` | A\* pathfinder + path smoothing |
| `Public/VolumetricSubsystem.h` | World subsystem; owns the live grid + console commands |
| `Private/VolumetricSubsystem.cpp` | Subsystem implementation |
| `Public/Voxelizers/FastCpuVoxelizer.*` | Triangle/primitive rasterizer (main path) |
| `Public/Voxelizers/SparseCollisionVoxelizer.*` | Collision-primitive rasterizer |
| `Private/GreedyNav.cpp` | Module entry, `Greedy.Bake` console command |

---

## Subsystem Lifetime

`UVolumetricSubsystem` is a **`UWorldSubsystem`**. It is created automatically when a world
initializes and destroyed when the world tears down.

- **`Initialize`** – Registers six console commands (`Nav.BakeSettings`, `Nav.Bake`,
  `Nav.Debug`, `Nav.Test`, `Nav.Probe`, `Nav.Slice`).
- **`Deinitialize`** – Calls `Super::Deinitialize()`, cleanly unregisters all six console command delegates from the engine console manager, and frees/nulls all command pointer handles to prevent delegate leaks and crashes.
- Owns `FSparseVolumetricGrid RawSparseGrid` — the live navigation data.

---

## Coordinate System

All internal positions are stored as **voxel grid indices** (flat `int64`). The grid origin is
the world-space minimum corner of the bake volume:

```
Origin = BakeCenter - BakeExtents
```

Helper conversions (all `FORCEINLINE` on `FSparseVolumetricGrid`):

| Method | Direction |
|--------|-----------|
| `GetIndexFromWorld(FVector)` | World → flat index |
| `GetWorldPos(int64)` | Flat index → world centre of voxel |
| `GetCoords(int64, X, Y, Z)` | Flat index → 3-D voxel coords |
| `GetIndex(X, Y, Z)` | 3-D coords → flat index (bounds-checked) |

---

## Threading Model

| Stage | Threading |
|-------|-----------|
| Voxelizer gather (component iteration) | **Main thread** |
| Voxelizer rasterize | `ParallelFor` — per-component task |
| Baker seed + BFS propagation | `ParallelFor` — per-frontier-node, sharded mutex |
| Baker brick allocation | **Main thread** (safe pointer setup before parallel write) |
| Baker compression pass | `ParallelFor` — per brick |
| Pathfinder (A\*) | **Caller thread** (synchronous, blocks game thread) |

> The bake pipeline runs synchronously — this is intentional for an **editor-time** workflow.
> See [Performance & Memory](./06_performance_and_memory.md) for timing details.

---

## Key Design Decisions

1. **Sparse Brick Map** — Instead of a dense 3-D array, the grid uses a flat `TArray<FSDFBrick*>`
   page table. Uniform bricks (all air or all solid) are collapsed to sentinel pointer values
   (`nullptr` / `BRICK_TAG_SOLID`), dramatically reducing RAM.

2. **Shell-Only SDF** — The baker only propagates the distance field for `MaxShellDist = 250 cm`
   around obstacles. Voxels further away are implicitly treated as open air (distance = 10000).
   This is a memory–accuracy trade-off.

3. **Weighted A\*** — `HEURISTIC_WEIGHT = 3.0` makes the search heavily greedy (faster but not
   guaranteed optimal). The pathfinder additionally uses SDF-derived **safety penalties** to
   push paths away from walls.
