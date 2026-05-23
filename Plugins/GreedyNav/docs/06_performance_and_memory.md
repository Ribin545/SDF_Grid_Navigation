# Performance & Memory Reference

A practical guide for evaluating GreedyNav's runtime cost and memory footprint under different
bake configurations.

---

## Memory Model

### Page Table (Always allocated)

```
PtrMem = BrickDimX × BrickDimY × BrickDimZ × sizeof(FSDFBrick*)
       = BrickDimX × BrickDimY × BrickDimZ × 8 bytes
```

For default settings (5000×5000×2000 cm, 50 cm voxels):
```
VoxelDim  = 200 × 200 × 80  = 3,200,000 voxels
BrickDim  = 13  × 13  × 5   = 845 bricks
PtrMem    = 845 × 8          ≈ 6.6 KB   (negligible)
```

### Per-Brick Data

Each occupied `FSDFBrick` is **4,096 bytes** (4 KB):
```cpp
struct FSDFBrick { int8 Distances[4096]; };
```

A brick covers a 16³ voxel cube. Bricks that are entirely air (`nullptr`) or entirely solid
(`BRICK_TAG_SOLID`) cost **zero** data bytes — only the page table pointer.

### Bit Mask (During Bake Only)

```
VisitedMask = TotalVoxels / 64 × 8 bytes
            = 3,200,000 / 64 × 8 ≈ 400 KB
```

Freed after `BakeSVO` returns.

### Pathfinder (Per Query)

```
Contexts  ≈ VisitedNodes × 24 bytes
OpenSet   ≈ QueuedNodes  × 12 bytes
```

Worst case on a large map with `MAX_PATH_STEPS = 2M`:
```
~2,000,000 × 24 = 48 MB (Contexts)
~2,000,000 × 12 = 24 MB (OpenSet)
```
> These are per-call, non-persistent allocations.

---

## Bake Time Estimator

Bake time scales with:

1. **Voxelizer:** `O(Components × TrianglesOrPrimitives × VoxelsPerPrimitive)`
2. **Baker BFS Levels:** `O(MaxShellDist / VoxelSize)` = 250/50 = **5 levels**
3. **Baker Serial Alloc:** `O(FrontierSize)` per level (bottleneck on large maps)
4. **Compression:** `O(TotalBricks × 4096)` for `IsUniform` scan

### Rough Performance Table

| Config / Extents (cm) | Resolution (cm) | Logical Voxels | Blocked Voxels | Measured Bake Time | RAM Footprint | Notes |
|---|---|---|---|---|---|---|
| 50000 × 50000 × 10000 | 50 | 1.6 B | 60,199,796 | **10.41 s** (Total) | **422.32 MB** | **Measured (Workstation)** |

> **Benchmark Details:**
> - **Voxelizer time:** 4,325.86 ms (Processing 43 static mesh components)
> - **SDF propagation & compression time:** 6,047.58 ms
> - **Total bake time:** 10,411.45 ms
> - **SVO Structure:** 107,351 Real Bricks, 1,250 Solid Tags
> - **Pathfinding query (~700 m):** 16.6217 ms (2,021 raw steps expanded → 7 final waypoints)

---

## Memory Management Checklist

| Item | Status | Notes |
|------|--------|-------|
| `FSDFBrick` cleanup on rebake | ✅ | `FSparseVolumetricGrid::Init()` deletes old bricks |
| `FSDFBrick` cleanup on world tear-down | ✅ | **Fixed** — Custom destructor `~FSparseVolumetricGrid()` deallocates all dynamic bricks; deep-copy constructors/operators protect against double-frees. |
| `VisitedMask` (bake) | ✅ | Stack-allocated — freed on scope exit |
| Pathfinder `Contexts` / `OpenSet` | ✅ | Stack-allocated `TArray`/`TMap` — freed on return |
| Console command handles | ✅ | **Fixed** — All six console commands stored as subsystem members and cleanly unregistered/nulled in `Deinitialize()`. |
| `FMassiveBitArray` thread safety | ✅ | Atomic set, non-atomic read (correct pattern) |
| Parallel `delete` in compression | ✅ | **Fixed** — Refactored to scan brick uniform values in parallel and run deallocations sequentially on the main thread, preventing race conditions. |

---
