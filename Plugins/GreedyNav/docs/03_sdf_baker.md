# SDF Baker — `FVolumetricBaker`

**File:** `Public/VolumetricBaker.h/.cpp`

The baker converts a raw set of blocked voxel indices into a full **Signed Distance Field (SDF)**
stored inside `FSparseVolumetricGrid`. It uses a parallel wavefront BFS and a post-pass
brick compression step.

---

## Entry Point

```cpp
static void BakeSVO(FSparseVolumetricGrid& Grid,
                    const TArray<int64>& InitialBlockedIndices);
```

`Grid.Init()` **must** be called before `BakeSVO`. The Subsystem guarantees this by calling the
voxelizer first (which internally calls `Grid.Init()`).

---

## Internal Helper Structures

### `FMassiveBitArray`
A flat `TArray<uint64>` bit-field used as the **visited mask** during BFS. Provides:

| Method | Description |
|--------|-------------|
| `Init(int64 TotalBits)` | Allocates `(TotalBits / 64) + 1` words |
| `AtomicSet(int64 Index)` | Thread-safe set via `FPlatformAtomics::InterlockedOr` |
| `IsSet(int64 Index)` | Non-atomic read (safe during serial merge phase) |

> ⚠️ **Memory:** For a 10k × 10k × 4k voxel world at 50 cm resolution, `TotalBits ≈ 64 billion`
> — this bit-array alone would require ~8 GB. In practice the plugin limits extents via
> `BakeExtents` (default 5000 × 5000 × 2000 cm ÷ 50 cm = 200 × 200 × 80 = 3.2 M voxels → 400 KB).

### `FFrontierShard`
Sharded work queue to reduce lock contention during parallel BFS expansion:

```cpp
struct FFrontierShard {
    FCriticalSection Mutex;
    TArray<int64>    NewNodes;
    uint8            Pad[64];   // Cache-line padding
};
FFrontierShard Shards[64];
```

Shard selection is `Index % 64` (where `Index` is the frontier array position, **not** the
voxel coordinate). This is a **modular-bias** mapping — it distributes load evenly but does not
attempt spatial locality.

---

## Algorithm: Parallel Wavefront BFS

### Phase 1 — Seed
All `InitialBlockedIndices` are marked visited and added to `CurrentFrontier`.
Each seed voxel receives distance value `0` (written via `BatchWriteToGrid`).

### Phase 2 — Shell Propagation

```
MaxShellDist = 250.0 cm
MaxSteps     = ceil(250 / VoxelSize)
```

For each BFS level `1..MaxSteps`:

```
A. ParallelFor CurrentFrontier:
     For each of 6 face-neighbours:
         If not visited → AtomicSet + add to shard
B. Merge shards → new CurrentFrontier
C. BatchWriteToGrid(CurrentFrontier, DistByte = clamp(Level, 1, 127))
```

**Distance encoding:** `int8` in range `[0, 127]`. Actual world distance in centimetres is
`DistByte × VoxelSize`. Maximum representable safe distance = `127 × VoxelSize`.

At 50 cm voxels → max safely-representable distance = **63.5 m** (plenty for the 250 cm shell,
but worth noting if resolution changes).

### Phase 3 — Brick Compression

After propagation, every `FSDFBrick` is inspected via `OptimizeBrick`:

```cpp
// In VolumetricTypes.h
void OptimizeBrick(int64 BrickIndex)
{
    int8 Val;
    if (Brick->IsUniform(Val))
    {
        delete Brick;
        FlatBricks[BrickIndex] = (Val <= 0) ? BRICK_TAG_SOLID : nullptr;
    }
}
```

This is run in a `ParallelFor` to scan for uniform bricks and cache their uniform values. To ensure 100% thread-safety and prevent concurrent deletes (which was a potential race condition on some allocators), the actual `delete` calls and page-table tagging are executed sequentially on the calling thread.

---

## `BatchWriteToGrid` — Two-Phase Write

To avoid allocating `FSDFBrick` objects from multiple threads simultaneously (which would cause
races on `new`), the helper uses a two-phase approach:

```
Phase A (Serial, Main Thread): AllocateBrick() for every index in the batch
Phase B (Parallel): Write the distance byte into the pre-allocated brick
```

This is correct but incurs an **O(N) serial allocation pass per BFS level**, which can
dominate time when the frontier is large.

---

## Memory Statistics Logged

After compression, the baker emits:

```
[Baker] Done in X ms.
   Final RAM:  X MB
   Real Bricks: X
   Solid Tags:  X
```

Calculation:
```cpp
int64 PtrMem  = FlatBricks.Num() * sizeof(FSDFBrick*);  // Page table overhead
int64 DataMem = RealBricks * sizeof(FSDFBrick);          // 4 KB per detailed brick
float TotalMB = (PtrMem + DataMem) / 1024.0f / 1024.0f;
```

> ⚠️ This does **not** account for `FMassiveBitArray` peak allocation (freed after bake) nor
> `TArray<int64>` frontier temporaries.

---

## Shell Distance Constant — `MaxShellDist`

```cpp
const float MaxShellDist = 250.0f; // cm — hardcoded
```

This is not configurable from the console or any property. If an agent is wider than 250 cm and
tries to navigate near walls, the SDF will report maximum (air) distance even though the voxel
is inside the clearance boundary.
