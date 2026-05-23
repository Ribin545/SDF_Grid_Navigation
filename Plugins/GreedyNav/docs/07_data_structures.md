# Data Structures — `VolumetricTypes.h`

**File:** `Public/VolumetricTypes.h`

This header defines every core type used by the GreedyNav pipeline.

---

## Compile-Time Constants

```cpp
#define SDF_BRICK_SIZE  16      // Voxels per brick edge
#define SDF_BRICK_SHIFT  4      // log2(16) — used for fast division
#define SDF_BRICK_MASK  15      // 0x0F     — used for fast modulo

#define BRICK_TAG_SOLID ((FSDFBrick*)0x1)  // Sentinel: all voxels are solid
```

`BRICK_TAG_SOLID` works because valid heap pointers are always **at least 8-byte aligned**.
The value `0x1` (odd integer) can never be a valid heap address, making it a safe sentinel.

`nullptr` serves as the complementary sentinel for **all-air** bricks.

---

## `FSDFBrick` — 4 KB Voxel Data Chunk

```cpp
struct FSDFBrick
{
    int8 Distances[4096];  // 16 × 16 × 16 = 4096 voxels
    FSDFBrick() { FMemory::Memset(Distances, 127, sizeof(Distances)); }
    bool IsUniform(int8& OutValue) const;
};
```

### Layout
Voxels are stored in **XYZ linear order** (X fastest, Z slowest):

```
Index = LocalX + LocalY×16 + LocalZ×256
```

This matches the lookup in `GetDistanceValues`:
```cpp
int32 LocalIdx = (X & SDF_BRICK_MASK)
               + ((Y & SDF_BRICK_MASK) << 4)
               + ((Z & SDF_BRICK_MASK) << 8);
```

### Default Value: `127`
All voxels are initialized to `int8(127)` — the maximum positive value. The subsystem
interprets `127` as **maximum safe distance (open air)**. Any voxel not written during
the BFS shell remains at this default.

### `IsUniform`
Linear scan — O(4096). Called once per brick during compression. Could be accelerated with
SIMD (`FPlatformMemory::Memcmp` against a reference array), but at bake time this is
not a bottleneck.

---

## `FSparseVolumetricGrid` — The Page Table

```cpp
USTRUCT()
struct FSparseVolumetricGrid
{
    GENERATED_BODY()

    TArray<FSDFBrick*> FlatBricks;   // Page table — one pointer per brick
    FVector  Origin;                  // World-space minimum corner
    FIntVector Dimensions;            // Total voxels (X, Y, Z)
    FIntVector BrickDimensions;       // Total bricks per axis
    float    VoxelSize;
};
```

### Indexing Scheme

Two-level lookup:
```
Brick Index = BrickX + BrickY × BrickDimX + BrickZ × BrickDimX × BrickDimY
LocalIndex  = LocalX + LocalY × 16        + LocalZ × 256
```

Both indices derived via bit-shift (no division at runtime).

### `Init(Center, Extents, VoxelSize)`

Initialises dimensions, cleans up any previously allocated bricks, and zeroes the pointer
array. This is the **only safe rebake entry point** — calling `BakeSVO` without `Init` would
write into stale brick pointers.

### `GetDistanceValues(X, Y, Z)` → `float`

Hot-path lookup used by both the Baker and Pathfinder:

| Pointer State | Return Value |
|---|---|
| `nullptr` (Air) | `10000.0f` |
| `BRICK_TAG_SOLID` | `0.0f` |
| Valid `FSDFBrick*` | `int8 × VoxelSize` |

The return type is `float` in world-space centimetres. Distance `≤ 0` means blocked.
Distance `≥ 10000` means open air (implicit — never stored).

### `AllocateBrick(X, Y, Z)` → `FSDFBrick*`

Creates a new brick if the slot is `nullptr` or was previously a `BRICK_TAG_SOLID`.
If the slot already holds a valid brick, returns it unchanged. Used by the baker's
serial allocation phase.

### `OptimizeBrick(BrickIndex)`

Post-bake compression helper. Checks uniformity; if uniform, frees the brick and replaces
with the appropriate sentinel. Called in parallel from `BakeSVO`.

---

## Pointer Arithmetic Safety Note

`BRICK_TAG_SOLID` is a **tagged pointer** — a value that is stored in a pointer field but is
not a valid address. This pattern is common in low-level allocators and SDF engines but
introduces subtle risks:

- Must **never** be dereferenced.
- Must be checked before every brick access (both the Baker and Pathfinder do this correctly).
- Is incompatible with smart pointers (`TUniquePtr` etc.), which is why raw pointers are used.
- If the code is ever ported to a platform with a smaller pointer alignment (< 2 bytes, which
  would be unusual), the sentinel value `0x1` would no longer be safe.
