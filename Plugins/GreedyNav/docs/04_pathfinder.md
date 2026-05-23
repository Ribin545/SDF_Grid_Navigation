# Pathfinder — `FVolumetricPathfinder`

**File:** `Public/VolumetricPathfinder.h/.cpp`

The pathfinder implements **weighted A\*** over the `FSparseVolumetricGrid`, followed by a
greedy line-of-sight **string-pulling** (Theta\*-style) smoothing pass.

---

## Public API

```cpp
// Main entry point — returns false if no path exists or grid is empty
static bool FindPath(const FVector& StartWorld, const FVector& EndWorld,
                     const FSparseVolumetricGrid& Grid,
                     TArray<FVector>& OutPath);

// Gradient of the SDF at a world position (currently unused — see below)
static FVector GetGradient(const FVector& WorldPos, const FSparseVolumetricGrid& Grid);

// DDA voxel march — returns false if line clips a voxel with distance < 10 cm
static bool IsLineSafe(const FVector& Start, const FVector& End,
                       const FSparseVolumetricGrid& Grid);

// Path shortcutting via IsLineSafe
static void SmoothPath(const TArray<FVector>& RawPath,
                       const FSparseVolumetricGrid& Grid,
                       TArray<FVector>& OutSmoothPath);

// BFS outward search for nearest unblocked voxel
static bool FindNearestSafeVoxel(const FVector& QueryPos,
                                 const FSparseVolumetricGrid& Grid,
                                 float MaxRadius, FVector& OutSafePos);

// O(1) distance lookup — made public for subsystem snap checks
static float GetDistance(int64 Index, const FSparseVolumetricGrid& Grid);
```

---

## A\* Implementation

### Constants

```cpp
const float HEURISTIC_WEIGHT = 3.0f;   // Weighted / inadmissible heuristic
const int32 MAX_PATH_STEPS   = 2000000; // Per-iteration cap (not node cap)
```

`HEURISTIC_WEIGHT = 3.0` makes this a **weighted A\*** (WA\*). The heuristic is **inadmissible** —
the algorithm will find a path but it is **not guaranteed to be optimal**. The log message on
failure correctly suggests increasing the weight.

### Node Representation

```cpp
struct FPathNode {
    int64 ID;    // Flat voxel index
    float F;     // f = g + h
};

struct FNodeContext {
    float G;
    int64 ParentID;
    bool  bClosed;
};
```

The open set is a `TArray<FPathNode>` used as a **binary heap** via UE's `HeapPush` / `HeapPop`.
Node contexts are stored in a `TMap<int64, FNodeContext>` (hash map).

### Cost Function

```cpp
// bInOpenAir = (current SDF distance > 400 cm)
if (bInOpenAir)
    NewG = CurrentG + Grid.VoxelSize;          // No penalty in open air
else
{
    float SafetyDist    = GetDistance(NeighborID, Grid);
    if (SafetyDist <= 0.0f) continue;           // Blocked — skip
    float SafetyPenalty = FMath::Max(0.0f, 200.0f - SafetyDist) * 2.0f;
    NewG = CurrentG + Grid.VoxelSize + SafetyPenalty;
}
```

**Safety penalty ramp:**
- Distance ≥ 200 cm → no penalty
- Distance = 100 cm → penalty = 200 cm worth of extra cost
- Distance = 0 cm → blocked (skipped)

This encourages the path to fly through the centre of corridors rather than hugging walls.

### Closed-Set Handling

Nodes are lazily marked closed when popped. A re-queued node with a better G is handled by
the `if (NewG < NeighborCtx->G)` guard. Stale entries are discarded via:

```cpp
if (Contexts[CurrentID].bClosed) continue;
```

This pattern (lazy deletion) is correct but leads to **heap bloat** — stale entries accumulate
in `OpenSet` and inflate memory usage on long paths.

---

## Path Smoothing — `SmoothPath`

A greedy string-pull using `IsLineSafe`:

```
Anchor = Path[0]
For each subsequent node:
    If line from Anchor → next node is safe: skip current node
    Else: commit current node as waypoint, advance Anchor
```

This is equivalent to **Theta\*** post-processing. The result is a minimal waypoint list along
the raw A\* path.

### `IsLineSafe` — Step-March Ray

```cpp
int32 Steps = CeilToInt(Dist / (VoxelSize * 0.5f)); // Half-voxel steps
if (Steps > 1000) Steps = 1000;                      // Hard cap
```

**Tunnelling risk:** The 0.5× step size fixes the original full-voxel-step tunnelling issue
(noted in a `// FIX 1` comment), but the **hard cap of 1000 steps** means rays longer than
`500 × VoxelSize` are not fully checked. At 50 cm resolution that is **250 m**. Paths in large
open spaces may shortcut through geometry that is further than 250 m from the start waypoint.

---

## `FindNearestSafeVoxel`

Expands outward shell-by-shell (Manhattan radius) from a blocked start/end position. Returns
the first voxel with positive SDF distance.

```
For R = 1..MaxSteps:
    For shell of radius R (all voxels where max(|dx|,|dy|,|dz|) == R):
        Check SDF distance → return first positive
```

**Worst-case complexity:** O(R³) checks, though only the shell is checked per level (~6R²).
Not a BFS — does not avoid re-checking. For large `MaxRadius` this is wasteful.

---

## `GetGradient` — Implemented

`GetGradient` computes the SDF gradient at any world position using a **6-neighbour central
finite difference** scheme:

```cpp
float DistXPlus  = Grid.GetDistanceValues(X + 1, Y, Z);
float DistXMinus = Grid.GetDistanceValues(X - 1, Y, Z);
// ... repeat for Y and Z ...
FVector Gradient(
    (DistXPlus - DistXMinus) / (2.0f * VoxelSize),
    (DistYPlus - DistYMinus) / (2.0f * VoxelSize),
    (DistZPlus - DistZMinus) / (2.0f * VoxelSize)
);
return Gradient.IsNearlyZero() ? FVector::ZeroVector : Gradient.GetSafeNormal();
```

The returned normalized vector points **away from the nearest wall** — ideal for gradient-descent
repulsion or smooth wall-avoidance movement. It is not yet called anywhere in the runtime system,
but is available as a public static for callers.

---
