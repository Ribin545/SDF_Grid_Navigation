# Console Commands Reference

All commands are registered by `UVolumetricSubsystem::Initialize()` and are available in the
Unreal console (tilde `~` key) at runtime.

---

## Bake Configuration

### `Nav.BakeSettings <ExtX> <ExtY> <ExtZ> <Resolution>`

Updates the bake volume without triggering a rebake.

| Argument | Type | Default | Description |
|---|---|---|---|
| ExtX | float | 5000 | Half-extent along X axis (cm) |
| ExtY | float | 5000 | Half-extent along Y axis (cm) |
| ExtZ | float | 2000 | Half-extent along Z axis (cm) |
| Resolution | float | 50 | Voxel edge size (cm) |

The bake volume is always centred at world origin `(0, 0, 0)`.

**Example:**
```
Nav.BakeSettings 3000 3000 1500 25
```

---

### `Nav.Bake`

Runs the full bake pipeline:
1. Calls `FFastCpuVoxelizer::Voxelize()`
2. Calls `FVolumetricBaker::BakeSVO()`

Intended as an **editor-time** command. Logs timing and memory statistics to the Output Log on completion.

---

## Debug Visualisation

### `Nav.Debug`

Draws persistent debug points for all non-air voxels within 50 m of the player pawn.

- **Red** = Distance 0 (solid surface)
- **Green** = Distance at max shell range
- Gradient between red and green = distance from nearest obstacle

Capped at **100,000 drawn points** for performance. Uses `goto DrawDone` as an early exit.

---

### `Nav.Slice <Z>`

Draws red debug boxes for all blocked voxels at the specified world Z coordinate (±VoxelSize).
Uses `LastKnownBlockedIndices` (raw voxelizer output — not the baked SDF), so it shows
geometry before distance propagation.

**Example:**
```
Nav.Slice 100
```

---

### `Nav.Probe <X> <Y> <Z>`

Queries the SDF distance at an exact world position and prints the result to the log.

**Output format:**
```
=== PROBE ===
Point: X=500.0 Y=0.0 Z=100.0
Index: 12345
Distance: 125.50 cm (Gradient Surface)
```

| Distance Range | Status String |
|---|---|
| ≥ 1000.0 | `(Open Air - Nullptr)` |
| ≤ 0.0 | `(Solid Wall - Tag)` |
| Otherwise | `(Gradient Surface)` |

Also draws a purple debug box at the queried voxel for 5 seconds.

---

## Pathfinding Test

### `Nav.Test path [Sx Sy Sz] [Ex Ey Ez]`

Runs a full path query and visualises the result.

**Arguments:**
- `path` — required mode string (only mode currently implemented)
- `Sx Sy Sz` — start world position (defaults to `0,0,0`)
- `Ex Ey Ez` — end world position (defaults to start + 5000 on X)

**Behaviour:**
1. Checks if Start/End are inside a solid voxel (SDF ≤ 0).
2. If blocked, calls `FindNearestSafeVoxel(MaxRadius=500cm)` to snap to safety.
   - Yellow line drawn from original to snapped position.
3. Calls `FVolumetricPathfinder::FindPath()`.
4. On success: draws green lines between waypoints, cyan dots every other step.
5. On failure: draws red line from start to end.

**Log output (success):**
```
--- PATH SUCCESS ---
Steps: 47
Time:  0.8321 ms
```

**Log output (failure):**
```
--- PATH FAILED ---
Time: 2541.3200 ms   ← indicates iteration limit hit
```

---

## `Greedy.Bake` (Module Command)

Registered in `FGreedyNavModule::StartupModule()`. Calls `Subsystem->ExecuteBakeMesh(TArray<FString>())` —
equivalent to `Nav.Bake`. Prefer `Nav.Bake` for direct subsystem control; `Greedy.Bake` is
useful for triggering bakes from automation scripts or the module level.

---

## Command Lifecycle

All six console commands (`Nav.BakeSettings`, `Nav.Bake`, `Nav.Debug`, `Nav.Test`,
`Nav.Probe`, `Nav.Slice`) are stored as `IConsoleCommand*` member pointers on
`UVolumetricSubsystem`. They are cleanly unregistered and nulled in `Deinitialize()`,
preventing stale delegate crashes on world reload or PIE stop.
