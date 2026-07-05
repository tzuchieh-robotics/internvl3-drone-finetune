# Obstacle field configuration for manual data collection

Used by `spawn_obstacles.py` in the Blocks AirSim environment.

## Goal

Train the drone to fly a roughly **straight line while continuously doing local
obstacle avoidance** -- not maze-style navigation with long detours or dead ends.
Density needs to stay high enough that a straight-ish path almost always has
something nearby to dodge; sparse patches just waste collection time on plain
"forward" frames.

## Reference sizes

- **Drone** (AirSim SimpleFlight default quadrotor): approximately **0.98m x 0.98m x 0.29m**
  (motor-to-motor bounding box). This is AirSim's commonly documented default quadrotor size,
  not something we measured empirically in this environment — attempts to measure it directly
  (mesh vertex buffer inspection, collision-distance probing) hit unreliable data and a
  simulator crash, so we fell back to the standard published figure. Treat as approximate.
- **Obstacles**: Cube/Cylinder/Cone/Sphere assets, where `scale=1` corresponds to roughly 1m,
  i.e. the same units as the drone size above.

## Current layout: zoned corridor

The flight corridor (x = 10..800) is cut into 80-unit zones. Each zone is randomly assigned
one of three layouts:

| Zone type | Weight | Description |
|---|---|---|
| `maze` | 50% | Walls perpendicular to the flight path, each with a single gap. The gap's Y position is kept close to the centerline (`GAP_CENTER_RANGE = (-15, 15)`) so the drone makes local dodges, not big detours. |
| `forest` | 20% | Sparse tall, thin cylinders ("tree trunks", diameter 4-8, height 25-40) to weave between. |
| `open` | 30% | Moderate-density random scatter (min 4m gap between edges, ~15% allowed to overlap/cluster), narrowed to `Y = (-40, 40)` so obstacles are actually near the flight path instead of scattered somewhere a straight flight would never reach. |

All obstacle centers vary in Z from -30 to -2 (NED); nominal flight altitude is -8, so some
obstacles float above/below cruise height instead of all sitting at flight level.
`collect_flight_frames.py` has Up/Down arrow key controls for ascend/descend to maneuver
around these -- maneuvering only, not logged as training samples (the label schema only
covers left/right/both, not climb/descend).

## Spawn-point safety check

A "too close to spawn" check keeps the takeoff point (0, 0, -8) clear so the camera never
ends up inside a mesh. This is a proper **per-axis (AABB vs. point) check** -- each of x/y/z
must clear `half-extent + 15m` independently. An earlier version used a single
Euclidean-distance-vs-radius check, which under-detects overlap for elongated shapes: a maze
wall (thin in x, long in y) can have its *center* far from the spawn point while its bounding
box still spans across it. That bug caused two all-black-camera incidents before being fixed.

## History
- 10 → 300 (no gap check) → 1500 (no gap check, crashed the sim twice) → 500 with a min-gap
  check → current zoned maze/forest/open system, added after realizing pure random scatter
  didn't produce the "straight line + continuous local avoidance" training signal we actually
  want.

## Re-running
```bash
cd vlm_nav   # needs an active AirSim/Blocks connection
python spawn_obstacles.py
```
Destroys any previously spawned `obs_*` objects first, then spawns a fresh batch.
(A copy of this script also lives in this repo's `data_collection/`; keep both in sync.)
