# Obstacle field configuration for manual data collection

Used by `spawn_obstacles.py` in the Blocks AirSim environment.

## Reference sizes

- **Drone** (AirSim SimpleFlight default quadrotor): approximately **0.98m x 0.98m x 0.29m**
  (motor-to-motor bounding box). This is AirSim's commonly documented default quadrotor size,
  not something we measured empirically in this environment — attempts to measure it directly
  (mesh vertex buffer inspection, collision-distance probing) hit unreliable data and a
  simulator crash, so we fell back to the standard published figure. Treat as approximate.
- **Obstacles**: Cube/Cylinder/Cone/Sphere assets, where `scale=1` corresponds to roughly 1m,
  i.e. the same units as the drone size above.

## Current settings (as of this commit)

| Parameter | Value | Notes |
|---|---|---|
| Obstacle count | 500 (target), ~460 actually placed | history: 10 → 300 (no gap check) → 1500 (no gap check, crashed the sim twice) → 500 with a gap check |
| Scale range | 10 - 40 | ~10-40x the drone's own size; initial version used 1.0-1.5 |
| X range | 10 - 800 | meters ahead of spawn |
| Y range | -400 - 400 | meters left/right of spawn |
| Z range | -30 - -2 | NED; nominal flight altitude is -8, so some obstacles float above/below cruise height instead of all sitting at flight level |
| Minimum spacing | 4m clearance between obstacle edges | most obstacles keep a passable gap; up to 20 placement retries per obstacle before it's skipped |
| Overlap chance | ~15% of obstacles | these skip the spacing check entirely and may cluster/overlap, for variety |
| Spawn-point clearance | 15m + obstacle half-size | keeps the takeoff point clear so the camera never spawns inside a mesh (this caused an all-black camera feed once) |
| Assets used | Cube, Cylinder, Cone, Sphere | all confirmed present via `simListAssets()` in this Blocks build |

## Rationale
- Density and scale were increased in response to too few/too small obstacles in the default
  Blocks map, which produced heavily imbalanced action labels (almost all "forward").
- 1500 fully-random overlapping obstacles crashed the Blocks process twice — dialed back to 500
  with a minimum-gap check (15% still allowed to overlap/cluster) for stability while keeping
  most gaps flyable.
- Z variation was added so obstacles aren't all at one altitude, adding vertical variety.
  `collect_flight_frames.py` accordingly added Up/Down arrow key controls for ascend/descend so
  the drone can maneuver around them — these are maneuvering-only and are not logged as training
  samples (the label schema only covers left/right/both, not climb/descend).

## Re-running
```bash
cd vlm_nav   # needs an active AirSim/Blocks connection
python spawn_obstacles.py
```
Destroys any previously spawned `obs_*` objects first, then spawns a fresh batch.
