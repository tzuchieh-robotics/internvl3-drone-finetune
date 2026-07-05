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
| Obstacle count | 1500 | increased from an initial 10 → 300 → 1500 over iterations |
| Scale range | 10 - 40 | ~10-40x the drone's own size; initial version used 1.0-1.5 |
| X range | 10 - 800 | meters ahead of spawn |
| Y range | -400 - 400 | meters left/right of spawn |
| Z range | -30 - -2 | NED; nominal flight altitude is -8, so some obstacles float above/below cruise height instead of all sitting at flight level |
| Minimum spacing | none | obstacles are placed purely at random and may cluster or overlap, forcing narrow gaps |
| Assets used | Cube, Cylinder, Cone, Sphere | all confirmed present via `simListAssets()` in this Blocks build |

## Rationale
- Density and scale were increased in response to too few/too small obstacles in the default
  Blocks map, which produced heavily imbalanced action labels (almost all "forward").
- Overlap is intentionally allowed (no minimum-gap rejection) to create tighter, more varied
  navigable gaps rather than a evenly-spaced obstacle course.
- Z variation was added so obstacles aren't all at one altitude, adding vertical variety even
  though the current data-collection action space (forward/yaw-left/yaw-right) doesn't include
  climb/descend.

## Re-running
```bash
cd vlm_nav   # needs an active AirSim/Blocks connection
python spawn_obstacles.py
```
Destroys any previously spawned `obs_*` objects first, then spawns a fresh batch.
