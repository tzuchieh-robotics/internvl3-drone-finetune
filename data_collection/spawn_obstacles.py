"""
Clears previously spawned obstacles and spawns a dense field of large
obstacles for manual data collection. No minimum-spacing check: obstacles
may cluster tightly or overlap, forcing the drone to thread narrow gaps.

Reference sizes (see OBSTACLE_CONFIG.md for source/caveats):
  - Drone (AirSim SimpleFlight default quadrotor): ~0.98m x 0.98m x 0.29m (documented default, not empirically measured here)
  - Obstacle scale range below is in the same meter units as the Cube/Cylinder/Cone/Sphere assets (scale=1 ~= 1m)
"""
import random

import cosysairsim as airsim

NUM_OBSTACLES = 1500            # previously 300, now 5x denser
SCALE_RANGE = (10, 40)           # unchanged: ~10-40x the drone's own size
X_RANGE = (10, 800)
Y_RANGE = (-400, 400)
FLIGHT_Z = -8                    # nominal flight altitude (NED), kept for reference
Z_RANGE = (-30, -2)               # obstacle centers vary from high up (-30) to near ground (-2),
                                  # so some obstacles float above/below flight altitude instead of
                                  # all sitting at the drone's cruise height
ASSETS = ["Cube", "Cylinder", "Cone", "Sphere"]
# No MIN_GAP: obstacles are placed purely at random and may touch/overlap.

client = airsim.MultirotorClient()
client.confirmConnection()

existing = client.simListSceneObjects("obs_.*")
for name in existing:
    client.simDestroyObject(name)
print(f"destroyed {len(existing)} previous obstacles")

spawned = 0
for i in range(NUM_OBSTACLES):
    x = random.uniform(*X_RANGE)
    y = random.uniform(*Y_RANGE)
    z = random.uniform(*Z_RANGE)
    scale = random.uniform(*SCALE_RANGE)
    asset = random.choice(ASSETS)
    name = f"obs_{i:04d}_{asset.lower()}"
    pose = airsim.Pose(airsim.Vector3r(x, y, z), airsim.Quaternionr(0, 0, 0, 1))
    try:
        client.simSpawnObject(name, asset, pose, airsim.Vector3r(scale, scale, scale), physics_enabled=False)
        spawned += 1
        if spawned % 100 == 0:
            print(f"spawned {spawned}/{NUM_OBSTACLES}")
    except Exception as e:
        print(f"failed to spawn {name}: {e}")

print(f"Done. Spawned {spawned}/{NUM_OBSTACLES} obstacles (scale {SCALE_RANGE[0]}-{SCALE_RANGE[1]}, "
      f"area x={X_RANGE} y={Y_RANGE} z={Z_RANGE}, overlap allowed).")
