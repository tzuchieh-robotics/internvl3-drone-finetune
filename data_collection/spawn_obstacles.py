"""
Clears previously spawned obstacles and spawns a moderate-density field of
large obstacles for manual data collection. Most obstacles keep a minimum
clearance from each other so passable gaps remain; a small fraction are
deliberately allowed to cluster/overlap for variety.

Reference sizes (see OBSTACLE_CONFIG.md for source/caveats):
  - Drone (AirSim SimpleFlight default quadrotor): ~0.98m x 0.98m x 0.29m (documented default, not empirically measured here)
  - Obstacle scale range below is in the same meter units as the Cube/Cylinder/Cone/Sphere assets (scale=1 ~= 1m)
"""
import random

import cosysairsim as airsim

NUM_OBSTACLES = 500              # previously 1500 (too dense, crashed the sim); dialed back
SCALE_RANGE = (10, 40)           # unchanged: ~10-40x the drone's own size
X_RANGE = (10, 800)
Y_RANGE = (-400, 400)
FLIGHT_Z = -8                    # nominal flight altitude (NED), kept for reference
Z_RANGE = (-30, -2)               # obstacle centers vary from high up (-30) to near ground (-2),
                                  # so some obstacles float above/below flight altitude instead of
                                  # all sitting at the drone's cruise height
ASSETS = ["Cube", "Cylinder", "Cone", "Sphere"]

MIN_GAP = 4                # minimum clearance between obstacle edges, so gaps stay passable
OVERLAP_CHANCE = 0.15       # fraction of obstacles allowed to skip the gap check and cluster/overlap
MAX_ATTEMPTS_PER_OBSTACLE = 20

SPAWN_POINT = (0, 0, -8)  # drone takeoff position, kept clear so the camera never spawns inside a mesh
SPAWN_CLEARANCE = 15      # extra margin beyond the obstacle's own half-size

client = airsim.MultirotorClient()
client.confirmConnection()

existing = client.simListSceneObjects("obs_.*")
for name in existing:
    client.simDestroyObject(name)
print(f"destroyed {len(existing)} previous obstacles")

placed = []  # (x, y, z, half_size)
spawned = 0
skipped_spawn_point = 0
skipped_no_room = 0

for i in range(NUM_OBSTACLES):
    allow_overlap = random.random() < OVERLAP_CHANCE
    placed_ok = False

    for _ in range(MAX_ATTEMPTS_PER_OBSTACLE):
        x = random.uniform(*X_RANGE)
        y = random.uniform(*Y_RANGE)
        z = random.uniform(*Z_RANGE)
        scale = random.uniform(*SCALE_RANGE)
        half_size = scale * 0.5

        dist_to_spawn = ((x - SPAWN_POINT[0]) ** 2 + (y - SPAWN_POINT[1]) ** 2 + (z - SPAWN_POINT[2]) ** 2) ** 0.5
        if dist_to_spawn < (half_size + SPAWN_CLEARANCE):
            continue

        if not allow_overlap:
            too_close = False
            for px, py, pz, phalf in placed:
                if (abs(x - px) < (half_size + phalf + MIN_GAP)
                        and abs(y - py) < (half_size + phalf + MIN_GAP)
                        and abs(z - pz) < (half_size + phalf + MIN_GAP)):
                    too_close = True
                    break
            if too_close:
                continue

        placed_ok = True
        break

    if not placed_ok:
        skipped_no_room += 1
        continue

    asset = random.choice(ASSETS)
    name = f"obs_{i:04d}_{asset.lower()}"
    pose = airsim.Pose(airsim.Vector3r(x, y, z), airsim.Quaternionr(0, 0, 0, 1))
    try:
        client.simSpawnObject(name, asset, pose, airsim.Vector3r(scale, scale, scale), physics_enabled=False)
        placed.append((x, y, z, half_size))
        spawned += 1
        if spawned % 100 == 0:
            print(f"spawned {spawned}/{NUM_OBSTACLES}")
    except Exception as e:
        print(f"failed to spawn {name}: {e}")

print(f"skipped {skipped_no_room} obstacles (couldn't find room after {MAX_ATTEMPTS_PER_OBSTACLE} tries each)")
print(f"Done. Spawned {spawned}/{NUM_OBSTACLES} obstacles (scale {SCALE_RANGE[0]}-{SCALE_RANGE[1]}, "
      f"area x={X_RANGE} y={Y_RANGE} z={Z_RANGE}, min_gap={MIN_GAP}, ~{int(OVERLAP_CHANCE*100)}% allowed to overlap).")
