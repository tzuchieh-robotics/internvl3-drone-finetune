"""
Clears previously spawned obstacles and spawns a structured obstacle course:
the flight corridor (x: X_START..X_END) is cut into zones, each randomly
assigned one of three layouts:
  - "maze":  a series of walls perpendicular to the flight path, each with a
             single gap at a random Y position (classic slalom/maze corridor)
  - "forest": sparse tall, thin cylinders (tree trunks) to weave between
  - "open":  the old moderate-density random scatter (some min-spacing,
             ~15% allowed to overlap/cluster)
Maze is the dominant layout; forest shows up occasionally.

Reference sizes (see OBSTACLE_CONFIG.md for source/caveats):
  - Drone (AirSim SimpleFlight default quadrotor): ~0.98m x 0.98m x 0.29m (documented default, not empirically measured here)
  - scale=1 on the Cube/Cylinder/Cone/Sphere assets corresponds to roughly 1m, same units as the drone size above.
"""
import random

import cosysairsim as airsim

X_START, X_END = 10, 800
ZONE_LENGTH = 80
CORRIDOR_Y_RANGE = (-40, 40)   # wall/tree extents live in this band
GAP_CENTER_RANGE = (-15, 15)   # maze gaps stay close to the centerline: local dodges, not big detours
OPEN_Y_RANGE = (-40, 40)       # narrowed from +-400 so "open" obstacles are actually near the flight path
                               # instead of scattered somewhere a mostly-straight flight would never reach
Z_RANGE = (-30, -2)            # NED; nominal flight altitude is -8
FLIGHT_Z = -8

# Goal: train straight-line flight with continuous local obstacle avoidance, not
# maze-style detours/dead-ends. Density is kept high throughout so a roughly-straight
# path almost always has something nearby to dodge -- sparse patches just waste
# collection time on plain "forward" frames.
ZONE_WEIGHTS = {"maze": 0.5, "forest": 0.2, "open": 0.3}

# maze params
MAZE_GATE_SPACING = 13     # distance between successive walls within a maze zone (denser than before)
MAZE_GAP_WIDTH = (16, 24)  # opening width, wide enough for the drone plus maneuvering room
MAZE_WALL_THICKNESS = 3
MAZE_WALL_HEIGHT = (15, 25)

# forest params
FOREST_TRUNK_COUNT_PER_ZONE = (10, 18)
FOREST_TRUNK_DIAMETER = (4, 8)
FOREST_TRUNK_HEIGHT = (25, 40)
FOREST_MIN_GAP = 5

# open params (same as the previous moderate-density version)
OPEN_OBSTACLES_PER_ZONE = (22, 35)
OPEN_SCALE_RANGE = (10, 40)
OPEN_MIN_GAP = 4
OPEN_OVERLAP_CHANCE = 0.15
MAX_ATTEMPTS_PER_OBSTACLE = 20

ASSETS_OPEN = ["Cube", "Cylinder", "Cone", "Sphere"]

SPAWN_POINT = (0, 0, -8)
SPAWN_CLEARANCE = 15

client = airsim.MultirotorClient()
client.confirmConnection()

existing = client.simListSceneObjects("obs_.*")
for name in existing:
    client.simDestroyObject(name)
print(f"destroyed {len(existing)} previous obstacles")

obj_counter = 0
spawned = 0
skipped = 0


def too_close_to_spawn(x, y, z, half_x, half_y, half_z):
    # Proper per-axis (AABB vs point) check -- a single Euclidean-distance-vs-radius
    # check underestimates overlap for elongated shapes like maze walls (thin in x,
    # very long in y): a wall can have its center far from the spawn point yet still
    # span across it. All three axes must clear for the obstacle to count as "safe".
    return (abs(x - SPAWN_POINT[0]) < half_x + SPAWN_CLEARANCE
            and abs(y - SPAWN_POINT[1]) < half_y + SPAWN_CLEARANCE
            and abs(z - SPAWN_POINT[2]) < half_z + SPAWN_CLEARANCE)


def spawn_box(x, y, z, scale_xyz, asset="Cube"):
    global obj_counter, spawned
    name = f"obs_{obj_counter:04d}_{asset.lower()}"
    obj_counter += 1
    pose = airsim.Pose(airsim.Vector3r(x, y, z), airsim.Quaternionr(0, 0, 0, 1))
    scale = airsim.Vector3r(*scale_xyz)
    try:
        client.simSpawnObject(name, asset, pose, scale, physics_enabled=False)
        spawned += 1
    except Exception as e:
        print(f"failed to spawn {name}: {e}")


def spawn_maze_zone(x0, x1):
    x = x0 + MAZE_GATE_SPACING / 2
    while x < x1:
        z = FLIGHT_Z
        wall_height = random.uniform(*MAZE_WALL_HEIGHT)
        gap_width = random.uniform(*MAZE_GAP_WIDTH)
        gap_center = random.uniform(*GAP_CENTER_RANGE)
        y0, y1 = CORRIDOR_Y_RANGE
        gap_start, gap_end = gap_center - gap_width / 2, gap_center + gap_width / 2

        for seg_y0, seg_y1 in ((y0, gap_start), (gap_end, y1)):
            length = seg_y1 - seg_y0
            if length <= 1:
                continue
            center_y = (seg_y0 + seg_y1) / 2
            if too_close_to_spawn(x, center_y, z, MAZE_WALL_THICKNESS / 2, length / 2, wall_height / 2):
                continue
            spawn_box(x, center_y, z, (MAZE_WALL_THICKNESS, length, wall_height))
        x += MAZE_GATE_SPACING


def spawn_forest_zone(x0, x1):
    n = random.randint(*FOREST_TRUNK_COUNT_PER_ZONE)
    placed = []
    for _ in range(n):
        for _ in range(MAX_ATTEMPTS_PER_OBSTACLE):
            x = random.uniform(x0, x1)
            y = random.uniform(*CORRIDOR_Y_RANGE)
            diameter = random.uniform(*FOREST_TRUNK_DIAMETER)
            height = random.uniform(*FOREST_TRUNK_HEIGHT)
            half = diameter / 2
            if too_close_to_spawn(x, y, FLIGHT_Z, half, half, height / 2):
                continue
            if any(abs(x - px) < (half + phalf + FOREST_MIN_GAP) and abs(y - py) < (half + phalf + FOREST_MIN_GAP)
                   for px, py, phalf in placed):
                continue
            spawn_box(x, y, FLIGHT_Z, (diameter, diameter, height), asset="Cylinder")
            placed.append((x, y, half))
            break


def spawn_open_zone(x0, x1):
    n = random.randint(*OPEN_OBSTACLES_PER_ZONE)
    placed = []
    for _ in range(n):
        allow_overlap = random.random() < OPEN_OVERLAP_CHANCE
        for _ in range(MAX_ATTEMPTS_PER_OBSTACLE):
            x = random.uniform(x0, x1)
            y = random.uniform(*OPEN_Y_RANGE)
            z = random.uniform(*Z_RANGE)
            scale = random.uniform(*OPEN_SCALE_RANGE)
            half = scale / 2
            if too_close_to_spawn(x, y, z, half, half, half):
                continue
            if not allow_overlap and any(
                abs(x - px) < (half + phalf + OPEN_MIN_GAP)
                and abs(y - py) < (half + phalf + OPEN_MIN_GAP)
                and abs(z - pz) < (half + phalf + OPEN_MIN_GAP)
                for px, py, pz, phalf in placed
            ):
                continue
            asset = random.choice(ASSETS_OPEN)
            spawn_box(x, y, z, (scale, scale, scale), asset=asset)
            placed.append((x, y, z, half))
            break


zone_x = X_START
zone_log = []
while zone_x < X_END:
    x1 = min(zone_x + ZONE_LENGTH, X_END)
    zone_type = random.choices(list(ZONE_WEIGHTS), weights=list(ZONE_WEIGHTS.values()))[0]
    zone_log.append(zone_type)
    if zone_type == "maze":
        spawn_maze_zone(zone_x, x1)
    elif zone_type == "forest":
        spawn_forest_zone(zone_x, x1)
    else:
        spawn_open_zone(zone_x, x1)
    zone_x = x1

print(f"zones ({len(zone_log)}): {zone_log}")
print(f"Done. Spawned {spawned} obstacles across x={X_START}-{X_END}.")
