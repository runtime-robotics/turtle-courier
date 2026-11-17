"""Measured-in-metres rehearsal map. Coordinates were estimated from photos."""
import heapq
import math

FIELD = (3.40, 3.44)
BODY_RADIUS = .132
TRACK = .185
WHEEL = .0325
CAMERA_X = .100
SONAR_X = .115
HFOV = 1.40
ARUCO_SIZE = .095

# Starts face their corresponding sign. The route planner only learns the start ID from camera.
POSES = {
    21: (-1.14, 1.05, math.pi),
    22: (-1.14, .00, math.pi),
    23: (-1.14, -1.01, math.pi),
    25: (1.29, .77, -math.pi / 2),
    26: (1.29, -.92, math.pi / 2),
    27: (.63, -.16, 0.0),
}
NODES = {i: pose[:2] for i, pose in POSES.items()} | {
    'north': (-.12, 1.05), 'middle': (-.12, 0.0),
    'tee': (-.12, -.16), 'south': (-.12, -1.01), 'bend': (-.12, -1.40),
    'right_north': (1.29, 1.05), 'right_south': (1.29, -1.40),
    'bypass_n': (.305, 1.05), 'bypass_s': (.305, .0),
}
EDGES = (
    (21, 'north'), (22, 'middle'), (23, 'south'), (27, 'tee'),
    (25, 'right_north'), (26, 'right_south'), ('north', 'middle'),
    ('middle', 'tee'), ('tee', 'south'), ('south', 'bend'),
    ('north', 'right_north'), ('bend', 'right_south'),
    ('north', 'bypass_n'), ('bypass_n', 'bypass_s'), ('bypass_s', 'middle'),
)
TAG_DISTANCE = {i: .335 for i in POSES}
TAG_DISTANCE[25] = .32
TAG_DISTANCE[26] = .33
TAG_DISTANCE[27] = .32

# x, y, width, depth. The yellow cubes on the islands mirror the photos.
BUILDINGS = (
    (-1.56, 1.06, .17, .92), (-1.56, 0, .17, .91), (-1.56, -1.06, .17, .91),
    (.95, 1.56, 1.23, .24),
    (1.34, -.07, .42, 1.04), (1.045, -.16, .19, .34),
)
ISLAND_CUBES = (
    (-.82, 1.47, .30), (-.76, .54, .31), (-.80, -.52, .32),
    (-.83, -1.49, .32), (.70, .55, .29), (.70, -.89, .33),
)
# Extra cube can obstruct the shortest road and force a real sonar replan.
BLOCKER = (-.12, .52, .235, .235)


def tag_pose(i):
    x, y, yaw = POSES[i]
    return x + TAG_DISTANCE[i]*math.cos(yaw), y + TAG_DISTANCE[i]*math.sin(yaw), yaw


def route(start, goal, closed):
    queue = [(0., 0, [start])]
    seen = set()
    count = 0
    while queue:
        cost, _, path = heapq.heappop(queue)
        here = path[-1]
        if here in seen:
            continue
        if here == goal:
            return path
        seen.add(here)
        for a, b in EDGES:
            if here not in (a, b) or frozenset((a, b)) in closed:
                continue
            there = b if here == a else a
            distance = math.dist(NODES[here], NODES[there])
            count += 1
            heapq.heappush(queue, (cost + distance, count, path + [there]))
    return []
