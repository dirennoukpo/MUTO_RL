"""Step terrain: square rings of boxes around the start point.

Seen from above, ring k covers the band  CENTER + (k-1)·TREAD <= max(|x|, |y|) < CENTER + k·TREAD.
Going outward the ring heights follow  h, 2h, h, 0, h, 2h, h, 0, ...  (the start square is at 0),
so whatever direction the robot walks, it climbs two steps up, then two steps down, and so on.
Each ring is 4 boxes on one mocap body; the step height h changes per episode by moving that body.
(Moving boxes attached to the world at runtime does NOT work: collision detection keeps them where they
were compiled, and feet fall through. Mocap bodies are made to be moved.)

Collision groups: ground (floor + steps) has contype 4, conaffinity 3, so it touches the robot (feet and
body contype 1, leg capsules contype 2) but not itself.
"""
import mujoco
import numpy as np

CENTER = 0.3  # [m] half-size of the flat start square (robot footprint is ~0.4 m wide)
TREAD = 0.2  # [m] width of each step
N_RINGS = 10  # out to 2.3 m: farther than an episode walks at 0.15 m/s with changing commands
PATTERN = (1, 2, 1, 0)  # ring k height = h · PATTERN[(k - 1) % 4]
HALF_Z = 0.1  # [m] box half-height; boxes reach down below the floor


def add_rings(spec):
    """Add one mocap body per ring (at height 0) carrying its boxes. Returns the body names."""
    names = []
    for k in range(1, N_RINGS + 1):
        body = spec.worldbody.add_body()
        body.name = f"ring{k}"
        body.mocap = True
        inner, outer = CENTER + (k - 1) * TREAD, CENTER + k * TREAD
        mid = (inner + outer) / 2
        # two full-width strips (front/back in y) and two strips between them (left/right in x)
        boxes = [((0, mid), (outer, TREAD / 2)), ((0, -mid), (outer, TREAD / 2)),
                 ((mid, 0), (TREAD / 2, inner)), ((-mid, 0), (TREAD / 2, inner))]
        for i, ((x, y), (sx, sy)) in enumerate(boxes):
            g = body.add_geom()
            g.name = f"step{k}_{i}"
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = [sx, sy, HALF_Z]
            g.pos = [x, y, -HALF_Z]
            g.contype, g.conaffinity = 4, 3
            g.friction = [1.0, 0.02, 0.01]
            g.rgba = [0.55, 0.5, 0.45, 1] if k % 2 else [0.45, 0.42, 0.38, 1]
        names.append(body.name)
    return names


def ring_tops(h):
    """Top height of each ring for step height h."""
    return np.array([h * PATTERN[(k - 1) % 4] for k in range(1, N_RINGS + 1)])


def height_at(x, y, h):
    """Ground height under the point (x, y)."""
    r = max(abs(x), abs(y))
    if r < CENTER:
        return 0.0
    k = int((r - CENTER) // TREAD) + 1
    return float(ring_tops(h)[k - 1]) if k <= N_RINGS else 0.0
