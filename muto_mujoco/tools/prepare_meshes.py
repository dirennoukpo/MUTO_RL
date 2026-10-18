"""Copy the SolidWorks STL meshes into assets/meshes/ at full detail.

MuJoCo refuses meshes with more than 200k triangles, so larger meshes
(base_link, IMU_Link) are cut into chunks: base_link_0.stl, base_link_1.stl, ...
The chunks are just consecutive slices of the triangle list; drawn together
they look exactly like the original. (Decimating instead destroyed thin parts
like the antenna brackets.)

    uv run python tools/prepare_meshes.py
"""

import pathlib

import numpy as np
import trimesh

SRC = pathlib.Path(__file__).resolve().parents[2] / "assets/yahboomcar_description/meshes"
DST = pathlib.Path(__file__).resolve().parents[1] / "assets/meshes"
MAX_FACES = 190_000  # MuJoCo limit is 200k

DST.mkdir(parents=True, exist_ok=True)
for old in DST.glob("*.stl"):
    old.unlink()
for stl in sorted(SRC.glob("*.STL")):
    mesh = trimesh.load_mesh(stl, process=False)
    n = len(mesh.faces)
    if n <= MAX_FACES:
        mesh.export(DST / f"{stl.stem}.stl")
        print(f"{stl.name:18s} {n:>7d} faces")
        continue
    for i, faces in enumerate(np.array_split(np.arange(n), int(np.ceil(n / MAX_FACES)))):
        mesh.submesh([faces], append=True).export(DST / f"{stl.stem}_{i}.stl")
    print(f"{stl.name:18s} {n:>7d} faces -> {i + 1} chunks")
