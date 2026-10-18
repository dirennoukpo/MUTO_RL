"""Save a PNG of the robot after settling: left = visual meshes, right = collision shapes.

    uv run python tools/render.py [out.png]
"""
import sys

import mujoco
import numpy as np
from PIL import Image

m = mujoco.MjModel.from_xml_path("model/scene.xml")
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)
for _ in range(500):
    mujoco.mj_step(m, d)

r = mujoco.Renderer(m, 480, 640)
cam = mujoco.MjvCamera()
cam.lookat[:] = [0, 0, 0.1]
cam.distance, cam.azimuth, cam.elevation = 0.75, 135, -20
r.update_scene(d, cam)
visual = r.render()
opt = mujoco.MjvOption()
opt.geomgroup[2], opt.geomgroup[3] = 0, 1
r.update_scene(d, cam, scene_option=opt)
collision = r.render()
out = sys.argv[1] if len(sys.argv) > 1 else "muto.png"
Image.fromarray(np.hstack([visual, collision])).save(out)
print("saved", out)
