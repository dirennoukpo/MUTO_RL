"""Generate model/muto.xml (MuJoCo MJCF) from the SolidWorks URDF.

Run again whenever muto_des.urdf or a constant below changes:
    uv run python tools/build_mjcf.py

What this does compared to a raw URDF import:
  * adds a free joint so the body is not bolted to the world
  * meshes are visual-only; collisions use simple shapes (2 boxes for the body,
    1 sphere per foot, 1 capsule per femur, 1 box per tibia shaft) which is faster and more stable for RL.
    Leg shapes only touch the ground/terrain (contype 2), so a shin can catch on a step edge
    but legs don't collide with each other or the body.
  * adds the servo masses the SolidWorks export is missing (see SERVO_MASS)
  * models the Yahboom 35 kg bus servo as a DC motor + position controller
  * orders the actuators by the real servo IDs 1..18
  * adds IMU sensors (gyro, accelerometer, orientation) and foot touch sensors

Leg names (pinyin): z = zuo 左 = LEFT (+y), y = you 右 = RIGHT (-y),
                    q = qian 前 = front (+x), z = zhong 中 = middle, h = hou 后 = rear (-x).
Joint 1 = hip yaw (coxa), 2 = hip pitch (femur), 3 = knee (tibia).
"""

import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from muto_env.robot import HOME_SERVO_DEG, SERVO_LIMIT_DEG, servo_to_urdf  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
URDF = ROOT.parent / "assets/yahboomcar_description/urdf/muto_des.urdf"
MESH_DIR = ROOT / "assets/meshes"
OUT = ROOT / "model/muto.xml"

# Real servo IDs (Yahboom "Control Bus Servo" tutorial): leg1 = right front ... leg6 = left front.
#   leg1 yq: 1 2 3   leg2 yz: 4 5 6    leg3 yh: 7 8 9
#   leg4 zh: 10 11 12  leg5 zz: 13 14 15  leg6 zq: 16 17 18
from muto_env.robot import LEGS_BY_SERVO_ID  # noqa: E402
FOOT_RADIUS = 0.008  # [m] rubber foot cap, ~16 mm diameter in photos
TIBIA_SHAFT_Z = (-0.128, -0.045)  # [m] tibia part covered by a box fitted to the mesh (link frame). It stops
#                                   15 mm above the foot bottom, so the foot stays the lowest point when tilted.
FEMUR_RADIUS = 0.012  # [m] [ESTIMATE] femur + servo bracket collision capsule

# ── Servo: Yahboom 35KG serial bus servo (datasheet: YB-SD35M "Introduction of 35kg bus servo") ──
# Values marked [DATASHEET] are from the datasheet at 7.4 V; [ESTIMATE] must be identified on the robot.
STALL_TORQUE = 35 * 0.0980665  # [N·m] [DATASHEET] stall torque >= 35 kgf·cm  -> 3.43 N·m
NO_LOAD_SPEED = (math.pi / 3) / 0.22  # [rad/s] [DATASHEET] <= 0.22 s / 60°     -> 4.76 rad/s
SERVO_MASS = 0.0615  # [kg] [DATASHEET] 61.5 g, 40 x 20 x 40.5 mm
SERVO_SIZE = (0.020, 0.040, 0.0405)  # [m] [DATASHEET]
# DC-motor model: tau = clip(KP * err, ±STALL_TORQUE) - BACK_EMF_DAMPING * qdot.
# With this damping the joint stalls at STALL_TORQUE and free-runs at NO_LOAD_SPEED, like the datasheet.
BACK_EMF_DAMPING = STALL_TORQUE / NO_LOAD_SPEED  # [N·m·s/rad] -> 0.72
# The next three come from Rhoban BAM's identification of the Feetech STS3215 (7.4 V), the closest
# servo with published measurements: armature 0.026, friction_base 0.052, friction_viscous 0.060.
ARMATURE = 0.03  # [kg·m²] [ESTIMATE] reflected rotor inertia; scaled up slightly for the stronger gearbox
FRICTION_LOSS = 0.05  # [N·m] [ESTIMATE] dry gearbox friction
VISCOUS_FRICTION = 0.06  # [N·m·s/rad] [ESTIMATE]
KP = 20.0  # [N·m/rad] [ESTIMATE] servo internal P gain: full torque at ~10° error

# Total mass if you weigh the real robot (kg); the missing mass is added to base_link. None = don't.
MEASURED_TOTAL_MASS = None


def fmt(v):
    return " ".join(f"{x:.6g}" for x in np.atleast_1d(v))


def joint_range(k):
    """URDF range (rad) of joint k (1..3) equivalent to the servo's [-90, 90] deg software limit."""
    lo = servo_to_urdf(*[-SERVO_LIMIT_DEG] * 3)[k - 1]
    hi = servo_to_urdf(*[SERVO_LIMIT_DEG] * 3)[k - 1]
    return min(lo, hi), max(lo, hi)


def mesh_parts(link_name):
    """Mesh asset names for a link: one per file, several if prepare_meshes.py chunked it."""
    if (MESH_DIR / f"{link_name}.stl").exists():
        return [link_name]
    return sorted(p.stem for p in MESH_DIR.glob(f"{link_name}_[0-9]*.stl"))


def foot_tip(leg):
    """Lowest vertex of the lower-leg mesh (link frame == world-aligned at zero pose)."""
    v = trimesh.load_mesh(MESH_DIR / f"{leg}3_Link.stl").vertices
    return v[v[:, 2].argmin()]


def tibia_box(leg):
    """Axis-aligned box (center, half-sizes) around the tibia shaft vertices, in the link frame."""
    v = trimesh.load_mesh(MESH_DIR / f"{leg}3_Link.stl").vertices
    v = v[(v[:, 2] >= TIBIA_SHAFT_Z[0]) & (v[:, 2] <= TIBIA_SHAFT_Z[1])]
    lo, hi = v.min(axis=0), v.max(axis=0)
    return (lo + hi) / 2, (hi - lo) / 2


def servo_inertia():
    a, b, c = SERVO_SIZE
    return SERVO_MASS / 12 * np.array([b * b + c * c, a * a + c * c, a * a + b * b])


def main():
    urdf = ET.parse(URDF).getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    children = {}
    for j in urdf.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)

    lines = []
    out = lines.append
    added_mass = 0.0

    def inertial(link, ind):
        i = link.find("inertial")
        I = i.find("inertia").attrib
        full = [I[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        out(f'{ind}<inertial pos="{i.find("origin").get("xyz")}" mass="{i.find("mass").get("value")}" '
            f'fullinertia="{" ".join(full)}"/>')

    def servo_body(name, pos, ind):
        """A welded child body carrying one servo's mass (the CAD servo parts are nearly massless)."""
        nonlocal added_mass
        added_mass += SERVO_MASS
        out(f'{ind}<body name="{name}" pos="{fmt(pos)}">'
            f'<inertial pos="0 0 0" mass="{SERVO_MASS}" diaginertia="{fmt(servo_inertia())}"/></body>')

    def visual(name, link, ind):
        rgba = link.find("visual/material/color").get("rgba")
        if name.endswith("2_Link"):  # export bug: yz2_Link came out white, the other femur servo blocks are dark
            rgba = links["zq2_Link"].find("visual/material/color").get("rgba")
        for part in mesh_parts(name):
            out(f'{ind}<geom class="visual" mesh="{part}" rgba="{rgba}"/>')

    def body(joint, ind):
        name = joint.find("child").get("link")
        link = links[name]
        leg = name[:2]
        out(f'{ind}<body name="{name}" pos="{joint.find("origin").get("xyz")}">')
        inertial(link, ind + "  ")
        if joint.get("type") == "revolute":
            lo, hi = joint_range(int(name[2]))
            out(f'{ind}  <joint name="{joint.get("name")}" class="servo" axis="{joint.find("axis").get("xyz")}" '
                f'range="{lo:.4f} {hi:.4f}"/>')
        visual(name, link, ind + "  ")
        if name.endswith("2_Link"):
            # SolidWorks lists both the femur (2) and knee (3) servos inside the *2_Link part
            knee = next(j for j in children[name] if j.get("type") == "revolute")
            servo_body(f"{leg}2_servo", [0, 0, 0], ind + "  ")
            knee_pos = [float(x) for x in knee.find("origin").get("xyz").split()]
            servo_body(f"{leg}3_servo", knee_pos, ind + "  ")
            out(f'{ind}  <geom name="{leg}_femur" class="leg" size="{FEMUR_RADIUS}" fromto="0 0 0 {fmt(knee_pos)}"/>')
        if name.endswith("3_Link"):
            tip = foot_tip(leg) + [0, 0, FOOT_RADIUS]
            out(f'{ind}  <geom name="{leg}_foot" class="foot" pos="{fmt(tip)}"/>')
            center, half = tibia_box(leg)
            out(f'{ind}  <geom name="{leg}_tibia" class="leg" type="box" pos="{fmt(center)}" size="{fmt(half)}"/>')
            out(f'{ind}  <site name="{leg}_foot" pos="{fmt(tip)}" size="{FOOT_RADIUS * 1.2:.4g}"/>')
        for child in children.get(name, []):
            body(child, ind + "  ")
        out(f"{ind}</body>")

    out("<!-- AUTO-GENERATED by tools/build_mjcf.py from muto_des.urdf. Edit the script, not this file. -->")
    out('<mujoco model="muto">')
    out('  <compiler angle="radian" meshdir="../assets/meshes" autolimits="true"/>')
    out("")
    out("  <default>")
    out('    <default class="muto">')
    out(f'      <joint damping="{BACK_EMF_DAMPING + VISCOUS_FRICTION:.4g}" armature="{ARMATURE}" '
        f'frictionloss="{FRICTION_LOSS}"/>')
    out(f'      <position kp="{KP}" forcerange="{-STALL_TORQUE:.4f} {STALL_TORQUE:.4f}"/>')
    out('      <default class="servo"/>')
    out('      <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>')
    out('      <default class="collision"><geom group="3" rgba="0.8 0.2 0.2 0.4"/></default>')
    out('      <default class="leg"><geom type="capsule" group="3" contype="2" conaffinity="0" density="0" '
        'rgba="0.9 0.6 0.2 0.4"/></default>')
    out(f'      <default class="foot"><geom type="sphere" size="{FOOT_RADIUS}" group="3" priority="1" '
        f'friction="1.0 0.02 0.01" condim="3" rgba="0.2 0.6 0.9 1"/></default>')
    out("    </default>")
    out("  </default>")
    out("")
    out("  <asset>")
    for name in links:
        for part in mesh_parts(name):
            out(f'    <mesh name="{part}" file="{part}.stl"/>')
    out("  </asset>")
    out("")
    out("  <worldbody>")
    base = links["base_link"]
    out('    <body name="base_link" pos="0 0 0.2" childclass="muto">')
    out('      <freejoint name="root"/>')
    inertial(base, "      ")
    visual("base_link", base, "      ")
    out("      <!-- collision: lower chassis + upper electronics stack (meshes are visual only) -->")
    out('      <geom name="chassis" class="collision" type="box" pos="0 0 -0.017" size="0.115 0.09 0.027"/>')
    out('      <geom name="top" class="collision" type="box" pos="0.005 0 0.075" size="0.095 0.05 0.065"/>')
    out('      <site name="imu" pos="0.05045 -0.0053215 0.0091914" size="0.005"/>')
    for hip in children["base_link"]:
        if hip.get("type") == "revolute":  # hip servo body sits under its horn, inside the chassis
            x, y, z = (float(v) for v in hip.find("origin").get("xyz").split())
            servo_body(f"{hip.get('name')[:2]}1_servo", [x, y, z - SERVO_SIZE[2] / 2], "      ")
    for child in children["base_link"]:
        body(child, "      ")
    out("    </body>")
    out("  </worldbody>")
    out("")
    out("  <actuator>")
    for i, leg in enumerate(LEGS_BY_SERVO_ID):
        for k in range(3):
            lo, hi = joint_range(k + 1)
            out(f'    <position name="{leg}{k + 1}" joint="{leg}{k + 1}_Joint" class="muto" ctrlrange="{lo:.4f} {hi:.4f}"/>'
                f"  <!-- servo ID {3 * i + k + 1} -->")
    out("  </actuator>")
    out("")
    out("  <sensor>")
    out('    <gyro name="imu_gyro" site="imu"/>')
    out('    <accelerometer name="imu_acc" site="imu"/>')
    out('    <framequat name="imu_quat" objtype="site" objname="imu"/>')
    for leg in LEGS_BY_SERVO_ID:
        out(f'    <touch name="{leg}_touch" site="{leg}_foot"/>')
    out("  </sensor>")
    out("</mujoco>")
    home = servo_to_urdf(*HOME_SERVO_DEG)

    urdf_mass = sum(float(link.find("inertial/mass").get("value")) for link in links.values())
    if MEASURED_TOTAL_MASS is not None:
        extra = MEASURED_TOTAL_MASS - urdf_mass - added_mass
        assert extra >= 0, f"measured mass is below URDF + servos ({urdf_mass + added_mass:.3f} kg)"
        idx = next(i for i, line in enumerate(lines) if '<freejoint name="root"/>' in line)
        lines.insert(idx + 2, f'      <body name="unmodeled_mass"><inertial pos="0 0 0" mass="{extra:.4f}" '
                              f'diaginertia="{fmt(extra / 12 * np.array([0.18**2 + 0.08**2, 0.23**2 + 0.08**2, 0.23**2 + 0.18**2]))}"/></body>')
        added_mass += extra

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines) + "\n")

    # home keyframe: Yahboom standby pose, base height so the feet just touch the ground
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(OUT))
    data = mujoco.MjData(model)
    qpos = np.zeros(model.nq)
    qpos[3] = 1
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name.endswith("_Joint"):
            qpos[model.jnt_qposadr[j]] = home[int(name[2]) - 1]
    data.qpos[:] = qpos
    mujoco.mj_kinematics(model, data)
    feet = [i for i in range(model.ngeom) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").endswith("_foot")]
    qpos[2] = -min(data.geom_xpos[i][2] for i in feet) + FOOT_RADIUS
    ctrl = [home[int(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)[2]) - 1] for a in range(model.nu)]
    lines.insert(-1, f'  <keyframe>\n    <!-- Yahboom standby pose, servos {HOME_SERVO_DEG} deg -->\n'
                     f'    <key name="home" qpos="{fmt(qpos)}" ctrl="{fmt(ctrl)}"/>\n  </keyframe>')
    OUT.write_text("\n".join(lines) + "\n")
    print(f"home pose: URDF q = {[round(math.degrees(v), 2) for v in home]} deg, base height {qpos[2]:.4f} m")
    print(f"wrote {OUT.relative_to(ROOT)}: URDF mass {urdf_mass:.3f} kg + added {added_mass:.3f} kg "
          f"= {urdf_mass + added_mass:.3f} kg")


if __name__ == "__main__":
    main()
