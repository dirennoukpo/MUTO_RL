"""MUTO RS walking on velocity commands (zero command = stand still in the home pose).

Policy interface — this is what the real robot must reproduce (see ROBOT_NOTES 6b/6c):
  obs (29) = projected gravity (3) | 0.25·gyro [rad/s] (3) | previous action (18) | command / CMD_MAX (3)
             | gait clock sin, cos (2π · GAIT_HZ · t) (2), or (0, 0) when the command is zero (= stand still)
  act (18) = joint targets: home + ACTION_SCALE · action, servo-ID order (robot.JOINT_NAMES),
             rounded to whole servo degrees, sent at 50 Hz
No measured joint angles in the observation: the real driver can't read them at 50 Hz.

Hardware effects simulated every episode: 1° command rounding, command latency, servo response lag,
25 Hz IMU with noise and mounting bias, and randomized mass, servo and floor parameters, plus pushes.
"""

from dataclasses import dataclass, field

import gymnasium as gym
import mujoco
import numpy as np

from . import robot

CMD_MAX = np.array([0.15, 0.10, 0.6])  # [m/s, m/s, rad/s] vx, vy, yaw rate. [ESTIMATE] real top speed unknown.
ACTION_SCALE = 0.3  # [rad]
GYRO_SCALE = 0.25
GAIT_HZ = 2.0  # tripod gait cycle frequency given to the policy as a clock
# tripod groups (indices in servo-ID leg order yq, yz, yh, zh, zz, zq): RF+RR+LM swing together, then RM+LR+LF
TRIPOD_A = np.array([True, False, True, False, True, False])


@dataclass
class EnvConfig:
    episode_s: float = 20.0
    command_resample_s: tuple = (3.0, 6.0)
    zero_command_prob: float = 0.2  # fraction of commands that mean "stand still"
    quantize_deg: bool = True
    # ── hardware effects (sampled once per episode) ──
    latency_s: tuple = (0.0, 0.02)
    servo_tau_s: tuple = (0.0, 0.04)  # PLACEHOLDER until the friend's servo_speed measurement is in
    imu_tilt_bias_deg: float = 1.5
    gyro_noise: float = 0.05  # [rad/s]
    gravity_noise: float = 0.02
    # ── physics randomization ──
    payload_kg: tuple = (0.0, 0.7)  # battery/electronics mass the CAD misses (ROBOT_NOTES 7)
    com_shift_m: float = 0.01
    kp_scale: tuple = (0.6, 1.5)
    torque_scale: tuple = (0.8, 1.1)  # stall torque drops with battery voltage
    armature: tuple = (0.02, 0.04)
    frictionloss: tuple = (0.02, 0.1)
    damping_scale: tuple = (0.8, 1.2)
    foot_friction: tuple = (0.5, 1.2)
    servo_zero_offset_deg: float = 2.0  # per-joint calibration error: each servo holds target + offset
    # ── perturbations ──
    push_interval_s: tuple = (3.0, 6.0)
    push_vel: float = 0.3  # [m/s] max horizontal velocity kick
    init_joint_noise: float = 0.05  # [rad]
    # ── reward weights (per 50 Hz step) ──
    w: dict = field(default_factory=lambda: dict(
        track_lin=2.0, track_yaw=1.0,            # follow the command
        upright=-5.0, height=-20.0,              # keep the body level and at standby height
        lin_vel_z=-2.0, ang_vel_xy=-0.05,        # no bouncing / rocking
        air_time=1.0, foot_slip=-0.5,            # take real steps, don't shuffle
        gait=0.5,                                # contact pattern follows the tripod clock
        stand_still=-1.0,                        # zero command: stay in the home pose,
        stand_feet=1.0, stand_motion=-0.05,      #   all six feet down, joints not moving
        action_rate=-0.02, torque=-0.002,        # smooth, low effort (servo lifetime, battery)
        fall=-20.0))


class MutoEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": robot.CONTROL_HZ}

    def __init__(self, cfg: EnvConfig | None = None, render_mode=None, seed=None):
        self.cfg = cfg or EnvConfig()
        self.render_mode = render_mode
        spec = mujoco.MjSpec.from_file(str(robot.SCENE_XML))
        spec.compiler.discardvisual = render_mode is None  # training: physics only, keeps memory low
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        m = self.model

        self.dt = 1.0 / robot.CONTROL_HZ
        self.substeps = round(self.dt / m.opt.timestep)
        self.imu_every = robot.CONTROL_HZ // robot.IMU_HZ
        self.max_steps = round(self.cfg.episode_s * robot.CONTROL_HZ)

        act_joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a, 0]) for a in range(m.nu)]
        assert act_joints == robot.JOINT_NAMES, act_joints
        self.qadr = m.jnt_qposadr[m.actuator_trnid[:, 0]]
        self.dadr = m.jnt_dofadr[m.actuator_trnid[:, 0]]
        self.ctrl_lo, self.ctrl_hi = m.actuator_ctrlrange[:, 0].copy(), m.actuator_ctrlrange[:, 1].copy()
        key = m.key("home")
        self.home_qpos, self.home_ctrl = key.qpos.copy(), key.ctrl.copy()
        self.servo_sign = np.tile(robot.SERVO_SIGN, 6)
        self.servo_off = np.tile(robot.SERVO_OFFSET_DEG, 6)

        self.base = m.body("base_link").id
        self.body_geoms = {m.geom("chassis").id, m.geom("top").id}
        self.floor = m.geom("floor").id
        self.feet = np.array([m.geom(f"{leg}_foot").id for leg in robot.LEGS_BY_SERVO_ID])
        self.gyro_adr = m.sensor("imu_gyro").adr[0]
        self.quat_adr = m.sensor("imu_quat").adr[0]
        self.touch_adr = np.array([m.sensor(f"{leg}_touch").adr[0] for leg in robot.LEGS_BY_SERVO_ID])

        self.nom = dict(
            mass=m.body_mass[self.base], inertia=m.body_inertia[self.base].copy(), ipos=m.body_ipos[self.base].copy(),
            kp=m.actuator_gainprm[:, 0].copy(), force=m.actuator_forcerange.copy(), damping=m.dof_damping.copy())

        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (3 + 3 + m.nu + 3 + 2,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (m.nu,), np.float32)
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        self._renderer = None

    # ── randomization ──────────────────────────────────────────────────────────
    def _u(self, rng, size=None):
        return self.np_random.uniform(*rng, size)

    def _randomize(self):
        m, c, nom = self.model, self.cfg, self.nom
        payload = self._u(c.payload_kg)
        m.body_mass[self.base] = nom["mass"] + payload
        m.body_inertia[self.base] = nom["inertia"] * (nom["mass"] + payload) / nom["mass"]
        m.body_ipos[self.base] = nom["ipos"] + self._u((-c.com_shift_m, c.com_shift_m), 3)
        kp = nom["kp"] * self._u(c.kp_scale, m.nu)
        m.actuator_gainprm[:, 0] = kp
        m.actuator_biasprm[:, 1] = -kp
        m.actuator_forcerange[:] = nom["force"] * self._u(c.torque_scale)
        m.dof_armature[self.dadr] = self._u(c.armature, m.nu)
        m.dof_frictionloss[self.dadr] = self._u(c.frictionloss, m.nu)
        m.dof_damping[self.dadr] = nom["damping"][self.dadr] * self._u(c.damping_scale, m.nu)
        m.geom_friction[self.feet, 0] = self._u(c.foot_friction)
        z = np.radians(c.servo_zero_offset_deg)
        self.zero_offset = self._u((-z, z), m.nu)
        self.latency_steps = int(round(self._u(c.latency_s) / m.opt.timestep))
        tau = self._u(c.servo_tau_s)
        self.servo_alpha = 1.0 if tau <= 0 else m.opt.timestep / (tau + m.opt.timestep)
        b = np.radians(c.imu_tilt_bias_deg)
        self.imu_bias = np.array([self._u((-b, b)), self._u((-b, b)), 0.0])

    def _resample_command(self):
        c = self.cfg
        if self.np_random.random() < c.zero_command_prob:
            self.command = np.zeros(3)
        else:
            self.command = self._u((-1, 1), 3) * CMD_MAX
        self.next_command = self.t + self._u(c.command_resample_s)

    # ── gym API ────────────────────────────────────────────────────────────────
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        m, d, c = self.model, self.data, self.cfg
        self._randomize()
        mujoco.mj_resetData(m, d)
        d.qpos[:] = self.home_qpos
        d.qpos[self.qadr] += self._u((-c.init_joint_noise, c.init_joint_noise), m.nu)
        yaw = self._u((-np.pi, np.pi))
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qpos[2] += 0.005
        d.ctrl[:] = self.home_ctrl
        mujoco.mj_forward(m, d)

        self.step_count, self.t = 0, 0.0
        self.cmd = self.home_ctrl.copy()  # what the servos track (after latency and lag)
        self.target = self.home_ctrl.copy()  # last target sent by the policy
        self.last_action = np.zeros(m.nu)
        self.air_time = np.zeros(6)
        self._foot_prev = d.geom_xpos[self.feet][:, :2].copy()
        self.next_push = self._u(c.push_interval_s)
        self._resample_command()
        if options and "command" in options:
            self.command = np.asarray(options["command"], dtype=float)
            self.next_command = np.inf
        self.imu_phase = self.np_random.integers(self.imu_every)
        self.gait_phase = self.np_random.random()
        self._read_imu()
        return self._obs(), {}

    def step(self, action):
        m, d, c = self.model, self.data, self.cfg
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        prev_target = self.target
        self.target = self._to_servo_grid(np.clip(self.home_ctrl + ACTION_SCALE * action, self.ctrl_lo, self.ctrl_hi))
        for i in range(self.substeps):
            goal = prev_target if i < self.latency_steps else self.target
            self.cmd += self.servo_alpha * (goal - self.cmd)
            d.ctrl[:] = self.cmd + self.zero_offset
            mujoco.mj_step(m, d)

        self.step_count += 1
        self.t = self.step_count * self.dt
        self.gait_phase = (self.gait_phase + GAIT_HZ * self.dt) % 1.0
        if self.t >= self.next_push:
            ang = self._u((-np.pi, np.pi))
            d.qvel[0:2] += self._u((0.0, c.push_vel)) * np.array([np.cos(ang), np.sin(ang)])
            self.next_push = self.t + self._u(c.push_interval_s)
        if (self.step_count + self.imu_phase) % self.imu_every == 0:
            self._read_imu()

        fell = self._fell()
        reward, terms = self._reward(action, fell)
        self.last_action = action
        if self.t >= self.next_command:
            self._resample_command()
        truncated = self.step_count >= self.max_steps
        return self._obs(), reward, fell, truncated, terms

    # ── helpers ────────────────────────────────────────────────────────────────
    def _to_servo_grid(self, q):
        """Round joint targets to whole servo degrees (real motor() takes int degrees)."""
        if not self.cfg.quantize_deg:
            return q
        servo = np.round(self.servo_sign * np.degrees(q) + self.servo_off)
        return np.radians((servo - self.servo_off) / self.servo_sign)

    @staticmethod
    def _rot(quat):
        r = np.empty(9)
        mujoco.mju_quat2Mat(r, quat)
        return r.reshape(3, 3)

    def _read_imu(self):
        """Sample the IMU like the real board (25 Hz, noisy, slightly tilted mounting)."""
        d, c = self.data, self.cfg
        g = self._rot(d.sensordata[self.quat_adr:self.quat_adr + 4]).T @ np.array([0.0, 0.0, -1.0])
        g = g + np.cross(self.imu_bias, g) + self.np_random.normal(0, c.gravity_noise, 3)
        self.imu_gravity = g / np.linalg.norm(g)
        self.imu_gyro = d.sensordata[self.gyro_adr:self.gyro_adr + 3] + self.np_random.normal(0, c.gyro_noise, 3)

    def _obs(self):
        clock = 2 * np.pi * self.gait_phase
        tick = [np.sin(clock), np.cos(clock)] if np.any(self.command != 0) else [0.0, 0.0]  # stopped clock = stand
        return np.concatenate([self.imu_gravity, GYRO_SCALE * self.imu_gyro, self.last_action,
                               self.command / CMD_MAX, tick]).astype(np.float32)

    def _fell(self):
        d = self.data
        if self._rot(d.qpos[3:7])[2, 2] < np.cos(np.radians(50)):
            return True
        for i in range(d.ncon):
            g1, g2 = d.contact[i].geom1, d.contact[i].geom2
            if (g1 == self.floor and g2 in self.body_geoms) or (g2 == self.floor and g1 in self.body_geoms):
                return True
        return False

    def _reward(self, action, fell):
        d, w = self.data, self.cfg.w
        R = self._rot(d.qpos[3:7])  # true state: rewards may use privileged information
        v_b = R.T @ d.qvel[0:3]  # base linear velocity, body frame
        w_b = d.qvel[3:6]  # free-joint angular velocity is already in the body frame
        g_b = R.T @ np.array([0.0, 0.0, -1.0])
        moving = np.any(self.command != 0)

        contact = d.sensordata[self.touch_adr] > 1.0  # [N]
        first_contact = contact & (self.air_time > 0)
        air_reward = np.sum((self.air_time - 0.1) * first_contact) if moving else 0.0
        self.air_time = np.where(contact, 0.0, self.air_time + self.dt)
        a_swing = np.sin(2 * np.pi * self.gait_phase) > 0  # first half of the cycle: group A in the air
        expected_contact = np.where(TRIPOD_A, not a_swing, a_swing)
        foot_vel = np.linalg.norm(d.geom_xpos[self.feet][:, :2] - self._foot_prev, axis=1) / self.dt
        self._foot_prev = d.geom_xpos[self.feet][:, :2].copy()

        terms = dict(
            track_lin=np.exp(-np.sum((self.command[:2] - v_b[:2]) ** 2) / 0.02),  # 14 cm/s error -> e^-1
            track_yaw=np.exp(-(self.command[2] - w_b[2]) ** 2 / 0.01),  # 0.1 rad/s error -> e^-1 (strict: no drift)
            upright=np.sum(g_b[:2] ** 2),
            height=(d.qpos[2] - self.home_qpos[2]) ** 2,
            lin_vel_z=v_b[2] ** 2,
            ang_vel_xy=np.sum(w_b[:2] ** 2),
            air_time=air_reward,
            foot_slip=np.sum(foot_vel ** 2 * contact),
            gait=np.mean(contact == expected_contact) if moving else 0.0,
            stand_still=0.0 if moving else np.sum(np.abs(d.qpos[self.qadr] - self.home_ctrl)),
            stand_feet=0.0 if moving else np.mean(contact),
            stand_motion=0.0 if moving else np.sum(d.qvel[self.dadr] ** 2),
            action_rate=np.sum((action - self.last_action) ** 2),
            torque=np.sum(d.actuator_force ** 2),
            fall=float(fell),
        )
        return float(sum(w[k] * v for k, v in terms.items())), terms

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 640)
            self._cam = mujoco.MjvCamera()
            self._cam.distance, self._cam.azimuth, self._cam.elevation = 0.9, 135, -20
        self._cam.lookat[:] = self.data.qpos[:3]
        self._renderer.update_scene(self.data, self._cam)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
