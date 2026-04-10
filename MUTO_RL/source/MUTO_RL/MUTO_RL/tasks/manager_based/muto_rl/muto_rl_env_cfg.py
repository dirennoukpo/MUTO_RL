# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Configuration MDP Push Recovery pour le MUTO-RS hexapode.

Remplace le CartPole du template par le vrai MDP hexapode.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp
from .muto_rs_robot_cfg import MUTO_RS_CFG


# ─── SCENE ────────────────────────────────────────────────────────────────────

@configclass
class MutoRlSceneCfg(InteractiveSceneCfg):
    """Sol plat + hexapode + lumière."""

    # Sol plat avec friction correcte pour les pattes
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # Lumière
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

    # Robot
    robot: ArticulationCfg = MUTO_RS_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


# ─── OBSERVATIONS (70 dims) ───────────────────────────────────────────────────

@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        """Ce que voit la policy — 70 dims."""

        # 4 : quaternion orientation
        base_quat = ObsTerm(
            func=mdp.base_quaternion,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        # 3 : vitesse angulaire
        base_ang_vel = ObsTerm(
            func=mdp.base_angular_velocity,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        # 3 : vitesse linéaire
        base_lin_vel = ObsTerm(
            func=mdp.base_linear_velocity,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        # 3 : gravité projetée
        proj_gravity = ObsTerm(
            func=mdp.projected_gravity,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        # 3 : commandes (zeros Phase 1)
        velocity_cmds = ObsTerm(func=mdp.velocity_commands)
        # 18 : positions joints normalisées
        joint_pos = ObsTerm(
            func=mdp.joint_positions_normalized,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        # 18 : vitesses joints
        joint_vel = ObsTerm(
            func=mdp.joint_velocities,
            params={"asset_cfg": SceneEntityCfg("robot")},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        # 18 : dernières actions
        last_act = ObsTerm(func=mdp.last_actions)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True  # → tenseur (N, 70)

    policy: PolicyCfg = PolicyCfg()


# ─── ACTIONS (18 dims) ────────────────────────────────────────────────────────

@configclass
class ActionsCfg:
    """18 positions de joints normalisées ∈ [-1,1] → dénorm. vers [-1.57, 1.57] rad."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*_Joint"],
        scale=1.57,
        use_default_offset=True,
    )


# ─── REWARDS ──────────────────────────────────────────────────────────────────

@configclass
class RewardsCfg:
    """
    6 termes de récompense pour la Phase 1.

    Tuning si pas de convergence après ~500k steps :
      - Augmenter base_orientation.weight (2.0 → 4.0)
      - Mettre joint_torques.weight à 0.0 pour commencer
      - Augmenter entropy dans skrl_ppo_cfg.yaml (0.005 → 0.02)
    """

    alive = RewTerm(func=mdp.alive, weight=1.0)

    base_orientation = RewTerm(
        func=mdp.base_orientation,
        weight=2.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    base_height = RewTerm(
        func=mdp.base_height,
        weight=1.5,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    base_ang_vel_xy = RewTerm(
        func=mdp.base_angular_velocity_xy,
        weight=0.5,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    action_rate = RewTerm(func=mdp.action_rate, weight=0.1)

    joint_torques = RewTerm(
        func=mdp.joint_torques,
        weight=0.05,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


# ─── TERMINATIONS ─────────────────────────────────────────────────────────────

@configclass
class TerminationsCfg:

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    robot_fell = DoneTerm(
        func=mdp.robot_fell,
        params={"asset_cfg": SceneEntityCfg("robot"), "min_height": 0.04},
    )


# ─── EVENTS ───────────────────────────────────────────────────────────────────

@configclass
class EventCfg:

    # Au reset de chaque épisode
    reset_scene = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset",
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    reset_root = EventTerm(
        func=mdp.reset_root_state_with_noise,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "position_noise": 0.05,
            "orientation_noise": 0.1,
        },
    )

   # randomize_mass = EventTerm(
    #    func=mdp.randomize_robot_mass,
     #   mode="reset",
      #  params={"asset_cfg": SceneEntityCfg("robot"), "mass_range": (0.85, 1.15)},
    #s)

    # Poussées aléatoires pendant l'épisode
    push_robot = EventTerm(
        func=mdp.push_robot,
        mode="interval",
        interval_range_s=(2.0, 4.0),
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "velocity_range": {"x": (-0.2, 0.2), "y": (-0.2, 0.2)},
        },
    )


# ─── ENV CFG PRINCIPAL ────────────────────────────────────────────────────────

@configclass
class MutoRlEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration complète Phase 1 — Push Recovery."""

    scene: MutoRlSceneCfg = MutoRlSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self) -> None:
        self.decimation = 4
        self.episode_length_s = 10.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        # Solver TGS — plus stable que PGS pour les robots articulés
        self.sim.physx.solver_type = 1
        # GPU physique
        self.sim.physx.gpu_max_rigid_contact_count = 524288
        self.sim.physx.gpu_max_rigid_patch_count = 163840
        self.sim.physx.gpu_found_lost_pairs_capacity = 4096
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 65536
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 4096

@configclass
class MutoRlEnvCfg_PLAY(MutoRlEnvCfg):
    """Variante play.py : 32 envs, pas de bruit, pas de push."""

    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 32
        self.scene.env_spacing = 3.0
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None       # pas de poussées pour la visu
        self.events.randomize_mass = None   # masse fixe pour la visu
