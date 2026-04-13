# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
ArticulationCfg pour le hexapode MUTO-RS (yahboomcar).

USD_PATH : résultat de l'import URDF via l'outil officiel NVIDIA.
À modifier si tu déplaces le fichier USD.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

# ─── CHEMIN USD ───────────────────────────────────────────────────────────────
# Généré par l'import URDF officiel NVIDIA
USD_PATH = "/home/scop/MUTO_RL/assets/yahboomcar_description/urdf/MUTO_fixed/MUTO_fixed.usd"

# Hauteur sol → base_link quand le robot est debout.
# Extrait du URDF : base_footprint à -0.113m sous base_link.
TARGET_HEIGHT = 0.113  # [m]

# Ordre des 18 joints — vérifie dans Isaac Sim : Window > Stage
# (doit correspondre à observation_spec_v3.py d'Edwin)
JOINT_NAMES = [
    "zq1_Joint", "zq2_Joint", "zq3_Joint",   # patte avant droite
    "zz1_Joint", "zz2_Joint", "zz3_Joint",   # patte centrale droite
    "zh1_Joint", "zh2_Joint", "zh3_Joint",   # patte arrière droite
    "yq1_Joint", "yq2_Joint", "yq3_Joint",   # patte avant gauche
    "yz1_Joint", "yz2_Joint", "yz3_Joint",   # patte centrale gauche
    "yh1_Joint", "yh2_Joint", "yh3_Joint",   # patte arrière gauche
]

MUTO_RS_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,  #was False, kinda counter-intuitive for me
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, TARGET_HEIGHT + 0.02),
        joint_pos={name: 0.0 for name in JOINT_NAMES},
        joint_vel={name: 0.0 for name in JOINT_NAMES},
    ),
    actuators={
        # LX-224 modélisés comme actuateurs implicites (PD interne PhysX)
        # Kp/Kd conservatives pour démarrer — tuner si oscillations
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_Joint"],
            effort_limit=1.5,    # du URDF
            velocity_limit=6.28, # du URDF
            stiffness=  5.0     ,   # Kp
            damping=  0.2   ,      # Kd
        ),
    },
    soft_joint_pos_limit_factor=0.9,
)
