# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Rewards pour MUTO-RS Push Recovery.

Toutes les fonctions retournent shape (N,) — une valeur par env.

Stratégie de tuning :
  1. Commencer avec alive + base_orientation seulement
  2. Ajouter base_height quand l'orientation est stable
  3. Ajouter action_rate + joint_torques pour lisser
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

_TARGET_HEIGHT = 0.113  # [m] — doit correspondre à muto_rs_robot_cfg.TARGET_HEIGHT


def alive(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Récompense constante à chaque step : encourage la survie."""
    return torch.ones(env.num_envs, device=env.device)


def base_orientation(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Pénalise la déviation par rapport à la verticale.
    gravity_b ≈ (0,0,-1) quand debout → composantes XY doivent être nulles.
    Retourne valeur dans (-inf, 0].
    """
    asset: Articulation = env.scene[asset_cfg.name]
    gravity_w = torch.tensor([0.0, 0.0, -1.0], device=env.device).expand(env.num_envs, -1)
    gravity_b = math_utils.quat_rotate_inverse(asset.data.root_quat_w, gravity_w)
    return -torch.sum(torch.square(gravity_b[:, :2]), dim=1)


def base_height(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Pénalise la déviation par rapport à TARGET_HEIGHT."""
    asset: Articulation = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return -torch.square(height - _TARGET_HEIGHT)


def base_angular_velocity_xy(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Pénalise la vitesse angulaire en X/Y (tangage/roulis)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return -torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)


def action_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Pénalise les variations brusques entre actions successives.
    Protège les servos LX-224 et favorise les mouvements fluides.
    """
    curr = env.action_manager.action
    prev = env.action_manager.prev_action
    return -torch.sum(torch.square(curr - prev), dim=1)


def joint_torques(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Pénalise les couples articulaires élevés — économie d'énergie."""
    asset: Articulation = env.scene[asset_cfg.name]
    return -torch.sum(torch.square(asset.data.applied_torque), dim=1)
