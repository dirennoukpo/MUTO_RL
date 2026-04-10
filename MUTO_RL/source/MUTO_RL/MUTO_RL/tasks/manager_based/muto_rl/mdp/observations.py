# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Observations pour MUTO-RS Push Recovery — 70 dimensions.

  base_quat          4   orientation absolue (world frame)
  base_ang_vel       3   vitesse angulaire (frame robot)
  base_lin_vel       3   vitesse linéaire  (frame robot)
  projected_gravity  3   vecteur gravité projeté — proxy roll/pitch
  velocity_commands  3   commandes (zeros Phase 1)
  joint_pos         18   positions normalisées ÷ 1.57
  joint_vel         18   vitesses des joints
  last_actions      18   dernières actions
                   ---
                    70
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def base_quaternion(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Quaternion [w,x,y,z] du base_link en world frame. Shape: (N, 4)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_quat_w


def base_angular_velocity(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Vitesse angulaire en frame robot. Shape: (N, 3)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_ang_vel_b


def base_linear_velocity(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Vitesse linéaire en frame robot. Shape: (N, 3)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_lin_vel_b


def projected_gravity(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Vecteur gravité normalisé projeté dans le frame robot.
    Vaut (0, 0, -1) quand le robot est parfaitement vertical.
    Shape: (N, 3).
    """
    asset: Articulation = env.scene[asset_cfg.name]
    gravity_w = torch.tensor([0.0, 0.0, -1.0], device=env.device).expand(env.num_envs, -1)
    quat = asset.data.root_quat_w
    return math_utils.quat_rotate_inverse(quat, gravity_w)


def velocity_commands(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Commandes [vx, vy, omega_z] — zeros en Phase 1. Shape: (N, 3)."""
    return torch.zeros(env.num_envs, 3, device=env.device)


def joint_positions_normalized(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Positions des 18 joints normalisées par 1.57 → [-1, 1]. Shape: (N, 18)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.joint_pos / 1.57


def joint_velocities(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Vitesses des 18 joints. Shape: (N, 18)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.joint_vel


def last_actions(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Dernières actions envoyées. Shape: (N, 18)."""
    return env.action_manager.action
