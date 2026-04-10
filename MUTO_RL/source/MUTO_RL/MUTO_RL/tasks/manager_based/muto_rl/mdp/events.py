# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""
Events pour MUTO-RS Push Recovery.

  reset_scene_to_default     : reset joints + root state au début de chaque épisode
  reset_root_state_with_noise: spawn avec légère perturbation d'orientation
  push_robot                 : impulsion aléatoire toutes les 2-4s
  randomize_robot_mass       : domain randomization masse ±15%
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def reset_scene_to_default(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
) -> None:
    """Reset joints à DEFAULT_JOINT_POS + petit bruit aléatoire."""
    asset: Articulation = env.scene[asset_cfg.name]
    asset.reset(env_ids)
    joint_pos = asset.data.default_joint_pos[env_ids].clone()
    joint_vel = asset.data.default_joint_vel[env_ids].clone()
    joint_pos += torch.randn_like(joint_pos) * 0.05  # ±0.05 rad
    joint_pos = joint_pos.clamp(-1.57, 1.57)
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)


def reset_root_state_with_noise(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    position_noise: float = 0.05,
    orientation_noise: float = 0.1,
) -> None:
    """Reset root state avec bruit en position XY et orientation."""
    asset: Articulation = env.scene[asset_cfg.name]
    default_pos = asset.data.default_root_state[env_ids, :3].clone()
    default_quat = asset.data.default_root_state[env_ids, 3:7].clone()

    # Bruit position XY
    default_pos[:, :2] += torch.randn(len(env_ids), 2, device=env.device) * position_noise

    # Bruit orientation (roll/pitch)
    noise_rpy = torch.zeros(len(env_ids), 3, device=env.device)
    noise_rpy[:, :2] = torch.randn(len(env_ids), 2, device=env.device) * orientation_noise
    noise_quat = math_utils.quat_from_euler_xyz(noise_rpy[:, 0], noise_rpy[:, 1], noise_rpy[:, 2])
    noisy_quat = math_utils.normalize(math_utils.quat_mul(default_quat, noise_quat))

    zero_vel = torch.zeros(len(env_ids), 6, device=env.device)
    root_state = torch.cat([default_pos, noisy_quat, zero_vel], dim=1)
    asset.write_root_state_to_sim(root_state, env_ids=env_ids)


def push_robot(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    velocity_range: dict,
) -> None:
    """Applique une impulsion de vitesse aléatoire sur le base_link.

    velocity_range exemple : {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}
    """
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_lin_vel_w[env_ids].clone()

    x_range = velocity_range.get("x", (-0.5, 0.5))
    y_range = velocity_range.get("y", (-0.5, 0.5))

    lin_vel[:, 0] += torch.FloatTensor(len(env_ids)).uniform_(*x_range).to(env.device)
    lin_vel[:, 1] += torch.FloatTensor(len(env_ids)).uniform_(*y_range).to(env.device)
    lin_vel = lin_vel.clamp(-2.0, 2.0)

    ang_vel = asset.data.root_ang_vel_w[env_ids].clone()
    root_vel = torch.cat([lin_vel, ang_vel], dim=1)
    asset.write_root_velocity_to_sim(root_vel, env_ids=env_ids)

def randomize_robot_mass(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    mass_range: tuple = (0.85, 1.15),
) -> None:
    asset: Articulation = env.scene[asset_cfg.name]
    # default_mass est sur CPU — forcer env_ids sur CPU pour l'indexing
    env_ids_cpu = env_ids.cpu()
    default_mass = asset.data.default_mass[env_ids_cpu, 0].clone()
    factor = torch.FloatTensor(len(env_ids)).uniform_(*mass_range).to(env.device)
    new_mass = (default_mass.to(env.device) * factor).unsqueeze(1)
    asset.write_body_mass_to_sim(new_mass, body_ids=[0], env_ids=env_ids)