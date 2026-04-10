# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Conditions de terminaison pour MUTO-RS Push Recovery."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def time_out(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Timeout : épisode terminé si durée > episode_length_s. Shape: (N,) bool."""
    return env.episode_length_buf >= env.max_episode_length


def robot_fell(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, min_height: float = 0.04) -> torch.Tensor:
    """Terminé si le base_link descend sous min_height (robot tombé).

    min_height = 0.04 m est bien en dessous de TARGET_HEIGHT=0.113 m.
    Préférer cette approche au ContactSensor pour éviter les complications d'import USD.
    Shape: (N,) bool.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    height = asset.data.root_pos_w[:, 2]
    return height < min_height
