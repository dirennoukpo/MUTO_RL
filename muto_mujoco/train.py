"""Train the MUTO walking policy with PPO (Stable-Baselines3, CPU).

    uv run python train.py                      # new run
    uv run python train.py --steps 2e6 --envs 6
    uv run tensorboard --logdir runs             # watch training at http://localhost:6006
"""

import argparse
import datetime
import os
import pathlib

# one math thread per process: 6 env processes on a 6-core laptop CPU scale much better
for var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(var, "1")

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize

from muto_env.env import MutoEnv

RUNS = pathlib.Path(__file__).parent / "runs"


class RewardTermsCallback(BaseCallback):
    """Log the mean of each reward term (from info) to TensorBoard."""

    def __init__(self):
        super().__init__()
        self.sums, self.n = {}, 0

    def _on_step(self):
        for info in self.locals["infos"]:
            for k, v in info.items():
                if isinstance(v, float | int | np.floating) and not k.startswith(("episode", "TimeLimit")):
                    self.sums[k] = self.sums.get(k, 0.0) + float(v)
            self.n += 1
        return True

    def _on_rollout_end(self):
        for k, v in self.sums.items():
            self.logger.record(f"reward_terms/{k}", v / max(self.n, 1))
        self.sums, self.n = {}, 0


def make_env(rank, seed):
    return lambda: MutoEnv(seed=seed + rank)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=float, default=5e6)
    p.add_argument("--envs", type=int, default=6)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--name", default="walk")
    args = p.parse_args()

    torch.set_num_threads(2)
    run = RUNS / f"{datetime.datetime.now():%Y-%m-%d_%H-%M}_{args.name}"
    run.mkdir(parents=True)

    venv = SubprocVecEnv([make_env(i, args.seed) for i in range(args.envs)], start_method="fork")
    venv = VecMonitor(venv)
    # rewards are normalized for learning; observations are NOT (the robot would need the stats)
    venv = VecNormalize(venv, norm_obs=False, norm_reward=True, gamma=0.99)

    model = PPO(
        "MlpPolicy", venv, seed=args.seed, verbose=1, tensorboard_log=str(run),
        n_steps=512, batch_size=1024, n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
        clip_range=0.2, ent_coef=0.0, max_grad_norm=1.0, target_kl=0.02,
        policy_kwargs=dict(net_arch=dict(pi=[256, 128, 64], vf=[256, 128, 64]),
                           activation_fn=torch.nn.ELU, log_std_init=-1.0),
    )
    print(f"run dir: {run}")
    model.learn(
        total_timesteps=int(args.steps),
        callback=[RewardTermsCallback(),
                  CheckpointCallback(save_freq=max(200_000 // args.envs, 1), save_path=str(run / "checkpoints"),
                                     save_vecnormalize=True)],
    )
    model.save(run / "final")
    venv.save(str(run / "vecnormalize.pkl"))
    print(f"saved {run / 'final.zip'}")


if __name__ == "__main__":
    main()
