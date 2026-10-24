"""Compare the clock / no-clock ablation runs: same tests on every run, then mean ± std per arm.

    uv run python tools/ablate_eval.py                 # all runs/*ablate_* with a final.zip
    uv run python tools/ablate_eval.py --robots 3      # quicker

Each policy is tested on --robots randomized robots (same robots for every run), no pushes,
7 fixed commands, 8 s each, measured over the last 6 s. No-clock runs are tested with the clock off,
as they were trained. Learning speed comes from the TensorBoard logs (track_lin vs steps).
"""
import argparse
import json
import pathlib
import sys
from multiprocessing import Pool

import numpy as np
from stable_baselines3 import PPO
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from muto_env.env import TRIPOD_A, EnvConfig, MutoEnv  # noqa: E402

COMMANDS = {"stand": (0, 0, 0), "forward": (0.15, 0, 0), "slow fwd": (0.07, 0, 0), "backward": (-0.15, 0, 0),
            "left": (0, 0.1, 0), "turn left": (0, 0, 0.6), "turn right": (0, 0, -0.6)}
MOVING = [k for k in COMMANDS if k != "stand"]


def test_run(job):
    """All metrics for one run, averaged over the robots."""
    run, n_robots = job
    use_clock = "noclock" not in run.name
    policy = PPO.load(run / "final.zip", device="cpu")
    env = MutoEnv(EnvConfig(push_vel=0.0, use_clock=use_clock), seed=0)
    out = {}
    for name, cmd in COMMANDS.items():
        rows = []
        for robot in range(n_robots):
            obs, _ = env.reset(seed=100 + robot, options={"command": cmd})
            v, slip, torque, joint_speed, tripod, all_down, touchdowns, falls = [], [], [], [], [], [], 0, 0
            prev = None
            for k in range(400):
                obs, _, term, _, info = env.step(policy.predict(obs, deterministic=True)[0])
                falls += term
                d = env.data
                contact = d.sensordata[env.touch_adr] > 0.1  # [N] a lightly loaded foot still counts as down
                if k >= 100:
                    v.append([*(env._rot(d.qpos[3:7]).T @ d.qvel[:3])[:2], d.qvel[5]])
                    slip.append(info["foot_slip"])
                    torque.append(info["torque"])
                    joint_speed.append(np.mean(np.abs(d.qvel[env.dadr])))
                    a, b = contact[TRIPOD_A], contact[~TRIPOD_A]
                    # tripod: each group moves as one, and the two groups alternate
                    tripod.append((a.all() and not b.any()) or (b.all() and not a.any()))
                    all_down.append(contact.all())
                    if prev is not None:
                        touchdowns += np.sum(contact & ~prev)
                prev = contact
                if term:
                    break
            v = np.mean(v, axis=0) if v else np.full(3, np.nan)
            rows.append(dict(
                lin_err=float(np.hypot(*(v[:2] - cmd[:2]))),
                yaw_err=float(abs(v[2] - cmd[2])),
                slip=float(np.mean(slip)), torque=float(np.mean(torque)),
                joint_speed_deg=float(np.degrees(np.mean(joint_speed))),
                tripod=float(np.mean(tripod)), all_down=float(np.mean(all_down)),
                steps_hz=touchdowns / 6 / 6, falls=falls))
        out[name] = {k: float(np.mean([r[k] for r in rows])) for k in rows[0]}
    return run.name, out


def learning_curve(run, threshold=0.8):  # 0.6 is reached by standing still on every command
    """track_lin vs steps from TensorBoard, and the first step where it passes the threshold."""
    ea = EventAccumulator(str(next(run.glob("PPO_*"))))
    ea.Reload()
    ev = ea.Scalars("reward_terms/track_lin")
    steps, vals = np.array([e.step for e in ev]), np.array([e.value for e in ev])
    smooth = np.convolve(vals, np.ones(5) / 5, mode="same")
    reached = steps[np.argmax(smooth > threshold)] if np.any(smooth > threshold) else None
    return dict(final_track_lin=float(vals[-10:].mean()), steps_to_threshold=None if reached is None else int(reached))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", type=int, default=10)
    ap.add_argument("--out", default=str(ROOT / "runs" / "ablation_results.json"))
    args = ap.parse_args()

    runs = sorted(r for r in (ROOT / "runs").glob("*ablate_*") if (r / "final.zip").exists())
    with Pool(len(runs)) as pool:
        results = dict(pool.map(test_run, [(r, args.robots) for r in runs]))
    for r in runs:
        results[r.name]["learning"] = learning_curve(r)

    arms = {"clock": [n for n in results if "noclock" not in n], "no clock": [n for n in results if "noclock" in n]}

    def stat(names, f):
        x = np.array([f(results[n]) for n in names])
        return f"{x.mean():.3f} ± {x.std(ddof=1) if len(x) > 1 else 0:.3f}"

    metrics = [
        ("speed error, moving cmds [m/s]", lambda r: np.mean([r[c]["lin_err"] for c in MOVING if c not in ("turn left", "turn right")])),
        ("turn rate error [rad/s]", lambda r: np.mean([r[c]["yaw_err"] for c in ("turn left", "turn right")])),
        ("standing: joint motion [deg/s]", lambda r: r["stand"]["joint_speed_deg"]),
        ("standing: body speed [m/s]", lambda r: r["stand"]["lin_err"]),
        ("foot slip (moving)", lambda r: np.mean([r[c]["slip"] for c in MOVING])),
        ("effort: sum torque^2 (moving)", lambda r: np.mean([r[c]["torque"] for c in MOVING])),
        ("tripod pattern, fraction of time", lambda r: np.mean([r[c]["tripod"] for c in MOVING])),
        ("all 6 feet down, fraction of time", lambda r: np.mean([r[c]["all_down"] for c in MOVING])),
        ("touchdowns per leg per s", lambda r: np.mean([r[c]["steps_hz"] for c in MOVING])),
        ("falls (total)", lambda r: sum(r[c]["falls"] for c in COMMANDS)),
        ("final training track_lin", lambda r: r["learning"]["final_track_lin"]),
        ("steps until track_lin > 0.8 [M]", lambda r: (r["learning"]["steps_to_threshold"] or np.nan) / 1e6),
    ]
    print(f"\n{len(arms['clock'])} clock runs vs {len(arms['no clock'])} no-clock runs, "
          f"{args.robots} randomized robots each, mean ± std over seeds\n")
    print(f"{'metric':36s} {'clock':>18s} {'no clock':>18s}")
    for label, f in metrics:
        print(f"{label:36s} {stat(arms['clock'], f):>18s} {stat(arms['no clock'], f):>18s}")

    print("\nper run:")
    for label, f in metrics[:3] + metrics[6:7]:
        print(f"  {label}: " + ", ".join(f"{n.split('ablate_')[1]} {f(results[n]):.3f}" for n in results))
    pathlib.Path(args.out).write_text(json.dumps(results, indent=1))
    print(f"\nraw numbers: {args.out}")


if __name__ == "__main__":
    main()
