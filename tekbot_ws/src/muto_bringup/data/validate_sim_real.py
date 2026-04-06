"""
Documentation FR: src/muto_bringup/data/validate_sim_real.py
Role: module du pipeline MUTO RS (robotique temps reel ROS2).
Details: ce fichier participe a la chaine capteurs->observation->decision->commande.
Contraintes: garder la coherence QoS, les unites SI et la robustesse aux timeouts.
Maintenance: toute evolution doit conserver la compatibilite des topics, services et paramètres.
"""

#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path
from typing import List

import numpy as np
from scipy.stats import ks_2samp


def read_csv_matrix(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    return np.asarray([[float(x) for x in row] for row in rows], dtype=np.float64)


def validate_phase_1(sim: np.ndarray, real: np.ndarray) -> tuple[bool, List[int]]:
    if sim.shape[1] != 70 or real.shape[1] != 70:
        raise ValueError("Les deux CSV doivent avoir exactement 70 colonnes")

    all_ok = True
    failed_idx: List[int] = []
    for i in range(70):
        _, p = ks_2samp(sim[:, i], real[:, i], mode="auto")
        if p <= 0.05:
            print(f"ECHEC KS index={i} p={p:.6f} (seuil > 0.05)")
            all_ok = False
            failed_idx.append(i)
    return all_ok, failed_idx


def validate_phase_2(commanded: np.ndarray, measured: np.ndarray, model_card: dict) -> bool:
    if commanded.shape[1] != 18 or measured.shape[1] != 18:
        raise ValueError("Les CSV commande/mesure doivent avoir exactement 18 colonnes")

    win = model_card.get("domain_rand", {}).get("latency_action_ms", [0, 10])
    min_ms = float(win[0])
    max_ms = float(win[1])

    all_ok = True
    for i in range(18):
        a = commanded[:, i] - np.mean(commanded[:, i])
        b = measured[:, i] - np.mean(measured[:, i])
        corr = np.correlate(b, a, mode="full")
        lag_idx = int(np.argmax(corr) - (len(a) - 1))
        lag_ms = lag_idx * 10.0
        valid = min_ms <= lag_ms <= max_ms
        print(f"servo={i} lag_ms={lag_ms:.2f} window=[{min_ms:.2f},{max_ms:.2f}] valid={valid}")
        if not valid:
            all_ok = False
    return all_ok


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", type=int, choices=[1, 2], required=True)
    parser.add_argument("--model-card", type=Path, required=True)
    parser.add_argument("--sim-csv", type=Path)
    parser.add_argument("--real-csv", type=Path)
    parser.add_argument("--commanded-csv", type=Path)
    parser.add_argument("--measured-csv", type=Path)
    args = parser.parse_args()

    model_card = json.loads(args.model_card.read_text(encoding="utf-8"))

    if args.phase == 1:
        if args.sim_csv is None or args.real_csv is None:
            raise ValueError("--sim-csv et --real-csv sont requis pour phase 1")
        sim = read_csv_matrix(args.sim_csv)
        real = read_csv_matrix(args.real_csv)
        ok, failed_idx = validate_phase_1(sim, real)
        model_card["sim_real_validated"] = ok
        model_card["ks_failed_indices"] = failed_idx
    else:
        if args.commanded_csv is None or args.measured_csv is None:
            raise ValueError("--commanded-csv et --measured-csv sont requis pour phase 2")
        cmd = read_csv_matrix(args.commanded_csv)
        meas = read_csv_matrix(args.measured_csv)
        ok = validate_phase_2(cmd, meas, model_card)
        model_card["sim_real_cross_corr_validated"] = ok

    model_card["dry_run_validated"] = bool(
        model_card.get("sim_real_validated", False)
        and model_card.get("sim_real_cross_corr_validated", False)
    )

    args.model_card.write_text(json.dumps(model_card, indent=2), encoding="utf-8")
    print(f"phase={args.phase} success={ok}")


if __name__ == "__main__":
    main()
