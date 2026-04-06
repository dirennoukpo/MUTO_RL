# Checklist terrain - MUTO RS
# A completer avant chaque session avec servos reels

## Avant allumage

- [ ] `colcon build --symlink-install` reussi sans warning
- [ ] `libmuto_link_cpp_lib.so` presente dans `/tmp/muto_install/lib/`
- [ ] `nm -D libmuto_link_cpp_lib.so | grep muto_read` confirme `muto_read_servo_angle_deg`
- [ ] `isolcpus=3` present dans `/boot/cmdline.txt` sur le Pi
- [ ] `chrony` actif sur Pi et Jetson, derive < 1 ms confirmee
- [ ] `tegrastats` confirme RAM disponible > 15% avant lancement
- [ ] `model_card_v003.json` charge et flags verifies

## Phase 1 - Pi seul

- [ ] `python3 test_usb_bridge_topics.py` -> tous PASS
- [ ] `python3 test_watchdog_timeouts.py` -> tous PASS
- [ ] `python3 test_mode_transitions.py` -> tous PASS
- [ ] `ros2 topic hz /imu/data` confirme 200 Hz +/- 5%
- [ ] `ros2 topic hz /joint_states` confirme 200 Hz +/- 5%
- [ ] Aucun WARN dans les logs pendant 60 secondes

## Phase 2 - Controle Jetson sans IA

- [ ] `python3 test_obs_builder_vector.py` -> tous PASS
- [ ] `python3 test_safety_filter.py` -> tous PASS
- [ ] CSV vecteur observation 70 valeurs logge et inspecte visuellement
- [ ] `/sync/missed_cycles` < 1% apres 10 minutes
- [ ] `validate_sim_real.py` phase 1 (KS) -> tous PASS, `sim_real_validated = true`

## Phase 3 - Perception sans IA

- [ ] `python3 test_perception_no_rl.py` -> tous PASS
- [ ] RViz2 : nuage de points filtre visible et coherent
- [ ] RViz2 : carte SLAM coherente apres 2 minutes de deplacement manuel
- [ ] RAM GPU < 85% avec TensorRT inactif + perception active
- [ ] Ventilateur 5V actif, temperature GPU < 65C apres 10 minutes

## Phase 4 - IA en dry-run

- [ ] `python3 test_rl_pipeline_hz.py` -> tous PASS (10 minutes)
- [ ] `python3 test_dry_run_commands.py` -> tous PASS
- [ ] `/commands_dry_run` inspecte : aucune valeur aberrante sur 5 minutes
- [ ] `validate_sim_real.py` phase 2 (cross-correlation) -> tous PASS, `sim_real_cross_corr_validated = true`
- [ ] Robot sureleve, servos en l'air, dry-run actif 5 minutes -> aucun EMERGENCY

## Avant passage en RL_ACTIVE

- [ ] Les 3 flags `model_card_v003.json` sont `true`
- [ ] `/mode_request` RL_ACTIVE retourne `success=true`
- [ ] Operateur present physiquement, coupure d'urgence accessible
- [ ] `ulimit -v` configure sur Jetson
- [ ] Watchdog memoire actif dans `diagnostics_node`
