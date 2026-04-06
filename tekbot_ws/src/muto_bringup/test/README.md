# Suite de tests terrain

Cette suite est organisee par phase de mise en service.

## Execution

Chaque script est autonome :

python3 <script>.py --timeout 30

Sortie standard : PASS / FAIL / WARN coloree.
Code retour : 0 si aucun FAIL, sinon 1.

## Phases

- phase1_hardware_pi : validation hardware/surete Pi
- phase2_control_jetson : observation 70D et safety filter
- phase3_perception : performance perception sans RL
- phase4_ai : pipeline RL et dry-run

## Checklist

Utiliser checklist_terrain.md avant toute session reels servos.
