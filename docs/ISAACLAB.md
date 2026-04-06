Le workspace est complet et validé. La suite logique est l'**entraînement Isaac Lab** et le **sim-to-real**. Voici le prompt :

---

**CONTEXTE — lis tout avant d'écrire une ligne**

```bash
cat /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/muto_rs_architecture_finale2.md
cat /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/tekbot_ws/src/muto_bringup/data/model_card_v003.json
cat /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/tekbot_ws/src/muto_bringup/data/validate_sim_real.py
cat /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/tekbot_ws/src/muto_inference/muto_inference/rl_policy_node.py
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "*.py" | grep -i isaac | sort
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "observation_spec*" | sort
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "norm_stats*" | sort
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "*.urdf" -o -name "*.xacro" | sort
ls /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/
```

---

**OBJECTIF**

Créer l'intégralité du pipeline d'entraînement Isaac Lab pour Muto RS, conforme à la spec v3 du fichier d'architecture. Tout va dans :

```
/home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/
└── isaac_lab_training/
    ├── envs/
    │   ├── muto_rs_env.py           # environnement Isaac Lab complet
    │   └── observation_spec_v3.py   # contrat obs/action versionné
    ├── tasks/
    │   └── locomotion_task.py       # reward v1 + domain randomization v3
    ├── training/
    │   ├── train.py                 # script d'entraînement principal
    │   └── config_ppo.yaml          # hyperparamètres PPO
    ├── export/
    │   ├── export_tensorrt.py       # export du modèle entraîné → TensorRT FP16
    │   └── generate_norm_stats.py   # calcul norm_stats_v3.json depuis rollouts
    └── tools/
        ├── record_real_obs.py       # enregistre observations réelles depuis ROS 2
        └── run_validate_sim_real.py # lance validate_sim_real.py avec les données enregistrées
```

---

**`observation_spec_v3.py` — contrat versionné, NE PAS MODIFIER sans incrémenter**

```python
# observation_spec_v3.py
# Version : v3 — 70 dimensions — NE PAS MODIFIER sans incrémenter OBS_SPEC_VERSION

OBS_SPEC_VERSION = "v3"
OBS_DIM = 70
ACTION_DIM = 18

# Slices par groupe — utiliser UNIQUEMENT ces constantes dans tout le code
QUATERNION       = slice(0,  4)   # w, x, y, z
ANGULAR_VEL      = slice(4,  7)   # roll_rate, pitch_rate, yaw_rate (rad/s)
LINEAR_ACCEL     = slice(7,  10)  # ax, ay, az (m/s²)
JOINT_ANGLES     = slice(10, 28)  # 18 joints en radians (ordre URDF strict)
JOINT_VELOCITIES = slice(28, 46)  # 18 joints en rad/s
LAST_ACTION      = slice(46, 64)  # 18 dernières actions envoyées (rad)
GOAL_VELOCITY    = slice(64, 67)  # vx, vy, yaw_rate normalisé
CONTACT_FORCES   = slice(67, 70)  # 6 pattes, scalaire normalisé [0, 1]

# Limites angulaires joints (ordre URDF strict, radians)
# Plage hardware : ±90° = ±π/2
JOINT_LIMITS_MIN = [-1.5708] * 18
JOINT_LIMITS_MAX = [ 1.5708] * 18

# Scale action → radians (clipping [-1, 1] × scale)
ACTION_SCALE_RAD = 0.5  # à calibrer empiriquement

def validate_obs_dim(obs_tensor):
    assert obs_tensor.shape[-1] == OBS_DIM, \
        f"Dimension obs incorrecte : {obs_tensor.shape[-1]} != {OBS_DIM}"

def validate_spec_version(model_card: dict):
    assert model_card.get("obs_spec_version") == OBS_SPEC_VERSION, \
        f"Version spec incorrecte : {model_card.get('obs_spec_version')} != {OBS_SPEC_VERSION}"
```

---

**`muto_rs_env.py` — environnement Isaac Lab complet**

Implémenter une classe `MutoRsEnv` héritant de `DirectRLEnv` (Isaac Lab API) avec :

**Observation space :**
```python
# 70 valeurs exactes selon observation_spec_v3.py
# Construit dans _get_observations() :

def _get_observations(self):
    obs = torch.zeros(self.num_envs, OBS_DIM, device=self.device)

    # 00-03 : quaternion corps
    obs[:, QUATERNION] = self.robot.data.root_quat_w  # (w, x, y, z)

    # 04-06 : vitesse angulaire corps (rad/s)
    obs[:, ANGULAR_VEL] = self.robot.data.root_ang_vel_b

    # 07-09 : accélération linéaire corps (m/s²)
    obs[:, LINEAR_ACCEL] = self.robot.data.root_lin_acc_b

    # 10-27 : angles 18 joints (rad)
    obs[:, JOINT_ANGLES] = self.robot.data.joint_pos

    # 28-45 : vitesses 18 joints (rad/s)
    obs[:, JOINT_VELOCITIES] = self.robot.data.joint_vel

    # 46-63 : dernière action envoyée
    obs[:, LAST_ACTION] = self.last_action

    # 64-66 : goal velocity normalisé
    obs[:, GOAL_VELOCITY] = self.goal_velocity

    # 67-69 : contact forces normalisées [0, 1] par patte (6 pattes)
    obs[:, CONTACT_FORCES] = self._compute_contact_forces()

    return obs
```

**Action space :**
```python
# 18 valeurs dans [-1, 1], scalées par ACTION_SCALE_RAD avant envoi aux joints
def _apply_action(self, action):
    clipped = torch.clamp(action, -1.0, 1.0)  # AVANT scaling — règle absolue
    target_angles = clipped * ACTION_SCALE_RAD
    self.robot.set_joint_position_target(target_angles)
    self.last_action = clipped * ACTION_SCALE_RAD
```

**Terminaison :**
```python
def _get_dones(self):
    # Chute : hauteur corps < 15 cm
    height = self.robot.data.root_pos_w[:, 2]
    fell = height < 0.15
    # Timeout épisode
    timed_out = self.episode_length_buf >= self.max_episode_length
    return fell | timed_out, timed_out
```

---

**`locomotion_task.py` — reward v1 + domain randomization v3**

**Reward function v1 — exactement comme dans l'architecture :**

```python
def compute_reward(self, obs, action, prev_action, contact_forces):
    # Terme 1 : suivi vitesse cible (dominant)
    goal_vel  = obs[:, GOAL_VELOCITY]
    actual_vel = self.robot.data.root_lin_vel_b[:, :2]  # vx, vy
    yaw_rate   = self.robot.data.root_ang_vel_b[:, 2:3]
    actual_vel_full = torch.cat([actual_vel, yaw_rate], dim=-1)
    r_velocity = -torch.norm(goal_vel - actual_vel_full, dim=-1)

    # Terme 2 : stabilité angulaire
    angular_vel = obs[:, ANGULAR_VEL]
    r_stability = -0.3 * torch.norm(angular_vel[:, :2], dim=-1)

    # Terme 3 : efficacité énergétique
    r_energy = -0.01 * torch.sum(action ** 2, dim=-1)

    # Terme 4 : douceur transitions
    r_smooth = -0.05 * torch.sum((action - prev_action) ** 2, dim=-1)

    # Terme 5 : contact au sol
    r_contact = 0.1 * torch.sum(contact_forces > 0.5, dim=-1).float()

    # Terme 6 : pénalité chute (terminale)
    height = self.robot.data.root_pos_w[:, 2]
    r_fall = -10.0 * (height < 0.15).float()

    return r_velocity + r_stability + r_energy + r_smooth + r_contact + r_fall
```

**Domain randomization v3 — exactement comme dans l'architecture :**

```python
# Randomization temporelle — reset à chaque épisode
obs_delay_ms    = Uniform(0.0, 10.0)   # délai observation
action_delay_ms = Uniform(0.0, 10.0)   # délai action
servo_jitter_ms = Uniform(-2.0, 2.0)   # par joint, indépendant

# Simulation pertes de paquets — 2.5% par step
PACKET_LOSS_RATE  = 0.025
BURST_RATE        = 0.005   # 5% des pertes → burst
BURST_LEN_MIN     = 3
BURST_LEN_MAX     = 8

def apply_packet_loss(self, obs):
    # Distribution interne des pertes :
    # 65% → répéter dernière obs (retard DDS)
    # 20% → bruit gaussien std=0.3
    # 10% → zéros (dropout complet)
    # 5%  → burst de 3 à 8 pertes consécutives
    ...

# Randomization physique — reset à chaque épisode
mass_variation     = Uniform(0.8, 1.2)    # ×masse nominale par segment
friction           = Uniform(0.4, 1.2)    # friction pattes
stiffness_mult     = Uniform(0.7, 1.3)    # raideur joints
damping_mult       = Uniform(0.7, 1.3)    # amortissement joints
terrain_height_std = 0.01                 # ±1 cm bruit terrain
imu_bias_gyro      = Uniform(-0.05, 0.05) # rad/s par épisode
```

---

**`export_tensorrt.py` — export modèle → TensorRT FP16**

```python
# Prend en entrée :
# - le checkpoint PyTorch entraîné (.pt)
# - model_card_v003.json
# Produit :
# - muto_rs_v003.trt (moteur TensorRT FP16)
# - norm_stats_v3.json (si non fourni, calculé depuis les rollouts d'export)

# Étapes :
# 1. Charger le modèle PyTorch
# 2. Vérifier que obs_spec_version == "v3" dans le model_card
# 3. Tracer le modèle avec torch.jit.trace sur input shape (1, 70) float16
# 4. Convertir via torch2trt ou tensorrt Python API
#    - Precision : FP16
#    - max_batch_size : 1
#    - input shape : (1, 70)
#    - output shape : (1, 18)
# 5. Sauvegarder le moteur sérialisé
# 6. Mettre à jour model_card : training_date, best_reward, dry_run_validated=false
# 7. Vérifier l'export : inférence de test sur 500 inputs aléatoires
#    → assert que output shape == (1, 18)
#    → assert que toutes les valeurs sont dans [-1, 1] après clip
#    → mesurer latence moyenne sur 500 inférences, logguer en ms
```

---

**`generate_norm_stats.py` — calcul `norm_stats_v3.json`**

```python
# Prend en entrée : un fichier CSV ou numpy array de rollouts simulés
# (N observations × 70 dimensions)
# Produit : norm_stats_v3.json avec mean[70] et std_dev[70]

# Étapes :
# 1. Charger les observations (CSV ou .npy)
# 2. Vérifier que shape[-1] == 70
# 3. Calculer mean et std par dimension
# 4. Vérifier que std > 1e-6 pour chaque dimension (sinon WARN — dimension constante)
# 5. Sauvegarder norm_stats_v3.json :
{
  "obs_spec_version": "v3",
  "obs_dim": 70,
  "mean": [...],   # 70 valeurs float
  "std_dev": [...]  # 70 valeurs float, jamais < 1e-6
}
# 6. Afficher un résumé par groupe (QUATERNION, ANGULAR_VEL, etc.)
#    avec mean ± std pour chaque groupe
```

---

**`record_real_obs.py` — enregistrement observations réelles depuis ROS 2**

```python
# Script ROS 2 autonome
# Subscribe à /observation (muto_msgs/Observation)
# Enregistre N observations dans un fichier .npy ou CSV

# Arguments CLI :
# --output observations_real.npy
# --count 5000          # nombre d'observations à enregistrer
# --timeout 120         # timeout en secondes

# Vérifications pendant l'enregistrement :
# → assert que chaque obs a bien 70 valeurs
# → logguer la progression toutes les 500 observations
# → à la fin : afficher mean ± std par groupe pour inspection rapide

# Ce fichier est ensuite passé à validate_sim_real.py phase 1
```

---

**`run_validate_sim_real.py` — orchestrateur sim-to-real**

```python
# Orchestre les deux phases de validate_sim_real.py

# Arguments CLI :
# --real-obs observations_real.npy    # enregistré par record_real_obs.py
# --sim-obs  observations_sim.npy     # généré pendant l'entraînement Isaac Lab
# --model-card model_card_v003.json
# --phase 1|2|all

# Phase 1 : KS test scipy.stats.ks_2samp sur les 70 dimensions
#   → seuil p > 0.05
#   → logger l'index exact de chaque dimension qui échoue
#   → afficher le groupe auquel appartient chaque dimension échouée
#     (ex: "Dimension 15 [JOINT_ANGLES index 5] : p=0.02 FAIL")
#   → si tous PASS : mettre sim_real_validated = true dans model_card

# Phase 2 : cross-corrélation 18 servos
#   → données : commanded et measured depuis /joint_states enregistré
#   → np.correlate mode='full', lag en ms = (argmax - len) * 10
#   → comparer à latency_action_ms dans model_card
#   → logger servos hors fenêtre avec leur ID Dynamixel
#   → si tous PASS : mettre sim_real_cross_corr_validated = true dans model_card

# Sortie finale : rapport complet + model_card mis à jour
```

---

**`train.py` — script d'entraînement principal**

```python
# Arguments CLI :
# --num-envs 4096
# --max-iterations 5000
# --checkpoint None     # reprendre depuis un checkpoint si fourni
# --export-on-success   # exporter automatiquement si reward > seuil

# Étapes :
# 1. Charger observation_spec_v3.py — vérifier OBS_SPEC_VERSION
# 2. Initialiser MutoRsEnv avec domain randomization v3
# 3. Entraîner avec PPO (config_ppo.yaml)
# 4. Logger les métriques toutes les 50 itérations :
#    - reward moyen, std
#    - épisodes réussis (> 10s debout)
#    - taux de chute
# 5. Sauvegarder checkpoint toutes les 500 itérations
# 6. À la fin :
#    - générer observations_sim.npy depuis les derniers rollouts
#    - appeler generate_norm_stats.py → norm_stats_v3.json
#    - mettre à jour model_card : best_reward, training_date
#    - si --export-on-success et reward > seuil : appeler export_tensorrt.py
```

---

**`config_ppo.yaml` — hyperparamètres PPO**

```yaml
algorithm: PPO

# Architecture réseau
network:
  hidden_dims: [512, 256, 128]
  activation: elu
  init_noise_std: 1.0

# PPO
ppo:
  clip_param: 0.2
  entropy_coef: 0.01
  value_loss_coef: 1.0
  max_grad_norm: 1.0
  use_clipped_value_loss: true

# Collecte
num_steps_per_env: 24
num_mini_batches: 4
num_epochs: 5
gamma: 0.99
lam: 0.95

# Optimiseur
optimizer:
  lr: 3.0e-4
  schedule: adaptive   # réduit lr si kl_div > kl_threshold
  kl_threshold: 0.01

# Entraînement
num_envs: 4096
max_iterations: 5000
save_interval: 500
log_interval: 50

# Seuil export automatique
export_reward_threshold: 5.0

# Objectif minimal phase 4 : tenir debout 10 secondes
min_episode_length_success_s: 10.0
```

---

**TRAVAIL À FAIRE — ordre strict**

**Étape 0 — Inventaire**

```bash
# Chercher ce qui existe déjà dans le dépôt
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "*.py" | grep -iv "__pycache__" | sort
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "*.urdf" -o -name "*.xacro" | sort
find /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL -name "*.yaml" | sort
# Vérifier si Isaac Lab est installé
python3 -c "import isaaclab; print(isaaclab.__version__)" 2>/dev/null || echo "Isaac Lab non installé"
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

**Étape 1 — Créer la structure**

```bash
mkdir -p /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/isaac_lab_training/{envs,tasks,training,export,tools}
```

**Étape 2 — Créer chaque fichier**

Dans l'ordre :
1. `envs/observation_spec_v3.py` — contrat versionné
2. `envs/muto_rs_env.py` — environnement complet
3. `tasks/locomotion_task.py` — reward v1 + domain rand v3
4. `training/config_ppo.yaml` — hyperparamètres
5. `training/train.py` — script principal
6. `export/generate_norm_stats.py`
7. `export/export_tensorrt.py`
8. `tools/record_real_obs.py`
9. `tools/run_validate_sim_real.py`

**Étape 3 — Vérification syntaxe**

```bash
cd /home/edwin/TEKBOT_ROBOTICS_BENIN/MUTO_RL/isaac_lab_training
python3 -m py_compile \
    envs/observation_spec_v3.py \
    envs/muto_rs_env.py \
    tasks/locomotion_task.py \
    training/train.py \
    export/generate_norm_stats.py \
    export/export_tensorrt.py \
    tools/record_real_obs.py \
    tools/run_validate_sim_real.py && echo "PY_COMPILE_OK"
```

**Étape 4 — Vérification cohérence**

```bash
# Vérifier que OBS_DIM == 70 est utilisé partout de façon cohérente
grep -rn "70\|OBS_DIM\|obs_dim" isaac_lab_training/ | grep -v ".pyc"

# Vérifier que ACTION_DIM == 18 est utilisé partout
grep -rn "18\|ACTION_DIM\|action_dim" isaac_lab_training/ | grep -v ".pyc"

# Vérifier qu'il n'y a aucun magic number de slice (tout via les constantes nommées)
grep -rn "slice(" isaac_lab_training/ | grep -v "observation_spec"
# → doit retourner vide ou uniquement les imports de slices depuis observation_spec_v3
```

---

**RÈGLES ABSOLUES**

1. `OBS_SPEC_VERSION = "v3"` et `OBS_DIM = 70` importés depuis `observation_spec_v3.py` dans **tous** les fichiers. Zéro magic number.
2. `torch.clamp(action, -1.0, 1.0)` **avant** multiplication par `ACTION_SCALE_RAD` dans `_apply_action`. Règle identique à `rl_policy_node.py`.
3. Domain randomization v3 implémentée exactement : 4 types de pertes avec les pourcentages exacts (65/20/10/5), burst 3-8.
4. Reward v1 : 6 termes exactement, coefficients exacts (0.3, 0.01, 0.05, 0.1, 10.0).
5. `validate_obs_dim()` appelé dans `_get_observations()` en mode debug.
6. `generate_norm_stats.py` refuse de produire un `std_dev` < 1e-6 — remplacer par 1.0 avec WARN.
7. `export_tensorrt.py` vérifie `obs_spec_version == "v3"` dans le model_card avant tout export.
8. `record_real_obs.py` vérifie que chaque observation a exactement 70 valeurs avant de l'enregistrer.
9. Fichiers livrés complets du début à la fin. Aucun `# ... reste inchangé`.
10. Si Isaac Lab n'est pas installé, le code utilise des stubs compatibles avec l'API Isaac Lab documentée — le code doit pouvoir être exécuté tel quel une fois Isaac Lab installé, sans modification.