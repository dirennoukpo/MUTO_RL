#!/bin/bash

################################################################################
# setup_jetson.sh — Bootstrap Jetson Nano dans le container Docker
#
# Exécuté UNE SEULE FOIS après le premier démarrage du container.
#
# PRÉREQUIS :
#   - Container Docker Jetson actif
#   - Variables d'environnement Docker définies : WORKING_DIR, ROBOT_ROLE, TARGET_IP
#   - Image Docker construite et démarrée (make run PROFILE=jetson)
#
# USAGE depuis le container (après make shell) :
#   $ bash setup_jetson.sh
#
# RÉSULTAT ATTENDU :
#   - Build complet du workspace ROS 2 (packages Jetson)
#   - Vérification de TensorRT
#   - Vérification de la mémoire disponible
#   - Avant de lancer : Pi et Jetson doivent être en réseau
#
################################################################################

set -e

# ─────────────────────────────────────────────────────────────────────────────
# VALIDATIONS PRÉALABLES
# ─────────────────────────────────────────────────────────────────────────────

: "${WORKING_DIR:?ERREUR: WORKING_DIR non défini. Ce script doit tourner dans le container Docker.}"
: "${ROBOT_ROLE:?ERREUR: ROBOT_ROLE non défini. Vérifiez docker/config/.env.jetson_nano}"
: "${CONTAINER_NAME:?ERREUR: CONTAINER_NAME non défini.}"
: "${TARGET_IP:?ERREUR: TARGET_IP non défini (adresse du Pi).}"

if [ "$ROBOT_ROLE" != "BRAIN" ]; then
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║ ERREUR : ce script est réservé à la Jetson                   ║"
    echo "║ Robot attendu : BRAIN                                        ║"
    echo "║ Robot reçu   : $ROBOT_ROLE                                   ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# INTERFACE UTILISATEUR
# ─────────────────────────────────────────────────────────────────────────────

HEADER="╔═══════════════════════════════════════════════════════════════╗"
FOOTER="╚═══════════════════════════════════════════════════════════════╝"
GREEN='\033[92m'
BLUE='\033[94m'
RED='\033[91m'
YELLOW='\033[93m'
RESET='\033[0m'

echo ""
echo "$HEADER"
echo "║  Bootstrap Jetson Nano — Container: $CONTAINER_NAME"
echo "║  WORKING_DIR : $WORKING_DIR"
echo "║  TARGET_IP (Pi) : $TARGET_IP"
echo "║  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "$FOOTER"
echo ""

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 1 : Créer le dossier models
# ─────────────────────────────────────────────────────────────────────────────

WS="$WORKING_DIR/tekbot_ws"
MODELS_DIR="$WORKING_DIR/models"

echo -e "${BLUE}[1/4]${RESET} Création dossier models..."
mkdir -p "$MODELS_DIR"
echo -e "${GREEN}  ✓ Dossier créé${RESET} : $MODELS_DIR"

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 2 : Build workspace ROS 2
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[2/4]${RESET} Build workspace ROS 2..."
if [ ! -f "$WS/src/muto_inference/setup.py" ]; then
    echo -e "${RED}  ✗ Workspace ROS 2 introuvable : $WS${RESET}"
    exit 1
fi

cd "$WS"
source /opt/ros/humble/setup.bash

echo "  Building packages: muto_msgs muto_control muto_inference muto_perception..."
colcon build --symlink-install \
    --packages-select \
        muto_msgs muto_control muto_inference \
        muto_perception muto_navigation muto_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tail -3

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo -e "${RED}  ✗ Build échoué${RESET}"
    exit 1
fi
echo -e "${GREEN}  ✓ Build réussi${RESET}"

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 3 : Vérifier TensorRT
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[3/4]${RESET} Vérification TensorRT..."
if python3 -c "import tensorrt as trt; print('  ✓ TensorRT', trt.__version__, 'installé')" 2>/dev/null; then
    :
else
    echo -e "  ${YELLOW}ⓘ${RESET} TensorRT NON installé dans ce container"
    echo "    Installer : pip install tensorrt (ou checkpoints .trt non disponibles)"
fi

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 4 : Vérifier RAM disponible
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[4/4]${RESET} Ressources système..."
echo -n "  "
free -h | grep Mem | awk '{printf "RAM: %s total, %s utilisée\n", $2, $3}'

LOAD=$(cat /proc/loadavg | awk '{print $1}')
echo -e "  CPU Load: $LOAD"

# ─────────────────────────────────────────────────────────────────────────────
# SUCCÈS + VÉRIFICATION RÉSEAU
# ─────────────────────────────────────────────────────────────────────────────

echo ""
echo "$HEADER"
echo "║  ${GREEN}✓ Bootstrap Jetson réussi${RESET}"
echo "║"
echo "║  AVANT DE LANCER — Vérifier la connectivité réseau :"
echo "║    • TARGET_IP défini et machine distante joignable"
echo "║    • Adresse locale de la machine correctement configurée"
echo "║    • DOMAIN_ID identique entre les deux machines"
echo "║"
echo "║  Prochaine étape :"
echo "║    source $WS/install/setup.bash"
echo "║    ros2 launch muto_bringup jetson_full.launch.py"
echo "║"
echo "║  Diagnostic :"
echo "║    ros2 node list (vérifier Pi et Jetson)"
echo "║    ros2 topic list (vérifier tous les topics)"
echo "$FOOTER"
echo ""
