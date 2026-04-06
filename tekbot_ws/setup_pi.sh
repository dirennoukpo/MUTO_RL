#!/bin/bash

################################################################################
# setup_pi.sh — Bootstrap Raspberry Pi dans le container Docker
#
# Exécuté UNE SEULE FOIS après le premier démarrage du container.
#
# PRÉREQUIS :
#   - Container Docker Pi actif
#   - Variables d'environnement Docker définies : WORKING_DIR, ROBOT_ROLE, CONTAINER_NAME
#   - Image Docker construite et démarrée (make run)
#
# USAGE depuis le container (après make shell) :
#   $ bash setup_pi.sh
#
# RÉSULTAT ATTENDU :
#   - Compilation et installation de muto_link_cpp dans ${WORKING_DIR}/muto_install
#   - Build complet du workspace ROS 2 (packages Pi)
#   - Vérification des symboles exportés
#   - Validation des constraints temps réel (isolcpus=3)
#
################################################################################

set -e

# ─────────────────────────────────────────────────────────────────────────────
# VALIDATIONS PRÉALABLES
# ─────────────────────────────────────────────────────────────────────────────

: "${WORKING_DIR:?ERREUR: WORKING_DIR non défini. Ce script doit tourner dans le container Docker.}"
: "${ROBOT_ROLE:?ERREUR: ROBOT_ROLE non défini. Vérifiez docker/config/.env.raspberrypi}"
: "${CONTAINER_NAME:?ERREUR: CONTAINER_NAME non défini.}"

if [ "$ROBOT_ROLE" != "DRIVER" ]; then
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║ ERREUR : ce script est réservé au Pi                         ║"
    echo "║ Robot attendu : DRIVER                                       ║"
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
RESET='\033[0m'

echo ""
echo "$HEADER"
echo "║  Bootstrap Raspberry Pi — Container: $CONTAINER_NAME"
echo "║  WORKING_DIR : $WORKING_DIR"
echo "║  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "$FOOTER"
echo ""

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 1 : Compiler et installer muto_link_cpp
# ─────────────────────────────────────────────────────────────────────────────

INSTALL_PREFIX="$WORKING_DIR/muto_install"
WS="$WORKING_DIR/tekbot_ws"

echo -e "${BLUE}[1/5]${RESET} Compilation muto_link_cpp..."
if [ ! -d "$WORKING_DIR/muto_link_cpp" ]; then
    echo -e "${RED}  ✗ Répertoire $WORKING_DIR/muto_link_cpp introuvable${RESET}"
    echo "    Assurez-vous que muto_link_cpp est monté via le volume Docker."
    exit 1
fi

cd "$WORKING_DIR/muto_link_cpp"
cmake -S . -B build -DMUTO_LINK_CPP_BUILD_SHARED=ON >/dev/null 2>&1
cmake --build build -j$(nproc) >/dev/null 2>&1
cmake --install build --prefix "$INSTALL_PREFIX" >/dev/null 2>&1
echo -e "${GREEN}  ✓ muto_link_cpp installée${RESET}"
echo "    Destination : $INSTALL_PREFIX/lib/"

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 2 : Vérifier les symboles exportés
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[2/5]${RESET} Vérification symboles exportés..."
SO_PATH="$INSTALL_PREFIX/lib/libmuto_link_cpp_lib.so"
if [ ! -f "$SO_PATH" ]; then
    echo -e "${RED}  ✗ .so non trouvée : $SO_PATH${RESET}"
    exit 1
fi

SYMBOLS=("muto_create_usb" "muto_open" "muto_read_servo_angle_deg" \
         "muto_write_servo_command_rad" "muto_get_imu_euler" "muto_close")
ALL_OK=true
for sym in "${SYMBOLS[@]}"; do
    if nm -D "$SO_PATH" | grep -q "$sym"; then
        echo -e "  ${GREEN}✓${RESET} $sym"
    else
        echo -e "  ${RED}✗${RESET} $sym (MANQUANT)"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = false ]; then
    echo -e "${RED}ERREUR : symboles manquants dans la .so${RESET}"
    exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 3 : Créer le dossier models
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[3/5]${RESET} Création dossier models..."
mkdir -p "$WORKING_DIR/models"
echo -e "${GREEN}  ✓ Dossier créé${RESET} : $WORKING_DIR/models"

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 4 : Build workspace ROS 2
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[4/5]${RESET} Build workspace ROS 2..."
if [ ! -f "$WS/src/muto_hardware/CMakeLists.txt" ]; then
    echo -e "${RED}  ✗ Workspace ROS 2 introuvable : $WS${RESET}"
    exit 1
fi

cd "$WS"
source /opt/ros/humble/setup.bash

# Nettoyer les builds antérieurs (optionnel)
# rm -rf build install log

echo "  Building packages: muto_msgs muto_hardware..."
colcon build --symlink-install \
    --packages-select muto_msgs muto_hardware \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tail -3

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo -e "${RED}  ✗ Build échoué${RESET}"
    exit 1
fi
echo -e "${GREEN}  ✓ Build réussi${RESET}"

# ─────────────────────────────────────────────────────────────────────────────
# ÉTAPE 5 : Vérifier isolcpus=3
# ─────────────────────────────────────────────────────────────────────────────

echo -e "${BLUE}[5/5]${RESET} Vérification contraintes temps réel..."
ISOLCPUS=$(cat /proc/cmdline 2>/dev/null | tr ' ' '\n' | grep isolcpus | head -1)
if [ -z "$ISOLCPUS" ]; then
    echo -e "  ${BLUE}ℹ${RESET}  isolcpus non activé dans le kernel"
    echo "    (optionnel : bootloader.txt sur Pi : isolcpus=3)"
else
    echo -e "  ${GREEN}✓${RESET} $ISOLCPUS"
fi

# ─────────────────────────────────────────────────────────────────────────────
# SUCCÈS
# ─────────────────────────────────────────────────────────────────────────────

echo ""
echo "$HEADER"
echo "║  ${GREEN}✓ Bootstrap Pi réussi${RESET}"
echo "║"
echo "║  Prochaine étape :"
echo "║    source $WS/install/setup.bash"
echo "║    ros2 launch muto_bringup pi_full.launch.py"
echo "║"
echo "║  Vérification :"
echo "║    htop (vérifier topologie CPU et RT priorities)"
echo "║    ros2 topic list (vérifier connectivité ROS 2)"
echo "$FOOTER"
echo ""
