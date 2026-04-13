#!/bin/bash
set -e

: "${WORKING_DIR:?WORKING_DIR must be set from env files}"
: "${MUTO_LINK_CPP_REPO:?MUTO_LINK_CPP_REPO must be set from env files}"
: "${MUTO_LINK_CPP_REF:?MUTO_LINK_CPP_REF must be set from env files}"

WORKSPACE_DIR=$WORKING_DIR
MUTO_LINK_CPP_DIR=$WORKSPACE_DIR/muto_link_cpp
TEKBOT_WS_DIR=$WORKSPACE_DIR/tekbot_ws

if [ -f /opt/tekbot_venv/bin/activate ]; then
    source /opt/tekbot_venv/bin/activate
fi

# ────────────────────────────────────────────────────────────────
# Source ROS 2
# ────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

if [ -d "$WORKSPACE_DIR" ]; then
    cd "$WORKSPACE_DIR"

    if [ -d "$MUTO_LINK_CPP_DIR/.git" ]; then
        echo "[entrypoint] Mise a jour de muto_link_cpp (git pull --ff-only)..."
        if ! git -C "$MUTO_LINK_CPP_DIR" pull --ff-only origin "$MUTO_LINK_CPP_REF"; then
            echo "[entrypoint] WARN: git pull muto_link_cpp echoue (conflit local ou branche absente)."
        fi

        echo "[entrypoint] Compilation de muto_link_cpp..."
        cd "$MUTO_LINK_CPP_DIR"
        mkdir -p build && cd build
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$WORKSPACE_DIR/muto_install" .. \
            && cmake --build . --config Release \
            && cmake --install . \
            || echo "[entrypoint] WARN: compilation de muto_link_cpp echouee."
        cd "$WORKSPACE_DIR"
    fi

    if [ ! -f "$MUTO_LINK_CPP_DIR/include/muto_link/c_api.h" ]; then
        echo "[entrypoint] muto_link_cpp introuvable, clonage dans $MUTO_LINK_CPP_DIR..."
        if ! git clone --depth 1 --branch "$MUTO_LINK_CPP_REF" "$MUTO_LINK_CPP_REPO" "$MUTO_LINK_CPP_DIR"; then
            echo "[entrypoint] WARN: impossible de cloner muto_link_cpp."
        else
            echo "[entrypoint] Compilation de muto_link_cpp..."
            cd "$MUTO_LINK_CPP_DIR"
            mkdir -p build && cd build
            cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$WORKSPACE_DIR/muto_install" .. \
                && cmake --build . --config Release \
                && cmake --install . \
                || echo "[entrypoint] WARN: compilation de muto_link_cpp échouée."
            cd "$WORKSPACE_DIR"
        fi
    fi

    if [ -f "$MUTO_LINK_CPP_DIR/include/muto_link/c_api.h" ]; then
        mkdir -p "$WORKSPACE_DIR/muto_install/include"
        ln -sfn "$MUTO_LINK_CPP_DIR/include/muto_link" "$WORKSPACE_DIR/muto_install/include/muto_link"
    else
        echo "[entrypoint] WARN: c_api.h toujours introuvable, build muto_hardware risque d'echouer."
    fi

    if [ ! -f "$WORKSPACE_DIR/muto_install/lib/libmuto_link_cpp_lib.so" ]; then
        echo "[entrypoint] WARN: libmuto_link_cpp_lib.so non trouvée, muto_hardware échouera au runtime."
    fi

    if [ -d "$TEKBOT_WS_DIR" ]; then
        echo "[entrypoint] Préparation de tekbot_ws..."
        cd "$TEKBOT_WS_DIR"

        if [ -d "$TEKBOT_WS_DIR/src" ]; then
            echo "[entrypoint] Build de tekbot_ws (colcon build)..."
            if ! colcon build --symlink-install; then
                echo "[entrypoint] WARN: build tekbot_ws échoué."
            fi
        else
            echo "[entrypoint] WARN: dossier src introuvable dans tekbot_ws, build ignoré."
        fi

        if [ -f "$TEKBOT_WS_DIR/install/setup.bash" ]; then
            echo "[entrypoint] Sourcing de tekbot_ws/install/setup.bash"
            source "$TEKBOT_WS_DIR/install/setup.bash"
        elif [ -f "$TEKBOT_WS_DIR/install/local_setup.bash" ]; then
            echo "[entrypoint] Sourcing de tekbot_ws/install/local_setup.bash"
            source "$TEKBOT_WS_DIR/install/local_setup.bash"
        else
            echo "[entrypoint] WARN: aucun setup bash trouvé dans tekbot_ws/install."
        fi

        cd "$WORKSPACE_DIR"
    else
        echo "[entrypoint] WARN: tekbot_ws introuvable dans $TEKBOT_WS_DIR"
    fi
fi

# ────────────────────────────────────────────────────────────────
# Tailscale — démarrage automatique du daemon + connexion
#
# TS_AUTHKEY  : clé d'authentification (depuis .env.user)
# TS_HOSTNAME : nom du nœud sur le tailnet
# TS_ACCEPT_ROUTES : accepter les routes annoncées (true/false)
# TS_ADVERTISE_EXIT_NODE : annoncer ce noeud en exit node (true/false)
# ────────────────────────────────────────────────────────────────
echo "[entrypoint] Démarrage de tailscaled..."
tailscaled --tun=userspace-networking \
           --socks5-server=localhost:1055 \
           --state=/var/lib/tailscale/tailscaled.state \
           &

# Attendre que le daemon soit prêt
sleep 2

if [ -n "$TS_AUTHKEY" ]; then
    echo "[entrypoint] Connexion Tailscale avec authkey..."
    tailscale up \
        --authkey="$TS_AUTHKEY" \
    --hostname="$TS_HOSTNAME" \
    --accept-routes="$TS_ACCEPT_ROUTES" \
    --advertise-exit-node="$TS_ADVERTISE_EXIT_NODE" \
        || echo "[entrypoint] WARN: tailscale up a échoué (déjà connecté ?)"
else
    echo "[entrypoint] WARN: TS_AUTHKEY non défini — Tailscale en attente d'auth manuelle."
    echo "[entrypoint]       Lancez : tailscale up --authkey=<votre_clé>"
fi

echo "[entrypoint] Statut Tailscale :"
tailscale status || true

# ────────────────────────────────────────────────────────────────
# Lancer la commande passée au container (par défaut : bash)
# ────────────────────────────────────────────────────────────────
exec "$@"