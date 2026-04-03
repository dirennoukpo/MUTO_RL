#!/bin/bash
set -e

# ────────────────────────────────────────────────────────────────
# Source ROS 2
# ────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
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
        --hostname="${TS_HOSTNAME:-tekbot-ros-dev}" \
        --accept-routes="${TS_ACCEPT_ROUTES:-false}" \
        --advertise-exit-node="${TS_ADVERTISE_EXIT_NODE:-false}" \
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