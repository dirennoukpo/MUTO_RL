#!/usr/bin/env bash
set -e

SCRIPT_PATH="${BASH_SOURCE[0]:-$0}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_DIR="$ROOT_DIR/docker/config"

load_env_file() {
	local file="$1"
	if [ ! -f "$file" ]; then
		return 0
	fi
	while IFS= read -r line || [ -n "$line" ]; do
		# Ignore comments/empty lines
		[[ -z "$line" || "$line" =~ ^[[:space:]]*# ]] && continue
		local key="${line%%=*}"
		local val="${line#*=}"
		key="$(echo "$key" | xargs)"
		val="$(echo "$val" | sed -e 's/^ *//' -e 's/ *$//')"
		export "$key=$val"
	done < "$file"
}

PROFILE_FILE=""
HOST_LOWER="$(hostname 2>/dev/null | tr '[:upper:]' '[:lower:]')"

if [[ "$HOST_LOWER" == *"jetson"* || "$HOST_LOWER" == *"yahboom"* ]] && [ -f "$ENV_DIR/.env.jetson_nano" ]; then
	PROFILE_FILE="$ENV_DIR/.env.jetson_nano"
elif [[ "$HOST_LOWER" == *"pi"* ]] && [ -f "$ENV_DIR/.env.raspberrypi" ]; then
	PROFILE_FILE="$ENV_DIR/.env.raspberrypi"
elif [ -f "$ENV_DIR/.env.$(whoami)" ]; then
	PROFILE_FILE="$ENV_DIR/.env.$(whoami)"
elif [ -f "$ENV_DIR/.env.default" ]; then
	PROFILE_FILE="$ENV_DIR/.env.default"
fi

load_env_file "$ENV_DIR/.env.base"
if [ -n "$PROFILE_FILE" ]; then
	load_env_file "$PROFILE_FILE"
fi

source /opt/ros/humble/setup.bash
cd "$SCRIPT_DIR"

if [ "${1:-}" = "--build" ]; then
	colcon build --symlink-install
fi

if [ -f install/setup.bash ]; then
	source install/setup.bash
fi

echo "Workspace tekbot_ws pret."
echo "Env active: ${PROFILE_FILE:-none}"
echo "ROBOT_ROLE=${ROBOT_ROLE:-unset} DOMAIN_ID=${DOMAIN_ID:-unset} TARGET_IP=${TARGET_IP:-unset}"
