set -a
source ./config/.env
set +a

scp ./scripts/dependances.sh jetson@$IP_JETSON:/home/jetson/
scp -r ./config/ jetson@$IP_JETSON:/home/jetson/
