set -a
source ./config/.env
set +a

scp ./scripts/dependances.sh pi@$IP_PI:/home/pi/
scp -r ./config/ pi@$IP_PI:/home/pi/
