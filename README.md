# MUTO_RL - Documentation de la branche

## But de la branche

Cette branche sert a automatiser le demarrage d'un Raspberry Pi 5 pour un environnement robotique/IA:

1. preparer la machine distante (paquets systeme),
2. installer les dependances utiles (reseau, docker, python),
3. deposer les fichiers de configuration depuis la machine locale vers le Raspberry Pi.

## Arborescence utile

- `setup_raspberrypi5.sh`: script lance en local pour preparer l'envoi vers le Pi.
- `scripts/dependances.sh`: script execute sur le Raspberry Pi pour installer/configurer les dependances.
- `config/.env`: variables d'environnement locales (exemple: IP du Raspberry Pi).

## Detail des scripts

### `setup_raspberrypi5.sh` (machine locale)

Ce script:

1. charge les variables du fichier `config/.env`,
2. copie `scripts/dependances.sh` dans `/home/pi/` sur le Pi,
3. copie le dossier `config/` dans `/home/pi/` sur le Pi.

Commande utilisee:

```bash
scp ./scripts/dependances.sh pi@$IP_PI:/home/pi/
scp -r ./config/ pi@$IP_PI:/home/pi/
```

### `scripts/dependances.sh` (Raspberry Pi)

Ce script execute les actions suivantes:

1. `apt-get update` + `apt-get upgrade` + `apt autoremove`,
2. installation et activation de Tailscale,
3. affichage d'etat reseau Tailscale (`tailscale ip`, `tailscale status`),
4. installation de Git,
5. installation de Docker via script officiel,
6. ajout de l'utilisateur courant au groupe docker,
7. installation de `python3-pip`, creation d'un environnement virtuel `venv`,
8. installation de librairies Python (`Adafruit-SSD1306`, `Adafruit-GPIO`, `Pillow`),
9. configuration d'une interface Ethernet statique via `nmcli`,
10. clonage du depot `MUTO_RL`.

## Prerequis

Avant execution, verifier:

1. acces SSH au Raspberry Pi (utilisateur `pi`),
2. variable `IP_PI` correctement renseignee dans `config/.env`,
3. droits `sudo` sur le Raspberry Pi,
4. commandes `scp` et `ssh` disponibles sur la machine locale.

Exemple minimal de `config/.env`:

```env
IP_PI=192.168.1.50
```

## Procedure conseillee

1. depuis la machine locale:

```bash
chmod +x setup_raspberrypi5.sh
./setup_raspberrypi5.sh
```

2. puis, sur le Raspberry Pi:

```bash
chmod +x /home/pi/dependances.sh
/home/pi/dependances.sh
```

## Points d'attention

- la commande `tailscale up` peut demander une action interactive,
- la commande `newgrp docker` peut ouvrir un nouveau contexte shell,
- la configuration reseau statique force `eth0` en `10.0.0.1/24` (adapter si besoin),
- certaines etapes sont potentiellement intrusives en production (upgrade systeme, reseau, docker).

## Etat actuel

La branche fournit une base de provisioning fonctionnelle, mais il est recommande d'ajouter ensuite:

1. des scripts idempotents,
2. des verifications d'erreur (`set -e`, logs),
3. une separation claire entre installation de base et configuration metier.
