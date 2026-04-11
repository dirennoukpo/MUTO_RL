# MUTO_RL - Documentation complete

## Objectif

Cette branche automatise la preparation d'une machine Jetson Nano pour un usage distant et deploiement rapide:

1. transfert des scripts/configuration depuis la machine locale,
2. installation des dependances systeme sur la Jetson,
3. configuration reseau et acces distant.

## Fichiers du projet

- `setup_jetson_nano.sh`:
	script lance sur la machine locale pour copier les fichiers vers la Jetson.
- `scripts/dependances.sh`:
	script a executer sur la Jetson pour installer/configurer l'environnement.
- `config/.env.exemple`:
	exemple de variables d'environnement a preparer localement.

## 1) Script local: `setup_jetson_nano.sh`

### Role

Preparer le deploiement vers la Jetson via SCP.

### Fonctionnement

1. charge les variables de `./config/.env`:

```bash
set -a
source ./config/.env
set +a
```

2. copie le script distant:

```bash
scp ./scripts/dependances.sh jetson@$IP_JETSON:/home/jetson/
```

3. copie le dossier de configuration:

```bash
scp -r ./config/ jetson@$IP_JETSON:/home/jetson/
```

### Prerequis

- compte SSH `jetson` accessible,
- variable `IP_JETSON` definie,
- commande `scp` disponible en local.

## 2) Script distant: `scripts/dependances.sh`

Ce script est prevu pour etre execute sur la Jetson Nano.

### A. Maintenance systeme

1. `sudo apt-get update`
2. `sudo apt-get upgrade -y`
3. `sudo apt autoremove -y`

### B. Tailscale

1. installe Tailscale via script officiel,
2. lance `sudo tailscale up`,
3. affiche `tailscale ip` et `tailscale status`.

### C. Cle SSH

1. cree `~/.ssh` si besoin,
2. ajoute une cle publique dans `~/.ssh/authorized_keys`,
3. fixe les permissions (`700` puis `600`),
4. affiche le contenu de `authorized_keys`.

### D. Reseau Ethernet statique

1. verifie les interfaces avec `nmcli device status`,
2. cree une connexion `static-eth0` sur `eth0` avec `10.0.0.2/24`,
3. configure passerelle (`10.0.0.254`) et DNS Google,
4. active la connexion,
5. verifie avec `ip addr show eth0`.

### E. Depot projet

Clone le depot:

```bash
git clone https://github.com/dirennoukpo/MUTO_RL.git
cd MUTO_RL/
```

## Variables d'environnement

Le script local attend un fichier `config/.env`.

Exemple minimal:

```env
IP_JETSON=192.168.1.60
```

Le fichier `config/.env.exemple` peut servir de base.

## Procedure complete recommandee

1. creer le fichier de variables:

```bash
cp config/.env.exemple config/.env
# puis renseigner IP_JETSON
```

2. depuis la machine locale:

```bash
chmod +x setup_jetson_nano.sh
./setup_jetson_nano.sh
```

3. se connecter a la Jetson et executer:

```bash
chmod +x /home/jetson/dependances.sh
/home/jetson/dependances.sh
```

## Points d'attention

- `tailscale up` peut demander une validation interactive,
- la cle publique est ajoutee en dur dans le script (a securiser selon le contexte),
- la config reseau statique peut couper la connectivite si le plan IP ne correspond pas,
- le script ne contient pas de gestion d'erreurs (`set -e`, logs),
- les commandes ne sont pas idempotentes (re-execution potentiellement problematique).

## Ameliorations conseillees

1. ajouter `set -euo pipefail` en tete des scripts,
2. tester l'existence des connexions `nmcli` avant creation,
3. externaliser la cle SSH et les IP dans `config/.env`,
4. ajouter des verifications post-installation (Tailscale, reseau, acces SSH).
