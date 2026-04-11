# But de cette branche

Cette branche a pour objectif de **préparer rapidement un Raspberry Pi 5** pour un usage distant, avec une configuration réseau via **Tailscale**.

## Ce que fait le script

Le script [setup_raspberrypi5.sh](setup_raspberrypi5.sh) automatise les étapes suivantes :

1. Mise à jour des paquets système.
2. Nettoyage des paquets inutiles.
3. Installation de Tailscale.
4. Connexion de la machine au réseau Tailscale.
5. Affichage de l'IP Tailscale et de l'état de connexion.

## Pourquoi cette branche existe

Cette branche sert de base d'initialisation pour :

- gagner du temps lors du provisioning d'un Raspberry Pi 5 ;
- obtenir rapidement un accès distant sécurisé ;
- standardiser la configuration de départ avant d'ajouter d'autres composants (robotique, IA, services, etc.).

## Utilisation

Depuis la racine du projet :

```bash
chmod +x setup_raspberrypi5.sh
./setup_raspberrypi5.sh
```

## Remarque

Le script nécessite des droits administrateur (`sudo`) et une interaction utilisateur peut être demandée lors de l'activation Tailscale.
