# Rapport Detaille - Diagnostic Raspberry Pi (2026-04-07)

## 1. Objectif de la mission
Realiser un diagnostic complet des erreurs observees sur la pile Raspberry Pi (usb_bridge_node, watchdog_node, mode_manager_node), corriger les causes reproductibles, puis stabiliser le systeme pour permettre la poursuite des tests.

Objectifs techniques:
- supprimer les erreurs USB critiques repetitives
- verifier si la cause est logicielle, materielle, ou protocolaire
- fournir une solution operationnelle immediate

## 2. Symptomes observes au depart
Lors des launches Pi, les logs montraient de facon recurrente:
- `Driver::read failed to read response header`
- `Driver::readServoAngleDeg expected 18 servo bytes in response`
- `Sensor::getImuAngle response too short`
- `Sensor::getRawImuData response too short`

Impact:
- instabilite du flux IMU/joint states
- baisse de robustesse de la boucle hardware
- echec de certains tests de performance/frequence

## 3. Demarche d'investigation (comment on est arrive au resultat)
La methode employeee a ete une elimination progressive des hypotheses, avec reproduction controlee.

### Etape A - Verification des configurations effectives
Actions:
- lecture des parametres dans `system_params.yaml`
- verification runtime des valeurs imprimees au boot du noeud

Constat:
- le systeme tournait a `baudrate=115200`
- des essais precedents a 460800 n'etaient pas le mode stable du banc

Decision:
- conserver 115200 pour alignement avec la config reelle du materiel branche.

### Etape B - Validation materielle hors ROS
Actions:
- execution de `muto_link_cpp_imu_raw_loop_loader`
- execution de `muto_link_cpp_servo_test`
- verification du device USB via `dmesg`

Constat:
- IMU stable en lecture directe
- servos actifs et pilotables (mouvement reussi)
- convertisseur CH341 detecte sur `/dev/ttyUSB0`

Conclusion:
- pas de panne materielle globale
- le probleme apparait surtout dans certains patterns d'acces.

### Etape C - Reproduction sous charge avec C API (hors ROS)
Actions:
- scripts de stress en Python via `libmuto_link_cpp_lib.so`
- cas testes:
  1) lectures servo seules
  2) lectures IMU seules
  3) ecritures servo seules
  4) melange ecritures servo + lectures servo + lectures IMU

Resultats:
- cas 1, 2, 3: stables
- cas 4: reproduction des erreurs `response too short` et `expected 18 servo bytes`

Conclusion technique solide:
- ce n'est pas un bug "mono-fonction"
- c'est un probleme de cohabitation/protocole sous transactions mixtes a cadence elevee.

### Etape D - Tentatives de correction intermediaires
Actions tentees:
- ajustement timeout/retry au niveau C API
- startup buffer
- serialisation par mutex des acces hardware
- ordonnancement exclusif partiel des familles de lecture

Constat:
- ameliorations ponctuelles possibles
- mais la source d'instabilite persistait tant que le feedback servo restait actif dans la boucle RT 200 Hz avec IMU.

### Etape E - Correctif pragmatique de stabilisation
Strategie retenue:
- desactiver temporairement la lecture feedback servo en boucle RT
- publier les angles commandes comme `measured_angles` en mode degrade stable

Pourquoi ce choix:
- supprime le pattern qui provoque les trames tronquees
- preserve le fonctionnement global du stack ROS2
- permet de continuer les validations watchdog/FSM/integration sans bloquer le projet

## 4. Corrections appliquees

### Fichier 1 - Parametrage Pi
`tekbot_ws/src/muto_bringup/config/system_params.yaml`

Changement principal:
- `servo_reads_per_cycle: 0`

Effet:
- stoppe les lectures feedback servo dans la boucle RT
- evite les collisions de transactions qui destabilisaient IMU

### Fichier 2 - Logique du bridge hardware
`tekbot_ws/src/muto_hardware/src/usb_bridge_node.cpp`

Changements principaux:
- accepter `servo_reads_per_cycle=0` comme valeur valide
- si feedback servo desactive:
  - remplir `measured_angles` avec `commanded_angles_rad_`
- conserver la lecture IMU sans surcharge due au polling servo

Effet:
- suppression du chemin d'erreur principal en exploitation courante
- boucle hardware plus stable pour l'usage de test/integration

## 5. Validation apres correctifs
Commande de verification utilisee:
- lancement Pi avec timeout et capture de logs

Constat de validation:
- le noeud demarre proprement
- les logs affichent explicitement `reads_per_cycle=0`
- absence d'avalanche d'erreurs observee dans la fenetre de test

Note sur `Exit Code 143`:
- attendu ici car le process est stoppe par `timeout`
- ce n'est pas une preuve de crash interne.

## 6. Resultat final obtenu
Resultat atteint:
- systeme stabilise pour continuer le developpement et les tests de la pile Pi
- elimination du pattern de communication le plus destructeur
- diagnostic cause racine etabli: transactions mixtes feedback servo + IMU sous charge

Compromis assume:
- feedback servo reel desactive temporairement
- `joint_states` publies en mode open-loop (angles commandes)

## 7. Ce qui reste ouvert (limites connues)
- la restauration du feedback servo reel en RT n'est pas encore revalidee
- le driver/protocole doit encore etre renforce pour supporter durablement les transactions mixtes sans trames tronquees

## 8. Recommandation de suite
Passer en phase 2 de reactivation progressive du feedback servo avec garde-fous, mesures, et criteres de passage/echec (documentee dans le plan separe).
