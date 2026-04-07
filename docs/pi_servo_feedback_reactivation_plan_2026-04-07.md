# Plan Technique - Reactivation du Feedback Servo (5 Etapes)

## 1. But du plan
Reintroduire le feedback servo reel de maniere controlee, sans reintroduire les erreurs:
- `response too short`
- `expected 18 servo bytes in response`

Approche:
- progression par paliers
- instrumentation
- criteres stricts de validation avant passage a l'etape suivante

## 2. Prerequis
- Stack Pi stable en mode actuel (`servo_reads_per_cycle=0`)
- IMU stable au lancement
- tests bas niveau OK (`muto_link_cpp_servo_test`, `muto_link_cpp_imu_raw_loop_loader`)
- meme configuration hardware (port, baudrate, alimentation)

## 3. Definition des metriques de controle
Pendant chaque palier, mesurer:
- taux d'erreurs servo read
- taux d'erreurs IMU raw
- taux d'erreurs IMU angles
- frequence effective de publication `/imu/data` et `/joint_states`
- jitter max et jitter moyen

Critere global de succes d'un palier:
- zero erreur critique repetee sur la fenetre de validation
- pas de degradation majeure de frequence
- pas de cascade d'erreurs apres quelques secondes

## 4. Execution en 5 etapes

## Etape 1 - Baseline instrumentee (sans feedback servo)
Configuration:
- `servo_reads_per_cycle=0`

Objectif:
- etablir une reference stable pour comparer les paliers suivants

Validation:
- execution 5 a 10 minutes
- confirmer qu'aucune erreur protocole n'apparait

Si echec:
- bloquer le plan, corriger la baseline avant tout.

## Etape 2 - Feedback minimal tres decime
Configuration proposee:
- activer feedback sur un sous-ensemble reduit
- cadence tres faible (ex: 1 lecture tous les N cycles, N eleve)

Objectif:
- verifier si la simple presence du feedback, a faible pression bus, reste stable

Validation:
- run 10 minutes
- erreurs zero ou quasi nulles

Si echec:
- revenir Etape 1
- ajouter mecanisme de resynchronisation driver apres read incomplet

## Etape 3 - Feedback complet mais cadence faible
Configuration proposee:
- couvrir les 18 IDs
- polling round-robin lent

Objectif:
- verifier la tenue sur l'ensemble des servos

Validation:
- run 15 minutes
- pas de drift vers erreurs IMU

Si echec:
- identifier segment d'IDs declencheur
- tester fenetrage ou batching alternatif

## Etape 4 - Augmentation progressive de cadence
Configuration:
- augmenter la frequence de feedback par paliers
- conserver protections d'ordonnancement (transactions non concurrentes)

Objectif:
- converger vers une cadence compatible avec les besoins de controle

Validation:
- a chaque palier, 10 minutes de run
- passage uniquement si metriques dans les seuils

Si echec:
- revenir au dernier palier stable
- ne jamais sauter directement a la cadence cible.

## Etape 5 - Validation finale systeme
Tests a lancer:
- launch Pi complet
- tests hardware Pi (topics, watchdog, transitions)
- observation continue (>= 20 min)

Objectif:
- confirmer la robustesse en conditions integration

Critere d'acceptation finale:
- aucune erreur protocolaire recurrente
- frequence/jitter compatibles avec l'usage cible
- watchdog et FSM restent nominals

## 5. Durcissements recommandes cote driver
Pour fiabiliser durablement:
1. ajouter une routine de desynchronisation/resynchronisation de trame
2. purger le buffer serie apres certains types d'erreurs
3. introduire des garde-fous sur l'enchainement read/write
4. tracer les longueurs reelles de payload et les en-tetes/tails en debug
5. ajouter un test de stress automatise reproduisant le pattern mixte

## 6. Strategie de rollback
A tout moment, en cas d'instabilite:
- remettre `servo_reads_per_cycle=0`
- relancer stack Pi
- confirmer retour immediate a l'etat stable

Ce rollback doit rester simple, rapide et documente pour eviter tout blocage d'exploitation.

## 7. Livrables attendus a la fin de la phase 2
- config finale stable de feedback servo
- courbe d'erreurs par palier
- seuils de fonctionnement recommandes
- mise a jour des tests pour prevenir regressions futures
