###############################################################################
# muto_hardware/CMakeLists.txt
# 
# Fichier de configuration CMake pour le package muto_hardware (nœud temps réel Pi).
#
# ARCHITECTURE
# ═════════════════════════════════════════════════════════════════════════════
# Machine cible :     Raspberry Pi (ROBOT_ROLE=DRIVER, image Docker Ubuntu 22.04)
# Python :            N/A (C++ uniquement)
# Build system :      colcon/ament_cmake
#
# DÉPENDANCES
# ───────────────────────────────────────────────────────────────────────────
# Système:  rclcpp (ROS 2 Humble C++ client library)
#           std_msgs, sensor_msgs (ROS 2 message types)
# Workspace: muto_msgs (custom message definitions)
# Externe:  muto_link_cpp_lib.so (libraire C dynamique pour USB/servos/IMU)
#           Montée via volume Docker ${WORKING_DIR}/muto_install/lib/
#
# TARGETS GÉNÉRÉS
# ───────────────────────────────────────────────────────────────────────────
# 1. usb_bridge_node     : Interface bas niveau servos/IMU → topics ROS 2 (RT 200Hz)
# 2. watchdog_node       : Superviseur de sécurité (timeouts, heartbeat)
# 3. mode_manager_node   : Gestionnaire FSM d'états centralisé
#
# INSTALLATION
# ───────────────────────────────────────────────────────────────────────────
# Cibles:      → lib/${PROJECT_NAME}/
# Headers:     → include/
# Ressources:  → share/${PROJECT_NAME}/launch, config/
#
# NOTES DE MAINTENANCE
# ───────────────────────────────────────────────────────────────────────────
# • WORKING_DIR doit être défini au moment du configure (variable d'env Docker)
# • Ne pas installer libmuto_link_cpp_lib.so via CMake (elle existe que sur Pi)
# • Le header c_api.h est dans ${WORKING_DIR}/muto_install/include (volume Docker)
# • Linkage dynamique à l'exécution via dlopen/dlsym (pas de -L/-l statique)
# • RT constraints: isolcpus=3 recommandé sur bootloader.txt du Pi
#
###############################################################################

# GUARD WORKING_DIR: validation au moment du configure
# ═════════════════════════════════════════════════════════════════════════════
# Si WORKING_DIR n'est pas défini, CMake échoue explicitement.
# Cela force l'utilisateur à verifier que le container Docker est actif
# et que le profil .env est chargé (make run).
if(NOT DEFINED ENV{WORKING_DIR} OR "$ENV{WORKING_DIR}" STREQUAL "")
  message(FATAL_ERROR
    "[muto_hardware] WORKING_DIR n'est pas défini.\n"
    "Ce package doit être compilé DANS le container Docker.\n"
    "Exécutez : make run (pour démarrer le container)\n"
    "Puis      : make shell (pour enter le container)\n"
    "Puis      : colcon build --packages-select muto_msgs muto_hardware\n"
    "WORKING_DIR doit être défini via docker/config/.env.raspberrypi"
  )
endif()

# Include directories guard
# ═════════════════════════════════════════════════════════════════════════════
# Les includes de la librairie C (muto_link_cpp.hpp) sont résolus à l'exécution,
# mais CMake a besoin du chemin pour la compilation (target_include_directories).
# Les symbols de la .so sont chargés dynamiquement via dlopen/dlsym.

cmake_minimum_required(VERSION 3.8)
project(muto_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(muto_msgs REQUIRED)

include_directories(include)

# ═════════════════════════════════════════════════════════════
# ARCHITECTURE DOCKER — résolution des chemins à l'exécution
# ═════════════════════════════════════════════════════════════
# La librairie muto_link_cpp_lib.so est montée dans le container
# via le volume Docker : ${USER_HOME}:${USER_HOME}:rw (docker-compose.yml)
# Le chemin est résolu à l'exécution depuis la variable WORKING_DIR.
# Ne pas installer la .so via CMake — elle n'existe que sur le Pi.

add_executable(usb_bridge_node src/usb_bridge_node.cpp)
ament_target_dependencies(usb_bridge_node rclcpp std_msgs sensor_msgs muto_msgs)
# Vérifier que WORKING_DIR est défini au moment du configure
if(NOT DEFINED ENV{WORKING_DIR} OR "$ENV{WORKING_DIR}" STREQUAL "")
  message(FATAL_ERROR
    "[muto_hardware] WORKING_DIR n'est pas défini.\n"
    "Ce package doit être compilé dans le container Docker.\n"
    "Vérifiez que docker/config/.env.raspberrypi est chargé\n"
    "et que le container est actif (make run).\n"
    "WORKING_DIR attendu : défini par le profil Docker."
  )
endif()
# Le header c_api.h est dans muto_install/include, monté via le volume Docker
# depuis le Pi. En développement : créer un lien symbolique local si nécessaire.
target_include_directories(usb_bridge_node PRIVATE
  $ENV{WORKING_DIR}/muto_install/include
)
# Linkage dynamique uniquement via dl (dlopen au runtime)
target_link_libraries(usb_bridge_node dl pthread)

add_executable(watchdog_node src/watchdog_node.cpp)
ament_target_dependencies(watchdog_node rclcpp std_msgs sensor_msgs muto_msgs)

add_executable(mode_manager_node src/mode_manager_node.cpp)
ament_target_dependencies(mode_manager_node rclcpp std_msgs sensor_msgs muto_msgs)

install(TARGETS
  usb_bridge_node
  watchdog_node
  mode_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
