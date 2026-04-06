/*
 * Documentation FR: src/muto_utils/include/muto_utils/logger_config.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <string>

namespace muto_utils {

inline std::string logger_name(const std::string &base) {
  return "muto_" + base;
}

}  // namespace muto_utils
