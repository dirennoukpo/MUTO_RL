/*
 * Documentation FR: src/muto_utils/include/muto_utils/timing_stats.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <cstdint>

namespace muto_utils {

struct TimingStats {
  uint64_t cycles = 0;
  double last_ms = 0.0;
};

}  // namespace muto_utils
