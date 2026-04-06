/*
 * Documentation FR: src/muto_perception/include/muto_perception/pcl_utils.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

namespace muto_perception {

inline float clamp_height(float h) {
  if (h < -1.0F) {
    return -1.0F;
  }
  if (h > 1.0F) {
    return 1.0F;
  }
  return h;
}

}  // namespace muto_perception
