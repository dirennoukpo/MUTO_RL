/*
 * Documentation FR: src/muto_control/include/muto_control/posture_validator.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <array>

namespace muto_control {

class PostureValidator {
public:
  bool valid(const std::array<float, 18> &angles,
             const std::array<float, 6> &contact_forces,
             float threshold) const {
    (void)angles;
    float support = 0.0F;
    for (float v : contact_forces) {
      support += v;
    }
    return support > threshold;
  }
};

}  // namespace muto_control
