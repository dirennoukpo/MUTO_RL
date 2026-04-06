/*
 * Documentation FR: src/muto_perception/include/muto_perception/thermal_monitor.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

namespace muto_perception {

class ThermalMonitor {
public:
  bool should_throttle(float gpu_freq_ratio) const {
    return gpu_freq_ratio < 0.8F;
  }
};

}  // namespace muto_perception
