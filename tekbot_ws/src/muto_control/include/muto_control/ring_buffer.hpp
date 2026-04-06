/*
 * Documentation FR: src/muto_control/include/muto_control/ring_buffer.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <array>
#include <cstddef>

namespace muto_control {

template <typename T, std::size_t N>
class RingBuffer {
public:
  void put(std::size_t idx, const T &value) {
    data_[idx % N] = value;
    valid_[idx % N] = true;
  }

  bool get(std::size_t idx, T &out) const {
    const auto i = idx % N;
    if (!valid_[i]) {
      return false;
    }
    out = data_[i];
    return true;
  }

  void invalidate(std::size_t idx) { valid_[idx % N] = false; }

private:
  std::array<T, N> data_{};
  std::array<bool, N> valid_{};
};

}  // namespace muto_control
