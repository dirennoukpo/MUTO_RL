/*
 * Documentation FR: src/muto_hardware/include/muto_hardware/complementary_filter.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

/**
 * @file complementary_filter.hpp
 * @brief Conversion orientee temps reel des angles Euler IMU vers quaternion.
 *
 * Ce filtre est volontairement minimaliste pour la phase actuelle : il priorise
 * la stabilite numerique et la reproductibilite du pipeline observation.
 *
 * LIMITATION : la fusion accelero/gyro est simplifiee et ne fait pas encore
 * d'integration dynamique avancee du biais.
 */
#pragma once

#include <array>
#include <cmath>

namespace muto_hardware {

/**
 * @class ComplementaryFilter
 * @brief Outil de conversion orientation Euler (degres) -> quaternion (w,x,y,z).
 *
 * Raison de design : dans la boucle 200 Hz, une formule fermee est privilegiee
 * pour eviter toute allocation et minimiser la variance de temps d'execution.
 */
class ComplementaryFilter {
public:
  /**
   * @brief Retourne un quaternion normalise a partir des angles Euler en degres.
   *
   * @param roll_deg Angle de roulis en degres.
   * @param pitch_deg Angle de tangage en degres.
   * @param yaw_deg Angle de lacet en degres.
   * @param gyro_x_rad_s Vitesse angulaire X en rad/s (non exploitee dans cette version).
   * @param gyro_y_rad_s Vitesse angulaire Y en rad/s (non exploitee dans cette version).
   * @param gyro_z_rad_s Vitesse angulaire Z en rad/s (non exploitee dans cette version).
   * @return std::array<float,4> Quaternion (w,x,y,z).
   *
   * @note Le calcul suit la decomposition Tait-Bryan et reste O(1).
   */
  std::array<float, 4> update(
      float roll_deg,
      float pitch_deg,
      float yaw_deg,
      float gyro_x_rad_s,
      float gyro_y_rad_s,
      float gyro_z_rad_s) {
    (void)gyro_x_rad_s;
    (void)gyro_y_rad_s;
    (void)gyro_z_rad_s;
    const float roll = roll_deg * static_cast<float>(M_PI / 180.0);
    const float pitch = pitch_deg * static_cast<float>(M_PI / 180.0);
    const float yaw = yaw_deg * static_cast<float>(M_PI / 180.0);

    const float cr = std::cos(roll * 0.5F);
    const float sr = std::sin(roll * 0.5F);
    const float cp = std::cos(pitch * 0.5F);
    const float sp = std::sin(pitch * 0.5F);
    const float cy = std::cos(yaw * 0.5F);
    const float sy = std::sin(yaw * 0.5F);

    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
  }
};

}  // namespace muto_hardware
