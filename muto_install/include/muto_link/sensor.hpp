#pragma once

#include <cstdint>

#include "muto_link/export.hpp"
#include "muto_link/driver.hpp"

namespace muto_link {

/**
 * @brief Données d'orientations 3D et température provenant de l'IMU fusionné.
 * 
 * Représente les angles d'Euler (Roll, Pitch, Yaw) calculés par les
 * capteurs IMU du MUTO-rs et la température du capteur.
 * 
 * Tous les champs d'angle sont en valeurs brutes (LSB = 0.01°).
 * Utilisez Sensor::toDegrees() pour convertir en degrés.
 */
struct MUTO_LINK_API IMUAngleData {
    uint16_t roll;              ///< Angle de roulis (Roll) brut, en LSB
    uint16_t pitch;             ///< Angle de tangage (Pitch) brut, en LSB
    uint16_t yaw;               ///< Angle de lacet (Yaw) brut, en LSB
    uint8_t temperature;        ///< Température du capteur en °C
};

/**
 * @brief Données brutes d'accélération, gyroscope et magnétomètre.
 * 
 * Contient les 9 axes de l'IMU (3 accéléromètres + 3 gyroscopes + 3 magnétomètres)
 * dans leur format brut (LSB = unités capteur).
 * 
 * Utilisez les fonctions Sensor::toXxx() pour convertir en unités physiques :
 * - Accélération : toMs2() pour m/s²
 * - Gyroscope : toDegPerSec() pour °/s
 * - Magnétomètre : aucune conversion fournie actuellement
 */
struct MUTO_LINK_API RawIMUData {
    uint16_t accel_x;           ///< Accélération X brute (LSB)
    uint16_t accel_y;           ///< Accélération Y brute (LSB)
    uint16_t accel_z;           ///< Accélération Z brute (LSB)
    uint16_t gyro_x;            ///< Vitesse angulaire X brute (LSB)
    uint16_t gyro_y;            ///< Vitesse angulaire Y brute (LSB)
    uint16_t gyro_z;            ///< Vitesse angulaire Z brute (LSB)
    uint16_t mag_x;             ///< Champ magnétique X brut (LSB)
    uint16_t mag_y;             ///< Champ magnétique Y brut (LSB)
    uint16_t mag_z;             ///< Champ magnétique Z brut (LSB)
};

struct MUTO_LINK_API EulerAnglesDeg {
    float roll;
    float pitch;
    float yaw;
    uint8_t temperature_c;
};

struct MUTO_LINK_API Imu9AxesPhysical {
    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float mag_x_raw;
    float mag_y_raw;
    float mag_z_raw;
};

/**
 * @brief Classe de gestion des capteurs IMU du MUTO-rs.
 * 
 * Cette classe étend Driver pour fournir des méthodes haut-niveau
 * de interaction avec l'IMU 9-axes du MUTO-rs.
 * 
 * Elle offre :
 * - Lecture des angles d'Euler (Roll/Pitch/Yaw) fusionnés
 * - Lecture des données brutes IMU (accél, gyro, magnéto)
 * - Fonctions de conversion entre format brut et unités physiques
 * 
 * Paramètres de conversion définissant la plage de mesure :
 * - Accélération : 4G full-scale (1G = 8192 LSB)
 * - Gyroscope : 2000 °/s full-scale (1 °/s = 16.4 LSB)
 * - Angles : 1° = 100 LSB
 * 
 * Exemple d'utilisation :
 * @code
 * auto transport = std::make_unique<UsbSerial>("/dev/ttyUSB0");
 * Sensor sensor(std::move(transport));
 * sensor.open();
 * 
 * // Lire les angles fusionnés
 * auto angles = sensor.getImuAngle();
 * float roll_deg = Sensor::toDegrees(angles.roll);
 * 
 * // Lire les données brutes
 * auto raw = sensor.getRawImuData();
 * float accel_x_ms2 = Sensor::toMs2(raw.accel_x);
 * float gyro_z_dps = Sensor::toDegPerSec(raw.gyro_z);
 * @endcode
 */
class MUTO_LINK_API Sensor final : public Driver {
public:
    using Driver::Driver;

    // === Constantes de conversion ===

    static constexpr float kGravityMs2 = 9.80665f;     ///< Accélération gravitationnelle standard (m/s²)
    static constexpr float kAccelScale = 8192.0f;      ///< Facteur d'échelle accélération : 1G = 8192 LSB (4G full-scale)
    static constexpr float kGyroScale = 16.4f;         ///< Facteur d'échelle gyroscope : 1 °/s = 16.4 LSB (2000 °/s full-scale)
    static constexpr float kAngleScale = 100.0f;       ///< Facteur d'échelle angle : 1° = 100 LSB

    // === Fonctions de conversion ===

    /**
     * @brief Convertit une valeur brute d'angle en degrés.
     * 
     * @param raw Valeur brute de l'angle (LSB).
     * @return Angle en degrés (positif ou négatif).
     */
    static float toDegrees(uint16_t raw);

    /**
     * @brief Convertit une valeur brute d'accélération en G (9.81 m/s²).
     * 
     * Suppose une plage full-scale de 4G avec résolution 8192 LSB/G.
     * 
     * @param raw Valeur brute de l'accélération (LSB).
     * @return Accélération en unités G.
     */
    static float toG(uint16_t raw);

    /**
     * @brief Convertit une valeur brute d'accélération en m/s².
     * 
     * Combine toG() avec l'accélération gravitationnelle standard.
     * 
     * @param raw Valeur brute de l'accélération (LSB).
     * @return Accélération en m/s².
     */
    static float toMs2(uint16_t raw);

    /**
     * @brief Convertit une valeur brute de gyroscope en degrés par seconde.
     * 
     * Suppose une plage full-scale de 2000 °/s avec résolution 16.4 LSB/°s.
     * 
     * @param raw Valeur brute de la vitesse angulaire (LSB).
     * @return Vitesse angulaire en °/s.
     */
    static float toDegPerSec(uint16_t raw);

    // === Méthodes de lecture capteur ===

    /**
     * @brief Lit les angles d'Euler fusionnés (Roll, Pitch, Yaw).
     * 
     * Récupère les angles d'orientation 3D calculés par la fusion IMU
     * et la température du capteur.
     * 
     * @return Structure IMUAngleData contenant :
     *         - roll, pitch, yaw en valeurs brutes (LSB)
     *         - temperature en °C
     * 
     * @throws std::runtime_error si la lecture échoue.
     */
    IMUAngleData getImuAngle();
    EulerAnglesDeg getImuAngleDegrees();

    /**
     * @brief Lit toutes les données brutes de l'IMU 9-axes.
     * 
     * Récupère les 9 axes complets :
     * - Accéléromètre (X, Y, Z)
     * - Gyroscope (X, Y, Z)
     * - Magnétomètre (X, Y, Z)
     * 
     * @return Structure RawIMUData contenant les 9 valeurs en LSB.
     * 
     * @throws std::runtime_error si la lecture échoue.
     */
    RawIMUData getRawImuData();
    Imu9AxesPhysical getImuPhysical();

private:
    static constexpr uint8_t kRegImuAngle = 0x60;      ///< Adresse de registre pour les angles fusionnés
    static constexpr uint8_t kRegImuRaw = 0x61;        ///< Adresse de registre pour les données brutes IMU
};

} // namespace muto_link
