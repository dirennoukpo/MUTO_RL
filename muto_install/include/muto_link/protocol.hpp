#pragma once

#include <cstdint>
#include <vector>

#include "muto_link/export.hpp"

namespace muto_link {

/**
 * @brief Énumération des types d'instructions du protocole MUTO.
 * 
 * Le protocole utilise ces trois types d'instructions pour communiquer
 * avec la carte baseboard MUTO :
 * - Write (0x01) : Envoyer des données à la carte
 * - Read (0x02) : Demander des données à la carte
 * - Reply (0x12) : Réponse de la carte à une lecture
 */
enum class Instruction : uint8_t {
    Write = 0x01,   ///< Instruction d'écriture
    Read = 0x02,    ///< Instruction de lecture
    Reply = 0x12,   ///< Réponse de la carte
};

/**
 * @brief Gestion du protocole de communication MUTO.
 * 
 * Cette classe encapsule tout ce qui concerne le protocole de communication
 * avec la carte MUTO-rs (baseboard). Elle définit les constantes et les
 * fonctions utilitaires pour :
 * - Construire des trames de communication
 * - Calculer et vérifier les sommes de contrôle (checksum)
 * - Valider les trames reçues
 * - Convertir les données entre les formats de la trame et les types C++
 * 
 * Format de trame:
 * @code
 * [0x55][0x00]  [LEN]  [INSTR]  [ADDR]  [DATA...]  [CHK]  [0x00][0xAA]
 *  HEADER        <- PAYLOAD (LEN octets) ->              TAIL
 * @endcode
 * 
 * Où:
 * - 0x55 0x00 = En-tête de trame
 * - LEN = Longueur totale de la trame en octets
 * - INSTR = Type d'instruction (Write/Read/Reply)
 * - ADDR = Adresse du registre cible
 * - DATA = Données (jusqu'à 250 octets)
 * - CHK = Somme de contrôle
 * - 0x00 0xAA = Queue de trame
 */
class MUTO_LINK_API Protocol {
public:
    static constexpr uint8_t kHeader1 = 0x55;      ///< Premier octet d'en-tête
    static constexpr uint8_t kHeader2 = 0x00;      ///< Deuxième octet d'en-tête
    static constexpr uint8_t kTail1 = 0x00;        ///< Premier octet de queue
    static constexpr uint8_t kTail2 = 0xAA;        ///< Deuxième octet de queue
    static constexpr std::size_t kMaxData = 250;   ///< Taille maximale des données par trame

    /**
     * @brief Construit une trame de protocole complète.
     * 
     * Assemble une trame valide en incluant :
     * - Les en-têtes et queue
     * - Le type d'instruction
     * - L'adresse du registre
     * - Les données
     * - La somme de contrôle
     * 
     * @param instruction Type d'instruction (Write/Read/Reply).
     * @param address Adresse du registre cible (0-255).
     * @param data Données à inclure dans la trame (max 250 octets).
     * 
     * @return Vecteur d'octets formant une trame complète et valide.
     * 
     * @throws std::invalid_argument si l'instruction est invalide ou les données trop longues.
     */
    static std::vector<uint8_t> buildFrame(uint8_t instruction, uint8_t address, const std::vector<uint8_t>& data);

    /**
     * @brief Calcule la somme de contrôle pour les données payload.
     * 
     * La somme de contrôle est calculée comme : 255 - (somme % 256).
     * Le payload incluant LEN, INSTR, ADDR et les données.
     * 
     * @param payload Données du payload sans en-tête ni queue.
     * @return Octet de somme de contrôle.
     */
    static uint8_t checksum(const std::vector<uint8_t>& payload);

    /**
     * @brief Valide qu'une trame reçue est correcte.
     * 
     * Vérifie :
     * - La présence des en-têtes et queue corrects
     * - La correspondance entre la longueur déclarée et la longueur réelle
     * - La validité de la somme de contrôle
     * 
     * @param frame Trame complète à valider.
     * @return true si la trame est valide, false sinon.
     */
    static bool validateFrame(const std::vector<uint8_t>& frame);

    /**
     * @brief Encode une valeur 16-bit en big-endian (réseau).
     * 
     * @param value Valeur 16-bit à encoder.
     * @return Vecteur de 2 octets en big-endian.
     */
    static std::vector<uint8_t> packUint16BE(uint16_t value);

    /**
     * @brief Décode une valeur 16-bit depuis big-endian.
     * 
     * @param bytes Vecteur contenant au moins 2 octets en big-endian.
     * @return Valeur 16-bit décodée.
     * 
     * @throws std::invalid_argument si bytes contient moins de 2 octets.
     */
    static uint16_t unpackUint16BE(const std::vector<uint8_t>& bytes);
};

} // namespace muto_link
