#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "muto_link/export.hpp"

namespace muto_link {

/**
 * @brief Interface abstraite pour la couche de transport de communication.
 * 
 * Cette classe définit l'interface que tous les transports (USB, UART, etc.)
 * doivent implémenter pour communiquer avec la carte MUTO via liaison série.
 * 
 * La couche de transport est responsable de :
 * - L'ouverture/fermeture de la connexion au port série
 * - L'envoi des données brutes
 * - La réception des données brutes
 * - La gestion des timeouts
 * 
 * Tous les transports utilisent une interface commune pour permettre
 * l'abstraction et la testabilité.
 */
class MUTO_LINK_API Transport {
public:
    virtual ~Transport() = default;

    /**
     * @brief Ouvre la connexion de transport.
     * 
     * Établit la connexion avec le périphérique (ouverture du port série,
     * configuration des paramètres, etc.). Doit être appelé avant toute
     * opération de lecture/écriture.
     * 
     * @throws std::runtime_error si l'ouverture échoue.
     */
    virtual void open() = 0;

    /**
     * @brief Ferme la connexion de transport.
     * 
     * Libère les ressources et ferme la connexion.
     * Peut être appelé plusieurs fois en toute sécurité.
     */
    virtual void close() = 0;

    /**
     * @brief Envoie des données brutes via le transport.
     * 
     * Écrit complètement les données dans le buffer de sortie jusqu'à
     * ce que tous les octets soient envoyés. Cette opération est synchrone
     * et attend que tous les données soient réellement transmises.
     * 
     * @param data Vecteur d'octets à envoyer.
     * @return Nombre d'octets réellement envoyés (équivaut à data.size()).
     * 
     * @throws std::runtime_error si l'envoi échoue.
     */
    virtual std::size_t write(const std::vector<uint8_t>& data) = 0;

    /**
     * @brief Reçoit des données brutes du transport.
     * 
     * Lit jusqu'à 'size' octets depuis le transport avec un timeout.
     * Si le timeout est atteint avant de recevoir tous les octets,
     * retourne les octets reçus jusqu'à présent.
     * 
     * @param size Nombre d'octets à lire.
     * @param timeout_sec Timeout en secondes (par défaut 0 = utiliser le timeout par défaut du transport).
     * @return Vecteur d'octets reçus (peut contenir moins que 'size' en cas de timeout).
     * 
     * @throws std::runtime_error si la lecture échoue.
     */
    virtual std::vector<uint8_t> read(std::size_t size, double timeout_sec = 0.0) = 0;
};

/**
 * @brief Implémentation du transport via port série USB/UART.
 * 
 * Cette classe offre une implémentation concrète de l'interface Transport
 * pour communiquer avec la carte MUTO via une liaison série USB ou UART native.
 * 
 * Caractéristiques :
 * - Ouverture/fermeture du port série
 * - Configuration automatique des paramètres (débit, bits, parité, etc.)
 * - Lecture/écriture non-bloquantes avec timeout
 * - Gestion robuste des erreurs système
 * - Utilisation d'API POSIX standards (fcntl, termios, select)
 * 
 * Débits supportés : 9600, 19200, 38400, 57600, 115200, 230400, 460800 (plateforme-dépendant).
 * 
 * Exemple d'utilisation :
 * @code
 * auto transport = std::make_unique<UsbSerial>("/dev/ttyUSB0", 115200);
 * transport->open();
 * transport->write({0x55, 0x00, ...});
 * auto data = transport->read(10);
 * transport->close();
 * @endcode
 */
class MUTO_LINK_API UsbSerial final : public Transport {
public:
    /**
     * @brief Construit un transport série USB/UART.
     * 
     * @param port Chemin du port série (ex: "/dev/ttyUSB0" sur Linux, "COM3" sur Windows).
     * @param baud Débit en bauds (par défaut 115200).
     * @param timeout_sec Timeout par défaut pour les opérations de lecture en secondes.
     *                   Doit être > 0 (par défaut 0.05 secondes = 50ms).
     * 
     * @throws std::invalid_argument si port est vide ou timeout_sec <= 0.
     */
    explicit UsbSerial(std::string port, int baud = 115200, double timeout_sec = 0.05)
        : port_(std::move(port)), baud_(baud), timeout_sec_(timeout_sec) {
        if (port_.empty()) {
            throw std::invalid_argument("UsbSerial port cannot be empty");
        }
        if (timeout_sec_ <= 0.0) {
            throw std::invalid_argument("UsbSerial timeout must be > 0");
        }
    }

    /**
     * @brief Destructeur. Ferme automatiquement le port s'il était ouvert.
     */
    ~UsbSerial() override;

    /**
     * @brief Ouvre le port série et configure les paramètres.
     * 
     * Configuration appliquée :
     * - Mode brut (raw mode)
     * - Pas de parité, 1 bit de stop, 8 bits de données
     * - Pas de contrôle de flux matériel (RTS/CTS)
     * - Mode non-bloquant
     * 
     * @throws std::runtime_error si l'ouverture ou la configuration échoue.
     */
    void open() override;

    /**
     * @brief Ferme le port série et libère les ressources.
     * 
     * Peut être appelée plusieurs fois en toute sécurité.
     */
    void close() override;

    /**
     * @brief Envoie des données sur le port série.
     * 
     * Écrit complètement les données jusqu'à ce que tous les octets
     * soient envoyés, puis attend que le buffer de sortie soit vidé.
     * 
     * @param data Vecteur d'octets à envoyer.
     * @return Nombre d'octets envoyés (équivaut à data.size()).
     * 
     * @throws std::runtime_error si le port est fermé ou si l'écriture échoue.
     */
    std::size_t write(const std::vector<uint8_t>& data) override;

    /**
     * @brief Reçoit des données du port série avec timeout.
     * 
     * Lit jusqu'à 'size' octets avec un timeout. Retourne les octets reçus
     * avant la fin du timeout. Utilise select() pour éviter les lectures bloquantes.
     * 
     * @param size Nombre d'octets à lire.
     * @param timeout_sec Timeout en secondes (0 = utiliser le timeout par défaut du transport).
     * @return Vecteur d'octets reçus (peut contenir moins que 'size' en cas de timeout).
     * 
     * @throws std::runtime_error si le port est fermé ou si la lecture échoue.
     */
    std::vector<uint8_t> read(std::size_t size, double timeout_sec = 0.0) override;

private:
    std::string port_;          ///< Chemin du port série
    int baud_;                  ///< Débit en bauds
    double timeout_sec_;        ///< Timeout par défaut pour les lectures (en secondes)
    int fd_{-1};                ///< Descripteur de fichier du port (-1 = fermé)
};

} // namespace muto_link
