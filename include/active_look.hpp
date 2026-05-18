#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

/**
 * @brief Classe gérant la communication avec les lunettes connectées
 * ActiveLook.
 *
 * Cette classe implémente le protocole propriétaire pour envoyer des commandes
 * et afficher des informations sur l'écran des lunettes via BLE.
 */
class ActiveLook {
public:
  /// Identifiants des caractéristiques Bluetooth utilisées pour les commandes.
  static constexpr uint16_t COMMAND_HANDLES[] = {35, 56, 32};

  /// Constantes du protocole ActiveLook
  enum class Command : uint8_t {
    POWER = 0x00,
    CLEAR = 0x01,
    CONFIG = 0x03,
    LUMA_TEXT = 0x37
  };

  /**
   * @brief Envoie une commande structurée aux lunettes.
   * @param cmd Code de la commande à exécuter.
   * @param payload Données optionnelles associées à la commande.
   */
  void sendCommand(Command cmd, std::span<const uint8_t> payload = {});

  /**
   * @brief Initialise l'affichage après la connexion.
   * @param conn_handle Identifiant de la connexion BLE.
   */
  void initializeDisplay(uint16_t conn_handle);

  /**
   * @brief Affiche du texte sur l'écran.
   * @param msg Message à afficher.
   */
  void displayText(const char *msg);

  /**
   * @brief Affiche un nombre entier.
   * @param value Valeur à afficher.
   */
  void displayNumber(int value);

  /**
   * @brief Vérifie l'état de la connexion.
   * @return true si les lunettes sont connectées.
   */
  bool isConnected() const { return connection_handle_.has_value(); }

  /**
   * @brief Réinitialise l'état lors de la déconnexion.
   */
  void disconnect() { connection_handle_.reset(); }

  /**
   * @brief Affiche un message d'attente pour le fix GPS.
   */
  void displayGpsWait();

  /**
   * @brief Affiche les coordonnées GPS sur l'écran.
   * @param lat Latitude.
   * @param lon Longitude.
   */
  void displayCoordinates(double lat, double lon);

private:
  std::optional<uint16_t> connection_handle_;
};
