#pragma once
#include <stdint.h>
#include <vector>

// Classe gérant la communication avec les lunettes connectées ActiveLook
class ActiveLook {
private:
  uint16_t connection_handle = 0xFFFF; // Handle BLE (0xFFFF = non connecté.
                                       // Attention, 0 est un handle valide !)
  // Liste des handles (caractéristiques Bluetooth) où nous envoyons nos
  // commandes
  const std::vector<uint16_t> handles = {35, 56, 32};

public:
  // Envoie une commande structurée aux lunettes (ex: allumer, configurer,
  // afficher texte)
  void sendCommand(uint8_t cmd, const uint8_t *payload = nullptr,
                   uint16_t len = 0);

  // Fonction principale pour initialiser l'écran et y afficher un message par
  // défaut
  void displayHello(uint16_t conn_h);

  // Affiche n'importe quel texte sur l'écran
  void displayText(const char *msg);

  // Affiche n'importe quel nombre (ex: vitesse du GPS) sur l'écran
  void displayNumber(int value);

  // Affiche la vitesse et les coordonnées GPS
  void displayGpsData(float speed, double lat, double lon);
};
