#include "active_look.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <string.h>

// Prépare et envoie une trame de données aux lunettes selon le protocole
// propriétaire ActiveLook
void ActiveLook::sendCommand(uint8_t cmd, const uint8_t *payload,
                             uint16_t len) {
  uint8_t buf[128];
  uint16_t total_len = 4 + len + 1; // Calcul de la taille de la trame

  buf[0] = 0xFF; // Octet de début de trame, constant
  buf[1] = cmd;  // Commande à exécuter (ex: 0x01 pour effacer)
  buf[2] = (uint8_t)(total_len >> 8);   // Taille (poids fort)
  buf[3] = (uint8_t)(total_len & 0xFF); // Taille (poids faible)

  // S'il y a des données utiles (coordonnées, texte...), on les insère dans le
  // buffer
  if (payload && len > 0)
    memcpy(&buf[4], payload, len);
  buf[4 + len] = 0xAA; // Octet de fin de trame, constant

  // Envoi de la trame via BLE sur chaque handle identifié
  for (uint16_t h : handles) {
    ble_gattc_write_no_rsp_flat(connection_handle, h, buf, total_len);
  }
}

// Allume l'écran des lunettes et y envoie un texte de test
void ActiveLook::displayHello(uint16_t conn_h) {
  connection_handle = conn_h; // Sauvegarde l'identifiant de la connexion active

  uint8_t vOn = 0x01, vFlip = 0x02;
  sendCommand(0x00, &vOn, 1); // Commande The Power: Allume l'écran
  sendCommand(
      0x03, &vFlip,
      1); // Configuration: Retourne l'écran si nécessaire ou change le mode
  sendCommand(0x01); // Commande Clear: Efface le contenu de l'écran

  // Attente brève (50ms) pour laisser le temps à l'écran de s'initialiser
  vTaskDelay(pdMS_TO_TICKS(50));

  displayText("RTA Connecté"); // Affiche le texte de test initial
}

// [NOUVEAU] Efface l'écran et affiche un nouveau texte
void ActiveLook::displayText(const char *msg) {
  if (connection_handle == 0xFFFF)
    return; // Ignore si on n'est pas connecté (0xFFFF)

  sendCommand(0x01); // Efface l'ancien texte pour éviter la superposition
  vTaskDelay(pdMS_TO_TICKS(
      50)); // DELAI OBLIGATOIRE avant d'envoyer le texte sinon il est annulé !

  // Préparation des paramètres de la commande LumaText
  // txt regroupe : {X, Y, Rotation, Font, Couleur}
  uint8_t txt[64] = {0x00, 0x99, 0x00, 0x60, 0x04, 0x02, 0x0F};

  // Sécurité: limite la taille du texte à 50 caractères pour entrer dans le
  // buffer
  size_t msg_len = strlen(msg) > 50 ? 50 : strlen(msg);
  memcpy(&txt[7], msg, msg_len);
  txt[7 + msg_len] = '\0'; // Caractère de fin

  // Envoi de la commande texte (0x37)
  sendCommand(0x37, txt, 7 + msg_len + 1);
}

// Affiche un nombre entier (converti en texte)
void ActiveLook::displayNumber(int value) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%d",
           value);     // Convertit le chiffre en texte
  displayText(buffer); // Appelle la méthode d'affichage
}

// Affiche la vitesse et les coordonnées GPS
void ActiveLook::displayGpsData(float speed, double lat, double lon) {
  if (connection_handle == 0xFFFF)
    return;

  sendCommand(0x01); // Efface l'écran
  vTaskDelay(pdMS_TO_TICKS(50));

  char buffer[64];

  // Affichage de la vitesse
  snprintf(buffer, sizeof(buffer), "%.1f km/h", speed);
  uint8_t txtSpeed[64] = {0x00, 0x10, 0x00, 0x40,
                          0x00, 0x02, 0x0F}; // X=16, Y=64
  size_t lenSpeed = strlen(buffer);
  memcpy(&txtSpeed[7], buffer, lenSpeed);
  txtSpeed[7 + lenSpeed] = '\0';
  sendCommand(0x37, txtSpeed, 7 + lenSpeed + 1);

  vTaskDelay(pdMS_TO_TICKS(50));

  // Affichage des coordonnées
  snprintf(buffer, sizeof(buffer), "%.4f, %.4f", lat, lon);
  uint8_t txtCoords[64] = {
      0x00, 0x10, 0x00, 0x80,
      0x00, 0x01, 0x0F}; // X=16, Y=128, Font=1 (plus petite)
  size_t lenCoords = strlen(buffer);
  memcpy(&txtCoords[7], buffer, lenCoords);
  txtCoords[7 + lenCoords] = '\0';
  sendCommand(0x37, txtCoords, 7 + lenCoords + 1);
}
