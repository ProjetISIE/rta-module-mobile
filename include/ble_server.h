#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise la pile BLE et le NVS.
 *
 * @param device_name Le nom qui sera diffusé par l'appareil.
 * @return int 0 en cas de succès, code d'erreur sinon.
 */
int ble_server_init(const char *device_name);

/**
 * @brief Enregistre les services GATT auprès du serveur BLE.
 * Doit être appelé AVANT ble_server_start().
 *
 * @param svcs Pointeur vers le tableau de définitions de services (doit se
 * terminer par une entrée vide).
 * @return int 0 en cas de succès.
 */
int ble_server_register_services(const struct ble_gatt_svc_def *svcs);

/**
 * @brief Démarre la pile BLE et lance la tâche hôte.
 *
 * @return int 0 en cas de succès.
 */
int ble_server_start(void);

/**
 * @brief Démarre l'annonce publicitaire.
 */
void ble_app_advertise(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVER_H
