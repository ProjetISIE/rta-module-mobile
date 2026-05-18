#pragma once

#include "host/ble_gap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise le gestionnaire BLE (Scan pour les lunettes).
 */
void ble_manager_init();

/**
 * @brief Démarre le scan pour trouver les lunettes ActiveLook (ENGO).
 */
void ble_manager_scan_start(void);

/**
 * @brief Gestionnaire d'événements GAP pour la découverte et la connexion.
 *
 * @param event Événement GAP reçu.
 * @param arg Argument utilisateur (non utilisé).
 * @return 0 en cas de succès.
 */
int ble_manager_gap_event(struct ble_gap_event *event, void *arg);

#ifdef __cplusplus
}
#endif
