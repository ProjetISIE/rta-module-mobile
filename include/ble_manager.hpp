#pragma once

#include "host/ble_gap.h"

// Initialise la couche réseau Bluetooth (Pile NimBLE),
// configure les événements et démarre le scan en arrière-plan.
void ble_manager_init();

// Démarre le scan pour les lunettes
void ble_manager_scan_start(void);

// Gestionnaire d'événements GAP
int ble_manager_gap_event(struct ble_gap_event *event, void *arg);
