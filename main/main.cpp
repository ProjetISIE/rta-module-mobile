#include "GpsService.hpp"
#include "active_look.hpp"
#include "ble_manager.hpp"
#include "ble_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include <cstdio>

static const char *TAG = "RTA";

// Instance globale des lunettes ActiveLook
ActiveLook myGlasses;

// The GATT service definition is exported by the C definition file
extern "C" const struct ble_gatt_svc_def gps_gatt_svcs[];

// Fonction de synchronisation combinée pour le serveur et le client BLE
void combined_on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);

  // Démarrer l'annonce (Serveur)
  ble_app_advertise();

  // Démarrer le scan (Client pour les lunettes)
  ble_manager_scan_start();

  ESP_LOGI(TAG, "BLE synchronized: Advertising and Scanning started");
}

void process_and_display_task(void *arg) {
  auto &gps = rta::GpsService::instance();
  while (true) {
    auto status = gps.getStatus();
    if (status.fix) {
      // Affichage local (console)
      std::printf(
          "\r[FIX OK] Sat: %2d | Speed: %6.2f km/h | L: %9.6f, %9.6f   ",
          status.satellites, status.speed_kmh, status.latitude,
          status.longitude);
      std::fflush(stdout);

      // Affichage sur les lunettes ActiveLook
      myGlasses.displayGpsData(status.speed_kmh, status.latitude,
                               status.longitude);
    } else {
      std::printf("\r[WAITING] No fix data parsed yet...          ");
      std::fflush(stdout);

      // On peut optionnellement afficher un message sur les lunettes
      // myGlasses.displayText("Searching GPS...");
    }
    vTaskDelay(pdMS_TO_TICKS(
        1000)); // Mise à jour toutes les secondes pour les lunettes
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting Modern C++ GPS BLE Server & ActiveLook Client");

  // 1. Initialize BLE stack
  if (ble_server_init("ESP32_GPS_MODERN") != 0) {
    ESP_LOGE(TAG, "Failed to initialize BLE server");
    return;
  }

  // 2. Override sync_cb for combined behavior
  ble_hs_cfg.sync_cb = combined_on_sync;

  // 3. Start GPS service
  auto &gps = rta::GpsService::instance();
  gps.start();

  // 4. Register and Start BLE
  ble_server_register_services(gps_gatt_svcs);
  ble_server_start();

  // 5. Start local display task
  xTaskCreate(process_and_display_task, "display_task", 4096, nullptr, 5,
              nullptr);
}
