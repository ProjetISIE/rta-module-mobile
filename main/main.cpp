#include "GpsService.hpp"
#include "active_look.hpp"
#include "ble_manager.hpp"
#include "ble_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nvs_flash.h"
#include <cstdio>

static const char *TAG = "RTA";

// Instance globale des lunettes ActiveLook
ActiveLook myGlasses;

// The GATT service definition is exported by the C definition file
extern "C" const struct ble_gatt_svc_def gps_gatt_svcs[];

// Fonction de synchronisation combinée pour le serveur et le client BLE
extern "C" void combined_on_sync(void) {
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
  int rta_ok_timer = 0;
  bool was_connected = false;
  bool last_fix = false;
  double last_lat = 0, last_lon = 0;
  bool force_update = false;

  while (true) {
    bool connected = myGlasses.isConnected();
    auto status = gps.getStatus();

    if (connected) {
      if (!was_connected) {
        rta_ok_timer = 5;
        was_connected = true;
        force_update = true;
      }

      if (rta_ok_timer > 0) {
        rta_ok_timer--;
      } else {
        bool fix_changed = (status.fix != last_fix);
        // On considère que la position a changé si on a un fix et que les
        // coordonnées bougent
        bool pos_changed = status.fix && (status.latitude != last_lat ||
                                          status.longitude != last_lon);

        if (force_update || fix_changed || pos_changed) {
          if (status.fix) {
            myGlasses.displayCoordinates(status.latitude, status.longitude);
            last_lat = status.latitude;
            last_lon = status.longitude;
          } else {
            myGlasses.displayGpsWait();
          }
          last_fix = status.fix;
          force_update = false;
        }
      }
    } else {
      was_connected = false;
      rta_ok_timer = 0;
      last_fix = false;
      force_update = false;
    }

    // Affichage local toujours actif pour le debug
    if (status.fix) {
      std::printf(
          "\r[FIX OK] Sat: %2d | Speed: %6.2f km/h | L: %9.6f, %9.6f   ",
          status.satellites, status.speed_kmh, status.latitude,
          status.longitude);
    } else {
      std::printf("\r[WAITING] No fix data parsed yet...          ");
    }
    std::fflush(stdout);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting Modern C++ GPS BLE Server & ActiveLook Client");

  // 0. Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

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
