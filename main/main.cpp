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
#include <print>
#include <string_view>

static constexpr std::string_view TAG = "RTA";

// Instance globale des lunettes ActiveLook
ActiveLook myGlasses;

// Définition des services GATT exportée depuis le fichier C++
extern "C" struct ble_gatt_svc_def gps_gatt_svcs[];

// Fonction de synchronisation combinée serveur/client BLE
extern "C" void combined_on_sync() {
  if (ble_hs_util_ensure_addr(0) != 0) {
    ESP_LOGE(TAG.data(), "Failed to ensure BLE address");
    return;
  }

  ble_app_advertise();      // Démarrer le serveur
  ble_manager_scan_start(); // Démarrer le scan client

  ESP_LOGI(TAG.data(), "BLE synchronized: Advertising and Scanning started");
}

/**
 * @brief Tâche de traitement et d'affichage des données GPS.
 */
void process_and_display_task(void *) {
  auto &gps = rta::GpsService::instance();
  int rta_ok_timer = 0;
  bool was_connected = false;
  bool last_fix = false;
  double last_lat = 0.0, last_lon = 0.0;
  bool force_update = false;

  while (true) {
    const bool connected = myGlasses.isConnected();
    const auto status = gps.getStatus();

    if (connected) {
      if (!was_connected) {
        rta_ok_timer = 5;
        was_connected = true;
        force_update = true;
      }

      if (rta_ok_timer > 0) {
        rta_ok_timer--;
      } else {
        const bool fix_changed = (status.fix != last_fix);
        const bool pos_changed = status.fix && (status.latitude != last_lat ||
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

    // Affichage console formaté via std::print (C++23)
    if (status.fix) {
      std::print("\r[FIX OK] Sat: {:2d} | Speed: {:6.2f} km/h | L: {:9.6f}, "
                 "{:9.6f}   ",
                 status.satellites, status.speed_kmh, status.latitude,
                 status.longitude);
    } else {
      std::print("\r[WAITING] No fix data parsed yet...          ");
    }
    std::fflush(stdout);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

extern "C" void app_main() {
  ESP_LOGI(TAG.data(),
           "Starting Modern C++ GPS BLE Server & ActiveLook Client");

  // Initialisation du NVS (nécessaire pour le Bluetooth)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialisation de la pile BLE
  if (ble_server_init("ESP32_RTA") != 0) {
    ESP_LOGE(TAG.data(), "Failed to initialize BLE server");
    return;
  }

  // Configuration du callback de synchronisation
  ble_hs_cfg.sync_cb = combined_on_sync;

  // Démarrage du service GPS
  rta::GpsService::instance().start();

  // Enregistrement des services et démarrage de la pile
  ble_server_register_services(gps_gatt_svcs);
  ble_server_start();

  // Création de la tâche d'affichage
  xTaskCreate(process_and_display_task, "display_task", 4096, nullptr, 5,
              nullptr);
}
