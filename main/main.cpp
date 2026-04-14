#include "GpsService.hpp"
#include "ble_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>

static const char *TAG = "RTA";

// The GATT service definition is exported by the C definition file
extern "C" const struct ble_gatt_svc_def gps_gatt_svcs[];

void process_and_display_task(void *arg) {
  auto &gps = rta::GpsService::instance();
  while (true) {
    auto status = gps.getStatus();
    if (status.fix) {
      std::printf("\r[FIX OK] Sat: %2d | Speed: %6.2f km/h | Odo: %8.3f km   ",
                  status.satellites, status.speed_kmh, status.odometer_km);
      std::fflush(stdout);
    } else {
      std::printf("\r[WAITING] No fix data parsed yet...          ");
      std::fflush(stdout);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Starting Modern C++ GPS BLE Server");

  // 1. Initialize BLE stack
  if (ble_server_init("ESP32_GPS_MODERN") != 0) {
    ESP_LOGE(TAG, "Failed to initialize BLE server");
    return;
  }

  // 2. Start GPS service
  auto &gps = rta::GpsService::instance();
  gps.start();

  // 3. Register and Start BLE
  ble_server_register_services(gps_gatt_svcs);
  ble_server_start();

  // 4. Start local display task
  xTaskCreate(process_and_display_task, "display_task", 4096, nullptr, 5,
              nullptr);
}
