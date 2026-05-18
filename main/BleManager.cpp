#include "BleManager.hpp"
#include "ActiveLook.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <cstring>
#include <string_view>

namespace rta {

namespace {
constexpr std::string_view TAG = "RTA_BLE_MANAGER";
}

BleManager *BleManager::instance_ = nullptr;

BleManager::BleManager(ActiveLook &glasses) : glasses_(glasses) {
  instance_ = this;
}

void BleManager::startScanning() {
  struct ble_gap_disc_params dp{};
  ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp,
               BleManager::gapEventCallback, this);
}

int BleManager::gapEventCallback(struct ble_gap_event *event, void *arg) {
  auto *manager = static_cast<BleManager *>(arg);

  switch (event->type) {
  case BLE_GAP_EVENT_DISC: {
    struct ble_hs_adv_fields fields;
    ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

    if (fields.name) {
      std::string_view name(reinterpret_cast<const char *>(fields.name),
                            fields.name_len);
      if (name.starts_with("ENGO")) {
        ESP_LOGI(TAG.data(), "ENGO found, connecting...");
        ble_gap_disc_cancel();

        struct ble_gap_conn_params cp{};
        cp.scan_itvl = 16;
        cp.scan_window = 16;
        cp.itvl_min = 24;
        cp.itvl_max = 40;
        cp.supervision_timeout = 500;

        if (ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, &cp,
                            BleManager::gapEventCallback, manager) != 0) {
          ESP_LOGE(TAG.data(), "Error connecting");
          manager->startScanning();
        }
      }
    }
    break;
  }

  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      ESP_LOGI(TAG.data(), "Connected to glasses");
      vTaskDelay(pdMS_TO_TICKS(1500));
      manager->glasses_.initializeDisplay(event->connect.conn_handle);
    } else {
      ESP_LOGE(TAG.data(), "Connection failed; status=%d",
               event->connect.status);
      manager->startScanning();
    }
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG.data(), "Disconnected from glasses; reason=%d",
             event->disconnect.reason);
    manager->glasses_.disconnect();
    manager->startScanning();
    break;
  }
  return 0;
}

} // namespace rta
