#include "ble_manager.hpp"
#include "active_look.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include <cstring>
#include <string_view>

static constexpr std::string_view TAG = "BLE_MANAGER";

// Instance globale définie dans main.cpp
extern ActiveLook myGlasses;

extern "C" int ble_manager_gap_event(struct ble_gap_event *event, void *) {
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
                            ble_manager_gap_event, nullptr) != 0) {
          ESP_LOGE(TAG.data(), "Error connecting");
          ble_manager_scan_start();
        }
      }
    }
    break;
  }

  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status == 0) {
      ESP_LOGI(TAG.data(), "Connected to glasses");
      vTaskDelay(pdMS_TO_TICKS(1500));
      myGlasses.initializeDisplay(event->connect.conn_handle);
    } else {
      ESP_LOGE(TAG.data(), "Connection failed; status=%d",
               event->connect.status);
      ble_manager_scan_start();
    }
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG.data(), "Disconnected from glasses; reason=%d",
             event->disconnect.reason);
    myGlasses.disconnect();
    ble_manager_scan_start();
    break;
  }
  return 0;
}

void ble_manager_scan_start(void) {
  struct ble_gap_disc_params dp{};
  ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp, ble_manager_gap_event,
               nullptr);
}

void ble_manager_init() { ble_manager_scan_start(); }
