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
constexpr std::string_view tag = "RTA_BLE_MANAGER";

void handleDiscovery(struct ble_gap_event* event, BleManager* manager,
                     ActiveLook& glasses) {
    struct ble_hs_adv_fields fields{};
    ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

    if (fields.name != nullptr) {
        std::string_view name(reinterpret_cast<const char*>(fields.name),
                              fields.name_len);
        if (name.starts_with("ENGO")) {
            ESP_LOGI(tag.data(), "ENGO found, connecting...");
            ble_gap_disc_cancel();

            struct ble_gap_conn_params conn_params{};
            conn_params.scan_itvl = 16;
            conn_params.scan_window = 16;
            conn_params.itvl_min = 24;
            conn_params.itvl_max = 40;
            conn_params.supervision_timeout = 500;

            if (ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000,
                                &conn_params, BleManager::gapEventCallback,
                                manager) != 0) {
                ESP_LOGE(tag.data(), "Error connecting");
                manager->startScanning();
            }
        }
    }
}
} // namespace

BleManager* BleManager::instance_ = nullptr;

BleManager::BleManager(ActiveLook& glasses) : glasses_(glasses) {
    instance_ = this;
}

void BleManager::startScanning() noexcept {
    struct ble_gap_disc_params disc_params{};
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                 BleManager::gapEventCallback, this);
}

int BleManager::gapEventCallback(struct ble_gap_event* event, void* arg) {
    auto* manager = static_cast<BleManager*>(arg);

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        handleDiscovery(event, manager, manager->glasses_);
        break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(tag.data(), "Connected to glasses");
            vTaskDelay(pdMS_TO_TICKS(1500));
            manager->glasses_.initializeDisplay(event->connect.conn_handle);
        } else {
            ESP_LOGE(tag.data(), "Connection failed; status=%d",
                     event->connect.status);
            manager->startScanning();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(tag.data(), "Disconnected from glasses; reason=%d",
                 event->disconnect.reason);
        manager->glasses_.disconnect();
        manager->startScanning();
        break;
    default: break;
    }
    return 0;
}

} // namespace rta
