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

// Service and Characteristic UUIDs for RTA_FIXE
const ble_uuid128_t fixed_svc_uuid =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

const ble_uuid128_t fixed_chr_uuid =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

void handleDiscovery(struct ble_gap_event* event, BleManager* manager) {
    struct ble_hs_adv_fields fields{};
    ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

    if (fields.name != nullptr) {
        std::string_view name(reinterpret_cast<const char*>(fields.name),
                              fields.name_len);

        bool found = false;
        if (name.starts_with("ENGO") &&
            manager->getGlassesConnHandle() == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(tag.data(), "ENGO found, connecting...");
            found = true;
        } else if (name == "RTA_FIXE" &&
                   manager->getFixedConnHandle() == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(tag.data(), "RTA_FIXE found, connecting...");
            found = true;
        }

        if (found) {
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
            }
        }
    }
}
} // namespace

BleManager* BleManager::instance = nullptr;
extern "C" float gattReceivedDistance;

BleManager::BleManager(ActiveLook& glasses) : glasses_(glasses) {
    instance = this;
}

void BleManager::startScanning() noexcept {
    struct ble_gap_disc_params disc_params{};
    disc_params.passive = 1;
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                 BleManager::gapEventCallback, this);
}

int BleManager::onFixedDiscService(uint16_t conn_handle,
                                   const struct ble_gatt_error* error,
                                   const struct ble_gatt_svc* service,
                                   void* arg) {
    auto* manager = static_cast<BleManager*>(arg);
    if (error->status == 0) {
        manager->fixedDistSvcStartHandle_ = service->start_handle;
        manager->fixedDistSvcEndHandle_ = service->end_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (manager->fixedDistSvcStartHandle_ != 0) {
            ble_gattc_disc_chrs_by_uuid(
                conn_handle, manager->fixedDistSvcStartHandle_,
                manager->fixedDistSvcEndHandle_, &fixed_chr_uuid.u,
                BleManager::onFixedDiscCharacteristic, manager);
        }
        return 0;
    }
    return 0;
}

int BleManager::onFixedDiscCharacteristic(uint16_t conn_handle,
                                          const struct ble_gatt_error* error,
                                          const struct ble_gatt_chr* chr,
                                          void* arg) {
    auto* manager = static_cast<BleManager*>(arg);
    if (error->status == 0) {
        manager->fixedDistChrValHandle_ = chr->val_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (manager->fixedDistChrValHandle_ != 0) {
            ESP_LOGI(tag.data(),
                     "Subscribing to RTA_FIXE distance notifications");
            uint8_t value[2] = {0x01, 0x00}; // Enable notifications
            ble_gattc_write_flat(conn_handle,
                                 manager->fixedDistChrValHandle_ + 1, value,
                                 sizeof(value), nullptr, nullptr);
        }
        return 0;
    }
    return 0;
}

int BleManager::gapEventCallback(struct ble_gap_event* event, void* arg) {
    auto* manager = static_cast<BleManager*>(arg);

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: handleDiscovery(event, manager); break;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            // Check if it's the fixed module or glasses
            // For simplicity, we use the name during discovery to tag,
            // but here we can check peer address or just rely on GATT discovery
            // We'll try to discover service to identify
            ble_gattc_disc_svc_by_uuid(event->connect.conn_handle,
                                       &fixed_svc_uuid.u,
                                       BleManager::onFixedDiscService, manager);

            // If it's glasses, manager will handle via onFixedDiscService
            // failure or simply time out Better: glasses handle is usually the
            // first one or we track address. Let's assume for now any
            // connection that doesn't have the fixed service is glasses.
            // Simplified: first connect is glasses, second is fixed (or vice
            // versa) Actual logic: glasses connect triggers initializeDisplay.
            // We'll check if glasses handle is empty.
            if (manager->glassesConnHandle_ == BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGI(tag.data(), "Connected to glasses");
                manager->glassesConnHandle_ = event->connect.conn_handle;
                vTaskDelay(pdMS_TO_TICKS(1500));
                manager->glasses_.initializeDisplay(event->connect.conn_handle);
            } else {
                ESP_LOGI(tag.data(), "Connected to RTA_FIXE");
                manager->fixedConnHandle_ = event->connect.conn_handle;
            }
        } else {
            ESP_LOGE(tag.data(), "Connection failed; status=%d",
                     event->connect.status);
            manager->startScanning();
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == manager->fixedConnHandle_ &&
            event->notify_rx.attr_handle == manager->fixedDistChrValHandle_) {
            if (OS_MBUF_PKTLEN(event->notify_rx.om) == sizeof(float)) {
                ble_hs_mbuf_to_flat(event->notify_rx.om, &gattReceivedDistance,
                                    sizeof(float), nullptr);
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        if (event->disconnect.conn.conn_handle == manager->glassesConnHandle_) {
            ESP_LOGI(tag.data(), "Disconnected from glasses");
            manager->glassesConnHandle_ = BLE_HS_CONN_HANDLE_NONE;
            manager->glasses_.disconnect();
        } else if (event->disconnect.conn.conn_handle ==
                   manager->fixedConnHandle_) {
            ESP_LOGI(tag.data(), "Disconnected from RTA_FIXE");
            manager->fixedConnHandle_ = BLE_HS_CONN_HANDLE_NONE;
            manager->fixedDistChrValHandle_ = 0;
        }
        manager->startScanning();
        break;
    default: break;
    }
    return 0;
}

} // namespace rta
