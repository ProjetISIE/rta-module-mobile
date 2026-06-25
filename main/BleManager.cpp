#include "BleManager.hpp"
#include "ActiveLook.hpp"

#include "EspNowMobile.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <cstring>
#include <string_view>

namespace rta {

namespace {
constexpr std::string_view tag = "RTA_BLE_MANAGER";
static bool connection_pending = false;
static bool connecting_to_glasses = false;

const ble_uuid128_t fixed_svc_uuid =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56);

const ble_uuid128_t fixed_chr_uuid =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56);

void handleDiscovery(struct ble_gap_event* event, BleManager* manager) {
    if (connection_pending) return;

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
            connecting_to_glasses = true;
        } else if (name.starts_with("RTA_FIXE") &&
                   manager->getFixedConnHandle() == BLE_HS_CONN_HANDLE_NONE) {
            if (!rta::espnow::isActive()) {
                ESP_LOGI(
                    tag.data(),
                    "RTA_FIXE found (ESP-NOW inactive), connecting via BLE...");
                found = true;
                connecting_to_glasses = false;
            } else {
                // Ignore BLE advertisement because ESP-NOW is active
            }
        }

        if (found) {
            connection_pending = true;

            // Stop scanning explicitly before connecting to avoid EBUSY (rc=15)
            // on some NimBLE versions/configurations
            ble_gap_disc_cancel();

            // Infer the best own address type (public or random)
            uint8_t own_addr_type;
            int rc = ble_hs_id_infer_auto(0, &own_addr_type);
            if (rc != 0) {
                ESP_LOGE(tag.data(), "Error inferring addr type; rc=%d", rc);
                connection_pending = false;
                manager->startScanning();
                return;
            }

            struct ble_gap_conn_params conn_params{};
            conn_params.scan_itvl = 16;
            conn_params.scan_window = 16;
            conn_params.itvl_min = 24;
            conn_params.itvl_max = 40;
            conn_params.supervision_timeout = 500;

            rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000,
                                 &conn_params, BleManager::gapEventCallback,
                                 manager);
            if (rc != 0) {
                if (rc != BLE_HS_EALREADY) {
                    ESP_LOGE(tag.data(), "Error connecting; rc=%d", rc);
                    connection_pending = false;
                    manager->startScanning(); // Restart if failed
                }
            }
        }
    }
}
} // namespace

BleManager::BleManager(ActiveLook& glasses, std::atomic<uint16_t>& distanceRef)
    : glasses_(glasses), distanceRef_(distanceRef) {}

void BleManager::startScanning() noexcept {
    if (connection_pending) return;

    struct ble_gap_disc_params disc_params{};
    disc_params.passive = 0;
    disc_params.filter_duplicates = 0;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                          BleManager::gapEventCallback, this);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(tag.data(), "Failed to start scanning; rc=%d", rc);
    }
}

void BleManager::disconnectFixed() {
    if (fixedConnHandle_ != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(fixedConnHandle_, BLE_ERR_REM_USER_CONN_TERM);
    }
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
        if (ble_uuid_cmp(&chr->uuid.u, &fixed_chr_uuid.u) == 0) {
            manager->fixedDistChrValHandle_ = chr->val_handle;
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (manager->fixedDistChrValHandle_ != 0) {
            ESP_LOGI(tag.data(), "Subscribing to RTA_FIXE notifications");
            uint8_t value[2] = {0x01, 0x00};
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
        connection_pending = false;
        if (event->connect.status == 0) {
            if (connecting_to_glasses) {
                ESP_LOGI(tag.data(), "Connected to glasses");
                manager->glassesConnHandle_ = event->connect.conn_handle;
                // Move initialization to a safe place if possible,
                // but for now just call it (ActiveLook should handle handles)
                manager->glasses_.initializeDisplay(event->connect.conn_handle);
            } else {
                ESP_LOGI(tag.data(), "Connected to RTA_FIXE");
                manager->fixedConnHandle_ = event->connect.conn_handle;
                ble_gattc_disc_svc_by_uuid(
                    manager->fixedConnHandle_, &fixed_svc_uuid.u,
                    BleManager::onFixedDiscService, manager);
            }

            // Re-start scan if needed after a small delay to avoid congestion
            if (manager->glassesConnHandle_ == BLE_HS_CONN_HANDLE_NONE ||
                manager->fixedConnHandle_ == BLE_HS_CONN_HANDLE_NONE) {
                manager->startScanning();
            }
        } else {
            ESP_LOGE(tag.data(), "Connect failed; status=%d",
                     event->connect.status);
            manager->startScanning();
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == manager->fixedConnHandle_ &&
            event->notify_rx.attr_handle == manager->fixedDistChrValHandle_) {
            if (OS_MBUF_PKTLEN(event->notify_rx.om) == sizeof(uint16_t)) {
                uint16_t dist = 0;
                ble_hs_mbuf_to_flat(event->notify_rx.om, &dist,
                                    sizeof(uint16_t), nullptr);
                manager->distanceRef_.store(dist, std::memory_order_relaxed);
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        connection_pending = false;
        if (event->disconnect.conn.conn_handle == manager->glassesConnHandle_) {
            ESP_LOGI(tag.data(), "Disconnected from glasses; reason=%d",
                     event->disconnect.reason);
            manager->glassesConnHandle_ = BLE_HS_CONN_HANDLE_NONE;
            manager->glasses_.disconnect();
        } else if (event->disconnect.conn.conn_handle ==
                   manager->fixedConnHandle_) {
            ESP_LOGI(tag.data(), "Disconnected from RTA_FIXE; reason=%d",
                     event->disconnect.reason);
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
