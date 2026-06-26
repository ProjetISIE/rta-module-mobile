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
constexpr std::string_view kTag = "RTA_BLE_MANAGER";
bool connectionPending = false;
bool connectingToGlasses = false;

const ble_uuid128_t kFixedSvcUuid =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56);

const ble_uuid128_t kFixedChrUuid =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56);

void handleDiscovery(struct ble_gap_event* event, BleManager* manager) {
    if (connectionPending) {
        return;
    }

    struct ble_hs_adv_fields fields{};
    ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

    if (fields.name != nullptr) {
        std::string_view name(reinterpret_cast<const char*>(fields.name),
                              fields.name_len);

        bool found = false;
        if (name.starts_with("ENGO") &&
            manager->getGlassesConnHandle() == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(kTag.data(), "ENGO found, connecting...");
            found = true;
            connectingToGlasses = true;
        } else if (name.starts_with("RTA_FIXE") &&
                   manager->getFixedConnHandle() == BLE_HS_CONN_HANDLE_NONE) {
            if (!rta::espnow::isActive()) {
                ESP_LOGI(
                    kTag.data(),
                    "RTA_FIXE found (ESP-NOW inactive), connecting via BLE...");
                found = true;
                connectingToGlasses = false;
            } else {
                // Ignore BLE advertisement because ESP-NOW is active
            }
        }

        if (found) {
            connectionPending = true;

            // Stop scanning explicitly before connecting to avoid EBUSY (rc=15)
            // on some NimBLE versions/configurations
            ble_gap_disc_cancel();

            // Infer the best own address type (public or random)
            uint8_t ownAddrType = 0;
            int returnCode = ble_hs_id_infer_auto(
                0, &ownAddrType); // NOLINT(readability-identifier-length)
            if (returnCode != 0) {
                ESP_LOGE(kTag.data(), "Error inferring addr type; rc=%d",
                         returnCode);
                connectionPending = false;
                manager->startScanning();
                return;
            }

            struct ble_gap_conn_params connParams{};
            // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
            connParams.scan_itvl = 16;
            connParams.scan_window = 16;
            connParams.itvl_min = 24;
            connParams.itvl_max = 40;
            connParams.supervision_timeout = 500;

            returnCode = ble_gap_connect(ownAddrType, &event->disc.addr, 30000,
                                         &connParams,
                                         BleManager::gapEventCallback, manager);
            // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
            if (returnCode != 0) {
                if (returnCode != BLE_HS_EALREADY) {
                    ESP_LOGE(kTag.data(), "Error connecting; rc=%d",
                             returnCode);
                    connectionPending = false;
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
    if (connectionPending) {
        return;
    }

    struct ble_gap_disc_params discParams{};
    discParams.passive = 0;
    discParams.filter_duplicates = 0;
    int returnCode =
        ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                     &discParams, // NOLINT(readability-identifier-length)
                     BleManager::gapEventCallback, this);
    if (returnCode != 0 && returnCode != BLE_HS_EALREADY) {
        ESP_LOGE(kTag.data(), "Failed to start scanning; rc=%d", returnCode);
    }
}

void BleManager::
    disconnectFixed() { // NOLINT(readability-make-member-function-const)
    if (fixedConnHandle_ != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(fixedConnHandle_, BLE_ERR_REM_USER_CONN_TERM);
    }
}

int BleManager::onFixedDiscService(uint16_t connHandle,
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
                connHandle, manager->fixedDistSvcStartHandle_,
                manager->fixedDistSvcEndHandle_, &kFixedChrUuid.u,
                BleManager::onFixedDiscCharacteristic, manager);
        }
        return 0;
    }
    return 0;
}

int BleManager::onFixedDiscCharacteristic(uint16_t connHandle,
                                          const struct ble_gatt_error* error,
                                          const struct ble_gatt_chr* chr,
                                          void* arg) {
    auto* manager = static_cast<BleManager*>(arg);
    if (error->status == 0) {
        if (ble_uuid_cmp(&chr->uuid.u, &kFixedChrUuid.u) == 0) {
            manager->fixedDistChrValHandle_ = chr->val_handle;
        }
        return 0;
    }
    if (error->status == BLE_HS_EDONE) {
        if (manager->fixedDistChrValHandle_ != 0) {
            ESP_LOGI(kTag.data(), "Subscribing to RTA_FIXE notifications");
            std::array<uint8_t, 2> value = {0x01, 0x00};
            ble_gattc_write_flat(connHandle,
                                 manager->fixedDistChrValHandle_ + 1,
                                 value.data(), value.size(), nullptr, nullptr);
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
        connectionPending = false;
        if (event->connect.status == 0) {
            if (connectingToGlasses) {
                ESP_LOGI(kTag.data(), "Connected to glasses");
                manager->glassesConnHandle_ = event->connect.conn_handle;
                // Move initialization to a safe place if possible,
                // but for now just call it (ActiveLook should handle handles)
                manager->glasses_.initializeDisplay(event->connect.conn_handle);
            } else {
                ESP_LOGI(kTag.data(), "Connected to RTA_FIXE");
                manager->fixedConnHandle_ = event->connect.conn_handle;
                ble_gattc_disc_svc_by_uuid(
                    manager->fixedConnHandle_, &kFixedSvcUuid.u,
                    BleManager::onFixedDiscService, manager);
            }

            // Re-start scan if needed after a small delay to avoid congestion
            if (manager->glassesConnHandle_ == BLE_HS_CONN_HANDLE_NONE ||
                manager->fixedConnHandle_ == BLE_HS_CONN_HANDLE_NONE) {
                manager->startScanning();
            }
        } else {
            ESP_LOGE(kTag.data(), "Connect failed; status=%d",
                     event->connect.status);
            manager->startScanning();
        }
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == manager->fixedConnHandle_ &&
            event->notify_rx.attr_handle == manager->fixedDistChrValHandle_) {
            // NOLINTNEXTLINE(bugprone-casting-through-void,cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
            if (OS_MBUF_PKTLEN(event->notify_rx.om) == sizeof(uint16_t)) {
                uint16_t dist = 0;
                ble_hs_mbuf_to_flat(event->notify_rx.om, &dist,
                                    sizeof(uint16_t), nullptr);
                manager->distanceRef_.store(dist, std::memory_order_relaxed);
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        connectionPending = false;
        if (event->disconnect.conn.conn_handle == manager->glassesConnHandle_) {
            ESP_LOGI(kTag.data(), "Disconnected from glasses; reason=%d",
                     event->disconnect.reason);
            manager->glassesConnHandle_ = BLE_HS_CONN_HANDLE_NONE;
            manager->glasses_.disconnect();
        } else if (event->disconnect.conn.conn_handle ==
                   manager->fixedConnHandle_) {
            ESP_LOGI(kTag.data(), "Disconnected from RTA_FIXE; reason=%d",
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
