#include "BleServer.hpp"
#include "esp_log.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace rta {

namespace {
constexpr std::string_view kTag = "RTA_BLE_SERVER";
}

BleServer* BleServer::instance = nullptr;

BleServer::BleServer(std::string_view deviceName) : deviceName_(deviceName) {
    instance = this;
}

int BleServer::init() noexcept {
    nimble_port_init();

    // Configure sync callback
    ble_hs_cfg.sync_cb = BleServer::onSync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    return ble_svc_gap_device_name_set(deviceName_.c_str());
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
int BleServer::registerServices(const struct ble_gatt_svc_def* svcs) noexcept {
    if (const int returnCode = ble_gatts_count_cfg(svcs); returnCode != 0) {
        return returnCode;
    }
    return ble_gatts_add_svcs(svcs);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
int BleServer::start() noexcept {
    nimble_port_freertos_init(BleServer::hostTask);
    return 0;
}

void BleServer::startAdvertising() {
    struct ble_gap_adv_params advParams{};
    struct ble_hs_adv_fields fields{};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<const uint8_t*>(deviceName_.c_str());
    fields.name_len = static_cast<uint8_t>(deviceName_.length());
    fields.name_is_complete = 1;

    if (const int returnCode = ble_gap_adv_set_fields(&fields);
        returnCode != 0) {
        ESP_LOGE(kTag.data(), "Error setting adv fields; rc=%d", returnCode);
        return;
    }

    advParams.conn_mode = BLE_GAP_CONN_MODE_UND;
    advParams.disc_mode = BLE_GAP_DISC_MODE_GEN;

    if (const int returnCode =
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                              &advParams, BleServer::gapEventCallback, this);
        returnCode != 0) {
        ESP_LOGE(kTag.data(), "Error starting advertising; rc=%d", returnCode);
        return;
    }
    ESP_LOGI(kTag.data(), "BLE Advertising started: %s", deviceName_.c_str());
}

int BleServer::gapEventCallback(struct ble_gap_event* event, void* arg) {
    auto* server = static_cast<BleServer*>(arg);
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(kTag.data(), "BLE Connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(kTag.data(), "BLE Disconnection; reason=%d",
                 event->disconnect.reason);
        server->startAdvertising();
        break;
    default: break;
    }
    return 0;
}

void BleServer::hostTask([[maybe_unused]] void* arg) {
    ESP_LOGI(kTag.data(), "BLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BleServer::onSync() {
    if (ble_hs_util_ensure_addr(0) == 0) {
        if (instance != nullptr && instance->syncCallback_) {
            instance->syncCallback_();
        } else if (instance != nullptr) {
            instance->startAdvertising();
        }
    }
}

} // namespace rta
