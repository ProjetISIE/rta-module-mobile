#include "BleServer.hpp"
#include "esp_log.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace rta {

namespace {
constexpr std::string_view TAG = "RTA_BLE_SERVER";
}

BleServer* BleServer::instance_ = nullptr;

BleServer::BleServer(std::string_view deviceName) : deviceName_(deviceName) {
    instance_ = this;
}

int BleServer::init() noexcept {
    nimble_port_init();

    // Configure sync callback
    ble_hs_cfg.sync_cb = BleServer::onSync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    return ble_svc_gap_device_name_set(deviceName_.c_str());
}

int BleServer::registerServices(const struct ble_gatt_svc_def* svcs) noexcept {
    if (const int rc = ble_gatts_count_cfg(svcs); rc != 0) {
        return rc;
    }
    return ble_gatts_add_svcs(svcs);
}

int BleServer::start() noexcept {
    nimble_port_freertos_init(BleServer::hostTask);
    return 0;
}

void BleServer::startAdvertising() {
    struct ble_gap_adv_params adv_params{};
    struct ble_hs_adv_fields fields{};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<const uint8_t*>(deviceName_.c_str());
    fields.name_len = static_cast<uint8_t>(deviceName_.length());
    fields.name_is_complete = 1;

    if (int rc = ble_gap_adv_set_fields(&fields); rc != 0) {
        ESP_LOGE(TAG.data(), "Error setting adv fields; rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    if (int rc =
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                              &adv_params, BleServer::gapEventCallback, this);
        rc != 0) {
        ESP_LOGE(TAG.data(), "Error starting advertising; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG.data(), "BLE Advertising started: %s", deviceName_.c_str());
}

int BleServer::gapEventCallback(struct ble_gap_event* event, void* arg) {
    auto* server = static_cast<BleServer*>(arg);
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG.data(), "BLE Connection %s",
                 event->connect.status == 0 ? "established" : "failed");
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG.data(), "BLE Disconnection; reason=%d",
                 event->disconnect.reason);
        server->startAdvertising();
        break;
    }
    return 0;
}

void BleServer::hostTask(void* arg) {
    ESP_LOGI(TAG.data(), "BLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BleServer::onSync(void) {
    if (ble_hs_util_ensure_addr(0) == 0) {
        if (instance_ && instance_->syncCallback_) {
            instance_->syncCallback_();
        } else if (instance_) {
            instance_->startAdvertising();
        }
    }
}

} // namespace rta
