#include "ble_server.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string>
#include <string_view>

/**
 * @file ble_server.cpp
 * @brief Implémentation du serveur BLE en C++23.
 */

namespace {
constexpr std::string_view TAG = "RTA_BLE_SERVER";
std::string ble_device_name = "RTA_MOBILE";
} // namespace

extern "C" {

static int ble_gap_event(struct ble_gap_event *event, void *arg);

void ble_app_advertise() {
  struct ble_gap_adv_params adv_params{};
  struct ble_hs_adv_fields fields{};

  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = reinterpret_cast<const uint8_t *>(ble_device_name.c_str());
  fields.name_len = static_cast<uint8_t>(ble_device_name.length());
  fields.name_is_complete = 1;

  if (int rc = ble_gap_adv_set_fields(&fields); rc != 0) {
    ESP_LOGE(TAG.data(), "Erreur fields; rc=%d", rc);
    return;
  }

  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  if (int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                                 &adv_params, ble_gap_event, nullptr);
      rc != 0) {
    ESP_LOGE(TAG.data(), "Erreur annonce; rc=%d", rc);
    return;
  }
  ESP_LOGI(TAG.data(), "Annonce BLE démarrée : %s", ble_device_name.c_str());
}

static void ble_on_sync() {
  if (ble_hs_util_ensure_addr(0) == 0) {
    ble_app_advertise();
  }
}

static int ble_gap_event(struct ble_gap_event *event, void *) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG.data(), "Connexion BLE %s",
             event->connect.status == 0 ? "établie" : "échouée");
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG.data(), "Déconnexion BLE; raison=%d",
             event->disconnect.reason);
    ble_app_advertise();
    break;
  }
  return 0;
}

static void ble_host_task(void *) {
  ESP_LOGI(TAG.data(), "Tâche hôte BLE démarrée");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

int ble_server_init(const char *device_name) {
  if (device_name) {
    ble_device_name = device_name;
  }

  nimble_port_init();
  ble_hs_cfg.sync_cb = ble_on_sync;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  return ble_svc_gap_device_name_set(ble_device_name.c_str());
}

int ble_server_register_services(const struct ble_gatt_svc_def *svcs) {
  if (int rc = ble_gatts_count_cfg(svcs); rc != 0)
    return rc;
  return ble_gatts_add_svcs(svcs);
}

int ble_server_start() {
  nimble_port_freertos_init(ble_host_task);
  return 0;
}

} // extern "C"
