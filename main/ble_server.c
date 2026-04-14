#include "ble_server.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "RTA_MOBILE_MOD";
static char ble_device_name[32] = "RTA_MOBILE";

static int ble_gap_event(struct ble_gap_event *event, void *arg);

void ble_app_advertise(void) {
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  int rc;

  memset(&fields, 0, sizeof(fields));
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = (uint8_t *)ble_device_name;
  fields.name_len = strlen(ble_device_name);
  fields.name_is_complete = 1;

  // In a generic server, we might want to allow setting service UUIDs in adv
  // data, but for now let's keep it simple.
  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
    return;
  }

  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    return;
  }
  ESP_LOGI(TAG, "BLE Advertising started");
}

static void ble_on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);
  ble_app_advertise();
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "BLE Connection %s",
             event->connect.status == 0 ? "established" : "failed");
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "BLE Disconnection; reason=%d", event->disconnect.reason);
    ble_app_advertise(); // Restart advertising
    break;
  }
  return 0;
}

static void ble_host_task(void *param) {
  ESP_LOGI(TAG, "BLE Host Task Started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

int ble_server_init(const char *device_name) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  strncpy(ble_device_name, device_name, sizeof(ble_device_name) - 1);

  nimble_port_init();
  ble_hs_cfg.sync_cb = ble_on_sync;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  int rc = ble_svc_gap_device_name_set(ble_device_name);
  return rc;
}

int ble_server_register_services(const struct ble_gatt_svc_def *svcs) {
  int rc = ble_gatts_count_cfg(svcs);
  if (rc != 0)
    return rc;

  rc = ble_gatts_add_svcs(svcs);
  return rc;
}

int ble_server_start(void) {
  nimble_port_freertos_init(ble_host_task);
  return 0;
}
