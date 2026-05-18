#include "ble_server.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

/**
 * @file ble_server.c
 * @brief Implémentation du serveur BLE pour l'application RTA.
 */

static const char *TAG = "RTA_BLE_SERVER";
static char ble_device_name[32] = "RTA_MOBILE";

static int ble_gap_event(struct ble_gap_event *event, void *arg);

/**
 * @brief Démarre l'annonce publicitaire BLE.
 */
void ble_app_advertise(void) {
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  int rc;

  memset(&fields, 0, sizeof(fields));
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = (uint8_t *)ble_device_name;
  fields.name_len = strlen(ble_device_name);
  fields.name_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "Erreur lors de la configuration des champs d'annonce; rc=%d",
             rc);
    return;
  }

  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "Erreur lors du démarrage de l'annonce; rc=%d", rc);
    return;
  }
  ESP_LOGI(TAG, "Annonce BLE démarrée");
}

static void ble_on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);
  ble_app_advertise();
}

/**
 * @brief Gestionnaire d'événements GAP pour le serveur.
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "Connexion BLE %s",
             event->connect.status == 0 ? "établie" : "échouée");
    break;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "Déconnexion BLE; raison=%d", event->disconnect.reason);
    ble_app_advertise(); // Redémarrer l'annonce après déconnexion
    break;
  }
  return 0;
}

static void ble_host_task(void *param) {
  ESP_LOGI(TAG, "Tâche hôte BLE démarrée");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

int ble_server_init(const char *device_name) {
  // Le NVS est déjà initialisé dans app_main, mais on garde une sécurité si
  // nécessaire esp_err_t ret = nvs_flash_init(); ...

  if (device_name) {
    strncpy(ble_device_name, device_name, sizeof(ble_device_name) - 1);
    ble_device_name[sizeof(ble_device_name) - 1] = '\0';
  }

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
