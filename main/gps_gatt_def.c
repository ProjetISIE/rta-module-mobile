#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

// Bridge function from GpsService.cpp
extern float gps_service_get_speed();

uint16_t gatt_svr_chr_spd_val_handle;

static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad,
                     0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef);

static const ble_uuid128_t gatt_svr_chr_spd_uuid =
    BLE_UUID128_INIT(0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa,
                     0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00);

static const ble_uuid16_t gatt_svr_dsc_uuid = BLE_UUID16_INIT(0x2901);

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ble_uuid_cmp(ctxt->chr->uuid, &gatt_svr_chr_spd_uuid.u) == 0) {
    float speed = gps_service_get_speed();
    return os_mbuf_append(ctxt->om, &speed, sizeof(speed)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
  const char *desc = "Speed (km/h)";
  return os_mbuf_append(ctxt->om, desc, strlen(desc));
}

const struct ble_gatt_svc_def gps_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &gatt_svr_chr_spd_uuid.u,
                    .access_cb = gatt_svr_chr_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &gatt_svr_chr_spd_val_handle,
                    .descriptors =
                        (struct ble_gatt_dsc_def[]){
                            {
                                .uuid = &gatt_svr_dsc_uuid.u,
                                .att_flags = BLE_ATT_F_READ,
                                .access_cb = gatt_svr_dsc_access,
                            },
                            {0}},
                },
                {0}},
    },
    {0}};
