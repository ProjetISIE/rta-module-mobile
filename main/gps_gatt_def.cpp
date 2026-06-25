#include "GpsService.hpp"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include <array>
#include <string_view>

/**
 * @file gps_gatt_def.cpp
 * @brief Définition du profil GATT en C++ pur.
 */

extern "C" {

uint16_t gattSvrChrSpdValHandle;

static const ble_uuid128_t GATT_SVR_SVC_UUID =
    BLE_UUID128_INIT(0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef, 0xde, 0xad,
                     0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef);

static const ble_uuid128_t GATT_SVR_CHR_SPD_UUID =
    BLE_UUID128_INIT(0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa,
                     0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00);

static const ble_uuid16_t GATT_SVR_DSC_UUID = BLE_UUID16_INIT(0x2901);

static int gattSvrChrAccess(uint16_t connHandle, uint16_t attrHandle,
                            struct ble_gatt_access_ctxt* ctxt, void* arg);

static int gattSvrDscAccess(uint16_t connHandle, uint16_t attrHandle,
                            struct ble_gatt_access_ctxt* ctxt, void* arg);

// Définition des descripteurs
static const struct ble_gatt_dsc_def GATT_SVR_DSCS[] = {
    {.uuid = &GATT_SVR_DSC_UUID.u,
     .att_flags = BLE_ATT_F_READ,
     .min_key_size = 0,
     .access_cb = gattSvrDscAccess,
     .arg = nullptr},
    {.uuid = nullptr,
     .att_flags = 0,
     .min_key_size = 0,
     .access_cb = nullptr,
     .arg = nullptr} // Sentinelle
};

// Définition des caractéristiques
static const struct ble_gatt_chr_def GATT_SVR_CHRS[] = {
    {.uuid = &GATT_SVR_CHR_SPD_UUID.u,
     .access_cb = gattSvrChrAccess,
     .arg = nullptr,
     .descriptors = const_cast<struct ble_gatt_dsc_def*>(GATT_SVR_DSCS),
     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
     .min_key_size = 0,
     .val_handle = &gattSvrChrSpdValHandle,
     .cpfd = nullptr},
    {.uuid = nullptr,
     .access_cb = nullptr,
     .arg = nullptr,
     .descriptors = nullptr,
     .flags = 0,
     .min_key_size = 0,
     .val_handle = nullptr,
     .cpfd = nullptr} // Sentinelle
};

// Définition finale des services
struct ble_gatt_svc_def gpsGattSvcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &GATT_SVR_SVC_UUID.u,
     .includes = nullptr,
     .characteristics = const_cast<struct ble_gatt_chr_def*>(GATT_SVR_CHRS)},
    {.type = 0,
     .uuid = nullptr,
     .includes = nullptr,
     .characteristics = nullptr} // Sentinelle
};

static int gattSvrChrAccess(uint16_t /*connHandle*/, uint16_t /*attrHandle*/,
                            struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    if (ble_uuid_cmp(ctxt->chr->uuid, &GATT_SVR_CHR_SPD_UUID.u) == 0) {
        const float speed = rta::GpsService::instance().getStatus().speedKmh_;
        return os_mbuf_append(ctxt->om, &speed, sizeof(speed)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int gattSvrDscAccess(uint16_t /*connHandle*/, uint16_t /*attrHandle*/,
                            struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    static constexpr std::string_view desc = "Vitesse (km/h)";
    return os_mbuf_append(ctxt->om, desc.data(), desc.size()) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

} // extern "C"
