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
namespace {
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
rta::GpsService* sGattGpsService = nullptr;
} // namespace

void setGattGpsService(rta::GpsService* gps) { sGattGpsService = gps; }

// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
static const ble_uuid128_t kGattSvrSvcUuid =
    BLE_UUID128_INIT(0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa,
                     0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00);

// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
static const ble_uuid128_t kGattSvrChrSpdUuid =
    BLE_UUID128_INIT(0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa,
                     0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x01);

// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
static const ble_uuid16_t kGattSvrDscUuid = BLE_UUID16_INIT(0x2901);

namespace {
int gattSvrChrAccess(uint16_t connHandle, uint16_t attrHandle,
                     struct ble_gatt_access_ctxt* ctxt, void* arg);

int gattSvrDscAccess(uint16_t connHandle, uint16_t attrHandle,
                     struct ble_gatt_access_ctxt* ctxt, void* arg);
} // namespace

// Définition des descripteurs
// NOLINTNEXTLINE(modernize-avoid-c-arrays, hicpp-avoid-c-arrays,
// cppcoreguidelines-avoid-c-arrays)
static struct ble_gatt_dsc_def kGattSvrDscs[] = {
    {.uuid = &kGattSvrDscUuid.u,
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
// NOLINTNEXTLINE(modernize-avoid-c-arrays, hicpp-avoid-c-arrays,
// cppcoreguidelines-avoid-c-arrays)
static struct ble_gatt_chr_def kGattSvrChrs[] = {
    {.uuid = &kGattSvrChrSpdUuid.u,
     .access_cb = gattSvrChrAccess,
     .arg = nullptr,
     .descriptors = kGattSvrDscs,
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
     .uuid = &kGattSvrSvcUuid.u,
     .includes = nullptr,
     .characteristics = kGattSvrChrs},
    {.type = 0,
     .uuid = nullptr,
     .includes = nullptr,
     .characteristics = nullptr} // Sentinelle
};

namespace {
int gattSvrChrAccess(uint16_t /*connHandle*/, uint16_t /*attrHandle*/,
                     struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    if (ble_uuid_cmp(ctxt->chr->uuid, &kGattSvrChrSpdUuid.u) == 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
        float speed = 0.0F;
        if (sGattGpsService != nullptr) {
            speed = sGattGpsService->getStatus().speedKmh_;
        }
        return os_mbuf_append(ctxt->om, &speed, sizeof(speed)) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

int gattSvrDscAccess(uint16_t /*connHandle*/, uint16_t /*attrHandle*/,
                     struct ble_gatt_access_ctxt* ctxt, void* /*arg*/) {
    static constexpr std::string_view kDesc = "Vitesse (km/h)";
    return os_mbuf_append(ctxt->om, kDesc.data(), kDesc.size()) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

} // namespace
} // extern "C"
