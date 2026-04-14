#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include "host/ble_hs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the BLE stack and NVS.
 *
 * @param device_name The name that will be advertised.
 * @return int 0 on success, non-zero error code otherwise.
 */
int ble_server_init(const char *device_name);

/**
 * @brief Register GATT services to the BLE server.
 * This must be called BEFORE ble_server_start().
 *
 * @param svcs Pointer to the array of service definitions (must end with empty
 * entry).
 * @return int 0 on success.
 */
int ble_server_register_services(const struct ble_gatt_svc_def *svcs);

/**
 * @brief Start the BLE stack and begin advertising.
 *
 * @return int 0 on success.
 */
int ble_server_start(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVER_H
