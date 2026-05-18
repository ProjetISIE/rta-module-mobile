#pragma once

#include "host/ble_hs.h"
#include <functional>
#include <string>
#include <string_view>

namespace rta {

/**
 * @brief Class managing the BLE Server functionality.
 */
class BleServer {
public:
  using SyncCallback = std::function<void()>;

  BleServer(std::string_view deviceName = "RTA_MOBILE");
  ~BleServer() = default;

  // Delete copy and move for safety
  BleServer(const BleServer &) = delete;
  BleServer &operator=(const BleServer &) = delete;
  BleServer(BleServer &&) = delete;
  BleServer &operator=(BleServer &&) = delete;

  /**
   * @brief Initializes the BLE stack.
   * @return 0 on success, error code otherwise.
   */
  int init();

  /**
   * @brief Registers GATT services.
   * @param svcs Pointer to service definition array (must end with empty
   * entry).
   * @return 0 on success.
   */
  int registerServices(const struct ble_gatt_svc_def *svcs);

  /**
   * @brief Starts the BLE stack and host task.
   * @return 0 on success.
   */
  int start();

  /**
   * @brief Starts advertising.
   */
  void startAdvertising();

  /**
   * @brief Sets the sync callback.
   * @param cb Callback function.
   */
  void setSyncCallback(SyncCallback cb) { syncCallback_ = std::move(cb); }

private:
  static int gapEventCallback(struct ble_gap_event *event, void *arg);
  static void hostTask(void *arg);
  static void onSync(void);

  std::string deviceName_;
  SyncCallback syncCallback_;

  // Static pointer for NimBLE callbacks
  static BleServer *instance_;
};

} // namespace rta
