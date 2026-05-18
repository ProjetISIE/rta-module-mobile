#ifndef RTA_BLE_MANAGER_HPP
#define RTA_BLE_MANAGER_HPP

#include "host/ble_gap.h"
#include <string_view>

namespace rta {

class ActiveLook;

/**
 * @brief Class managing BLE scanning and connection to ActiveLook glasses.
 */
class BleManager {
  public:
    explicit BleManager(ActiveLook& glasses);
    ~BleManager() = default;

    // Delete copy and move
    BleManager(const BleManager&) = delete;
    BleManager& operator=(const BleManager&) = delete;
    BleManager(BleManager&&) = delete;
    BleManager& operator=(BleManager&&) = delete;

    /**
     * @brief Starts scanning for ActiveLook (ENGO) glasses.
     */
    void startScanning() noexcept;

    static int gapEventCallback(struct ble_gap_event* event, void* arg);

  private:
    ActiveLook& glasses_;

    // Static pointer for NimBLE callbacks
    static BleManager* instance;
};

} // namespace rta

#endif // RTA_BLE_MANAGER_HPP
