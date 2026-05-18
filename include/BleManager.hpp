#pragma once

#include "host/ble_gap.h"
#include <string_view>

namespace rta {

class ActiveLook;

/**
 * @brief Class managing BLE scanning and connection to ActiveLook glasses.
 */
class BleManager {
  public:
    BleManager(ActiveLook& glasses);
    ~BleManager() = default;

    // Delete copy and move
    BleManager(const BleManager&) = delete;
    BleManager& operator=(const BleManager&) = delete;

    /**
     * @brief Starts scanning for ActiveLook (ENGO) glasses.
     */
    void startScanning() noexcept;

  private:
    static int gapEventCallback(struct ble_gap_event* event, void* arg);

    ActiveLook& glasses_;

    // Static pointer for NimBLE callbacks
    static BleManager* instance_;
};

} // namespace rta
