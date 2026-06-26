#ifndef RTA_BLE_MANAGER_HPP
#define RTA_BLE_MANAGER_HPP

#include "host/ble_gap.h"
#include <atomic>
#include <string_view>

namespace rta {

class ActiveLook;

/**
 * @brief Class managing BLE scanning and connection to ActiveLook glasses.
 */
class BleManager {
  public:
    explicit BleManager(ActiveLook& glasses,
                        std::atomic<uint16_t>& distanceRef);
    ~BleManager() = default;
    BleManager(const BleManager&) = delete;
    BleManager& operator=(const BleManager&) = delete;
    BleManager(BleManager&&) = delete;
    BleManager& operator=(BleManager&&) = delete;

    // ... (copy/move delete)

    void startScanning() noexcept;
    void disconnectFixed();

    static int gapEventCallback(struct ble_gap_event* event, void* arg);

    [[nodiscard]] uint16_t getGlassesConnHandle() const {
        return glassesConnHandle_;
    }
    [[nodiscard]] uint16_t getFixedConnHandle() const {
        return fixedConnHandle_;
    }
    [[nodiscard]] bool isFixedConnected() const {
        return fixedConnHandle_ != BLE_HS_CONN_HANDLE_NONE;
    }

  private:
    static int onFixedDiscService(uint16_t connHandle,
                                  const struct ble_gatt_error* error,
                                  const struct ble_gatt_svc* service,
                                  void* arg);
    static int onFixedDiscCharacteristic(uint16_t connHandle,
                                         const struct ble_gatt_error* error,
                                         const struct ble_gatt_chr* chr,
                                         void* arg);

    ActiveLook&
        glasses_; // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    std::atomic<uint16_t>&
        distanceRef_; // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    uint16_t glassesConnHandle_{BLE_HS_CONN_HANDLE_NONE};
    uint16_t fixedConnHandle_{BLE_HS_CONN_HANDLE_NONE};

    // Handles for fixed module GATT
    uint16_t fixedDistSvcStartHandle_{0};
    uint16_t fixedDistSvcEndHandle_{0};
    uint16_t fixedDistChrValHandle_{0};
};

} // namespace rta

#endif // RTA_BLE_MANAGER_HPP
