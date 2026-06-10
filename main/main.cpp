#include "ActiveLook.hpp"
#include "BleManager.hpp"
#include "BleServer.hpp"
#include "GpsService.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "host/ble_gatt.h" // For ble_gatt_svc_def
#include <cstdio>
#include <memory>
#include <print>

namespace {
constexpr const char* tag = "RTA";

class AppContext {
  public:
    AppContext() : server_("RTA_MOBILE"), manager_(glasses_) {}

    rta::ActiveLook& glasses() { return glasses_; }
    rta::BleServer& server() { return server_; }
    rta::BleManager& manager() { return manager_; }

  private:
    rta::ActiveLook glasses_;
    rta::BleServer server_;
    rta::BleManager manager_;
};

struct DisplayState {
    bool wasConnected_{false};
    bool lastFix_{false};
    double lastLat_{0.0};
    double lastLon_{0.0};
    bool forceUpdate_{false};
    int rtaOkTimer_{0};
};

// GATT services defined in gps_gatt_def.cpp
extern "C" struct ble_gatt_svc_def gpsGattSvcs[];
extern "C" float gattReceivedDistance;

void updateGlassesDisplay(AppContext& context, const rta::GpsStatus& status,
                          DisplayState& state) {
    const bool connected = context.glasses().isConnected();

    if (connected) {
        if (!state.wasConnected_) {
            state.rtaOkTimer_ = 5;
            state.wasConnected_ = true;
            state.forceUpdate_ = true;
        }

        if (state.rtaOkTimer_ > 0) {
            state.rtaOkTimer_--;
        } else {
            const bool fixChanged = (status.fix_ != state.lastFix_);
            const bool posChanged =
                status.fix_ && (status.latitude_ != state.lastLat_ ||
                                status.longitude_ != state.lastLon_);

            if (state.forceUpdate_ || fixChanged || posChanged) {
                if (status.fix_) {
                    context.glasses().displayCoordinates(status.latitude_,
                                                         status.longitude_);
                    state.lastLat_ = status.latitude_;
                    state.lastLon_ = status.longitude_;
                } else {
                    context.glasses().displayGpsWait();
                }
                state.lastFix_ = status.fix_;
                state.forceUpdate_ = false;
            }
        }
    } else {
        state.wasConnected_ = false;
        state.rtaOkTimer_ = 0;
        state.lastFix_ = false;
        state.forceUpdate_ = false;
    }
}

void processAndDisplayTask(void* pvParameters) {
    auto* context = static_cast<AppContext*>(pvParameters);
    auto& gps = rta::GpsService::instance();
    DisplayState state;

    while (true) {
        const auto status = gps.getStatus();

        updateGlassesDisplay(*context, status, state);

        // Formatted console output via std::print (C++23)
        if (status.fix_) {
            std::print("\r[FIX OK] Sat: {:2d} | Speed: {:6.2f} km/h | Dist: "
                       "{:6.2f}m | "
                       "L: {:9.6f}, {:9.6f}   ",
                       status.satellites_, status.speedKmh_,
                       gattReceivedDistance, status.latitude_,
                       status.longitude_);
        } else {
            std::print("\r[WAITING] Dist: {:6.2f}m | No fix data parsed yet... "
                       "         ",
                       gattReceivedDistance);
        }
        (void)std::fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

} // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void app_main() {
    static auto context = std::make_unique<AppContext>();

    // Initialisation du serveur BLE
    if (context->server().init() != 0) {
        ESP_LOGE(tag, "Failed to initialize BLE server");
        return;
    }

    // Set sync callback to start both advertising and scanning
    context->server().setSyncCallback([]() {
        context->server().startAdvertising();
        context->manager().startScanning();
        ESP_LOGI(tag, "BLE synchronized: Advertising and Scanning started");
    });

    // Start GPS Service
    rta::GpsService::instance().start();

    // Register services and start BLE stack
    if (context->server().registerServices(gpsGattSvcs) != 0) {
        ESP_LOGE(tag, "Failed to register BLE services");
        return;
    }

    if (context->server().start() != 0) {
        ESP_LOGE(tag, "Failed to start BLE stack");
        return;
    }

    // Create display task
    xTaskCreate(processAndDisplayTask, "display_task", 4096, context.get(), 5,
                nullptr);
}
