#include "ActiveLook.hpp"
#include "BleManager.hpp"
#include "BleServer.hpp"
#include "GpsService.hpp"
#include "LoggerService.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include "host/ble_gatt.h" // For ble_gatt_svc_def
#include "nvs_flash.h"
#include <cstdio>
#include <memory>
#include <optional>
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
    float lastSpeedKmh_{0.0f};
    uint16_t lastDistance_{0xFFFF};
    bool forceUpdate_{false};
    int rtaOkTimer_{0};
};

// GATT services defined in gps_gatt_def.cpp
extern "C" struct ble_gatt_svc_def gpsGattSvcs[];
extern "C" uint16_t gattReceivedDistance;

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
            const bool speedChanged =
                status.fix_ && (status.speedKmh_ != state.lastSpeedKmh_);
            const bool distChanged =
                (gattReceivedDistance != state.lastDistance_);

            if (state.forceUpdate_ || fixChanged || speedChanged ||
                distChanged) {
                std::optional<double> speedVal;
                if (status.fix_) {
                    speedVal = status.speedKmh_;
                }
                std::optional<double> distVal;
                if (gattReceivedDistance != 0xFFFF) {
                    distVal =
                        static_cast<double>(gattReceivedDistance) / 1000.0;
                }

                context.glasses().displaySpeedAndDistance(speedVal, distVal);

                state.lastFix_ = status.fix_;
                state.lastSpeedKmh_ = status.fix_ ? status.speedKmh_ : 0.0f;
                state.lastDistance_ = gattReceivedDistance;
                state.forceUpdate_ = false;
            }
        }
    } else {
        state.wasConnected_ = false;
        state.rtaOkTimer_ = 0;
        state.lastFix_ = false;
        state.lastSpeedKmh_ = 0.0f;
        state.lastDistance_ = 0xFFFF;
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
            if (gattReceivedDistance == 0xFFFF) {
                std::print(
                    "\r[FIX OK] Sat: {:2d} | Speed: {:6.2f} km/h | Dist: "
                    "--- | "
                    "L: {:9.6f}, {:9.6f}   ",
                    status.satellites_, status.speedKmh_, status.latitude_,
                    status.longitude_);
            } else {
                std::print(
                    "\r[FIX OK] Sat: {:2d} | Speed: {:6.2f} km/h | Dist: "
                    "{:6.2f}m | "
                    "L: {:9.6f}, {:9.6f}   ",
                    status.satellites_, status.speedKmh_,
                    static_cast<float>(gattReceivedDistance) / 1000.0F,
                    status.latitude_, status.longitude_);
            }
        } else {
            if (gattReceivedDistance == 0xFFFF) {
                std::print("\r[WAITING] Dist: --- | No fix data parsed yet... "
                           "         ");
            } else {
                std::print(
                    "\r[WAITING] Dist: {:6.2f}m | No fix data parsed yet... "
                    "         ",
                    static_cast<float>(gattReceivedDistance) / 1000.0F);
            }
        }
        (void)std::fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

} // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" void app_main() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

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

    // Start Logger Service and Console Reader
    rta::LoggerService::instance().start();
    rta::LoggerService::instance().startConsoleReader();

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
