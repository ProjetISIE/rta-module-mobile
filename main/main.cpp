#include "ActiveLook.hpp"
#include "BleManager.hpp"
#include "BleServer.hpp"
#include "GpsService.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "nvs_flash.h"
#include <memory>
#include <print>
#include <string_view>

namespace {
constexpr std::string_view TAG = "RTA";

struct AppContext {
    rta::ActiveLook glasses;
    rta::BleServer server;
    rta::BleManager manager;

    AppContext() : server("ESP32_RTA"), manager(glasses) {}
};

// GATT services defined in gps_gatt_def.cpp
extern "C" struct ble_gatt_svc_def gps_gatt_svcs[];

void processAndDisplayTask(void* pvParameters) {
    auto* ctx = static_cast<AppContext*>(pvParameters);
    auto& gps = rta::GpsService::instance();

    int rtaOkTimer = 0;
    bool wasConnected = false;
    bool lastFix = false;
    double lastLat = 0.0, lastLon = 0.0;
    bool forceUpdate = false;

    while (true) {
        const bool connected = ctx->glasses.isConnected();
        const auto status = gps.getStatus();

        if (connected) {
            if (!wasConnected) {
                rtaOkTimer = 5;
                wasConnected = true;
                forceUpdate = true;
            }

            if (rtaOkTimer > 0) {
                rtaOkTimer--;
            } else {
                const bool fixChanged = (status.fix != lastFix);
                const bool posChanged =
                    status.fix &&
                    (status.latitude != lastLat || status.longitude != lastLon);

                if (forceUpdate || fixChanged || posChanged) {
                    if (status.fix) {
                        ctx->glasses.displayCoordinates(status.latitude,
                                                        status.longitude);
                        lastLat = status.latitude;
                        lastLon = status.longitude;
                    } else {
                        ctx->glasses.displayGpsWait();
                    }
                    lastFix = status.fix;
                    forceUpdate = false;
                }
            }
        } else {
            wasConnected = false;
            rtaOkTimer = 0;
            lastFix = false;
            forceUpdate = false;
        }

        // Formatted console output via std::print (C++23)
        if (status.fix) {
            std::print(
                "\r[FIX OK] Sat: {:2d} | Speed: {:6.2f} km/h | L: {:9.6f}, "
                "{:9.6f}   ",
                status.satellites, status.speedKmh, status.latitude,
                status.longitude);
        } else {
            std::print("\r[WAITING] No fix data parsed yet...          ");
        }
        std::fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
} // namespace

extern "C" void app_main() {
    ESP_LOGI(TAG.data(),
             "Starting Modern C++ GPS BLE Server & ActiveLook Client");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // App Context stays alive for the duration of the app
    static AppContext ctx;

    // Initialize BLE Server
    if (ctx.server.init() != 0) {
        ESP_LOGE(TAG.data(), "Failed to initialize BLE server");
        return;
    }

    // Set sync callback to start both advertising and scanning
    ctx.server.setSyncCallback([]() {
        ctx.server.startAdvertising();
        ctx.manager.startScanning();
        ESP_LOGI(TAG.data(),
                 "BLE synchronized: Advertising and Scanning started");
    });

    // Start GPS Service
    rta::GpsService::instance().start();

    // Register services and start BLE stack
    ctx.server.registerServices(gps_gatt_svcs);
    ctx.server.start();

    // Create display task
    xTaskCreate(processAndDisplayTask, "display_task", 4096, &ctx, 5, nullptr);
}
