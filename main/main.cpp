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
constexpr std::string_view tag = "RTA";

struct AppContext {
    rta::ActiveLook glasses;
    rta::BleServer server;
    rta::BleManager manager;

    AppContext() : server("ESP32_RTA"), manager(glasses) {}
};

// GATT services defined in gps_gatt_def.cpp
extern "C" struct ble_gatt_svc_def gps_gatt_svcs[];

void updateGlassesDisplay(AppContext& context, const rta::GpsStatus& status,
                          bool& wasConnected, bool& lastFix, double& lastLat,
                          double& lastLon, bool& forceUpdate, int& rtaOkTimer) {
    const bool connected = context.glasses.isConnected();

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
            const bool posChanged = status.fix && (status.latitude != lastLat ||
                                                   status.longitude != lastLon);

            if (forceUpdate || fixChanged || posChanged) {
                if (status.fix) {
                    context.glasses.displayCoordinates(status.latitude,
                                                       status.longitude);
                    lastLat = status.latitude;
                    lastLon = status.longitude;
                } else {
                    context.glasses.displayGpsWait();
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
}

void processAndDisplayTask(void* pvParameters) {
    auto* context = static_cast<AppContext*>(pvParameters);
    auto& gps = rta::GpsService::instance();

    int rtaOkTimer = 0;
    bool wasConnected = false;
    bool lastFix = false;
    double lastLat = 0.0;
    double lastLon = 0.0;
    bool forceUpdate = false;

    while (true) {
        const auto status = gps.getStatus();

        updateGlassesDisplay(*context, status, wasConnected, lastFix, lastLat,
                             lastLon, forceUpdate, rtaOkTimer);

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
    ESP_LOGI(tag.data(),
             "Starting Modern C++ GPS BLE Server & ActiveLook Client");

    // Initialize NVS
    esp_err_t return_code = nvs_flash_init();
    if (return_code == ESP_ERR_NVS_NO_FREE_PAGES ||
        return_code == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        return_code = nvs_flash_init();
    }
    ESP_ERROR_CHECK(return_code);

    // App Context stays alive for the duration of the app
    static AppContext context;

    // Initialize BLE Server
    if (context.server.init() != 0) {
        ESP_LOGE(tag.data(), "Failed to initialize BLE server");
        return;
    }

    // Set sync callback to start both advertising and scanning
    context.server.setSyncCallback([]() {
        context.server.startAdvertising();
        context.manager.startScanning();
        ESP_LOGI(tag.data(),
                 "BLE synchronized: Advertising and Scanning started");
    });

    // Start GPS Service
    rta::GpsService::instance().start();

    // Register services and start BLE stack
    context.server.registerServices(gps_gatt_svcs);
    context.server.start();

    // Create display task
    xTaskCreate(processAndDisplayTask, "display_task", 4096, &context, 5,
                nullptr);
}
