#include "LoggerService.hpp"
#include "GpsService.hpp"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <algorithm>
#include <cstdio>
#include <string_view>
#include <sys/stat.h>

extern "C" uint16_t gattReceivedDistance;

namespace rta {

namespace {
constexpr std::string_view tag = "RTA_LOGGER";
}

void LoggerService::start() {
    std::scoped_lock lock(mutex_);
    if (isLoggingActive_) {
        return;
    }

    // 1. Initialize NVS (in case it wasn't)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 2. Initialize SPIFFS
    ESP_LOGI(tag.data(), "Initializing SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                  .partition_label = "storage",
                                  .max_files = 10,
                                  .format_if_mount_failed = true};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(tag.data(), "Failed to mount SPIFFS (%s)",
                 esp_err_to_name(ret));
        return;
    }

    // 3. Load active file index and increment for the new boot session
    loadActiveFileIndex();
    activeFileIdx_ = (activeFileIdx_ + 1) % 6;
    saveActiveFileIndex();
    linesWritten_ = 0;

    // 4. Open new active file in truncate mode and write CSV header
    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/session_%d.csv",
             activeFileIdx_);
    FILE* f = fopen(filename, "w");
    if (f != nullptr) {
        fprintf(f, "timestamp_ms,speed_kmh,distance_m\n");
        fclose(f);
    }

    isLoggingActive_ = true;

    // 5. Start logging task
    xTaskCreate(loggerTask, "logger_task", 4096, this, 3, nullptr);

    ESP_LOGI(tag.data(),
             "Logger service started successfully. Session active file: %s",
             filename);
}

void LoggerService::startConsoleReader() {
    xTaskCreate(consoleTask, "console_task", 4096, this, 2, nullptr);
    ESP_LOGI(tag.data(), "Console command reader started.");
}

void LoggerService::loggerTask(void* arg) {
    auto* self = static_cast<LoggerService*>(arg);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz (100 ms)

    while (self->isLoggingActive_) {
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Get GPS status
        auto status = GpsService::instance().getStatus();

        // Get BLE distance
        std::optional<double> distVal;
        if (gattReceivedDistance != 0xFFFF) {
            distVal = static_cast<double>(gattReceivedDistance) / 1000.0;
        }

        self->writeRecord(status.fix_ ? status.speedKmh_ : 0.0, distVal);
    }
    vTaskDelete(nullptr);
}

void LoggerService::consoleTask(void* arg) {
    auto* self = static_cast<LoggerService*>(arg);
    char line[64];
    while (true) {
        // Read from stdin. fgets is blocking.
        if (fgets(line, sizeof(line), stdin) != nullptr) {
            std::string_view command(line);
            while (!command.empty() &&
                   (command.back() == '\n' || command.back() == '\r')) {
                command.remove_suffix(1);
            }
            if (command == "DUMP") {
                self->dumpLogs();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(nullptr);
}

void LoggerService::writeRecord(double speed, std::optional<double> distance) {
    std::scoped_lock lock(mutex_);

    checkAndRotateFile();

    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/session_%d.csv",
             activeFileIdx_);
    FILE* f = fopen(filename, "a");
    if (f == nullptr) {
        return;
    }

    int64_t timestamp = esp_timer_get_time() / 1000;

    if (distance.has_value()) {
        fprintf(f, "%lld,%.2f,%.2f\n", (long long)timestamp, speed, *distance);
    } else {
        fprintf(f, "%lld,%.2f,---\n", (long long)timestamp, speed);
    }
    fclose(f);

    linesWritten_++;
}

void LoggerService::checkAndRotateFile() {
    // 18000 lines is 30 minutes of logging at 10 Hz
    if (linesWritten_ >= 18000) {
        activeFileIdx_ = (activeFileIdx_ + 1) % 6;
        saveActiveFileIndex();
        linesWritten_ = 0;

        char filename[32];
        snprintf(filename, sizeof(filename), "/spiffs/session_%d.csv",
                 activeFileIdx_);
        FILE* f = fopen(filename, "w");
        if (f != nullptr) {
            fprintf(f, "timestamp_ms,speed_kmh,distance_m\n");
            fclose(f);
        }
        ESP_LOGI(tag.data(), "Session rotated. Active file: %s", filename);
    }
}

void LoggerService::loadActiveFileIndex() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        uint8_t idx = 0;
        err = nvs_get_u8(my_handle, "active_file", &idx);
        if (err == ESP_OK && idx < 6) {
            activeFileIdx_ = idx;
        } else {
            activeFileIdx_ = 0;
        }
        nvs_close(my_handle);
    } else {
        activeFileIdx_ = 0;
    }
}

void LoggerService::saveActiveFileIndex() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_u8(my_handle, "active_file", activeFileIdx_);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

void LoggerService::dumpLogs() {
    std::scoped_lock lock(mutex_);

    // 1. Temporarily disable logging to avoid log pollution
    esp_log_level_set("*", ESP_LOG_NONE);

    // 2. Print start marker
    printf("\n===START_DUMP===\n");

    auto dumpFile = [](const char* filepath) {
        FILE* f = fopen(filepath, "r");
        if (f != nullptr) {
            char buf[128];
            while (fgets(buf, sizeof(buf), f) != nullptr) {
                printf("%s", buf);
            }
            fclose(f);
        }
    };

    // Dump chronologically: from activeFileIdx_ + 1 to activeFileIdx_ (modulo
    // 6)
    for (int i = 1; i <= 6; ++i) {
        int idx = (activeFileIdx_ + i) % 6;
        char filename[32];
        snprintf(filename, sizeof(filename), "/spiffs/session_%d.csv", idx);
        dumpFile(filename);
    }

    // 3. Print end marker
    printf("===END_DUMP===\n");

    fflush(stdout);

    // 4. Restore log level to the default compiled level
    esp_log_level_set("*",
                      static_cast<esp_log_level_t>(CONFIG_LOG_DEFAULT_LEVEL));
}

} // namespace rta
