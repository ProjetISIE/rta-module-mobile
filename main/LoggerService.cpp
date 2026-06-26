#include "LoggerService.hpp"
#include "GpsService.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <dirent.h>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <unistd.h>

namespace rta {

namespace {
constexpr std::string_view kTag = "RTA_LOGGER";

#pragma pack(push, 1)
struct LogRecord {
    uint32_t timestampMs_;
    uint32_t utcEpochS_;
    float speedKmh_;
    uint16_t distanceMm_;
};
#pragma pack(pop)
} // namespace

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
    ESP_LOGI(kTag.data(), "Initializing SPIFFS...");
    const esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                        .partition_label = "storage",
                                        .max_files = 10,
                                        .format_if_mount_failed = true};
    const esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag.data(), "Failed to mount SPIFFS (%s)",
                 esp_err_to_name(ret));
        return;
    }

    // 3. Clean up legacy V1 files safely
    std::vector<std::string> filesToDelete;
    DIR* dir = opendir("/spiffs");
    if (dir != nullptr) {
        struct dirent* ent = nullptr;
        while ((ent = readdir(dir)) != nullptr) {
            if (strncmp(ent->d_name, "session_", 8) == 0) {
                filesToDelete.push_back(ent->d_name);
            }
        }
        closedir(dir);
    }

    for (const auto& fName : filesToDelete) {
        char filepath[300];
        snprintf(filepath, sizeof(filepath), "/spiffs/%s", fName.c_str());
        unlink(filepath);
        ESP_LOGI(kTag.data(), "Deleted legacy file: %s", filepath);
    }

    // 4. Load active file index dynamically by scanning directory
    auto files = getSessionFiles();
    if (!files.empty()) {
        activeFileIdx_ = files.back() + 1;
    } else {
        activeFileIdx_ = 0;
    }
    linesWritten_ = 0;

    // 5. Free up space and open new active file
    freeUpSpaceIfNeeded();
    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/log_v2_%lu.bin",
             (unsigned long)activeFileIdx_);
    FILE* file = fopen(filename, "wb");
    if (file != nullptr) {
        fclose(file);
    }

    isLoggingActive_ = true;

    // 5. Start logging task
    xTaskCreate(loggerTask, "logger_task", 4096, this, 3, nullptr);

    ESP_LOGI(kTag.data(),
             "Logger service started successfully. Session active file: %s",
             filename);
}

void LoggerService::startConsoleReader() {
    xTaskCreate(consoleTask, "console_task", 4096, this, 2, nullptr);
    ESP_LOGI(kTag.data(), "Console command reader started.");
}

void LoggerService::loggerTask(void* arg) {
    auto* self = static_cast<LoggerService*>(arg);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10 Hz (100 ms)

    while (self->isLoggingActive_) {
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        auto status = self->gps_.getStatus();

        // Get BLE distance
        std::optional<double> distVal;
        const uint16_t dist =
            self->distanceRef_.load(std::memory_order_relaxed);
        if (dist != 0xFFFF) {
            distVal = static_cast<double>(dist) / 1000.0;
        }

        self->writeRecord(status, distVal);
    }
    vTaskDelete(nullptr);
}

void LoggerService::consoleTask(void* arg) {
    auto* self = static_cast<LoggerService*>(arg);

    // Install UART driver on UART0 if not already installed.
    // If a driver is already installed, this will fail safely or do nothing.
    uart_config_t uartConfig = {};
    uartConfig.baud_rate = 115200;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.rx_flow_ctrl_thresh = 0;
    uartConfig.source_clk = UART_SCLK_DEFAULT;
    uart_driver_install(UART_NUM_0, 256, 256, 0, nullptr, 0);
    uart_param_config(UART_NUM_0, &uartConfig);

    char line[64];
    int lineLen = 0;

    while (true) {
        uint8_t byte = 0;
        const int readBytes =
            uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(50));
        if (readBytes > 0) {
            // Echo character back to screen
            uart_write_bytes(UART_NUM_0, &byte, 1);

            if (byte == '\r' || byte == '\n') {
                // Echo carriage return and newline
                const char nlSeq[] = {'\r', '\n'};
                uart_write_bytes(UART_NUM_0, nlSeq, sizeof(nlSeq));

                // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                line[lineLen] = '\0';
                if (lineLen > 0) {
                    std::string cmd(line);
                    // Convert to uppercase
                    std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                                   [](unsigned char charCode) {
                                       return std::toupper(charCode);
                                   }); // NOLINT(readability-identifier-length)
                    if (cmd.find("DUMP") != std::string::npos) {
                        self->dumpLogs();
                    } else {
                        printf("\n[DEBUG] Ignored command: '%s' (len: %d)\n",
                               cmd.c_str(), static_cast<int>(cmd.length()));
                        for (size_t i = 0; i < cmd.length(); i++) {
                            printf("%02X ", static_cast<uint8_t>(cmd[i]));
                        }
                        printf("\n");
                    }
                }
                lineLen = 0;
            } else if (lineLen < static_cast<int>(sizeof(line)) - 1) {
                if (byte == 8 || byte == 127) {
                    if (lineLen > 0) {
                        lineLen--;
                        // Erase character from user terminal
                        const char bsSeq[] = {8, ' ', 8};
                        uart_write_bytes(UART_NUM_0, bsSeq, sizeof(bsSeq));
                    }
                } else {
                    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
                    line[lineLen++] = static_cast<char>(byte);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(nullptr);
}

void LoggerService::writeRecord(const GpsStatus& status,
                                std::optional<double> distance) {
    std::scoped_lock lock(mutex_);

    checkAndRotateFile();

    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/log_v2_%lu.bin",
             (unsigned long)activeFileIdx_);
    // hicpp-no-array-decay, readability-identifier-length)
    FILE* file = fopen(filename, "ab");
    if (file == nullptr) {
        return;
    }

    LogRecord record{};
    record.timestampMs_ = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    record.utcEpochS_ = status.utcEpoch_;
    record.speedKmh_ = status.fix_
                           ? status.speedKmh_
                           : 0.0F; // NOLINT(readability-redundant-casting)
    if (distance.has_value()) {
        record.distanceMm_ = static_cast<uint16_t>(*distance * 1000.0);
    } else {
        record.distanceMm_ = 0xFFFF;
    }

    fwrite(&record, sizeof(LogRecord), 1, file);
    fclose(file);

    linesWritten_++;
}

void LoggerService::checkAndRotateFile() {
    // 18000 lines is 30 minutes of logging at 10 Hz
    if (linesWritten_ >= 18000) {
        activeFileIdx_++;
        linesWritten_ = 0;

        freeUpSpaceIfNeeded();

        char filename[32];
        snprintf(filename, sizeof(filename), "/spiffs/log_v2_%lu.bin",
                 (unsigned long)activeFileIdx_);
        // hicpp-no-array-decay, readability-identifier-length)
        FILE* file = fopen(filename, "wb");
        if (file != nullptr) {
            fclose(file);
        }
        ESP_LOGI(kTag.data(), "Session rotated. Active file: %s", filename);
    }
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
std::vector<uint32_t> LoggerService::getSessionFiles() {
    std::vector<uint32_t> indices;
    DIR* dir = opendir("/spiffs");
    if (dir != nullptr) {
        struct dirent* ent = nullptr;
        while ((ent = readdir(dir)) != nullptr) {
            uint64_t idxUl = 0;
            if (sscanf(ent->d_name, "log_v2_%llu.bin", &idxUl) == 1) {
                indices.push_back(static_cast<uint32_t>(idxUl));
            }
        }
        closedir(dir);
    }
    std::sort(indices.begin(), indices.end());
    return indices;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void LoggerService::freeUpSpaceIfNeeded() {
    size_t total = 0;
    size_t used = 0;
    if (esp_spiffs_info("storage", &total, &used) != ESP_OK) {
        return;
    }
    // Maintain at least 300KB free
    while (total > 300000 && (total - used) < 300000) {
        auto files = getSessionFiles();
        if (files.empty()) {
            break;
        }

        char filepath[64];
        snprintf(filepath, sizeof(filepath), "/spiffs/log_v2_%lu.bin",
                 (unsigned long)files.front());
        unlink(filepath);
        ESP_LOGI(kTag.data(), "Deleted oldest session to free space: %s",
                 filepath);

        if (esp_spiffs_info("storage", &total, &used) != ESP_OK) {
            break;
        }
    }
}

void LoggerService::dumpLogs() {
    std::scoped_lock lock(mutex_);
    isDumping_ = true;

    // 1. Temporarily disable logging to avoid log pollution
    esp_log_level_set("*", ESP_LOG_NONE);

    // 2. Print start marker and CSV header
    printf("\n===START_DUMP===\n");
    printf("timestamp_ms,utc_epoch_s,speed_kmh,distance_m\n");

    auto dumpFile = [](const char* filepath) {
        FILE* file = fopen(filepath, "rb");
        if (file != nullptr) {
            LogRecord record{};
            int lineCount = 0;
            while (fread(&record, sizeof(LogRecord), 1, file) == 1) {
                if (record.distanceMm_ != 0xFFFF) {
                    printf("%lu,%lu,%.2f,%.2f\n",
                           (unsigned long)record.timestampMs_,
                           (unsigned long)record.utcEpochS_, record.speedKmh_,
                           record.distanceMm_ / 1000.0F);
                } else {
                    printf("%lu,%lu,%.2f,---\n",
                           (unsigned long)record.timestampMs_,
                           (unsigned long)record.utcEpochS_, record.speedKmh_);
                }
                lineCount++;
                if (lineCount % 100 == 0) {
                    vTaskDelay(1); // Yield to let Idle Task feed the watchdog
                }
            }
            fclose(file);
        }
    };

    // Dump chronologically by retrieving sorted files
    auto files = getSessionFiles();
    for (uint32_t idx : files) {
        char filename[32];
        snprintf(filename, sizeof(filename), "/spiffs/log_v2_%lu.bin",
                 (unsigned long)idx);
        dumpFile(filename);
    }

    // 3. Print end marker
    printf("===END_DUMP===\n");

    fflush(stdout);

    // 4. Restore log level to the default compiled level
    esp_log_level_set("*",
                      static_cast<esp_log_level_t>(CONFIG_LOG_DEFAULT_LEVEL));
    isDumping_ = false;
}
} // namespace rta
