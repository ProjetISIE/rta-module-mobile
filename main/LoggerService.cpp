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
#include <cstdio>
#include <string_view>
#include <sys/stat.h>

extern "C" uint16_t gattReceivedDistance;

namespace rta {

namespace {
constexpr std::string_view tag = "RTA_LOGGER";

#pragma pack(push, 1)
struct LogRecord {
    uint32_t timestamp_ms;
    float speed_kmh;
    uint16_t distance_mm;
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

    // 4. Open new active file in truncate mode to clear it
    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/session_%d.bin",
             activeFileIdx_);
    FILE* f = fopen(filename, "wb");
    if (f != nullptr) {
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

    // Install UART driver on UART0 if not already installed.
    // If a driver is already installed, this will fail safely or do nothing.
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 256, 256, 0, nullptr, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    char line[64];
    int line_len = 0;

    while (true) {
        uint8_t byte;
        int read_bytes =
            uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(50));
        if (read_bytes > 0) {
            // Echo character back to screen
            uart_write_bytes(UART_NUM_0, &byte, 1);

            if (byte == '\r' || byte == '\n') {
                // Echo carriage return and newline
                const char nl_seq[] = {'\r', '\n'};
                uart_write_bytes(UART_NUM_0, nl_seq, sizeof(nl_seq));

                line[line_len] = '\0';
                if (line_len > 0) {
                    std::string_view command(line);
                    if (command == "DUMP") {
                        self->dumpLogs();
                    }
                }
                line_len = 0;
            } else if (line_len < static_cast<int>(sizeof(line)) - 1) {
                if (byte == 8 || byte == 127) {
                    if (line_len > 0) {
                        line_len--;
                        // Erase character from user terminal
                        const char bs_seq[] = {8, ' ', 8};
                        uart_write_bytes(UART_NUM_0, bs_seq, sizeof(bs_seq));
                    }
                } else {
                    line[line_len++] = static_cast<char>(byte);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(nullptr);
}

void LoggerService::writeRecord(double speed, std::optional<double> distance) {
    std::scoped_lock lock(mutex_);

    checkAndRotateFile();

    char filename[32];
    snprintf(filename, sizeof(filename), "/spiffs/session_%d.bin",
             activeFileIdx_);
    FILE* f = fopen(filename, "ab");
    if (f == nullptr) {
        return;
    }

    LogRecord record;
    record.timestamp_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    record.speed_kmh = static_cast<float>(speed);
    if (distance.has_value()) {
        record.distance_mm = static_cast<uint16_t>(*distance * 1000.0);
    } else {
        record.distance_mm = 0xFFFF;
    }

    fwrite(&record, sizeof(LogRecord), 1, f);
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
        snprintf(filename, sizeof(filename), "/spiffs/session_%d.bin",
                 activeFileIdx_);
        FILE* f = fopen(filename, "wb");
        if (f != nullptr) {
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
    isDumping_ = true;

    // 1. Temporarily disable logging to avoid log pollution
    esp_log_level_set("*", ESP_LOG_NONE);

    // 2. Print start marker and CSV header
    printf("\n===START_DUMP===\n");
    printf("timestamp_ms,speed_kmh,distance_m\n");

    auto dumpFile = [](const char* filepath) {
        FILE* f = fopen(filepath, "rb");
        if (f != nullptr) {
            LogRecord record;
            int lineCount = 0;
            while (fread(&record, sizeof(LogRecord), 1, f) == 1) {
                if (record.distance_mm != 0xFFFF) {
                    printf("%lu,%.2f,%.2f\n",
                           (unsigned long)record.timestamp_ms, record.speed_kmh,
                           record.distance_mm / 1000.0f);
                } else {
                    printf("%lu,%.2f,---\n", (unsigned long)record.timestamp_ms,
                           record.speed_kmh);
                }
                lineCount++;
                if (lineCount % 100 == 0) {
                    vTaskDelay(1); // Yield to let Idle Task feed the watchdog
                }
            }
            fclose(f);
        }
    };

    // Dump chronologically: from activeFileIdx_ + 1 to activeFileIdx_ (modulo
    // 6)
    for (int i = 1; i <= 6; ++i) {
        int idx = (activeFileIdx_ + i) % 6;
        char filename[32];
        snprintf(filename, sizeof(filename), "/spiffs/session_%d.bin", idx);
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
