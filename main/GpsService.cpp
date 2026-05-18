#include "GpsService.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpgga.h"
#include "gprmc.h"
#include "host/ble_gatt.h"
#include "nmea.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <span>
#include <vector>

namespace rta {

namespace {
constexpr double EARTH_RADIUS_KM = 6371.0;
constexpr double MIN_ODOMETER_ACCURACY_KM = 0.001;
constexpr float MIN_SPEED_KMH_THRESHOLD = 0.5f;

// UART Config
constexpr uart_port_t GPS_UART_PORT = UART_NUM_2;
constexpr int GPS_RX_PIN = 16;
constexpr int GPS_TX_PIN = 17;
constexpr int UART_BUF_SIZE = 1024;

// PMTK Commands
constexpr std::string_view PMTK_SET_BAUD_115200 = "$PMTK251,115200*1F\r\n";
constexpr std::string_view PMTK_SET_NMEA_OUTPUT_RMCGGA =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
constexpr std::string_view PMTK_SET_NMEA_UPDATE_5HZ = "$PMTK220,200*2C\r\n";

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    using namespace std::numbers;
    auto dLat = (lat2 - lat1) * pi / 180.0;
    auto dLon = (lon2 - lon1) * pi / 180.0;
    lat1 = lat1 * pi / 180.0;
    lat2 = lat2 * pi / 180.0;
    auto a = std::pow(std::sin(dLat / 2), 2) +
             std::pow(std::sin(dLon / 2), 2) * std::cos(lat1) * std::cos(lat2);
    auto c = 2 * std::asin(std::sqrt(a));
    return EARTH_RADIUS_KM * c;
}

double convertPositionToDecimal(nmea_position pos) {
    auto decimalDegrees = pos.degrees + (pos.minutes / 60.0);
    if (pos.cardinal == 'S' || pos.cardinal == 'W') {
        decimalDegrees = -decimalDegrees;
    }
    return decimalDegrees;
}
} // namespace

extern "C" uint16_t gatt_svr_chr_spd_val_handle;

GpsStatus GpsService::getStatus() const noexcept {
    std::scoped_lock lock(mutex_);
    return status_;
}

void GpsService::processNmeaSentence(std::string_view sentence) {
    // libnmea requires a mutable char*
    static thread_local std::vector<char> mutableSentence;
    mutableSentence.assign(sentence.begin(), sentence.end());

    nmea_s* data =
        nmea_parse(mutableSentence.data(), mutableSentence.size(), 1);
    if (!data) return;

    {
        std::scoped_lock lock(mutex_);
        if (data->type == NMEA_GPRMC) {
            auto* rmc = reinterpret_cast<nmea_gprmc_s*>(data);
            if (rmc->valid) {
                status_.fix = true;
                status_.latitude = convertPositionToDecimal(rmc->latitude);
                status_.longitude = convertPositionToDecimal(rmc->longitude);
                status_.speedKmh = rmc->gndspd_knots * 1.852f;

                if (firstFix_) {
                    lastLat_ = status_.latitude;
                    lastLon_ = status_.longitude;
                    firstFix_ = false;
                } else if (status_.speedKmh > MIN_SPEED_KMH_THRESHOLD) {
                    auto dist =
                        calculateDistance(lastLat_, lastLon_, status_.latitude,
                                          status_.longitude);
                    if (dist > MIN_ODOMETER_ACCURACY_KM) {
                        status_.odometerKm += dist;
                        lastLat_ = status_.latitude;
                        lastLon_ = status_.longitude;
                    }
                }
                ble_gatts_chr_updated(gatt_svr_chr_spd_val_handle);
            } else {
                status_.fix = false;
            }
        } else if (data->type == NMEA_GPGGA) {
            auto* gga = reinterpret_cast<nmea_gpgga_s*>(data);
            if (gga->position_fix > 0) {
                status_.fix = true;
                status_.satellites = gga->n_satellites;
            } else {
                status_.fix = false;
            }
        }
    }
    nmea_free(data);
}

void GpsService::readerTask(void*) {
    auto& service = GpsService::instance();

    uart_config_t uartConfig = {.baud_rate = 9600,
                                .data_bits = UART_DATA_8_BITS,
                                .parity = UART_PARITY_DISABLE,
                                .stop_bits = UART_STOP_BITS_1,
                                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                .rx_flow_ctrl_thresh = 0,
                                .source_clk = UART_SCLK_DEFAULT,
                                .flags = {}};

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, UART_BUF_SIZE * 2, 0, 0,
                                        nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(GPS_UART_PORT, PMTK_SET_BAUD_115200.data(),
                     PMTK_SET_BAUD_115200.size());
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_PORT, 115200));

    uart_write_bytes(GPS_UART_PORT, PMTK_SET_NMEA_OUTPUT_RMCGGA.data(),
                     PMTK_SET_NMEA_OUTPUT_RMCGGA.size());
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(GPS_UART_PORT, PMTK_SET_NMEA_UPDATE_5HZ.data(),
                     PMTK_SET_NMEA_UPDATE_5HZ.size());

    std::vector<uint8_t> buffer(UART_BUF_SIZE);
    size_t totalBytes = 0;

    while (true) {
        const int len =
            uart_read_bytes(GPS_UART_PORT, buffer.data() + totalBytes,
                            buffer.size() - totalBytes, pdMS_TO_TICKS(100));
        if (len > 0) {
            totalBytes += len;

            auto currentRange = std::span{buffer.data(), totalBytes};

            while (!currentRange.empty()) {
                auto itStart =
                    std::find(currentRange.begin(), currentRange.end(), '$');
                if (itStart == currentRange.end()) {
                    totalBytes = 0;
                    break;
                }

                auto itEnd = std::find(itStart, currentRange.end(), '\r');
                if (itEnd == currentRange.end() ||
                    std::next(itEnd) == currentRange.end() ||
                    *std::next(itEnd) != '\n') {
                    // Incomplete frame, shift remaining data to start
                    const size_t remaining =
                        std::distance(itStart, currentRange.end());
                    std::memmove(buffer.data(), &(*itStart), remaining);
                    totalBytes = remaining;
                    break;
                }

                // Complete frame found
                const size_t sentenceLen = std::distance(itStart, itEnd) + 2;
                if (sentenceLen <= NMEA_MAX_LENGTH) {
                    service.processNmeaSentence(std::string_view(
                        reinterpret_cast<const char*>(&(*itStart)),
                        sentenceLen));
                }

                const size_t consumed =
                    std::distance(currentRange.begin(), itEnd) + 2;
                currentRange = currentRange.subspan(consumed);

                if (currentRange.empty()) {
                    totalBytes = 0;
                } else {
                    totalBytes = currentRange.size();
                    // We don't necessarily need to memmove every time if we're
                    // careful, but for simplicity and to avoid complex ring
                    // buffer logic, we shift if needed at the end of loop.
                }
            }

            // If we still have data but didn't find a complete frame in the
            // current chunk, it's already handled by the memmove above if it
            // was an incomplete frame.
        }
    }
}

void GpsService::start() {
    xTaskCreate(readerTask, "gps_reader_task", 4096, nullptr, 5, nullptr);
}

} // namespace rta
