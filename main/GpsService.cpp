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
#include <ctime>
#include <numbers>
#include <span>
#include <vector>

namespace rta {

namespace {
constexpr std::string_view tag = "RTA_GPS";
constexpr double earthRadiusKm = 6371.0;
constexpr double minOdometerAccuracyKm = 0.001;
constexpr float minSpeedKmhThreshold = 0.5F;

// UART Config
constexpr uart_port_t gpsUartPort = UART_NUM_2;
constexpr int gpsRxPin = 16;
constexpr int gpsTxPin = 17;
constexpr int uartBufSize = 1024;

// PMTK Commands
constexpr std::string_view pmtkSetBaud115200 = "$PMTK251,115200*1F\r\n";
constexpr std::string_view pmtkSetNmeaOutputRmcgga =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
constexpr std::string_view pmtkSetNmeaUpdate5Hz = "$PMTK220,200*2C\r\n";

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    using namespace std::numbers;
    const auto deltaLat = (lat2 - lat1) * pi / 180.0;
    const auto deltaLon = (lon2 - lon1) * pi / 180.0;
    const auto lat1Rad = lat1 * pi / 180.0;
    const auto lat2Rad = lat2 * pi / 180.0;
    const auto arc = std::pow(std::sin(deltaLat / 2), 2) +
                     std::pow(std::sin(deltaLon / 2), 2) * std::cos(lat1Rad) *
                         std::cos(lat2Rad);
    const auto centralAngle = 2 * std::asin(std::sqrt(arc));
    return earthRadiusKm * centralAngle;
}

double convertPositionToDecimal(nmea_position pos) {
    auto decimalDegrees = pos.degrees + (pos.minutes / 60.0);
    if (pos.cardinal == 'S' || pos.cardinal == 'W') {
        decimalDegrees = -decimalDegrees;
    }
    return decimalDegrees;
}

uint32_t convertUtcTmToEpoch(const struct tm& timeInfo) {
    int year = timeInfo.tm_year + 1900;
    int month = timeInfo.tm_mon + 1;
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    long days = (365L * year) + (year / 4) - (year / 100) + (year / 400);
    days += (367L * month - 362) / 12;
    days += timeInfo.tm_mday - 1;
    days -= 719499L; // Days from year 0 to 1970

    return static_cast<uint32_t>(days * 86400L + timeInfo.tm_hour * 3600L +
                                 timeInfo.tm_min * 60L + timeInfo.tm_sec);
}
} // namespace

extern "C" uint16_t gattSvrChrSpdValHandle;

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
    if (data == nullptr) {
        return;
    }

    {
        std::scoped_lock lock(mutex_);
        if (data->type == NMEA_GPRMC) {
            auto* rmc = reinterpret_cast<nmea_gprmc_s*>(data);
            if (rmc->valid) {
                status_.fix_ = true;
                status_.latitude_ = convertPositionToDecimal(rmc->latitude);
                status_.longitude_ = convertPositionToDecimal(rmc->longitude);
                status_.speedKmh_ = rmc->gndspd_knots * 1.852F;
                status_.utcEpoch_ =
                    static_cast<uint32_t>(timegm(&rmc->date_time));

                if (firstFix_) {
                    lastLat_ = status_.latitude_;
                    lastLon_ = status_.longitude_;
                    firstFix_ = false;
                } else if (status_.speedKmh_ > minSpeedKmhThreshold) {
                    const auto distance =
                        calculateDistance(lastLat_, lastLon_, status_.latitude_,
                                          status_.longitude_);
                    if (distance > minOdometerAccuracyKm) {
                        status_.odometerKm_ += distance;
                        lastLat_ = status_.latitude_;
                        lastLon_ = status_.longitude_;
                    }
                }
                ble_gatts_chr_updated(gattSvrChrSpdValHandle);
            } else {
                status_.fix_ = false;
            }
        } else if (data->type == NMEA_GPGGA) {
            auto* gga = reinterpret_cast<nmea_gpgga_s*>(data);
            if (gga->position_fix > 0) {
                status_.fix_ = true;
                status_.satellites_ = gga->n_satellites;
            } else {
                status_.fix_ = false;
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

    ESP_ERROR_CHECK(
        uart_driver_install(gpsUartPort, uartBufSize * 2, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(gpsUartPort, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(gpsUartPort, gpsTxPin, gpsRxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(gpsUartPort, pmtkSetBaud115200.data(),
                     pmtkSetBaud115200.size());
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(uart_set_baudrate(gpsUartPort, 115200));

    uart_write_bytes(gpsUartPort, pmtkSetNmeaOutputRmcgga.data(),
                     pmtkSetNmeaOutputRmcgga.size());
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(gpsUartPort, pmtkSetNmeaUpdate5Hz.data(),
                     pmtkSetNmeaUpdate5Hz.size());

    std::vector<uint8_t> buffer(uartBufSize);
    size_t totalBytes = 0;

    while (true) {
        const int len =
            uart_read_bytes(gpsUartPort, buffer.data() + totalBytes,
                            buffer.size() - totalBytes, pdMS_TO_TICKS(100));
        if (len > 0) {
            totalBytes += static_cast<size_t>(len);

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
                    const size_t remaining = static_cast<size_t>(
                        std::distance(itStart, currentRange.end()));
                    std::memmove(buffer.data(), &(*itStart), remaining);
                    totalBytes = remaining;
                    break;
                }

                // Complete frame found
                const size_t sentenceLen =
                    static_cast<size_t>(std::distance(itStart, itEnd)) + 2;
                if (sentenceLen <= NMEA_MAX_LENGTH) {
                    service.processNmeaSentence(std::string_view(
                        reinterpret_cast<const char*>(&(*itStart)),
                        sentenceLen));
                }

                const size_t consumed = static_cast<size_t>(std::distance(
                                            currentRange.begin(), itEnd)) +
                                        2;
                currentRange = currentRange.subspan(consumed);

                if (currentRange.empty()) {
                    totalBytes = 0;
                } else {
                    totalBytes = currentRange.size();
                }
            }
        }
    }
}

void GpsService::start() {
    xTaskCreate(readerTask, "gps_reader_task", 4096, nullptr, 5, nullptr);
}

} // namespace rta
