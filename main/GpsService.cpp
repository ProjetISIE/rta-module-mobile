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
constexpr std::string_view kTag = "RTA_GPS";
constexpr double kEarthRadiusKm = 6371.0;
constexpr double kMinOdometerAccuracyKm = 0.001;
constexpr float kMinSpeedKmhThreshold = 0.5F;

// UART Config
constexpr uart_port_t kGpsUartPort = UART_NUM_2;
constexpr int kGpsRxPin = 16;
constexpr int kGpsTxPin = 17;
constexpr int kUartBufSize = 1024;

// PMTK Commands
constexpr std::string_view kPmtkSetBaud115200 = "$PMTK251,115200*1F\r\n";
constexpr std::string_view kPmtkSetNmeaOutputRmcgga =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
constexpr std::string_view kPmtkSetNmeaUpdate5Hz = "$PMTK220,200*2C\r\n";

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    using std::numbers::pi;
    const auto deltaLat = (lat2 - lat1) * pi / 180.0;
    const auto deltaLon = (lon2 - lon1) * pi / 180.0;
    const auto lat1Rad = lat1 * pi / 180.0;
    const auto lat2Rad = lat2 * pi / 180.0;
    const auto arc = std::pow(std::sin(deltaLat / 2), 2) +
                     (std::pow(std::sin(deltaLon / 2), 2) * std::cos(lat1Rad) *
                      std::cos(lat2Rad));
    const auto centralAngle = 2 * std::asin(std::sqrt(arc));
    return kEarthRadiusKm * centralAngle;
}

double convertPositionToDecimal(nmea_position pos) {
    auto decimalDegrees =
        pos.degrees +
        (pos.minutes / 60.0); // NOLINT(cppcoreguidelines-avoid-magic-numbers)
    if (pos.cardinal == 'S' || pos.cardinal == 'W') {
        decimalDegrees = -decimalDegrees;
    }
    return decimalDegrees;
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
uint32_t utcTmToEpoch(const struct tm& timeStruct) {
    int year = timeStruct.tm_year + 1900;
    int month = timeStruct.tm_mon + 1; // 1-12
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    const int days = 365 * year + year / 4 - year / 100 + year / 400 +
                     367 * month / 12 - 30 + timeStruct.tm_mday - 719499;

    return static_cast<uint32_t>(days * 86400 + timeStruct.tm_hour * 3600 +
                                 timeStruct.tm_min * 60 + timeStruct.tm_sec);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
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
                status_.utcEpoch_ = utcTmToEpoch(rmc->date_time);

                if (firstFix_) {
                    lastLat_ = status_.latitude_;
                    lastLon_ = status_.longitude_;
                    firstFix_ = false;
                } else if (status_.speedKmh_ > kMinSpeedKmhThreshold) {
                    const auto distance =
                        calculateDistance(lastLat_, lastLon_, status_.latitude_,
                                          status_.longitude_);
                    if (distance > kMinOdometerAccuracyKm) {
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

void GpsService::readerTask(void* arg) {
    auto* service = static_cast<GpsService*>(arg);

    uart_config_t uartConfig = {};
    uartConfig.baud_rate = 9600;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_DISABLE;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.rx_flow_ctrl_thresh = 0;
    uartConfig.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(
        uart_driver_install(kGpsUartPort, kUartBufSize * 2, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(kGpsUartPort, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(kGpsUartPort, kGpsTxPin, kGpsRxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(kGpsUartPort, kPmtkSetBaud115200.data(),
                     kPmtkSetBaud115200.size());
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(uart_set_baudrate(kGpsUartPort, 115200));

    uart_write_bytes(kGpsUartPort, kPmtkSetNmeaOutputRmcgga.data(),
                     kPmtkSetNmeaOutputRmcgga.size());
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(kGpsUartPort, kPmtkSetNmeaUpdate5Hz.data(),
                     kPmtkSetNmeaUpdate5Hz.size());

    std::vector<uint8_t> buffer(kUartBufSize);
    size_t totalBytes = 0;

    while (true) {
        const int len =
            uart_read_bytes(kGpsUartPort, buffer.data() + totalBytes,
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
                    service->processNmeaSentence(std::string_view(
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
    xTaskCreate(readerTask, "gps_reader_task", 4096, this, 5, nullptr);
}

} // namespace rta
