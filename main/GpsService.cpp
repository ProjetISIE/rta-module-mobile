#include "GpsService.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// libnmea headers
#include "gpgga.h"
#include "gprmc.h"
#include "nmea.h"

// NimBLE headers
#include "host/ble_hs.h"

static const double EARTH_RADIUS_KM = 6371.0;
static const double MIN_ODOMETER_ACCURACY_KM = 0.001;
static const double MIN_SPEED_KMH_THRESHOLD = 0.5;

// UART Config
#define GPS_UART_PORT UART_NUM_2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define UART_BUF_SIZE 1024

// PMTK Commands
static const char *PMTK_SET_BAUD_115200 = "$PMTK251,115200*1F\r\n";
static const char *PMTK_SET_NMEA_OUTPUT_RMCGGA =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
static const char *PMTK_SET_NMEA_UPDATE_5HZ = "$PMTK220,200*2C\r\n";

extern "C" uint16_t gatt_svr_chr_spd_val_handle;

namespace rta {

static double calculateDistance(double lat1, double long1, double lat2,
                                double long2) {
  auto dLat = (lat2 - lat1) * M_PI / 180.0;
  auto dLon = (long2 - long1) * M_PI / 180.0;
  lat1 = lat1 * M_PI / 180.0;
  lat2 = lat2 * M_PI / 180.0;
  auto a = std::pow(std::sin(dLat / 2), 2) +
           std::pow(std::sin(dLon / 2), 2) * std::cos(lat1) * std::cos(lat2);
  auto c = 2 * std::asin(std::sqrt(a));
  return EARTH_RADIUS_KM * c;
}

static double convert_position_to_decimal(nmea_position pos) {
  auto decimal_degrees = pos.degrees + (pos.minutes / 60.0);
  if (pos.cardinal == 'S' || pos.cardinal == 'W') {
    decimal_degrees = -decimal_degrees;
  }
  return decimal_degrees;
}

gps_status_t GpsService::getStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

void GpsService::processNmeaSentence(const char *sentence, size_t len) {
  nmea_s *data = nmea_parse(const_cast<char *>(sentence), len, 1);
  if (!data)
    return;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (data->type == NMEA_GPRMC) {
      auto *rmc = reinterpret_cast<nmea_gprmc_s *>(data);
      if (rmc->valid) {
        status_.fix = true;
        status_.latitude = convert_position_to_decimal(rmc->latitude);
        status_.longitude = convert_position_to_decimal(rmc->longitude);
        status_.speed_kmh = rmc->gndspd_knots * 1.852f;

        if (firstFix_) {
          lastLat_ = status_.latitude;
          lastLon_ = status_.longitude;
          firstFix_ = false;
        } else if (status_.speed_kmh > MIN_SPEED_KMH_THRESHOLD) {
          auto dist = calculateDistance(lastLat_, lastLon_, status_.latitude,
                                        status_.longitude);
          if (dist > MIN_ODOMETER_ACCURACY_KM) {
            status_.odometer_km += dist;
            lastLat_ = status_.latitude;
            lastLon_ = status_.longitude;
          }
        }
        ble_gatts_chr_updated(gatt_svr_chr_spd_val_handle);
      } else {
        status_.fix = false;
      }
    } else if (data->type == NMEA_GPGGA) {
      auto *gga = reinterpret_cast<nmea_gpgga_s *>(data);
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

static void reader_task(void *arg) {
  auto &service = GpsService::instance();

  uart_config_t uart_config = {.baud_rate = 9600,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .rx_flow_ctrl_thresh = 0,
                               .source_clk = UART_SCLK_DEFAULT,
                               .flags = {}};

  ESP_ERROR_CHECK(
      uart_driver_install(GPS_UART_PORT, UART_BUF_SIZE * 2, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  vTaskDelay(pdMS_TO_TICKS(100));
  uart_write_bytes(GPS_UART_PORT, PMTK_SET_BAUD_115200,
                   std::strlen(PMTK_SET_BAUD_115200));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_PORT, 115200));

  uart_write_bytes(GPS_UART_PORT, PMTK_SET_NMEA_OUTPUT_RMCGGA,
                   std::strlen(PMTK_SET_NMEA_OUTPUT_RMCGGA));
  vTaskDelay(pdMS_TO_TICKS(100));
  uart_write_bytes(GPS_UART_PORT, PMTK_SET_NMEA_UPDATE_5HZ,
                   std::strlen(PMTK_SET_NMEA_UPDATE_5HZ));

  auto *buffer = static_cast<char *>(std::malloc(UART_BUF_SIZE));
  size_t total_bytes = 0;

  while (true) {
    int len = uart_read_bytes(GPS_UART_PORT,
                              reinterpret_cast<uint8_t *>(buffer) + total_bytes,
                              UART_BUF_SIZE - total_bytes, pdMS_TO_TICKS(100));
    if (len > 0) {
      total_bytes += len;
      while (total_bytes > 0) {
        char *start =
            static_cast<char *>(std::memchr(buffer, '$', total_bytes));
        if (!start) {
          total_bytes = 0;
          break;
        }
        if (start != buffer) {
          size_t skip = start - buffer;
          std::memmove(buffer, start, total_bytes - skip);
          total_bytes -= skip;
          start = buffer;
        }
        char *end = static_cast<char *>(std::memchr(start, '\r', total_bytes));
        if (!end || (end + 1 >= buffer + total_bytes) || *(end + 1) != '\n')
          break;
        end++;

        size_t sentence_len = end - start + 1;
        if (sentence_len <= NMEA_MAX_LENGTH) {
          char sentence[NMEA_MAX_LENGTH + 1];
          std::memcpy(sentence, start, sentence_len);
          sentence[sentence_len] = '\0';
          service.processNmeaSentence(sentence, sentence_len);
        }

        size_t consumed = sentence_len;
        if (consumed < total_bytes) {
          std::memmove(buffer, start + consumed, total_bytes - consumed);
          total_bytes -= consumed;
        } else {
          total_bytes = 0;
        }
      }
    }
  }
}

void GpsService::start() {
  xTaskCreate(reader_task, "gps_reader_task", 4096, nullptr, 5, nullptr);
}

} // namespace rta

// Bridge for the C GATT table access
extern "C" float gps_service_get_speed() {
  return rta::GpsService::instance().getStatus().speed_kmh;
}
