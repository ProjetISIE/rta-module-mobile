#include "GpsService.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <vector>

// libnmea headers
#include "gpgga.h"
#include "gprmc.h"
#include "nmea.h"

// NimBLE headers
#include "host/ble_hs.h"

namespace rta {

namespace {
constexpr double EARTH_RADIUS_KM = 6371.0;
constexpr double MIN_ODOMETER_ACCURACY_KM = 0.001;
constexpr float MIN_SPEED_KMH_THRESHOLD = 0.5f;

// Configuration UART
constexpr uart_port_t GPS_UART_PORT = UART_NUM_2;
constexpr int GPS_RX_PIN = 16;
constexpr int GPS_TX_PIN = 17;
constexpr int UART_BUF_SIZE = 1024;

// Commandes PMTK
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
  auto decimal_degrees = pos.degrees + (pos.minutes / 60.0);
  if (pos.cardinal == 'S' || pos.cardinal == 'W') {
    decimal_degrees = -decimal_degrees;
  }
  return decimal_degrees;
}
} // namespace

extern "C" uint16_t gatt_svr_chr_spd_val_handle;

GpsStatus GpsService::getStatus() const {
  std::lock_guard lock(mutex_);
  return status_;
}

void GpsService::processNmeaSentence(std::string_view sentence) {
  // libnmea nécessite un char* mutable
  std::vector<char> mutable_sentence(sentence.begin(), sentence.end());
  nmea_s *data =
      nmea_parse(mutable_sentence.data(), mutable_sentence.size(), 1);
  if (!data)
    return;

  {
    std::lock_guard lock(mutex_);
    if (data->type == NMEA_GPRMC) {
      auto *rmc = reinterpret_cast<nmea_gprmc_s *>(data);
      if (rmc->valid) {
        status_.fix = true;
        status_.latitude = convertPositionToDecimal(rmc->latitude);
        status_.longitude = convertPositionToDecimal(rmc->longitude);
        status_.speed_kmh = rmc->gndspd_knots * 1.852f;

        if (first_fix_) {
          last_lat_ = status_.latitude;
          last_lon_ = status_.longitude;
          first_fix_ = false;
        } else if (status_.speed_kmh > MIN_SPEED_KMH_THRESHOLD) {
          auto dist = calculateDistance(last_lat_, last_lon_, status_.latitude,
                                        status_.longitude);
          if (dist > MIN_ODOMETER_ACCURACY_KM) {
            status_.odometer_km += dist;
            last_lat_ = status_.latitude;
            last_lon_ = status_.longitude;
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

static void reader_task(void *) {
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
  size_t total_bytes = 0;

  while (true) {
    int len = uart_read_bytes(GPS_UART_PORT, buffer.data() + total_bytes,
                              buffer.size() - total_bytes, pdMS_TO_TICKS(100));
    if (len > 0) {
      total_bytes += len;

      size_t consumed_up_to = 0;
      while (consumed_up_to < total_bytes) {
        auto *current_ptr = buffer.data() + consumed_up_to;
        auto *end_ptr = buffer.data() + total_bytes;

        auto *sentence_start = std::find(current_ptr, end_ptr, '$');
        if (sentence_start == end_ptr) {
          total_bytes = 0;
          break;
        }

        auto *sentence_end = std::find(sentence_start, end_ptr, '\r');
        if (sentence_end == end_ptr || (sentence_end + 1 == end_ptr) ||
            *(sentence_end + 1) != '\n') {
          // Trame incomplète, on décale les données restantes au début
          size_t remaining = end_ptr - sentence_start;
          std::memmove(buffer.data(), sentence_start, remaining);
          total_bytes = remaining;
          consumed_up_to = total_bytes; // Pour sortir de la boucle
          break;
        }

        // Trame complète trouvée
        size_t sentence_len = (sentence_end + 2) - sentence_start;
        if (sentence_len <= NMEA_MAX_LENGTH) {
          service.processNmeaSentence(std::string_view(
              reinterpret_cast<const char *>(sentence_start), sentence_len));
        }
        consumed_up_to = (sentence_end + 2) - buffer.data();

        if (consumed_up_to == total_bytes) {
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

extern "C" float gps_service_get_speed() {
  return rta::GpsService::instance().getStatus().speed_kmh;
}
