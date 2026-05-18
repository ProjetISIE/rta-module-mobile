#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string_view>

namespace rta {

/**
 * @brief Structure représentant l'état du GPS.
 */
struct GpsStatus {
  double latitude{0.0};
  double longitude{0.0};
  float speed_kmh{0.0f};
  double odometer_km{0.0};
  int satellites{0};
  bool fix{false};
};

/**
 * @brief Service de gestion du GPS et du traitement des trames NMEA.
 *
 * Suit le pattern Singleton pour un accès global au sein de l'application.
 */
class GpsService {
public:
  static GpsService &instance() {
    static GpsService inst;
    return inst;
  }

  /// Démarre la tâche de lecture UART.
  void start();

  /// Récupère une copie de l'état actuel (Thread-safe).
  [[nodiscard]] GpsStatus getStatus() const;

  /// Traite une trame NMEA brute.
  void processNmeaSentence(std::string_view sentence);

private:
  GpsService() = default;

  mutable std::mutex mutex_;
  GpsStatus status_{};

  double last_lat_{0.0};
  double last_lon_{0.0};
  bool first_fix_{true};
};

} // namespace rta
