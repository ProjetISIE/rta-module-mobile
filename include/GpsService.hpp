#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace rta {

typedef struct {
  double latitude;
  double longitude;
  float speed_kmh;
  double odometer_km;
  int satellites;
  bool fix;
} gps_status_t;

class GpsService {
public:
  static GpsService &instance() {
    static GpsService inst;
    return inst;
  }

  void start();
  gps_status_t getStatus() const;

  // Internal callback for the task
  void processNmeaSentence(const char *sentence, size_t len);

private:
  GpsService() = default;

  mutable std::mutex mutex_;
  gps_status_t status_{};

  double lastLat_{0.0};
  double lastLon_{0.0};
  bool firstFix_{true};
};

} // namespace rta
