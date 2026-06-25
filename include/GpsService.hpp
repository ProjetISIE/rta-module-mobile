#ifndef RTA_GPS_SERVICE_HPP
#define RTA_GPS_SERVICE_HPP

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string_view>

namespace rta {

/**
 * @brief Structure representing the GPS status.
 */
struct GpsStatus {
    double latitude_{0.0};
    double longitude_{0.0};
    float speedKmh_{0.0F};
    double odometerKm_{0.0};
    int satellites_{0};
    uint32_t utcEpoch_{0};
    bool fix_{false};
};
/**
 * @brief Service for GPS management and NMEA sentence processing.
 *
 * Follows the Singleton pattern for global access within the application.
 */
class GpsService {
  public:
    GpsService() = default;

    /// Starts the UART reader task.
    void start();

    /// Retrieves a copy of the current status (Thread-safe).
    [[nodiscard]] GpsStatus getStatus() const noexcept;

    /// Processes a raw NMEA sentence.
    void processNmeaSentence(std::string_view sentence);

  private:
    static void readerTask(void* arg);

    mutable std::mutex mutex_;
    GpsStatus status_{};

    double lastLat_{0.0};
    double lastLon_{0.0};
    bool firstFix_{true};
};

} // namespace rta

#endif // RTA_GPS_SERVICE_HPP
