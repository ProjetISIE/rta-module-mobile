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
    double latitude{0.0};
    double longitude{0.0};
    float speedKmh{0.0F};
    double odometerKm{0.0};
    int satellites{0};
    bool fix{false};
};
/**
 * @brief Service for GPS management and NMEA sentence processing.
 *
 * Follows the Singleton pattern for global access within the application.
 */
class GpsService {
  public:
    static GpsService& instance() {
        static GpsService inst;
        return inst;
    }

    /// Starts the UART reader task.
    void start();

    /// Retrieves a copy of the current status (Thread-safe).
    [[nodiscard]] GpsStatus getStatus() const noexcept;

    /// Processes a raw NMEA sentence.
    void processNmeaSentence(std::string_view sentence);

  private:
    GpsService() = default;

    static void readerTask(void* arg);

    mutable std::mutex mutex_;
    GpsStatus status_{};

    double lastLat_{0.0};
    double lastLon_{0.0};
    bool firstFix_{true};
};

} // namespace rta

#endif // RTA_GPS_SERVICE_HPP
