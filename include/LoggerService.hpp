#ifndef RTA_LOGGER_SERVICE_HPP
#define RTA_LOGGER_SERVICE_HPP

#include <mutex>
#include <optional>
#include <vector>

#include "GpsService.hpp"
#include <atomic>

namespace rta {

class LoggerService {
  public:
    LoggerService(GpsService& gps, std::atomic<uint16_t>& distanceRef)
        : gps_(gps), distanceRef_(distanceRef) {}

    /// Initializes SPIFFS, NVS, and starts the 10Hz logging task.
    void start();

    /// Reads from stdin to listen for the "DUMP" command.
    void startConsoleReader();

    /// Dumps the logs from both files to standard output.
    void dumpLogs();

    /// Returns true if a dump is currently in progress.
    [[nodiscard]] bool isDumping() const { return isDumping_; }

  private:
    GpsService& gps_;
    std::atomic<uint16_t>& distanceRef_;

    static void loggerTask(void* arg);
    static void consoleTask(void* arg);

    void writeRecord(const GpsStatus& status, std::optional<double> distance);
    void checkAndRotateFile();
    void freeUpSpaceIfNeeded();
    std::vector<uint32_t> getSessionFiles();

    mutable std::mutex mutex_;
    uint32_t activeFileIdx_{0};
    uint32_t linesWritten_{0};
    bool isLoggingActive_{false};
    bool isDumping_{false};
};

} // namespace rta

#endif // RTA_LOGGER_SERVICE_HPP
