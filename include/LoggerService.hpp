#ifndef RTA_LOGGER_SERVICE_HPP
#define RTA_LOGGER_SERVICE_HPP

#include <mutex>
#include <optional>

namespace rta {

class LoggerService {
  public:
    static LoggerService& instance() {
        static LoggerService inst;
        return inst;
    }

    /// Initializes SPIFFS, NVS, and starts the 10Hz logging task.
    void start();

    /// Reads from stdin to listen for the "DUMP" command.
    void startConsoleReader();

    /// Dumps the logs from both files to standard output.
    void dumpLogs();

    /// Returns true if a dump is currently in progress.
    [[nodiscard]] bool isDumping() const { return isDumping_; }

  private:
    LoggerService() = default;

    static void loggerTask(void* arg);
    static void consoleTask(void* arg);

    void writeRecord(double speed, std::optional<double> distance);
    void checkAndRotateFile();
    void loadActiveFileIndex();
    void saveActiveFileIndex();

    mutable std::mutex mutex_;
    uint8_t activeFileIdx_{0};
    uint32_t linesWritten_{0};
    bool isLoggingActive_{false};
    bool isDumping_{false};
};

} // namespace rta

#endif // RTA_LOGGER_SERVICE_HPP
