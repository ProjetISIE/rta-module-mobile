#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace rta {

/**
 * @brief Class managing communication with ActiveLook connected glasses.
 */
class ActiveLook {
  public:
    /// Bluetooth characteristic handles used for commands.
    static constexpr std::array<uint16_t, 3> COMMAND_HANDLES = {35, 56, 32};

    /// ActiveLook protocol constants
    enum class Command : uint8_t {
        POWER = 0x00,
        CLEAR = 0x01,
        CONFIG = 0x03,
        LUMA_TEXT = 0x37
    };

    ActiveLook() = default;

    /**
     * @brief Sends a structured command to the glasses.
     * @param cmd Command code to execute.
     * @param payload Optional data associated with the command.
     */
    void sendCommand(Command cmd, std::span<const uint8_t> payload = {});

    /**
     * @brief Initializes the display after connection.
     * @param connHandle BLE connection handle.
     */
    void initializeDisplay(uint16_t connHandle);

    /**
     * @brief Displays text on the screen.
     * @param msg Message to display.
     */
    void displayText(std::string_view msg);

    /**
     * @brief Displays an integer value.
     * @param value Value to display.
     */
    void displayNumber(int value);

    /**
     * @brief Checks connection status.
     * @return true if glasses are connected.
     */
    [[nodiscard]] bool isConnected() const {
        return connectionHandle_.has_value();
    }

    /**
     * @brief Resets state upon disconnection.
     */
    void disconnect() { connectionHandle_.reset(); }

    /**
     * @brief Displays a waiting message for GPS fix.
     */
    void displayGpsWait();

    /**
     * @brief Displays GPS coordinates on the screen.
     * @param lat Latitude.
     * @param lon Longitude.
     */
    void displayCoordinates(double lat, double lon);

  private:
    std::optional<uint16_t> connectionHandle_;
};

} // namespace rta
