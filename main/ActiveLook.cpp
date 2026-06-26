#include "ActiveLook.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <algorithm>
#include <array>
#include <format>
#include <string>
#include <vector>

namespace rta {

namespace {
constexpr std::array<uint8_t, 7> kLumaTextConfigDefault = {
    0x00, 0x99, 0x00, 0x60, 0x04, 0x02, 0x0F};
constexpr uint8_t kYPosSpeed = 0x40;
constexpr uint8_t kYPosDistance = 0x80;
} // namespace

void ActiveLook::sendCommand(Command cmd, std::span<const uint8_t> payload) {
    if (!connectionHandle_) {
        return;
    }

    // Frame construction: [0xFF, CMD, LEN_H, LEN_L, PAYLOAD..., 0xAA]
    const size_t totalLen = 4 + payload.size() + 1;

    std::vector<uint8_t> buf;
    buf.reserve(totalLen);

    buf.push_back(0xFF);
    buf.push_back(static_cast<uint8_t>(cmd));
    buf.push_back(static_cast<uint8_t>(static_cast<uint32_t>(totalLen) >> 8));
    buf.push_back(static_cast<uint8_t>(static_cast<uint32_t>(totalLen) & 0xFF));

    if (!payload.empty()) {
        buf.insert(buf.end(), payload.begin(), payload.end());
    }
    buf.push_back(0xAA);

    for (const uint16_t handle : kCommandHandles) {
        ble_gattc_write_no_rsp_flat(*connectionHandle_, handle, buf.data(),
                                    buf.size());
    }
}

void ActiveLook::initializeDisplay(uint16_t connHandle) {
    connectionHandle_ = connHandle;

    // Power on and configure display
    static constexpr uint8_t kOn = 0x01;
    static constexpr uint8_t kFlip = 0x02;
    sendCommand(Command::POWER, std::span{&kOn, 1});
    sendCommand(Command::CONFIG, std::span{&kFlip, 1});

    vTaskDelay(pdMS_TO_TICKS(50));
    displayText("RTA OK");
}

void ActiveLook::displayText(std::string_view msg) {
    if (!connectionHandle_) {
        return;
    }

    sendCommand(Command::CLEAR);
    vTaskDelay(pdMS_TO_TICKS(50));

    // LumaText config: {X, Y, Rotation, Font, Color}
    std::vector<uint8_t> txt;
    txt.reserve(kLumaTextConfigDefault.size() + msg.length() + 1);
    txt.insert(txt.end(), std::begin(kLumaTextConfigDefault),
               std::end(kLumaTextConfigDefault));

    const auto displayMsg = msg.substr(0, std::min(msg.length(), size_t{50}));

    for (const char character : displayMsg) {
        txt.push_back(static_cast<uint8_t>(character));
    }
    txt.push_back('\0');

    sendCommand(Command::LUMA_TEXT, txt);
}

void ActiveLook::displayNumber(int value) {
    displayText(std::format("{}", value));
}

void ActiveLook::displayGpsWait() { displayText("Wait..."); }

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void ActiveLook::displaySpeedAndDistance(std::optional<double> speedKmh,
                                         std::optional<double> distanceM) {
    if (!connectionHandle_) {
        return;
    }

    sendCommand(Command::CLEAR);
    vTaskDelay(pdMS_TO_TICKS(50));

    auto formatValue = [](std::optional<double> val, int maxLen) {
        if (!val.has_value() || *val < 0.0) {
            return std::string("---");
        }
        double value = *val;
        std::string formatted = std::format("{:.2f}", value);
        std::replace(formatted.begin(), formatted.end(), '.', ',');
        if (formatted.length() > maxLen) {
            formatted = std::format("{:.1f}", value);
            std::replace(formatted.begin(), formatted.end(), '.', ',');
        }
        if (formatted.length() > maxLen) {
            formatted = std::format("{:.0f}", value);
            std::replace(formatted.begin(), formatted.end(), '.', ',');
        }
        if (formatted.length() > maxLen) {
            formatted.resize(maxLen);
        }
        return formatted;
    };

    auto buildPayload = [](const std::string& formatted, uint8_t yPos) {
        std::vector<uint8_t> payload = {0x00, 0x99, 0x00, yPos,
                                        0x04, 0x02, 0x0F};
        payload.reserve(payload.size() + formatted.length() + 1);

        for (const char character : formatted) {
            payload.push_back(static_cast<uint8_t>(character));
        }
        payload.push_back('\0');
        return payload;
    };

    std::string speedStr = formatValue(speedKmh, 3);
    if (speedStr != "---") {
        speedStr += "km/h";
    }
    std::string distanceStr = formatValue(distanceM, 5);
    if (distanceStr != "---") {
        distanceStr += "m";
    }

    sendCommand(Command::LUMA_TEXT, buildPayload(speedStr, kYPosSpeed));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendCommand(Command::LUMA_TEXT, buildPayload(distanceStr, kYPosDistance));
}

} // namespace rta
