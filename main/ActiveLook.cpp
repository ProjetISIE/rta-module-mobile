#include "ActiveLook.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <array>
#include <format>
#include <string>

namespace rta {

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

    for (const uint16_t handle : commandHandles) {
        ble_gattc_write_no_rsp_flat(*connectionHandle_, handle, buf.data(),
                                    buf.size());
    }
}

void ActiveLook::initializeDisplay(uint16_t connHandle) {
    connectionHandle_ = connHandle;

    // Power on and configure display
    static constexpr uint8_t vOn = 0x01;
    static constexpr uint8_t vFlip = 0x02;
    sendCommand(Command::POWER, std::span{&vOn, 1});
    sendCommand(Command::CONFIG, std::span{&vFlip, 1});

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
    std::vector<uint8_t> txt = {0x00, 0x99, 0x00, 0x60, 0x04, 0x02, 0x0F};

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

void ActiveLook::displayCoordinates(double lat, double lon) {
    if (!connectionHandle_) {
        return;
    }

    sendCommand(Command::CLEAR);
    vTaskDelay(pdMS_TO_TICKS(50));

    auto formatCoord = [](double val, uint8_t yPos) {
        auto formatted = std::format("{:.4f}", val);
        if (formatted.length() > 6) {
            formatted.resize(6);
        }

        std::vector<uint8_t> payload = {0x00, 0x99, 0x00, yPos,
                                        0x04, 0x02, 0x0F};
        payload.reserve(payload.size() + formatted.length() + 1);

        for (const char character : formatted) {
            payload.push_back(static_cast<uint8_t>(character));
        }
        payload.push_back('\0');
        return payload;
    };

    sendCommand(Command::LUMA_TEXT, formatCoord(lat, 0x40));
    vTaskDelay(pdMS_TO_TICKS(50));
    sendCommand(Command::LUMA_TEXT, formatCoord(lon, 0x80));
}

} // namespace rta
