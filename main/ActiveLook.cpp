#include "ActiveLook.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <array>
#include <format>
#include <string>

namespace rta {

void ActiveLook::sendCommand(Command cmd, std::span<const uint8_t> payload) {
  if (!connectionHandle_)
    return;

  // Frame construction: [0xFF, CMD, LEN_H, LEN_L, PAYLOAD..., 0xAA]
  const size_t totalLen = 4 + payload.size() + 1;
  std::vector<uint8_t> buf;
  buf.reserve(totalLen);

  buf.push_back(0xFF);
  buf.push_back(static_cast<uint8_t>(cmd));
  buf.push_back(static_cast<uint8_t>(totalLen >> 8));
  buf.push_back(static_cast<uint8_t>(totalLen & 0xFF));

  if (!payload.empty()) {
    buf.insert(buf.end(), payload.begin(), payload.end());
  }
  buf.push_back(0xAA);

  for (uint16_t h : COMMAND_HANDLES) {
    ble_gattc_write_no_rsp_flat(*connectionHandle_, h, buf.data(), buf.size());
  }
}

void ActiveLook::initializeDisplay(uint16_t connHandle) {
  connectionHandle_ = connHandle;

  // Power on and configure display
  const uint8_t vOn = 0x01;
  const uint8_t vFlip = 0x02;
  sendCommand(Command::POWER, std::span(&vOn, 1));
  sendCommand(Command::CONFIG, std::span(&vFlip, 1));

  vTaskDelay(pdMS_TO_TICKS(50));
  displayText("RTA OK");
}

void ActiveLook::displayText(std::string_view msg) {
  if (!connectionHandle_)
    return;

  sendCommand(Command::CLEAR);
  vTaskDelay(pdMS_TO_TICKS(50));

  // LumaText config: {X, Y, Rotation, Font, Color}
  std::vector<uint8_t> txt = {0x00, 0x99, 0x00, 0x60, 0x04, 0x02, 0x0F};

  if (msg.length() > 50)
    msg = msg.substr(0, 50);

  for (char c : msg)
    txt.push_back(static_cast<uint8_t>(c));
  txt.push_back('\0');

  sendCommand(Command::LUMA_TEXT, txt);
}

void ActiveLook::displayNumber(int value) {
  displayText(std::format("{}", value));
}

void ActiveLook::displayGpsWait() { displayText("Wait..."); }

void ActiveLook::displayCoordinates(double lat, double lon) {
  if (!connectionHandle_)
    return;

  sendCommand(Command::CLEAR);
  vTaskDelay(pdMS_TO_TICKS(50));

  auto formatCoord = [](double val, uint8_t yPos) {
    std::string s = std::format("{:.4f}", val);
    if (s.length() > 6)
      s = s.substr(0, 6);

    std::vector<uint8_t> payload = {0x00, 0x99, 0x00, yPos, 0x04, 0x02, 0x0F};
    for (char c : s)
      payload.push_back(static_cast<uint8_t>(c));
    payload.push_back('\0');
    return payload;
  };

  sendCommand(Command::LUMA_TEXT, formatCoord(lat, 0x40));
  vTaskDelay(pdMS_TO_TICKS(50));
  sendCommand(Command::LUMA_TEXT, formatCoord(lon, 0x80));
}

} // namespace rta
