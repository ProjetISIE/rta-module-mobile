#include "active_look.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include <array>
#include <cstring>
#include <format>
#include <string>

void ActiveLook::sendCommand(Command cmd, std::span<const uint8_t> payload) {
  if (!connection_handle_)
    return;

  // Construction de la trame : [0xFF, CMD, LEN_H, LEN_L, PAYLOAD..., 0xAA]
  const size_t total_len = 4 + payload.size() + 1;
  std::vector<uint8_t> buf;
  buf.reserve(total_len);

  buf.push_back(0xFF);
  buf.push_back(static_cast<uint8_t>(cmd));
  buf.push_back(static_cast<uint8_t>(total_len >> 8));
  buf.push_back(static_cast<uint8_t>(total_len & 0xFF));

  if (!payload.empty()) {
    buf.insert(buf.end(), payload.begin(), payload.end());
  }
  buf.push_back(0xAA);

  for (uint16_t h : COMMAND_HANDLES) {
    ble_gattc_write_no_rsp_flat(*connection_handle_, h, buf.data(), buf.size());
  }
}

void ActiveLook::initializeDisplay(uint16_t conn_handle) {
  connection_handle_ = conn_handle;

  // Allumage et configuration de l'écran
  const uint8_t vOn = 0x01;
  const uint8_t vFlip = 0x02;
  sendCommand(Command::POWER, std::span(&vOn, 1));
  sendCommand(Command::CONFIG, std::span(&vFlip, 1));

  vTaskDelay(pdMS_TO_TICKS(50));
  displayText("RTA OK");
}

void ActiveLook::displayText(const char *msg) {
  if (!connection_handle_)
    return;

  sendCommand(Command::CLEAR);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Configuration LumaText : {X, Y, Rotation, Font, Couleur}
  // Ici on utilise des valeurs par défaut pour centrer ou positionner
  std::vector<uint8_t> txt = {0x00, 0x99, 0x00, 0x60, 0x04, 0x02, 0x0F};

  std::string_view sv(msg);
  if (sv.length() > 50)
    sv = sv.substr(0, 50);

  for (char c : sv)
    txt.push_back(static_cast<uint8_t>(c));
  txt.push_back('\0');

  sendCommand(Command::LUMA_TEXT, txt);
}

void ActiveLook::displayNumber(int value) {
  displayText(std::format("{}", value).c_str());
}

void ActiveLook::displayGpsWait() { displayText("Wait..."); }

void ActiveLook::displayCoordinates(double lat, double lon) {
  if (!connection_handle_)
    return;

  sendCommand(Command::CLEAR);
  vTaskDelay(pdMS_TO_TICKS(50));

  auto format_coord = [](double val, uint8_t y_pos) {
    // Formatage : "%.4f" puis troncature à 6 caractères
    std::string s = std::format("{:.4f}", val);
    if (s.length() > 6)
      s = s.substr(0, 6);

    std::vector<uint8_t> payload = {0x00, 0x99, 0x00, y_pos, 0x04, 0x02, 0x0F};
    for (char c : s)
      payload.push_back(static_cast<uint8_t>(c));
    payload.push_back('\0');
    return payload;
  };

  sendCommand(Command::LUMA_TEXT, format_coord(lat, 0x40));
  vTaskDelay(pdMS_TO_TICKS(50));
  sendCommand(Command::LUMA_TEXT, format_coord(lon, 0x80));
}
