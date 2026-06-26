#include "EspNowMobile.hpp"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include <atomic>

namespace rta::espnow {

namespace {
constexpr const char* kTag = "ESP_NOW_MOBILE";
std::atomic<int64_t> lastPacketTimeUs{0};
std::atomic<uint16_t>* sDistanceRef = nullptr;
} // namespace

#pragma pack(push, 1)
struct EspNowDistancePacket {
    uint8_t magic_[4]; // NOLINT(modernize-avoid-c-arrays, hicpp-avoid-c-arrays,
                       // cppcoreguidelines-avoid-c-arrays)
    uint16_t distanceMm_;
};
#pragma pack(pop)

namespace {
void recvCb([[maybe_unused]] const esp_now_recv_info_t* espNowInfo,
            const uint8_t* data, int len) {
    if (len == sizeof(EspNowDistancePacket)) {
        auto* pkt = reinterpret_cast<const EspNowDistancePacket*>(data);
        if (pkt->magic_[0] == 'R' && pkt->magic_[1] == 'T' &&
            pkt->magic_[2] == 'A' && pkt->magic_[3] == '!') {
            if (sDistanceRef != nullptr) {
                sDistanceRef->store(pkt->distanceMm_,
                                    std::memory_order_relaxed);
            }
            lastPacketTimeUs.store(esp_timer_get_time());
        }
    }
}
} // namespace

bool isActive() {
    int64_t last = lastPacketTimeUs.load();
    if (last == 0) {
        return false;
    }
    return (esp_timer_get_time() - last) <
           5000000; // NOLINT(cppcoreguidelines-avoid-magic-numbers)
}

void init(std::atomic<uint16_t>& distanceRef) {
    sDistanceRef = &distanceRef;
    // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,misc-const-correctness)
    ESP_LOGI(kTag, "Initializing ESP-NOW (Mobile)...");

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                         WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recvCb));

    ESP_LOGI(kTag, "ESP-NOW mobile initialized and listening");
    // NOLINTEND(cppcoreguidelines-pro-type-vararg,misc-const-correctness)
}

} // namespace rta::espnow
