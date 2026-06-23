#include "EspNowMobile.hpp"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include <atomic>

extern "C" uint16_t gattReceivedDistance;

namespace rta {
namespace espnow {

static const char* TAG = "ESP_NOW_MOBILE";
static std::atomic<int64_t> lastPacketTimeUs{0};

#pragma pack(push, 1)
struct EspNowDistancePacket {
    uint8_t magic[4];
    uint16_t distance_mm;
};
#pragma pack(pop)

static void recv_cb(const esp_now_recv_info_t* esp_now_info,
                    const uint8_t* data, int len) {
    if (len == sizeof(EspNowDistancePacket)) {
        const EspNowDistancePacket* pkt =
            reinterpret_cast<const EspNowDistancePacket*>(data);
        if (pkt->magic[0] == 'R' && pkt->magic[1] == 'T' &&
            pkt->magic[2] == 'A' && pkt->magic[3] == '!') {
            gattReceivedDistance = pkt->distance_mm;
            lastPacketTimeUs.store(esp_timer_get_time());
        }
    }
}

bool isActive() {
    int64_t last = lastPacketTimeUs.load();
    if (last == 0) return false;
    return (esp_timer_get_time() - last) < 5000000; // 5 seconds timeout
}

void init() {
    ESP_LOGI(TAG, "Initializing ESP-NOW (Mobile)...");

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_protocol(
        WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                         WIFI_PROTOCOL_LR));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    ESP_LOGI(TAG, "ESP-NOW mobile initialized and listening");
}

} // namespace espnow
} // namespace rta
