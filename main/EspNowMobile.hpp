#ifndef RTA_ESPNOW_MOBILE_HPP
#define RTA_ESPNOW_MOBILE_HPP

#include <cstdint>
#include <atomic>

namespace rta::espnow {

void init(std::atomic<uint16_t>& distanceRef);
bool isActive();

} // namespace rta::espnow

#endif // RTA_ESPNOW_MOBILE_HPP
