#pragma once
#include <cstdint>

#include <atomic>

namespace rta {
namespace espnow {

void init(std::atomic<uint16_t>& distanceRef);
bool isActive();

} // namespace espnow
} // namespace rta
