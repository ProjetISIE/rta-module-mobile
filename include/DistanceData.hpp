#ifndef RTA_DISTANCE_DATA_HPP
#define RTA_DISTANCE_DATA_HPP

#include <atomic>
#include <cstdint>

namespace rta {

/**
 * @brief Thread-safe storage for the received distance from the fixed module.
 * Value is in millimeters. 0xFFFF indicates invalid/unavailable distance.
 */
inline std::atomic<uint16_t> globalReceivedDistance{0xFFFF};

} // namespace rta

#endif // RTA_DISTANCE_DATA_HPP
