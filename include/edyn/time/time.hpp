#ifndef EDYN_TIME_TIME_HPP
#define EDYN_TIME_TIME_HPP

#include <cstdint>

namespace edyn {

/**
 * @brief Get the number of milliseconds since library initialization.
 * @return Milliseconds since the library initialized.
 */
uint32_t ticks();

/**
 * @brief Wait a specified number of milliseconds before returning.
 * @param ms The number of milliseconds to delay.
 */
void delay(uint32_t ms);

/**
 * @brief Get the current value of the high resolution counter.
 * @return Counter value.
 */
uint64_t performance_counter();

/**
 * @brief Get the count per second of the high resolution counter.
 * @return Count per second.
 */
uint64_t performance_frequency();

/**
 * @brief Returns the current time in seconds using the performance counter.
 * I.e. `performance_counter` divided by `performance_frequency`.
 * @return Current time in seconds.
 */
double performance_time();

}

#endif // EDYN_TIME_TIME_HPP
