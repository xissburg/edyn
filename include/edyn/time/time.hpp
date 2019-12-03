#ifndef EDYN_TIME_TIME_HPP
#define EDYN_TIME_TIME_HPP

#include <cstdint>

namespace edyn {

uint32_t ticks();
void delay(uint32_t ms);
uint64_t performance_counter();
uint64_t performance_frequency();

}

#endif // EDYN_TIME_TIME_HPP
