#ifndef EDYN_NETWORKING_COMP_EXPORTER_MODIFIED_COMPONENTS_HPP
#define EDYN_NETWORKING_COMP_EXPORTER_MODIFIED_COMPONENTS_HPP

#include "edyn/config/config.h"
#include <cstdint>
#include <array>

namespace edyn {

template<uint16_t NumComponents>
struct exporter_modified_components {
    struct comp_index_time {
        uint16_t index;
        uint16_t remaining;
    };

    std::array<comp_index_time, NumComponents> entry {};
    uint16_t count {};
    static const uint16_t duration = 400;

    bool empty() const {
        for (unsigned i = 0; i < count; ++i) {
            if (entry[i].remaining > 0) {
                return false;
            }
        }
        return true;
    }

    void bump_index(uint16_t index) {
        EDYN_ASSERT(count < NumComponents);
        bool found = false;

        for (unsigned i = 0; i < count; ++i) {
            if (entry[i].index == index) {
                entry[i].remaining = duration;
                found = true;
            }
        }

        if (!found) {
            entry[count++] = {index, duration};
        }
    }

    void decay(unsigned elapsed_ms) {
        for (unsigned i = 0; i < count;) {
            if (elapsed_ms > entry[i].remaining) {
                // Assign value of last and decrement count.
                // Note that `i` isn't incremented in this case.
                entry[i] = entry[--count];
            } else {
                entry[i].remaining -= elapsed_ms;
                ++i;
            }
        }
    }
};

}

#endif // EDYN_NETWORKING_COMP_EXPORTER_MODIFIED_COMPONENTS_HPP
