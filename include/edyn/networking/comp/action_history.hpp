#ifndef EDYN_NETWORKING_COMP_ACTION_HISTORY_HPP
#define EDYN_NETWORKING_COMP_ACTION_HISTORY_HPP

#include "edyn/config/config.h"
#include <algorithm>
#include <limits>
#include <vector>
#include <cstdint>

namespace edyn {

/**
 * @brief A timestamped history of actions performed by a user.
 */
struct action_history {
    struct entry {
        double timestamp;
        std::vector<uint8_t> data;

        entry() = default;
        entry(double timestamp, std::vector<uint8_t> &&data)
            : timestamp(timestamp)
            , data(std::move(data))
        {}
    };

    using action_index_type = uint8_t;
    action_index_type action_index;
    std::vector<entry> entries;

    void erase_until(double timestamp) {
        auto it = std::find_if(entries.begin(), entries.end(),
                               [timestamp](auto &&entry) { return entry.timestamp >= timestamp; });
        entries.erase(entries.begin(), it);
    }

    void merge(const action_history &other) {
        EDYN_ASSERT(action_index == other.action_index);

        if (entries.empty()) {
            entries = other.entries;
            return;
        }

        // Only append newer entries.
        auto last_timestamp = other.entries.back().timestamp;
        auto other_begin = std::find_if(other.entries.begin(), other.entries.end(),
                                        [&](auto &&entry) { return entry.timestamp >= last_timestamp; });
        entries.insert(entries.end(), other_begin, other.entries.end());
    }

    void sort() {
        std::sort(entries.begin(), entries.end(), [] (auto &&lhs, auto &&rhs) {
            return lhs.timestamp < rhs.timestamp;
        });
    }
};

template<typename Archive>
void serialize(Archive &archive, action_history &history) {
    archive(history.action_index);
    using size_type = uint8_t;
    size_type size = static_cast<size_type>(std::min(history.entries.size(),
                                            static_cast<size_t>(std::numeric_limits<size_type>::max())));
    archive(size);
    history.entries.resize(size);

    for (size_type i = 0; i < size; ++i) {
        archive(history.entries[i].timestamp);
        archive(history.entries[i].data);
    }
}

}

#endif // EDYN_NETWORKING_COMP_ACTION_HISTORY_HPP
