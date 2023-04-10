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
    using action_index_type = uint8_t;

    struct entry {
        double timestamp;
        action_index_type action_index; // Index of action type.
        std::vector<uint8_t> data; // Serialized action list binary data.

        entry() = default;
        entry(double timestamp, action_index_type action_index, std::vector<uint8_t> &&data)
            : timestamp(timestamp)
            , action_index(action_index)
            , data(std::move(data))
        {}
    };

    std::vector<entry> entries;
    double last_timestamp {};

    bool empty() const {
        return entries.empty();
    }

    void erase_until(double timestamp) {
        auto it = std::find_if(entries.begin(), entries.end(),
                               [timestamp](auto &&entry) { return entry.timestamp >= timestamp; });
        entries.erase(entries.begin(), it);
    }

    void merge(const action_history &other) {
        EDYN_ASSERT(!other.empty());

        // Only append newer entries.
        auto other_begin = std::find_if(other.entries.begin(), other.entries.end(),
                                        [&](auto &&entry) { return entry.timestamp > last_timestamp; });
        entries.insert(entries.end(), other_begin, other.entries.end());

        if (!entries.empty()) {
            // Assign new highest timestamp yet inserted.
            last_timestamp = entries.back().timestamp;
        }
    }

    void sort() {
        std::sort(entries.begin(), entries.end(), [](auto &&lhs, auto &&rhs) {
            return lhs.timestamp < rhs.timestamp;
        });
    }
};

template<typename Archive>
void serialize(Archive &archive, action_history &history) {
    using size_type = uint8_t;
    size_type size = static_cast<size_type>(std::min(history.entries.size(),
                                            static_cast<size_t>(std::numeric_limits<size_type>::max())));
    archive(size);
    history.entries.resize(size);

    for (size_type i = 0; i < size; ++i) {
        archive(history.entries[i].timestamp);
        archive(history.entries[i].action_index);
        archive(history.entries[i].data);
    }
}

}

#endif // EDYN_NETWORKING_COMP_ACTION_HISTORY_HPP
