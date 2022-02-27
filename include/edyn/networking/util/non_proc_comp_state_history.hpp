#ifndef EDYN_NETWORKING_NON_PROC_COMP_STATE_HISTORY_HPP
#define EDYN_NETWORKING_NON_PROC_COMP_STATE_HISTORY_HPP

#include <deque>
#include <mutex>
#include <shared_mutex>
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/make_island_delta_builder.hpp"

namespace edyn {

class non_proc_comp_state_history {
public:
    struct element {
        island_delta delta;
        double timestamp;

        element() = default;
        element(island_delta &&delta, double timestamp)
            : delta(std::move(delta))
            , timestamp(timestamp)
        {}
    };

    void emplace(island_delta &&delta, double timestamp) {
        std::lock_guard lock(mutex);

        // Sorted insertion.
        auto it = std::find_if(history.begin(), history.end(),
                               [timestamp] (auto &&elem) { return elem.timestamp > timestamp; });
        history.insert(it, {std::move(delta), timestamp});

        if (history.size() > max_size) {
            history.pop_front();
        }
    }

    template<typename Func>
    void each(double start_time, double length_of_time, Func func) {
        std::lock_guard lock(mutex);

        for (auto &elem : history) {
            if (elem.timestamp >= start_time + length_of_time) {
                break;
            }

            if (elem.timestamp >= start_time) {
                func(elem.delta, elem.timestamp);
            }
        }
    }

    template<typename Func>
    void until(double time, Func func) {
        std::lock_guard lock(mutex);

        for (auto &elem : history) {
            if (elem.timestamp >= time) {
                break;
            }

            func(elem.delta, elem.timestamp);
        }
    }

private:
    std::deque<element> history;
    size_t max_size {100};
    mutable std::mutex mutex;
};

}

#endif // EDYN_NETWORKING_NON_PROC_COMP_STATE_HISTORY_HPP
