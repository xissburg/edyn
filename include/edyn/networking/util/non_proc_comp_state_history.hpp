#ifndef EDYN_NETWORKING_NON_PROC_COMP_STATE_HISTORY_HPP
#define EDYN_NETWORKING_NON_PROC_COMP_STATE_HISTORY_HPP

#include <deque>
#include <mutex>
#include <shared_mutex>
#include "edyn/parallel/island_delta.hpp"
#include "edyn/parallel/make_island_delta_builder.hpp"
#include "edyn/networking/comp/non_proc_comp_list.hpp"

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

    #ifdef EDYN_DEBUG
        if (!history.empty()) {
            EDYN_ASSERT(history.back().timestamp < timestamp);
        }
    #endif

        history.emplace_back(std::move(delta), timestamp);

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

    island_delta * get_first_before(double time) {
        for (auto it = history.rbegin(); it != history.rend(); ++it) {
            if (it->timestamp < time) {
                return &it->delta;
            }
        }
        return nullptr;
    }

private:
    std::deque<element> history;
    size_t max_size {100};
    std::mutex mutex;
};

}

#endif // EDYN_NETWORKING_NON_PROC_COMP_STATE_HISTORY_HPP
