#ifndef EDYN_COMP_ACTION_LIST_HPP
#define EDYN_COMP_ACTION_LIST_HPP

#include <limits>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace edyn {

template<typename Action>
struct action_list {
    std::vector<Action> actions;
};

template<typename Archive, typename Action>
void serialize(Archive &archive, action_list<Action> &list) {
    using size_type = uint8_t;
    auto size = static_cast<size_type>(std::min(list.actions.size(),
                                       static_cast<size_t>(std::numeric_limits<size_type>::max())));
    archive(size);
    list.actions.resize(size);

    for (size_type i = 0; i < size; ++i) {
        archive(list.actions[i]);
    }
}

// Declare custom merge function to accumulate actions instead of replace.
template<typename Action>
void merge_component(action_list<Action> &list, const action_list<Action> &new_value) {
    // Accumulate received actions. Action updates are only sent from
    // coordinator to worker.
    list.actions.insert(list.actions.end(), new_value.actions.begin(), new_value.actions.end());
}

}

#endif // EDYN_COMP_ACTION_LIST_HPP
