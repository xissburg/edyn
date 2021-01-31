#ifndef EDYN_COMP_CONSTRAINT_GROUP_HPP
#define EDYN_COMP_CONSTRAINT_GROUP_HPP

#include <cstdint>

namespace edyn {

struct constraint_group {
    using value_t = uint32_t;
    static constexpr value_t null_group = 0;
    static constexpr value_t stitch_group = std::numeric_limits<value_t>::max();
    value_t value;
};

struct constraint_graph_node {};
struct constraint_graph_edge {};

}

#endif // EDYN_COMP_CONSTRAINT_GROUP_HPP