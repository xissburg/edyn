#ifndef EDYN_COMP_CONSTRAINT_GROUP_HPP
#define EDYN_COMP_CONSTRAINT_GROUP_HPP

#include <cstdint>

namespace edyn {

struct constraint_group {
    using value_t = uint32_t;
    static constexpr value_t null_group = 0;
    static constexpr value_t first_group = 1;
    static constexpr value_t seam_group = std::numeric_limits<value_t>::max();
    value_t value {null_group};
};

struct constraint_graph_node {
    constraint_group::value_t group_value {constraint_group::null_group};
};

struct constraint_graph_edge {
    constraint_group::value_t group_value {constraint_group::null_group};
};

}

#endif // EDYN_COMP_CONSTRAINT_GROUP_HPP