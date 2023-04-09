#ifndef EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_OPERATION_HPP
#define EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_OPERATION_HPP

#include "edyn/replication/registry_operation.hpp"

namespace edyn {

struct extrapolation_operation_create {
    registry_operation ops;
    std::vector<entt::entity> owned_entities;
};

struct extrapolation_operation_destroy {
    std::vector<entt::entity> entities;
};

}

#endif // EDYN_NETWORKING_EXTRAPOLATION_EXTRAPOLATION_OPERATION_HPP
