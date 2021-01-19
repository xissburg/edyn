#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

static constexpr size_t max_constrained_entities = 2;

struct constraint_row {
    std::array<entt::entity, max_constrained_entities> entity =
        make_array<max_constrained_entities>(entt::entity{entt::null});

    scalar error;

    // Error reduction parameter.
    scalar erp {0.2};

    scalar restitution {0};

    int priority {0};
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP