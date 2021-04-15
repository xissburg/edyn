#ifndef EDYN_COMP_CONTACT_CONSTRAINT_HPP
#define EDYN_COMP_CONTACT_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "edyn/math/constants.hpp"

namespace edyn {

class row_cache;
struct constraint;
struct constraint_row_data;

struct contact_constraint {
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    void prepare(entt::entity, const constraint &, entt::registry &, row_cache &cache, scalar dt);
    void iteration(entt::entity, const constraint &, entt::registry &, row_cache &, size_t row_index, scalar dt);

private:
    scalar m_friction;
};

}

#endif // EDYN_COMP_CONTACT_CONSTRAINT_HPP