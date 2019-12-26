#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT

#include <entt/fwd.hpp>
#include "constraint_base.hpp"

namespace edyn {

struct contact_patch_constraint : public constraint_base<contact_patch_constraint> {

    void init(entt::entity, constraint &, const relation &, entt::registry &);
    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT