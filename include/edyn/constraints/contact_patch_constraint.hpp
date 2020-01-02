#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT

#include <map>
#include <utility>
#include <entt/fwd.hpp>
#include "constraint_base.hpp"

namespace edyn {

struct contact_patch_constraint : public constraint_base<contact_patch_constraint> {
    struct brush_bristle {
        entt::entity entity;
        vector3 pivotA;
        vector3 pivotB;
    };

    static constexpr size_t num_rows = 3;

    scalar tread_stiffness {2000000};

    entt::entity normal_row_entity {entt::null};
    std::map<std::pair<size_t, size_t>, brush_bristle> bristles{};
    scalar prev_patch_start[num_rows] {0,0,0};
    scalar prev_patch_end[num_rows] {0,0,0};

    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT