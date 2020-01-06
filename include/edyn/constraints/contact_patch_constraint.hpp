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
        scalar tread_area;
        scalar force;
        scalar friction;
    };

    struct tread_row {
        scalar prev_patch_start {0};
        scalar prev_patch_end {0};
        scalar tread_width;
        scalar patch_half_length;
        std::map<size_t, brush_bristle> bristles;
    };

    static constexpr size_t num_rows = 3;

    scalar friction_coefficient {1};
    scalar speed_sensitivity {0.03};
    scalar tread_stiffness {2000000};

    entt::entity normal_row_entity {entt::null};
    std::array<tread_row, num_rows> tread_rows{};

    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT