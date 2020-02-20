#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_SPRING_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_SPRING_CONSTRAINT

#include <map>
#include <array>
#include <utility>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/constants.hpp"
#include "constraint_base.hpp"

namespace edyn {

struct contact_patch_spring_constraint : public constraint_base<contact_patch_spring_constraint> {
    struct brush_bristle {
        entt::entity entity;
        vector3 pivotA;
        vector3 pivotB;
        scalar tread_area;
        scalar force;
        scalar friction;
    };

    struct tread_row {
        scalar prev_patch_start_angle {0};
        scalar prev_patch_end_angle {0};
        scalar tread_width;
        scalar patch_half_length;
        std::map<size_t, brush_bristle> bristles;
    };

    scalar stiffness {large_scalar};
    scalar damping {large_scalar};
    scalar friction_coefficient {1};
    scalar speed_sensitivity {0.03};
    scalar tread_stiffness {7000000};

    static constexpr size_t num_tread_rows = 3;

    entt::entity normal_row_entity {entt::null};
    std::array<tread_row, num_tread_rows> tread_rows{};

    void clear(entt::registry &, constraint &);

    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_SPRING_CONSTRAINT