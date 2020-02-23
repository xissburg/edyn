#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT

#include <map>
#include <array>
#include <utility>
#include <entt/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/constants.hpp"
#include "constraint_base.hpp"

namespace edyn {

struct contact_patch_constraint : public constraint_base<contact_patch_constraint> {
    struct brush_bristle {
        vector3 pivotA;
        vector3 pivotB;
        vector3 root;
        vector3 tip;
        vector3 deflection;
        vector3 damping_force;
        scalar tread_area;
        scalar friction;
    };

    struct tread_row {
        scalar prev_patch_start_angle {0};
        scalar prev_patch_end_angle {0};
        scalar tread_width;
        scalar patch_half_length;
        std::map<size_t, brush_bristle> bristles;
    };

    scalar m_stiffness {large_scalar};
    scalar m_damping {large_scalar};
    scalar m_friction_coefficient {1};
    scalar m_speed_sensitivity {0.03};
    scalar m_tread_stiffness {7000000};

    vector3 m_lon_dir;
    vector3 m_lat_dir;
    vector3 m_patch_center;

    static constexpr size_t num_tread_rows = 3;
    std::array<tread_row, num_tread_rows> m_tread_rows{};

    void clear(entt::registry &, constraint &);

    void prepare(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
    void iteration(entt::entity, constraint &, const relation &, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT