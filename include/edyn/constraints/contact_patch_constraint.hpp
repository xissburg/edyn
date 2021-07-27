#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT

#include <map>
#include <array>
#include <utility>
#include "edyn/math/vector3.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct contact_patch_constraint : public constraint_base {
    static constexpr size_t bristles_per_row = 10;
    static constexpr size_t num_tread_rows = 3;

    struct brush_bristle {
        vector3 pivotA; // Root in A's object space.
        vector3 pivotB; // Tip in B's object space.
        vector3 root; // Root in world space.
        vector3 tip; // Tip in world space.
        scalar friction;
        scalar sliding_spd {0};
    };

    struct tread_row {
        scalar half_angle {0};
        scalar half_length {0};
        vector3 start_pos; // Position where it starts in world space.
        vector3 end_pos; // Position where it ends in world space.
        vector3 start_posB; // Position where it starts in B's object space.
        vector3 end_posB; // Position where it ends in B's object space.
        std::array<brush_bristle, bristles_per_row> bristles;
    };

    // Tire material properties.
    scalar m_normal_stiffness {100000};
    scalar m_normal_damping {400};
    scalar m_speed_sensitivity {0.05};
    scalar m_load_sensitivity {0.03};
    scalar m_lon_tread_stiffness {3000000};
    scalar m_lat_tread_stiffness {1800000};

    // Spin angle at contact point.
    scalar m_contact_angle;
    long m_spin_count {0};

    // Read-only stats.
    vector3 m_lon_dir; // Longitudinal tire direction.
    vector3 m_lat_dir; // Lateral tire direction.
    vector3 m_pivot; // Center of pressure where forces are applied.
    vector3 m_center; // Geometric center of contact patch.
    scalar m_deflection {0}; // Vertical tire deflection.
    scalar m_sin_camber; // Sine of camber angle.
    scalar m_sliding_spd_avg; // Average of sliding speed of all bristles.
    scalar m_sliding_ratio; // Percentage of bristles which are sliding.
    scalar m_contact_width; // Width of contact patch.

    std::array<tread_row, num_tread_rows> m_tread_rows{};
};

void initialize_contact_patch_constraint(entt::registry &, entt::entity);

template<>
void prepare_constraints<contact_patch_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT