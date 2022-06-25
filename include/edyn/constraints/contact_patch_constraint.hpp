#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT

#include <map>
#include <array>
#include <utility>
#include "edyn/config/constants.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/comp/spin.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/util/array.hpp"

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
        scalar sliding_spd{0};
    };

    struct tread_row {
        scalar half_angle{0};
        scalar half_length{0};
        vector3 start_pos; // Position where it starts in world space.
        vector3 end_pos; // Position where it ends in world space.
        vector3 start_posB; // Position where it starts in B's object space.
        vector3 end_posB; // Position where it ends in B's object space.
        std::array<brush_bristle, bristles_per_row> bristles;
    };

    struct contact_patch {
        std::array<tread_row, num_tread_rows> tread_rows{};

        // Spin angle at contact point.
        scalar angle{};
        long spin_count{0};

        // Read-only stats.
        vector3 normal;
        scalar normal_impulse;
        scalar friction;
        uint32_t lifetime {0};
        vector3 lon_dir; // Longitudinal tire direction.
        vector3 lat_dir; // Lateral tire direction.
        vector3 pivot; // Center of pressure where forces are applied.
        vector3 center; // Geometric center of contact patch.
        scalar deflection{0}; // Vertical tire deflection.
        scalar sin_camber; // Sine of camber angle.
        scalar sliding_spd_avg; // Average of sliding speed of all bristles.
        scalar sliding_ratio; // Percentage of bristles which are sliding.
        scalar width; // Width of contact patch.
        scalar length;
    };

    uint8_t num_patches {0};
    std::array<contact_patch, max_contacts> patch;

    // Tire material properties.
    scalar m_normal_stiffness{100000};
    scalar m_normal_damping{400};
    scalar m_speed_sensitivity{0.05};
    scalar m_load_sensitivity{0.03};
    scalar m_lon_tread_stiffness{3000000};
    scalar m_lat_tread_stiffness{1800000};
    scalar m_sidewall_height{0.13};

    static const auto num_rows = 4 * max_contacts;
    std::array<scalar, num_rows> impulse = make_array<num_rows>(scalar{});
};

template<>
void prepare_constraints<contact_patch_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
