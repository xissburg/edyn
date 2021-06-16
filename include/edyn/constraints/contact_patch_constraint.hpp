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
    struct brush_bristle {
        vector3 pivotA;
        vector3 pivotB;
        vector3 root;
        vector3 tip;
        vector3 deflection;
        scalar friction;
        scalar sliding_spd {0};
    };

    struct tread_row {
        scalar prev_contact_angle {0};
        scalar prev_row_half_angle {0};
        long prev_spin_count {0};
        scalar tread_width;
        scalar tread_area;
        scalar patch_half_length;
        std::map<size_t, brush_bristle> bristles;
    };

    scalar m_normal_stiffness {100000};
    scalar m_normal_damping {400};
    scalar m_speed_sensitivity {0.05};
    scalar m_load_sensitivity {0.03};
    scalar m_lon_tread_stiffness {3000000};
    scalar m_lat_tread_stiffness {1800000};
    scalar m_tread_damping {2000};

    vector3 m_lon_dir;
    vector3 m_lat_dir;
    vector3 m_pivot;
    vector3 m_center;
    vector3 m_normal;
    scalar m_deflection;
    scalar m_sin_camber;
    scalar m_sliding_spd_avg;
    scalar m_contact_len_avg;
    scalar m_contact_width;

    scalar m_normal_relspd;
    scalar m_lon_damping;
    scalar m_lat_damping;
    scalar m_aligning_damping;

    static constexpr size_t num_tread_rows = 3;
    std::array<tread_row, num_tread_rows> m_tread_rows{};

    void clear();
};

template<>
void prepare_constraints<contact_patch_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
void iterate_constraints<contact_patch_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT