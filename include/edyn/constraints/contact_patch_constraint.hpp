#ifndef EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
#define EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT

#include <map>
#include <array>
#include <utility>
#include <vector>
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct constraint_row_prep_cache;
struct contact_manifold;

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
        scalar length; // Average length of contact patch.
        contact_manifold::contact_id_type contact_id;
        vector3 centerA;
        vector3 normalA;

        struct {
            scalar normal {};
            scalar longitudinal {};
            scalar lateral {};
            scalar aligning {};
        } applied_impulse {};
    };

    uint8_t num_patches {0};
    std::array<contact_patch, max_contacts> patches;

    void prepare(const entt::registry &, entt::entity, const contact_manifold &,
                 constraint_row_prep_cache &cache, scalar dt,
                 const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses, contact_manifold &manifold);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_PATCH_CONSTRAINT
