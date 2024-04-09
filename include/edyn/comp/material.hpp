#ifndef EDYN_COMP_MATERIAL_HPP
#define EDYN_COMP_MATERIAL_HPP

#include <cstdint>
#include <limits>
#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

/**
 * @brief Base material info which can be set into the material mixing table
 * for a pair of material ids.
 */
struct material_base {
    scalar restitution {0};
    scalar friction {0.5};
    scalar spin_friction {0};
    scalar roll_friction {0};
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};
};

/**
 * @brief Rigid body material component with optional identifier for material mixing.
 */
struct material : public material_base {
    using id_type = uint16_t;
    static constexpr auto UnassignedID = std::numeric_limits<id_type>::max();
    id_type id {UnassignedID};
};

template<typename Archive>
void serialize(Archive &archive, material &m) {
    archive(m.restitution);
    archive(m.friction);
    archive(m.spin_friction);
    archive(m.roll_friction);
    archive(m.stiffness);
    archive(m.damping);
}

}

#endif // EDYN_COMP_MATERIAL_HPP
