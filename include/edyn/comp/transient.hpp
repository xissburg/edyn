#ifndef EDYN_COMP_TRANSIENT_HPP
#define EDYN_COMP_TRANSIENT_HPP

#include <entt/core/fwd.hpp>
#include <entt/core/hashed_string.hpp>
#include <edyn/serialization/std_s11n.hpp>
#include <vector>

namespace edyn {

/**
 * @brief Holds a list of component type ids that must be synchronized with the
 * main registry after every step of the simulation.
 * @remark Only relevant when running Edyn in asynchronous mode.
 */
struct transient {
    std::vector<entt::id_type> ids;
};

template<typename Archive>
void serialize(Archive &archive, transient &tr) {
    archive(tr.ids);
}

}

#endif // EDYN_COMP_TRANSIENT_HPP
