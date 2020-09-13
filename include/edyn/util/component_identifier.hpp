#ifndef EDYN_UTIL_COMPONENT_IDENTIFIER_HPP
#define EDYN_UTIL_COMPONENT_IDENTIFIER_HPP

#include <cstdint>
#include "edyn/util/tuple.hpp"

namespace edyn {

using component_identifier_t = uint16_t;

template<typename... Components>
struct component_identifier {
    template<typename Component>
    inline static constexpr auto value = index_of_v<component_identifier_t, Component, Components...>;
};

}

#endif // EDYN_UTIL_COMPONENT_IDENTIFIER_HPP