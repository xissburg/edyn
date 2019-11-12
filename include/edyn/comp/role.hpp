#ifndef EDYN_COMP_ROLE_HPP
#define EDYN_COMP_ROLE_HPP

#include <cstdint>

namespace edyn {

/**
 * Roles for lock-free triple buffering.
 */
enum comp_role : uint8_t {
    none,
    ready,
    inprogress,
    presented
};

}


#endif // EDYN_COMP_ROLE_HPP