#ifndef EDYN_CONTEXT_ASYNC_SETTINGS_HPP
#define EDYN_CONTEXT_ASYNC_SETTINGS_HPP

#include "edyn/comp/transient.hpp"
#include <optional>

namespace edyn {

/**
 * @brief Settings specific to asynchronous execution.
 */
struct async_settings {
    std::optional<transient> contact_points_transient;
};

}

#endif // EDYN_CONTEXT_ASYNC_SETTINGS_HPP
