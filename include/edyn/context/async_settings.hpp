#ifndef EDYN_CONTEXT_ASYNC_SETTINGS_HPP
#define EDYN_CONTEXT_ASYNC_SETTINGS_HPP

namespace edyn {

/**
 * @brief Settings specific to asynchronous execution.
 */
struct async_settings {
    bool sync_contact_points {false};
};

}

#endif // EDYN_CONTEXT_ASYNC_SETTINGS_HPP
