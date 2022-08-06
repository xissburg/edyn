#ifndef EDYN_CONFIG_EXECUTION_MODE_HPP
#define EDYN_CONFIG_EXECUTION_MODE_HPP

namespace edyn {

enum class execution_mode {
    sequential,
    sequential_multithreaded,
    asynchronous
};

}

#endif // EDYN_CONFIG_EXECUTION_MODE_HPP
