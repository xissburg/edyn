#ifndef EDYN_PARALLEL_MESSAGE_HPP
#define EDYN_PARALLEL_MESSAGE_HPP

#include <vector>
#include <cstdint>

namespace edyn::msg {

struct registry_snapshot {
    std::vector<uint8_t> data;
};

}

#endif // EDYN_PARALLEL_MESSAGE_HPP