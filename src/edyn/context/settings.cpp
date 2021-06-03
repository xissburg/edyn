#include "edyn/context/settings.hpp"
#include "edyn/parallel/island_delta_builder.hpp"

namespace edyn {

std::unique_ptr<island_delta_builder> make_island_delta_builder_default() {
    return std::unique_ptr<island_delta_builder>(
        new island_delta_builder_impl(shared_components));
}

}
