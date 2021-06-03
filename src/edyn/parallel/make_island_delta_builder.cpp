#include "edyn/parallel/make_island_delta_builder.hpp"
#include "edyn/parallel/island_delta_builder.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/config/config.h"
#include <entt/entt.hpp>

namespace edyn {

std::unique_ptr<island_delta_builder> make_island_delta_builder(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    EDYN_ASSERT(settings.make_island_delta_builder != nullptr);
    return (*settings.make_island_delta_builder)();
}

}
