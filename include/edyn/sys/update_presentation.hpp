#ifndef EDYN_SYS_UPDATE_PRESENTATION_HPP
#define EDYN_SYS_UPDATE_PRESENTATION_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

void update_presentation(entt::registry &registry, double time);

void snap_presentation(entt::registry &registry);

}

#endif // EDYN_SYS_UPDATE_PRESENTATION_HPP