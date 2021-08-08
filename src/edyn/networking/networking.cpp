#include "edyn/networking/networking.hpp"
#include "edyn/networking/client_networking_context.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void init_networking_server(entt::registry &) {

}

void init_networking_client(entt::registry &registry) {
    registry.set<client_networking_context>();
}

}
