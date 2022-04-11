#include "edyn/networking/networking_external.hpp"
#include "edyn/networking/comp/networked_comp.hpp"

namespace edyn {

bool(*g_is_networked_component)(entt::id_type) = internal::make_default_is_networked_component_func();

bool(*g_is_networked_input_component)(entt::id_type) = [](entt::id_type id) { return false; };

void(*g_mark_replaced_network_dirty)(entt::registry &, const registry_operation_collection &,
    const entity_map &, double timestamp) = internal::make_mark_replaced_network_dirty_func(networked_components);

}
