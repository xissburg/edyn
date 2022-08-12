#include "edyn/networking/networking_external.hpp"
#include "edyn/networking/comp/networked_comp.hpp"

namespace edyn {

bool(*g_is_networked_component)(entt::id_type) = internal::make_is_networked_component_func(networked_components);

bool(*g_is_networked_input_component)(entt::id_type) = internal::make_is_network_input_component_func(networked_components);

bool(*g_is_action_list_component)(entt::id_type) = [](entt::id_type) { return false; };

void(*g_mark_replaced_network_dirty)(entt::registry &, const registry_operation &,
    const entity_map &, double timestamp) = internal::make_mark_replaced_network_dirty_func(networked_components);

}
