#include "edyn/init.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/gravity.hpp"
#include <entt/meta/factory.hpp>

namespace edyn {

static bool g_edyn_initialized {false};

static void init_meta() {
    entt::meta<island>().type()
        .data<&island::entities, entt::as_ref_t>("entities"_hs);

    entt::meta<island_node>().type()
        .data<&island_node::entities, entt::as_ref_t>("entities"_hs);

    entt::meta<island_container>().type()
        .data<&island_container::entities, entt::as_ref_t>("entities"_hs);

    entt::meta<contact_point>().type()
        .data<&contact_point::body, entt::as_ref_t>("parent"_hs);

    entt::meta<contact_manifold>().type()
        .data<&contact_manifold::body, entt::as_ref_t>("body"_hs)
        .data<&contact_manifold::point, entt::as_ref_t>("point"_hs);

    entt::meta<constraint_row>().type()
        .data<&constraint_row::entity, entt::as_ref_t>("entity"_hs);

    entt::meta<constraint>().type()
        .data<&constraint::body, entt::as_ref_t>("entity"_hs)
        .data<&constraint::row, entt::as_ref_t>("row"_hs);

    entt::meta<gravity>().type()
        .data<&gravity::body, entt::as_ref_t>("body"_hs);

}

void init() {
    if (g_edyn_initialized) return;

    init_meta();
    edyn::job_dispatcher::global().start();

    g_edyn_initialized = true;
}

}