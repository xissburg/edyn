#include "edyn/edyn.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/collision/broadphase_main.hpp"
#include "edyn/networking/comp/entity_owner.hpp"
#include "edyn/parallel/island_coordinator.hpp"
#include "edyn/sys/update_presentation.hpp"
#include "edyn/dynamics/material_mixing.hpp"
#include "edyn/collision/tree_view.hpp"
#include <entt/meta/factory.hpp>
#include <entt/core/hashed_string.hpp>

namespace edyn {

static void init_meta() {
    using namespace entt::literals;

    entt::meta<contact_manifold>().type()
        .data<&contact_manifold::body, entt::as_ref_t>("body"_hs);

    entt::meta<collision_exclusion>().type()
        .data<&collision_exclusion::entity, entt::as_ref_t>("entity"_hs);

    std::apply([&] (auto ... c) {
        (entt::meta<decltype(c)>().type().template data<&decltype(c)::body, entt::as_ref_t>("body"_hs), ...);
    }, constraints_tuple);

    entt::meta<tree_view>().type()
        .data<&tree_view::m_nodes, entt::as_ref_t>("nodes"_hs);
    entt::meta<tree_view::tree_node>().type()
        .data<&tree_view::tree_node::entity, entt::as_ref_t>("entity"_hs);

    entt::meta<entity_owner>().type()
        .data<&entity_owner::client_entity, entt::as_ref_t>("client_entity"_hs);
}

void init(const init_config &config) {
    init_meta();

    auto &dispatcher = job_dispatcher::global();

    if (!dispatcher.running()) {
        if (config.num_worker_threads == 0) {
            dispatcher.start();
        } else {
            dispatcher.start(config.num_worker_threads);
        }

        dispatcher.assure_current_queue();
    }
}

void deinit() {
    job_dispatcher::global().stop();
}

void attach(entt::registry &registry) {
    registry.set<settings>();
    registry.set<entity_graph>();
    registry.set<contact_manifold_map>(registry);
    registry.set<island_coordinator>(registry);
    registry.set<broadphase_main>(registry);
    registry.set<material_mix_table>();
}

void detach(entt::registry &registry) {
    registry.unset<settings>();
    registry.unset<entity_graph>();
    registry.unset<contact_manifold_map>();
    registry.unset<island_coordinator>();
    registry.unset<broadphase_main>();
    registry.unset<material_mix_table>();
}

scalar get_fixed_dt(const entt::registry &registry) {
    return registry.ctx<settings>().fixed_dt;
}

void set_fixed_dt(entt::registry &registry, scalar dt) {
    registry.ctx<settings>().fixed_dt = dt;
    registry.ctx<island_coordinator>().settings_changed();
}

bool is_paused(const entt::registry &registry) {
    return registry.ctx<settings>().paused;
}

void set_paused(entt::registry &registry, bool paused) {
    registry.ctx<settings>().paused = paused;
    registry.ctx<island_coordinator>().set_paused(paused);
}

void update(entt::registry &registry) {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    // Do island management. Merge updated entity state into main registry.
    registry.ctx<island_coordinator>().update();

    // Perform broad-phase between different islands and create contact manifolds
    // between them which will later cause islands to be merged into one.
    registry.ctx<broadphase_main>().update();

    if (is_paused(registry)) {
        snap_presentation(registry);
    } else {
        auto time = performance_time();
        update_presentation(registry, time);
    }
}

void step_simulation(entt::registry &registry) {
    EDYN_ASSERT(is_paused(registry));
    registry.ctx<island_coordinator>().step_simulation();
}

void remove_external_components(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.make_reg_op_builder = &make_reg_op_builder_default;
    settings.index_source.reset(new component_index_source_impl(shared_components));
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_init(entt::registry &registry, external_system_func_t func) {
    registry.ctx<settings>().external_system_init = func;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_pre_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx<settings>().external_system_pre_step = func;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_post_step(entt::registry &registry, external_system_func_t func) {
    registry.ctx<settings>().external_system_post_step = func;
    registry.ctx<island_coordinator>().settings_changed();
}

void set_external_system_functions(entt::registry &registry,
                                   external_system_func_t init_func,
                                   external_system_func_t pre_step_func,
                                   external_system_func_t post_step_func) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.external_system_init = init_func;
    settings.external_system_pre_step = pre_step_func;
    settings.external_system_post_step = post_step_func;
    registry.ctx<island_coordinator>().settings_changed();
}

void remove_external_systems(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.external_system_init = nullptr;
    settings.external_system_pre_step = nullptr;
    settings.external_system_post_step = nullptr;
    registry.ctx<island_coordinator>().settings_changed();
}

void tag_external_entity(entt::registry &registry, entt::entity entity, bool procedural) {
    if (procedural) {
        registry.emplace<edyn::procedural_tag>(entity);
    }

    registry.emplace<edyn::external_tag>(entity);
    auto non_connecting = !procedural;
    auto node_index = registry.ctx<edyn::entity_graph>().insert_node(entity, non_connecting);
    registry.emplace<edyn::graph_node>(entity, node_index);
}

void set_should_collide(entt::registry &registry, should_collide_func_t func) {
    registry.ctx<settings>().should_collide_func = func;
    registry.ctx<island_coordinator>().settings_changed();
}

bool manifold_exists(entt::registry &registry, entt::entity first, entt::entity second) {
    return manifold_exists(registry, entity_pair{first, second});
}

bool manifold_exists(entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx<contact_manifold_map>();
    return manifold_map.contains(entities);
}

entt::entity get_manifold_entity(const entt::registry &registry, entt::entity first, entt::entity second) {
    return get_manifold_entity(registry, entity_pair{first, second});
}

entt::entity get_manifold_entity(const entt::registry &registry, entity_pair entities) {
    auto &manifold_map = registry.ctx<contact_manifold_map>();
    return manifold_map.get(entities);
}

entt::sink<void(entt::entity)> on_contact_started(entt::registry &registry) {
    return registry.ctx<island_coordinator>().contact_started_sink();
}

entt::sink<void(entt::entity)> on_contact_ended(entt::registry &registry) {
    return registry.ctx<island_coordinator>().contact_ended_sink();
}

entt::sink<void(entt::entity, contact_manifold::contact_id_type)> on_contact_point_created(entt::registry &registry) {
    return registry.ctx<island_coordinator>().contact_point_created_sink();
}

entt::sink<void(entt::entity, contact_manifold::contact_id_type)> on_contact_point_destroyed(entt::registry &registry) {
    return registry.ctx<island_coordinator>().contact_point_destroyed_sink();
}

vector3 get_gravity(const entt::registry &registry) {
    return registry.ctx<settings>().gravity;
}

void set_gravity(entt::registry &registry, vector3 gravity) {
    registry.ctx<settings>().gravity = gravity;

    auto view = registry.view<edyn::gravity, procedural_tag, rigidbody_tag>();

    for (auto entity : view) {
        view.get<edyn::gravity>(entity) = gravity;
        refresh<edyn::gravity>(registry, entity);
    }
}

unsigned get_solver_velocity_iterations(const entt::registry &registry) {
    return registry.ctx<settings>().num_solver_velocity_iterations;
}

void set_solver_velocity_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.num_solver_velocity_iterations = iterations;
    registry.ctx<island_coordinator>().settings_changed();
}

unsigned get_solver_position_iterations(const entt::registry &registry) {
    return registry.ctx<settings>().num_solver_position_iterations;
}

void set_solver_position_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.num_solver_position_iterations = iterations;
    registry.ctx<island_coordinator>().settings_changed();
}

unsigned get_solver_restitution_iterations(const entt::registry &registry) {
    return registry.ctx<settings>().num_restitution_iterations;
}

void set_solver_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.num_restitution_iterations = iterations;
    registry.ctx<island_coordinator>().settings_changed();
}

unsigned get_solver_individual_restitution_iterations(const entt::registry &registry) {
    return registry.ctx<settings>().num_individual_restitution_iterations;
}

void set_solver_individual_restitution_iterations(entt::registry &registry, unsigned iterations) {
    auto &settings = registry.ctx<edyn::settings>();
    settings.num_individual_restitution_iterations = iterations;
    registry.ctx<island_coordinator>().settings_changed();
}

void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material) {
    auto &material_table = registry.ctx<material_mix_table>();
    material_table.insert({material_id0, material_id1}, material);
    registry.ctx<island_coordinator>().material_table_changed();
}

}
