#include <type_traits>
#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_present_position.hpp"
#include "edyn/sys/update_present_orientation.hpp"
#include "edyn/time/time.hpp"
#include "edyn/comp.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/s11n.hpp"
#include "edyn/dynamics/island_util.hpp"
#include "edyn/parallel/registry_snapshot.hpp"

namespace edyn {

void on_construct_or_replace_mass(entt::entity entity, entt::registry &registry, mass &m) {
    EDYN_ASSERT(m > 0);
    registry.assign_or_replace<mass_inv>(entity, m < EDYN_SCALAR_MAX ? 1 / m : 0);
}

void on_destroy_mass(entt::entity entity, entt::registry &registry) {
    registry.reset<mass_inv>(entity);
}

void on_construct_or_replace_inertia(entt::entity entity, entt::registry &registry, inertia &i) {
    EDYN_ASSERT(i > vector3_zero);
    auto &invI = registry.assign_or_replace<inertia_inv>(entity, i.x < EDYN_SCALAR_MAX ? 1 / i.x : 0, 
                                                         i.y < EDYN_SCALAR_MAX ? 1 / i.y : 0, 
                                                         i.z < EDYN_SCALAR_MAX ? 1 / i.z : 0);
    registry.assign_or_replace<inertia_world_inv>(entity, diagonal(invI));
}

void on_destroy_inertia(entt::entity entity, entt::registry &registry) {
    registry.reset<inertia_inv>(entity);
    registry.reset<inertia_world_inv>(entity);
}

void on_construct_shape(entt::entity entity, entt::registry &registry, shape &) {
    registry.assign<AABB>(entity);
    registry.assign<collision_filter>(entity);
}

void on_destroy_shape(entt::entity entity, entt::registry &registry) {
    registry.reset<AABB>(entity);
    registry.reset<collision_filter>(entity);
}

template<typename Snapshot>
void on_registry_snapshot(entt::registry &registry, const Snapshot &snapshot) {
    // Load snapshot from island. It is already mapped into the main
    // registry's domain.
    snapshot.load(registry);
}

void on_construct_dynamic_tag(entt::entity entity, entt::registry &registry, dynamic_tag) {
    // For each new dynamic entity, a new island is created and the dynamic entity
    // is inserted into the new island.
    auto island_ent = registry.create();
    auto &isle = registry.assign<island>(island_ent);
    isle.entities.push_back(entity);

    auto &node = registry.assign<island_node>(entity);
    node.island_entity = island_ent;

    // Send a snapshot containing the new entity and its components to the island
    // worker.
    using registry_snapshot_type = decltype(registry_snapshot(all_components{}));
    auto snapshot = registry_snapshot(all_components{});
    snapshot.updated(island_ent, isle);
    std::apply([&] (auto &&... comp) {
        ((registry.has<std::decay_t<decltype(comp)>>(entity) ?
                snapshot.updated(entity, registry.get<std::decay_t<decltype(comp)>>(entity)) : (void)0), ...);
    }, all_components{});

    auto &wrld = registry.ctx<world>();
    wrld.m_island_info_map.at(island_ent).m_message_queue.send<registry_snapshot_type>(snapshot);
}

void on_destroy_dynamic_tag(entt::entity entity, entt::registry &registry) {
    auto &node = registry.get<island_node>(entity);
    auto &isle = registry.get<island>(node.island_entity);
    auto it = std::find(isle.entities.begin(), isle.entities.end(), entity);
    std::swap(*it, *(isle.entities.end() - 1));
    isle.entities.pop_back();

    if (isle.entities.empty()) {
        registry.destroy(node.island_entity);
    }

    registry.remove<island_node>(entity);
}

void on_construct_island(entt::entity entity, entt::registry &registry, island &isle) {
    isle.timestamp = (double)performance_counter() / (double)performance_frequency();

    auto [main_queue_input, main_queue_output] = make_message_queue_input_output();
    auto [isle_queue_input, isle_queue_output] = make_message_queue_input_output();

    // The `island_worker_context` is dynamically allocated and kept alive while
    // the associated island lives. The job that's created for it calls its
    // `update` function which reschedules itself to be run over and over again.
    // After the `finish` function is called on it (when the island is destroyed),
    // it will be deallocated on the next run.
    auto *worker = new island_worker_context(message_queue_in_out(main_queue_input, isle_queue_output),
                                             all_components{});

    auto info = island_info(worker, message_queue_in_out(isle_queue_input, main_queue_output));
    auto &wrld = registry.ctx<world>();
    wrld.m_island_info_map.insert(std::make_pair(entity, info));

    // Send over a snapshot containing this island entity to the island worker
    // before it even starts.
    using registry_snapshot_type = decltype(registry_snapshot(all_components{}));
    auto snapshot = registry_snapshot(all_components{});
    snapshot.updated(entity, isle);
    info.m_message_queue.send<registry_snapshot_type>(snapshot);

    info.m_message_queue.sink<registry_snapshot_type>()
        .connect<&on_registry_snapshot<registry_snapshot_type>>(registry);
    //info.m_message_queue.sink<msg::entity_created>().connect<&world::on_entity_created>(*this);

    auto j = job();
    j.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(worker);
    archive(ctx_intptr);
    job_dispatcher::global().async(j);
}

void on_destroy_island(entt::entity entity, entt::registry &registry) {
    auto &wrld = registry.ctx<world>();
    auto &info = wrld.m_island_info_map.at(entity);
    info.m_worker->finish();
    wrld.m_island_info_map.erase(entity);
}

world::world(entt::registry &reg) 
    : registry(&reg)
    , sol(reg)
    , bphase(reg)
{
    connections.push_back(reg.on_construct<mass>().connect<&on_construct_or_replace_mass>());
    connections.push_back(reg.on_replace<mass>().connect<&on_construct_or_replace_mass>());
    connections.push_back(reg.on_destroy<mass>().connect<&on_destroy_mass>());

    connections.push_back(reg.on_construct<inertia>().connect<&on_construct_or_replace_inertia>());
    connections.push_back(reg.on_replace<inertia>().connect<&on_construct_or_replace_inertia>());
    connections.push_back(reg.on_destroy<inertia>().connect<&on_destroy_inertia>());

    connections.push_back(reg.on_construct<shape>().connect<&on_construct_shape>());
    connections.push_back(reg.on_destroy<shape>().connect<&on_destroy_shape>());

    connections.push_back(reg.on_construct<dynamic_tag>().connect<&on_construct_dynamic_tag>());
    connections.push_back(reg.on_destroy<dynamic_tag>().connect<&on_destroy_dynamic_tag>());

    connections.push_back(reg.on_construct<island>().connect<&on_construct_island>());
    connections.push_back(reg.on_destroy<island>().connect<&on_destroy_island>());

    connections.push_back(reg.on_construct<relation>().connect<&island_on_construct_relation>());
    connections.push_back(reg.on_destroy<relation>().connect<&island_on_destroy_relation>());

    // Associate a `contact_manifold` to every broadphase relation that's created.
    connections.push_back(bphase.construct_relation_sink().connect<&entt::registry::assign<contact_manifold>>(reg));

    job_dispatcher::global().assure_current_queue();
}

world::~world() {
    
}

void world::update(scalar dt) {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    for (auto &pair : m_island_info_map) {
        pair.second.m_message_queue.update();

        // Update visual representation for this island to reflect the state
        // at the current time.
        /* const auto present_dt = residual_dt - fixed_dt;
        update_present_position(*registry, present_dt);
        update_present_orientation(*registry, present_dt); */
    }

    bphase.update();

    update_signal.publish(dt);
}

void world::run() {
    running = true;

    const auto freq = performance_frequency();
    const auto timescale = 1.0 / freq;
    const auto t0 = performance_counter();
    auto ti = t0;

    // Use an Integral Controller to calculate the right amount of delay to
    // keep `dt` as close as possible to `fixed_dt`.
    const scalar I = 0.5;
    scalar delay_dt = 0;

    while (running) {
        const auto t = performance_counter();
        const auto dt = (t - ti) * timescale;
        update(dt);
        ti = t;
        local_time_ = t * timescale - residual_dt;

        auto err_dt = fixed_dt - dt;
        delay_dt += err_dt * I;

        delay(delay_dt * 1000);
    }
}

void world::quit() {
    running = false;
}

}