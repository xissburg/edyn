#include <type_traits>
#include "edyn/dynamics/world.hpp"
#include "edyn/sys/update_present_position.hpp"
#include "edyn/sys/update_present_orientation.hpp"
#include "edyn/time/time.hpp"
#include "edyn/comp/constraint_row.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/mass.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/shape.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/collision_filter.hpp"
#include "edyn/comp/constraint.hpp"
#include "edyn/comp/gravity.hpp"
#include "edyn/comp/linacc.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/buffer_sync.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/island_worker.hpp"

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

void on_construct_dynamic_tag(entt::entity entity, entt::registry &registry, dynamic_tag) {
    auto island_ent = registry.create();
    auto &isle = registry.assign<island>(island_ent);
    isle.entities.push_back(entity);

    auto &node = registry.assign<island_node>(entity);
    node.island_entity = island_ent;

    auto [main_queue_input, main_queue_output] = make_message_queue_input_output();
    auto [isle_queue_input, isle_queue_output] = make_message_queue_input_output();
    auto buffer_sync = make_buffer_sync();

    isle.worker = new island_worker_context<
            AABB, angvel, collision_filter, constraint, constraint_row, 
            gravity, inertia, inertia_inv, inertia_world_inv,
            island, island_node, linacc, linvel, mass, mass_inv, material, orientation,
            position, relation, shape, dynamic_tag, kinematic_tag, static_tag,
            sleeping_tag, sleeping_disabled_tag, disabled_tag
        >(buffer_sync.second, message_queue_in_out(main_queue_input, isle_queue_output));

    // Leaking
    isle.reader = new buffer_sync_reader(buffer_sync.first);
    isle.message_queue = new message_queue_in_out(isle_queue_input, main_queue_output);

    std::vector<entt::entity> entities;
    entities.push_back(island_ent);
    entities.push_back(entity);

    auto buffer = memory_output_archive::buffer_type();
    auto output = memory_output_archive(buffer);
    registry.snapshot()
        .component<
            AABB, angvel, collision_filter, constraint, constraint_row, 
            gravity, inertia, inertia_inv, inertia_world_inv,
            island, island_node, linacc, linvel, mass, mass_inv, material, orientation,
            position, relation, shape, dynamic_tag, kinematic_tag, static_tag,
            sleeping_tag, sleeping_disabled_tag, disabled_tag
        >(output, entities.begin(), entities.end());
    isle.message_queue->send(msg::registry_snapshot{buffer});

    isle.timestamp = (double)performance_counter() / (double)performance_frequency();

    //isle.message_queue.sink<msg::entity_created>.connect<&world::on_entity_created>(*this);

    auto j = job();
    j.func = &island_worker_func;
    auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
    auto ctx_intptr = reinterpret_cast<intptr_t>(isle.worker);
    archive(ctx_intptr);
    job_dispatcher::global().async(j);
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

world::world(entt::registry &reg) 
    : registry(&reg)
    , sol(reg)
    , bphase(reg)
    , nphase(reg)
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

    // Associate a `contact_manifold` to every broadphase relation that's created.
    connections.push_back(bphase.construct_relation_sink().connect<&entt::registry::assign<contact_manifold>>(reg));

    job_dispatcher::global().assure_current_queue();
}

world::~world() {
    
}

void world::update(scalar dt) {
    // Run jobs scheduled in physics thread.
    job_dispatcher::global().once_current_queue();

    auto view = registry->view<island>();
    view.each([&] (auto, island &isle) {
        isle.message_queue->update();

        buffer_sync_context::data_type data;
        isle.reader->read(data);
        auto input = memory_input_archive(data.data(), data.size());
        registry->loader()
            .entities(input)
            .destroyed(input)
            .component<
                AABB, angvel, collision_filter, constraint, constraint_row, 
                gravity, inertia, inertia_inv, inertia_world_inv,
                island, island_node, linacc, linvel, mass, mass_inv, material, orientation,
                position, relation, shape, dynamic_tag, kinematic_tag, static_tag,
                sleeping_tag, sleeping_disabled_tag, disabled_tag
            >(input);

        // Update visual representation for this island to reflect the state
        // at the current time.
        /* const auto present_dt = residual_dt - fixed_dt;
        update_present_position(*registry, present_dt);
        update_present_orientation(*registry, present_dt); */
    });

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