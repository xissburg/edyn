#ifndef EDYN_PARALLEL_ISLAND_WORKER_HPP
#define EDYN_PARALLEL_ISLAND_WORKER_HPP

#include <atomic>
#include <entt/entt.hpp>
#include "edyn/serialization/memory_archive.hpp"
#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/message.hpp"
#include "edyn/parallel/job.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/time/time.hpp"
#include "edyn/collision/broadphase.hpp"
#include "edyn/collision/narrowphase.hpp"
#include "edyn/dynamics/solver.hpp"
#include "edyn/comp.hpp"
#include "edyn/util/tuple.hpp"
#include "edyn/parallel/registry_snapshot.hpp"
#include "edyn/collision/contact_manifold.hpp"

namespace edyn {

class job_dispatcher;

void island_worker_func(job::data_type &);

class island_worker_context_base {
public:
    virtual ~island_worker_context_base() {}
    virtual void update() = 0;

    bool is_finished() const {
        return m_finished.load(std::memory_order_relaxed);
    }

    void finish() {
        m_finished.store(true, std::memory_order_relaxed);
    }

private:
    std::atomic<bool> m_finished {false};
};

template<typename... Component>
class island_worker_context: public island_worker_context_base {
public:
    using components_tuple = std::tuple<Component...>;

    island_worker_context(entt::entity island_entity, scalar fixed_dt, message_queue_in_out message_queue, [[maybe_unused]] components_tuple)
        : m_message_queue(message_queue)
        , m_fixed_dt(fixed_dt)
        , m_paused(false)
        , m_bphase(m_registry)
        , m_nphase(m_registry)
        , m_sol(m_registry)
        , m_snapshot_builder(m_entity_map)
        , m_importing_snapshot(false)
    {
        m_island_entity = m_registry.create();
        m_entity_map.insert(island_entity, m_island_entity);

        m_message_queue.sink<registry_snapshot<Component...>>().template connect<&island_worker_context<Component...>::on_registry_snapshot>(*this);
        m_message_queue.sink<msg::set_paused>().template connect<&island_worker_context<Component...>::on_set_paused>(*this);
        m_message_queue.sink<msg::step_simulation>().template connect<&island_worker_context<Component...>::on_step_simulation>(*this);

        (m_registry.on_destroy<Component>().template connect<&island_worker_context<Component...>::on_destroy_component<Component>>(*this), ...);
        (m_registry.on_replace<Component>().template connect<&island_worker_context<Component...>::on_replace_component<Component>>(*this), ...);
        
        m_registry.on_destroy<relation>().template connect<&island_worker_context<Component...>::on_destroy_relation>(*this);
        m_registry.on_destroy<relation_container>().template connect<&island_worker_context<Component...>::on_destroy_relation_container>(*this);
        
        m_registry.on_destroy<island_node>().template connect<&island_worker_context<Component...>::on_destroy_island_node>(*this);

        m_registry.on_construct<constraint>().connect<&island_worker_context<Component...>::on_construct_constraint>(*this);
        m_registry.on_destroy<constraint>().connect<&island_worker_context<Component...>::on_destroy_constraint>(*this);

        // Associate a `contact_manifold` to every broadphase relation that's created.
        m_bphase.construct_relation_sink().connect<&entt::registry::assign<contact_manifold>>(m_registry);
    }

    virtual ~island_worker_context() {}

    void on_registry_snapshot(const registry_snapshot<Component...> &snapshot) {
        // Import components from main registry.
        m_importing_snapshot = true;
        snapshot.template import(m_registry, m_entity_map, all_components_entity_pointer_to_member);
        m_importing_snapshot = false;
    }

    void sync() {
        // Add island and transient components to snapshot before sending it over
        // to the main registry.
        auto &isle = m_registry.get<island>(m_island_entity);
        m_snapshot_builder.template updated<island>(m_island_entity, isle, &island::entities);

        for (auto entity : isle.entities) {
            m_snapshot_builder.template maybe_updated(entity, m_registry, transient_components{});
        }

        m_message_queue.send<registry_snapshot<Component...>>(m_snapshot_builder.template get_snapshot());

        // Clear snapshot for the next run.
        m_snapshot_builder.clear();
    }

    void update() override {
        // Process messages.
        m_message_queue.update();

        if (!m_paused) {
            auto &isle = m_registry.get<island>(m_island_entity);
            auto timestamp = (double)performance_counter() / (double)performance_frequency();
            auto dt = timestamp - isle.timestamp;

            if (dt >= m_fixed_dt) {
                step();
            }
        }

        // Reschedule this job.
        reschedule();
    }

    void step() {
        m_bphase.update();
        m_nphase.update();
        m_sol.update(0, m_fixed_dt);

        auto &isle = m_registry.get<island>(m_island_entity);
        isle.timestamp += m_fixed_dt;

        sync();
    }

    void reschedule() {
        auto j = job();
        j.func = &island_worker_func;
        auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
        auto ctx_intptr = reinterpret_cast<intptr_t>(this);
        archive(ctx_intptr);
        //job_dispatcher::global()::async_after(isle.timestamp + m_fixed_dt - timestamp, j);
        job_dispatcher::global().async(j);
    }

    template<typename Comp>
    void on_destroy_component(entt::entity entity, entt::registry &registry) {
        if (m_importing_snapshot) return;
        m_snapshot_builder.template destroyed<Comp>(entity);
    }

    template<typename Comp>
    void on_replace_component(entt::entity entity, entt::registry &registry, const Comp &comp) {
        if (m_importing_snapshot) return;
        m_snapshot_builder.template updated<Comp>(entity, comp, all_components_entity_pointer_to_member);
    }

    void on_set_paused(const msg::set_paused &msg) {
        m_paused = msg.paused;
        auto &isle = m_registry.get<island>(m_island_entity);
        auto timestamp = (double)performance_counter() / (double)performance_frequency();
        isle.timestamp = timestamp;
    }

    void on_step_simulation(const msg::step_simulation &msg) {
        step();
    }

    void on_destroy_relation(entt::entity entity, entt::registry &registry) {
        // Remove it from containers.
        auto &rel = registry.get<relation>(entity);

        for (auto e : rel.entity) {
            if (e == entt::null) continue;
            if (auto *container = registry.try_get<relation_container>(e)) {
                container->entities.erase(
                    std::remove(
                        container->entities.begin(),
                        container->entities.end(), entity), 
                    container->entities.end());
            }
        }
    }

    void on_destroy_relation_container(entt::entity entity, entt::registry &registry) {
        // Destroy relations.
        auto &container = registry.get<relation_container>(entity);
        // Make a copy to prevent modification to the vector during iteration since
        // `on_destroy_relation` will be called.
        auto relation_entities = container.entities;
        for (auto rel_entity : relation_entities) {
            registry.destroy(rel_entity);
        }
    }

    void on_destroy_island_node(entt::entity entity, entt::registry &registry) {
        // Remove from island.
        auto &node = registry.get<island_node>(entity);
        auto island_entity = node.island_entities.front();
        auto &isle = registry.get<island>(island_entity);
        auto it = std::find(isle.entities.begin(), isle.entities.end(), entity);
        std::swap(*it, *(isle.entities.end() - 1));
        isle.entities.pop_back();
    }

    void on_construct_constraint(entt::entity entity, entt::registry &registry, constraint &con) {
        if (m_importing_snapshot) return;

        auto &rel = registry.get<relation>(entity);

        std::visit([&] (auto &&c) {
            // Initialize actual constraint.
            c.update(solver_stage_value_t<solver_stage::init>{}, entity, con, rel, registry, 0);
        }, con.var);
    }

    void on_destroy_constraint(entt::entity entity, entt::registry &registry) {
        auto &con = registry.get<constraint>(entity);

        // Destroy all constraint rows.
        for (size_t i = 0; i < con.num_rows; ++i) {
            registry.destroy(con.row[i]);
        }
    }

private:
    entt::registry m_registry;
    entt::entity m_island_entity;
    entity_map m_entity_map;
    broadphase m_bphase;
    narrowphase m_nphase;
    solver m_sol;
    message_queue_in_out m_message_queue;
    double m_fixed_dt;
    bool m_paused;
    registry_snapshot_builder<Component...> m_snapshot_builder;
    bool m_importing_snapshot;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP