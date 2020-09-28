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
    {
        m_island_entity = m_registry.create();
        m_entity_map.insert(island_entity, m_island_entity);

        m_message_queue.sink<registry_snapshot<Component...>>().template connect<&island_worker_context<Component...>::on_registry_snapshot>(*this);
        m_message_queue.sink<msg::set_paused>().template connect<&island_worker_context<Component...>::on_set_paused>(*this);
        m_message_queue.sink<msg::step_simulation>().template connect<&island_worker_context<Component...>::on_step_simulation>(*this);

        (m_registry.on_destroy<Component>().template connect<&island_worker_context<Component...>::on_destroy_component<Component>>(*this), ...);
        (m_registry.on_replace<Component>().template connect<&island_worker_context<Component...>::on_replace_component<Component>>(*this), ...);
        
        m_registry.on_destroy<relation>().template connect<&island_worker_context<Component...>::on_destroy_relation>(*this);

        // Associate a `contact_manifold` to every broadphase relation that's created.
        m_bphase.construct_relation_sink().connect<&entt::registry::assign<contact_manifold>>(m_registry);
    }

    virtual ~island_worker_context() {}

    void on_registry_snapshot(const registry_snapshot<Component...> &snapshot) {
        // Import components from main registry.
        snapshot.template import(m_registry, m_entity_map, 
            &relation::entity, 
            &island::entities, 
            &contact_manifold::point_entity, 
            &contact_point::parent);
    }

    void sync() {
        // Add island and transient components to snapshot before sending it over
        // to the main registry.
        auto &isle = m_registry.get<island>(m_island_entity);
        m_snapshot.template updated(m_island_entity, isle);

        for (auto entity : isle.entities) {
            std::apply([&] (auto &&... args) {
                ((m_registry.has<std::decay_t<decltype(args)>>(entity) ? 
                    m_snapshot.template updated(entity, m_registry.get<std::decay_t<decltype(args)>>(entity)) : (void)0), ...);
            }, transient_components{});
        }

        // Map entities into the domain of the main registry.
        auto snapshot = m_snapshot.template map_entities(m_entity_map, 
            &relation::entity, 
            &island::entities, 
            &contact_manifold::point_entity, 
            &contact_point::parent);
        m_message_queue.send<decltype(snapshot)>(snapshot);

        // Clear snapshot for the next run.
        m_snapshot = {};
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
        m_snapshot.template destroyed<Comp>(entity);
    }

    template<typename Comp>
    void on_replace_component(entt::entity entity, entt::registry &registry, const Comp &comp) {
        m_snapshot.template updated(entity, comp);
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
    registry_snapshot<Component...> m_snapshot;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP