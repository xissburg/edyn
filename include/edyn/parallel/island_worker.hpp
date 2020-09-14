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
#include "edyn/serialization/registry_s11n.hpp"
#include "edyn/util/tuple.hpp"

namespace edyn {

class job_dispatcher;

void island_worker_func(job::data_type &);

class island_worker_context_base {
public:
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

    island_worker_context(message_queue_in_out message_queue, [[maybe_unused]] components_tuple)
        : m_message_queue(message_queue)
        , m_bphase(m_registry)
        , m_nphase(m_registry)
        , m_sol(m_registry)
    {
        m_message_queue.sink<msg::registry_snapshot>().template connect<&island_worker_context::on_registry_snapshot>(*this);

        (m_registry.on_destroy<Component>().template connect<&island_worker_context<Component...>::on_destroy_component<Component>>(*this), ...);
        (m_registry.on_replace<Component>().template connect<&island_worker_context<Component...>::on_replace_component<Component>>(*this), ...);
    }

    void on_registry_snapshot(const msg::registry_snapshot &snapshot) {
        auto input = memory_input_archive(snapshot.data);
        auto importer = registry_snapshot_importer<Component...>(m_registry, m_entity_map);
        importer.template serialize(input, &relation::entity, &island::entities);
    }

    void sync() {
        auto buffer = memory_output_archive::buffer_type();
        auto output = memory_output_archive(buffer);
        auto exporter = registry_snapshot_exporter<Component...>(m_registry, m_entity_map);

        auto &isle = m_registry.get<island>(m_island_entity);
        exporter.template updated<island>(m_island_entity);

        for (auto entity : isle.entities) {
            exporter.template updated(entity, transient_components{});
        }

        exporter.template updated(m_updated_components.begin(), m_updated_components.end());
        exporter.template destroyed(m_destroyed_components.begin(), m_destroyed_components.end());
        exporter.template serialize(output, &relation::entity, &island::entities);
        m_message_queue.send<msg::registry_snapshot>(buffer);
        m_destroyed_components.clear();
    }

    void update() override {
        // Process messages.
        m_message_queue.update();

        auto &isle = m_registry.get<island>(m_island_entity);
        auto timestamp = (double)performance_counter() / (double)performance_frequency();
        auto dt = timestamp - isle.timestamp;

        if (dt >= fixed_dt) {
            m_bphase.update();
            m_nphase.update();
            m_sol.update(0, dt);

            // Do not use the same `isle` instance because component references
            // are not stable.
            auto &isle = m_registry.get<island>(m_island_entity);
            isle.timestamp += dt;

            sync();
        }

        // Reschedule this job.
        auto j = job();
        j.func = &island_worker_func;
        auto archive = fixed_memory_output_archive(j.data.data(), j.data.size());
        auto ctx_intptr = reinterpret_cast<intptr_t>(this);
        archive(ctx_intptr);

        //job_dispatcher::global()::async_after(isle.timestamp + fixed_dt - timestamp, j);
        job_dispatcher::global().async(j);
    }

    template<typename Comp>
    void on_destroy_component(entt::entity entity, entt::registry &registry) {
        auto comp_id = index_of_v<size_t, Comp, Component...>;
        m_destroyed_components.emplace_back(entity, comp_id);
    }

    template<typename Comp>
    void on_replace_component(entt::entity entity, entt::registry &registry) {
        auto comp_id = index_of_v<size_t, Comp, Component...>;
        m_updated_components.emplace_back(entity, comp_id);
    }

private:
    entt::registry m_registry;
    entt::entity m_island_entity;
    entity_map m_entity_map;
    broadphase m_bphase;
    narrowphase m_nphase;
    solver m_sol;
    message_queue_in_out m_message_queue;
    double fixed_dt;
    std::vector<entity_comp_id_pair> m_destroyed_components;
    std::vector<entity_comp_id_pair> m_updated_components;
};

}

#endif // EDYN_PARALLEL_ISLAND_WORKER_HPP