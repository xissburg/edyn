#ifndef EDYN_DYNAMICS_WORLD_HPP
#define EDYN_DYNAMICS_WORLD_HPP

#include <atomic>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <entt/entt.hpp>

#include "solver.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/collision/simple_broadphase.hpp"

#include "edyn/parallel/message_queue.hpp"
#include "edyn/parallel/island_worker.hpp"

namespace edyn {

class world;

struct snapshot_context {
    world *m_world;
    entt::entity m_island_entity;
};

struct island_info {
    island_worker_context_base *m_worker;
    message_queue_in_out m_message_queue;
    entity_map m_entity_map;

    island_info(island_worker_context_base *worker,
                message_queue_in_out message_queue)
        : m_worker(worker)
        , m_message_queue(message_queue)
    {}
};

class world final {
    struct destroyed_rigidbody_info {
        entt::entity entity;
        std::vector<entt::entity> island_entities;
    };

    struct destroyed_relation_info {
        entt::entity entity;
        relation rel;
        std::unordered_set<entt::entity> island_entities;
    };

    template<typename Message, typename... Args>
    void send_message(entt::entity island_entity, Args &&... args) {
        m_island_info_map.at(island_entity).m_message_queue.send<Message>(std::forward<Args>(args)...);
    }

public:
    world(entt::registry &);
    ~world();

    void update(scalar dt);

    uint64_t current_step() const {
        return step_;
    }

    double local_time() const {
        return local_time_;
    }

    void run();

    void quit();

    void set_paused(bool);
    void step();

    using update_signal_func_t = void(scalar);
    using step_signal_func_t = void(uint64_t);

    entt::sink<update_signal_func_t> update_sink() {
        return {update_signal};
    }

    template<typename... Component, typename... Type, typename... Member>
    void refresh(entt::entity entity, Member Type:: *...member) {
        auto &node = m_registry->get<island_node>(entity);

        for (auto island_entity : node.island_entities) {
            auto &info = m_island_info_map.at(island_entity);
            auto builder = registry_snapshot_builder(info.m_entity_map, all_components{});
            (builder.template updated<Component>(entity, m_registry->get<Component>(entity), member...), ...);
            info.m_message_queue.send<typename decltype(builder)::registry_snapshot_t>(builder.template get_snapshot());
        }
    }

    void refresh_all(entt::entity);

    void init_new_dynamic_entities();
    void init_new_static_kinematic_entities();
    void init_new_relations();
    void init_new_entities();

    void destroy_pending_rigidbodies();
    void destroy_pending_relations();
    void destroy_pending_entities();

    void on_broadphase_intersect(entt::entity, entt::entity);

    void on_construct_constraint(entt::entity, entt::registry &, constraint &);
    void on_destroy_constraint(entt::entity, entt::registry &);

    void on_construct_island(entt::entity entity, entt::registry &registry, island &isle);
    
    void on_construct_dynamic_tag(entt::entity, entt::registry &, dynamic_tag);
    void on_destroy_dynamic_tag(entt::entity, entt::registry &);
    
    void on_construct_static_tag(entt::entity, entt::registry &, static_tag);
    void on_destroy_static_tag(entt::entity, entt::registry &);
    
    void on_construct_kinematic_tag(entt::entity, entt::registry &, kinematic_tag);
    void on_destroy_kinematic_tag(entt::entity, entt::registry &);

    void on_construct_rigidbody(entt::entity);
    void on_destroy_rigidbody(entt::entity);

    void on_construct_relation(entt::entity, entt::registry &, relation &);
    void on_destroy_relation(entt::entity, entt::registry &);

    void on_destroy_relation_container(entt::entity, entt::registry &);
    void on_destroy_island_node(entt::entity, entt::registry &);

    template<typename Snapshot>
    void import_snapshot(entt::entity island_entity, const Snapshot &snapshot) {
        // Load snapshot from island. It is already mapped into the main
        // registry's domain.
        auto &info = m_island_info_map.at(island_entity);
        snapshot.import(*m_registry, info.m_entity_map, all_components_entity_pointer_to_member);
    }

    void merge_entities(entt::entity, entt::entity, entt::entity rel_entity);
    void merge_dynamic_with_static_or_kinematic(entt::entity, entt::entity, entt::entity rel_entity);
    void merge_islands(entt::entity, entt::entity);

    scalar fixed_dt {1.0/60};
    solver sol;
    std::unordered_map<entt::entity, island_info> m_island_info_map;

private:
    entt::registry* m_registry;
    simple_broadphase bphase;
    std::vector<entt::scoped_connection> connections;

    std::vector<entt::entity> m_new_dynamic_entities;
    std::vector<entt::entity> m_new_static_kinematic_entities;
    std::vector<entt::entity> m_new_relations;
    std::vector<destroyed_rigidbody_info> m_destroyed_rigidbodies;
    std::vector<destroyed_relation_info> m_destroyed_relations;
    bool m_importing_snapshot {false};

    scalar residual_dt {0};
    std::atomic<uint64_t> step_ {0};
    std::atomic<double> local_time_;
    std::atomic_bool running {false};
    bool m_paused {false};
    entt::sigh<update_signal_func_t> update_signal;
};

}

#endif // EDYN_DYNAMICS_WORLD_HPP