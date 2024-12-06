#include "edyn/simulation/island_manager.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/graph_node.hpp"
#include "edyn/comp/graph_edge.hpp"
#include "edyn/comp/island.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/config/execution_mode.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/island_util.hpp"
#include "edyn/util/vector_util.hpp"
#include "edyn/util/entt_util.hpp"
#include <entt/entity/registry.hpp>
#include <set>

namespace edyn {

island_manager::island_manager(entt::registry &registry)
    : m_registry(&registry)
{
    m_connections.push_back(registry.on_construct<graph_node>().connect<&island_manager::on_construct_graph_node>(*this));
    m_connections.push_back(registry.on_construct<graph_edge>().connect<&island_manager::on_construct_graph_edge>(*this));
    m_connections.push_back(registry.on_destroy<graph_node>().connect<&island_manager::on_destroy_graph_node>(*this));
    m_connections.push_back(registry.on_destroy<graph_edge>().connect<&island_manager::on_destroy_graph_edge>(*this));
    m_connections.push_back(registry.on_destroy<island_resident>().connect<&island_manager::on_destroy_island_resident>(*this));
    m_connections.push_back(registry.on_destroy<multi_island_resident>().connect<&island_manager::on_destroy_multi_island_resident>(*this));
}

island_manager::~island_manager() {
    // Clear here to avoid invalid island entities in `on_destroy<island_resident>` events.
    m_connections.clear();

    // Destroy all island entities created by this manager.
    auto island_view = m_registry->view<island>();
    m_registry->destroy(island_view.begin(), island_view.end());
}

void island_manager::on_construct_graph_node(entt::registry &registry, entt::entity entity) {
    m_new_graph_nodes.push_back(entity);
}

void island_manager::on_construct_graph_edge(entt::registry &registry, entt::entity entity) {
    m_new_graph_edges.push_back(entity);
}

void island_manager::on_destroy_graph_node(entt::registry &registry, entt::entity entity) {
    // Temporarily disable the `on_destroy<graph_edge>` signal because all
    // edges connected to this node will be destroyed along with it in a
    // more efficient manner.
    registry.on_destroy<graph_edge>().disconnect<&island_manager::on_destroy_graph_edge>(*this);

    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx().get<entity_graph>();

    graph.visit_edges(node.node_index, [&](auto edge_index) {
        auto edge_entity = graph.edge_entity(edge_index);
        registry.destroy(edge_entity);
    });

    graph.remove_all_edges(node.node_index);
    graph.remove_node(node.node_index);

    // Reconnect `on_destroy<graph_edge>` signal.
    registry.on_destroy<graph_edge>().connect<&island_manager::on_destroy_graph_edge>(*this);
}

void island_manager::on_destroy_graph_edge(entt::registry &registry, entt::entity entity) {
    auto &graph = registry.ctx().get<entity_graph>();
    auto &edge = registry.get<graph_edge>(entity);
    graph.remove_edge(edge.edge_index);
}

void island_manager::on_destroy_island_resident(entt::registry &registry, entt::entity entity) {
    const auto &resident = registry.get<island_resident>(entity);

    if (resident.island_entity == entt::null) {
        return;
    }

    auto &island = registry.get<edyn::island>(resident.island_entity);

    if (island.nodes.contains(entity)) {
        island.nodes.erase(entity);
    } else if (island.edges.contains(entity)) {
        island.edges.erase(entity);
    }

    // Island could have been split.
    if (!m_islands_to_split.contains(resident.island_entity)) {
        m_islands_to_split.push(resident.island_entity);
    }

    if (!m_islands_to_wake_up.contains(resident.island_entity)) {
        m_islands_to_wake_up.push(resident.island_entity);
    }
}

void island_manager::on_destroy_multi_island_resident(entt::registry &registry, entt::entity entity) {
    auto &resident = registry.get<const multi_island_resident>(entity);
    auto island_view = registry.view<edyn::island>();

    for (auto island_entity : resident.island_entities) {
        auto [island] = island_view.get(island_entity);
        island.nodes.erase(entity);

        // Non-procedural entities do not form islands thus there's no need to
        // check whether this island was split by its removal. It is necessary
        // to wake the island up though, as it might cause other entities to
        // start moving.
        if (!m_islands_to_wake_up.contains(island_entity)) {
            m_islands_to_wake_up.push(island_entity);
        }
    }
}

void island_manager::init_new_nodes_and_edges() {
    // Entities that were created and destroyed before a call to `edyn::update`
    // are still in these collections, thus remove invalid entities first.
    entity_vector_erase_invalid(m_new_graph_nodes, *m_registry);
    entity_vector_erase_invalid(m_new_graph_edges, *m_registry);

    if (m_new_graph_nodes.empty() && m_new_graph_edges.empty()) return;

    auto &graph = m_registry->ctx().get<entity_graph>();
    auto node_view = m_registry->view<graph_node>();
    auto edge_view = m_registry->view<graph_edge>();
    std::set<entity_graph::index_type> procedural_node_indices;

    for (auto entity : m_new_graph_nodes) {
        if (m_registry->any_of<procedural_tag>(entity)) {
            auto &node = node_view.get<graph_node>(entity);
            procedural_node_indices.insert(node.node_index);
        }
    }

    for (auto edge_entity : m_new_graph_edges) {
        auto &edge = edge_view.get<graph_edge>(edge_entity);
        auto node_entities = graph.edge_node_entities(edge.edge_index);

        if (m_registry->any_of<procedural_tag>(node_entities.first)) {
            auto &node = node_view.get<graph_node>(node_entities.first);
            procedural_node_indices.insert(node.node_index);
        }

        if (m_registry->any_of<procedural_tag>(node_entities.second)) {
            auto &node = node_view.get<graph_node>(node_entities.second);
            procedural_node_indices.insert(node.node_index);
        }
    }

    m_new_graph_nodes.clear();
    m_new_graph_edges.clear();

    if (procedural_node_indices.empty()) return;

    std::vector<entt::entity> connected_nodes;
    std::vector<entt::entity> connected_edges;
    std::vector<entt::entity> island_entities;
    auto resident_view = m_registry->view<const island_resident>();
    auto procedural_view = m_registry->view<procedural_tag>();

    graph.reach(
        procedural_node_indices.begin(), procedural_node_indices.end(),
        [&](entt::entity entity) { // visit_node_func
            // We only visit procedural nodes.
            EDYN_ASSERT(procedural_view.contains(entity));
            auto [resident] = resident_view.get(entity);

            if (resident.island_entity == entt::null) {
                connected_nodes.push_back(entity);
            }
        },
        [&](entt::entity entity) { // visit_edge_func
            auto [resident] = resident_view.get(entity);

            if (resident.island_entity == entt::null) {
                connected_edges.push_back(entity);
            } else {
                auto contains_island = vector_contains(island_entities, resident.island_entity);

                if (!contains_island) {
                    island_entities.push_back(resident.island_entity);
                }
            }
        },
        [&](entity_graph::index_type node_index) { // should_visit_func
            auto other_entity = graph.node_entity(node_index);

            // Do not visit non-procedural nodes.
            if (!procedural_view.contains(other_entity)) {
                // However, add them to the island.
                if (!vector_contains(connected_nodes, other_entity)) {
                    connected_nodes.push_back(other_entity);
                }

                return false;
            }

            // Visit neighbor node if it's not in an island yet.
            auto [other_resident] = resident_view.get(other_entity);

            if (other_resident.island_entity == entt::null) {
                return true;
            }

            auto contains_island = vector_contains(island_entities, other_resident.island_entity);

            // Collect islands involved in this connected component.
            if (!contains_island) {
                island_entities.push_back(other_resident.island_entity);
            }

            bool continue_visiting = false;

            // Visit neighbor if it contains an edge that is not in an island yet.
            graph.visit_edges(node_index, [&](auto edge_index) {
                auto edge_entity = graph.edge_entity(edge_index);
                if (std::get<0>(resident_view.get(edge_entity)).island_entity == entt::null) {
                    continue_visiting = true;
                }
            });

            return continue_visiting;
        },
        [&]() { // connected_component_func
            if (island_entities.size() <= 1) {
                // Assign island to the residents.
                auto island_entity = entt::entity{};

                if (island_entities.empty()) {
                    island_entity = create_island();
                } else {
                    island_entity = island_entities.front();
                }

                insert_to_island(island_entity, connected_nodes, connected_edges);
            } else {
                // Islands have to be merged.
                merge_islands(island_entities, connected_nodes, connected_edges);
            }

            connected_nodes.clear();
            connected_edges.clear();
            island_entities.clear();
        });
}

entt::entity island_manager::create_island() {
    auto island_entity = m_registry->create();
    m_registry->emplace<island>(island_entity);
    m_registry->emplace<island_AABB>(island_entity);
    m_registry->emplace<island_tag>(island_entity);
    return island_entity;
}

void island_manager::insert_to_island(entt::entity island_entity,
                                      const std::vector<entt::entity> &nodes,
                                      const std::vector<entt::entity> &edges) {
    auto resident_view = m_registry->view<island_resident>();
    auto multi_resident_view = m_registry->view<multi_island_resident>();
    auto &island = m_registry->get<edyn::island>(island_entity);

    for (auto entity : nodes) {
        if (resident_view.contains(entity)) {
            island.nodes.push(entity);
            m_registry->patch<island_resident>(entity, [island_entity](island_resident &resident) {
                resident.island_entity = island_entity;
            });
        } else {
            auto [resident] = multi_resident_view.get(entity);

            if (!resident.island_entities.contains(island_entity)) {
                resident.island_entities.push(island_entity);
            }

            if (!island.nodes.contains(entity)) {
                island.nodes.push(entity);
            }
        }

        m_registry->remove<sleeping_tag>(entity);
    }

    for (auto entity : edges) {
        m_registry->patch<island_resident>(entity, [island_entity](island_resident &resident) {
            resident.island_entity = island_entity;
        });
        m_registry->remove<sleeping_tag>(entity);
    }

    island.edges.push(edges.begin(), edges.end());

    wake_up_island(*m_registry, island_entity);
}

entt::entity island_manager::merge_islands(const std::vector<entt::entity> &island_entities,
                                           const std::vector<entt::entity> &new_nodes,
                                           const std::vector<entt::entity> &new_edges) {
    EDYN_ASSERT(island_entities.size() > 1);

    // Pick biggest island and move the other entities into it.
    entt::entity island_entity = entt::null;
    size_t biggest_size = 0;
    auto island_view = m_registry->view<island>();

    for (auto entity : island_entities) {
        auto [island] = island_view.get(entity);
        auto size = island.nodes.size() + island.edges.size();

        if (size > biggest_size) {
            biggest_size = size;
            island_entity = entity;
        }
    }

    EDYN_ASSERT(island_entity != entt::null);

    auto other_island_entities = island_entities;
    vector_erase(other_island_entities, island_entity);

    auto all_nodes = new_nodes;
    auto all_edges = new_edges;

    for (auto other_island_entity : other_island_entities) {
        auto &island = island_view.get<edyn::island>(other_island_entity);
        all_edges.insert(all_edges.end(), island.edges.begin(), island.edges.end());

        // There could be duplicate nodes since non-procedural entities can be
        // in more than one island.
        for (auto entity : island.nodes) {
            if (!vector_contains(all_nodes, entity)) {
                all_nodes.push_back(entity);
            }
        }
    }

    insert_to_island(island_entity, all_nodes, all_edges);

    // Destroy empty islands.
    m_registry->destroy(other_island_entities.begin(), other_island_entities.end());

    // Remove destroyed islands from residents.
    for (auto [entity, resident] : m_registry->view<multi_island_resident>().each()) {
        resident.island_entities.remove(other_island_entities.begin(), other_island_entities.end());
    }

    // Return island that survived the merge.
    return island_entity;
}

void island_manager::split_islands() {
    for (auto island_entity : m_islands_to_split) {
        if (!m_registry->valid(island_entity)) {
            m_islands_to_split.erase(island_entity);
        }
    }

    if (m_islands_to_split.empty()) return;

    auto island_view = m_registry->view<island, island_AABB>();
    auto node_view = m_registry->view<graph_node>();
    auto multi_resident_view = m_registry->view<multi_island_resident>();
    auto aabb_view = m_registry->view<AABB>();
    auto procedural_view = m_registry->view<procedural_tag>();
    auto disabled_view = m_registry->view<disabled_tag>();
    auto &graph = m_registry->ctx().get<entity_graph>();

    for (auto source_island_entity : m_islands_to_split) {
        auto &source_island = island_view.get<edyn::island>(source_island_entity);

        // Island could now be empty or contain only non-procedural entities.
        if (source_island.nodes.empty() ||
            std::find_if(source_island.nodes.begin(), source_island.nodes.end(),
                         [&](auto entity){return procedural_view.contains(entity);}) == source_island.nodes.end()) {
            EDYN_ASSERT(source_island.edges.empty());

            // Remove destroyed island from non-procedural entities.
            for (auto entity : source_island.nodes) {
                // All nodes are non-procedural at this point so there's no
                // need to check.
                auto [resident] = multi_resident_view.get(entity);
                resident.island_entities.erase(source_island_entity);
            }

            m_registry->destroy(source_island_entity);
            continue;
        }

        auto all_nodes = entt::sparse_set{};
        all_nodes.push(source_island.nodes.begin(), source_island.nodes.end());
        std::vector<edyn::island> islands;

        while (!all_nodes.empty()) {
            auto start_node_entity = *all_nodes.begin();

            if (!procedural_view.contains(start_node_entity)) {
                all_nodes.erase(start_node_entity);
                continue;
            }

            auto start_node_index = node_view.get<graph_node>(start_node_entity).node_index;
            auto &curr_island = islands.emplace_back();

            graph.traverse(start_node_index,
                [&](auto node_index) {
                    // Add node to island and assign island to resident.
                    auto node_entity = graph.node_entity(node_index);
                    curr_island.nodes.push(node_entity);

                    // Remove visited entity from list of entities to be visited.
                    all_nodes.remove(node_entity);
                }, [&](auto edge_index) {
                    auto edge_entity = graph.edge_entity(edge_index);
                    curr_island.edges.push(edge_entity);
                });
        }

        EDYN_ASSERT(!islands.empty());

        if (islands.size() == 1) {
            // Island is a single connected component in the entity graph.
            continue;
        }

        // Find biggest island among all and move that into the original as to
        // minimize the amount of changes.
        unsigned biggest_size = 0;
        unsigned biggest_idx = 0;

        for (unsigned i = 0; i < islands.size(); ++i) {
            auto &island = islands[i];

            if (island.nodes.size() > biggest_size) {
                biggest_size = island.nodes.size();
                biggest_idx = i;
            }
        }

        source_island = std::move(islands[biggest_idx]);
        // swap with last and pop.
        islands[biggest_idx] = std::move(islands.back());
        islands.pop_back();

        remove_sleeping_tag_from_island(*m_registry, source_island_entity, source_island);
        const bool disabled = disabled_view.contains(source_island_entity);

        /* Update island AABB. */ {
            auto is_first_node = true;
            auto &island_aabb = island_view.get<island_AABB>(source_island_entity);

            for (auto entity : source_island.nodes) {
                if (procedural_view.contains(entity) && aabb_view.contains(entity)) {
                    auto [node_aabb] = aabb_view.get(entity);

                    if (is_first_node) {
                        island_aabb = {node_aabb};
                        is_first_node = false;
                    } else {
                        island_aabb = {enclosing_aabb(island_aabb, node_aabb)};
                    }
                }
            }
        }

        for (auto &other_island : islands) {
            auto island_entity_new = m_registry->create();
            auto &island_new = m_registry->emplace<edyn::island>(island_entity_new, std::move(other_island));
            auto &aabb = m_registry->emplace<island_AABB>(island_entity_new);
            auto is_first_node = true;

            for (auto node_entity : island_new.nodes) {
                bool is_procedural = procedural_view.contains(node_entity);

                if (is_procedural) {
                    m_registry->patch<island_resident>(node_entity, [island_entity_new](island_resident &resident) {
                        resident.island_entity = island_entity_new;
                    });
                } else {
                    auto [resident] = multi_resident_view.get(node_entity);
                    resident.island_entities.push(island_entity_new);

                    // Remove the original island if this non-procedural entity
                    // is not contained in it anymore.
                    auto &original_island = m_registry->get<edyn::island>(source_island_entity);
                    if (!original_island.nodes.contains(node_entity)) {
                        resident.island_entities.remove(source_island_entity);
                    }
                }

                // Update island AABB by uniting all AABBs of all
                // procedural entities.
                if (is_procedural && aabb_view.contains(node_entity)) {
                    auto [node_aabb] = aabb_view.get(node_entity);

                    if (is_first_node) {
                        aabb = {node_aabb};
                        is_first_node = false;
                    } else {
                        aabb = {enclosing_aabb(aabb, node_aabb)};
                    }
                }
            }

            for (auto edge_entity : island_new.edges) {
                m_registry->patch<island_resident>(edge_entity, [island_entity_new](island_resident &resident) {
                    resident.island_entity = island_entity_new;
                });
            }

            remove_sleeping_tag_from_island(*m_registry, island_entity_new, island_new);

            m_registry->emplace<island_tag>(island_entity_new);

            // Inherit disabled status.
            if (disabled) {
                m_registry->emplace<disabled_tag>(island_entity_new);
            }
        }
    }

    m_islands_to_split.clear();
}

void island_manager::wake_up_islands() {
    for (auto island_entity : m_islands_to_wake_up) {
        if (m_registry->valid(island_entity)) {
            wake_up_island(*m_registry, island_entity);
        }
    }
    m_islands_to_wake_up.clear();
}

void island_manager::update(double timestamp) {
    wake_up_islands();
    init_new_nodes_and_edges();
    split_islands();
    put_islands_to_sleep();
    m_last_time = timestamp;
}

void island_manager::put_to_sleep(entt::entity island_entity) {
    m_registry->emplace<sleeping_tag>(island_entity);

    auto &island = m_registry->get<edyn::island>(island_entity);
    auto procedural_view = m_registry->view<procedural_tag>();

    // Assign `sleeping_tag` to all procedural entities.
    for (auto entity : island.nodes) {
        if (!procedural_view.contains(entity)) continue;

        m_registry->emplace<sleeping_tag>(entity);

        if (m_registry->all_of<linvel>(entity)) {
            m_registry->get<linvel>(entity) = vector3_zero;
        }

        if (m_registry->all_of<angvel>(entity)) {
            m_registry->get<angvel>(entity) = vector3_zero;
        }
    }

    for (auto entity : island.edges) {
        m_registry->emplace<sleeping_tag>(entity);
    }
}

void island_manager::put_all_to_sleep() {
    for (auto island_entity : m_registry->view<island>(exclude_sleeping_disabled)) {
        put_to_sleep(island_entity);
    }
}

bool island_manager::could_go_to_sleep(entt::entity island_entity) const {
    auto &island = m_registry->get<edyn::island>(island_entity);
    auto sleeping_disabled_view = m_registry->view<sleeping_disabled_tag>();

    // If any entity has a `sleeping_disabled_tag` then the island should
    // not go to sleep, since the movement of all entities depend on one
    // another in the same island.
    for (auto entity : island.nodes) {
        if (sleeping_disabled_view.contains(entity)) {
            return false;
        }
    }

    // Check if there are any entities moving faster than the sleep threshold.
    auto vel_view = m_registry->view<linvel, angvel>();

    for (auto entity : island.nodes) {
        if (!vel_view.contains(entity)) {
            continue;
        }

        auto [v, w] = vel_view.get(entity);

        if ((length_sqr(v) > island_linear_sleep_threshold * island_linear_sleep_threshold) ||
            (length_sqr(w) > island_angular_sleep_threshold * island_angular_sleep_threshold)) {
            return false;
        }
    }

    return true;
}

void island_manager::put_islands_to_sleep() {
    auto island_view = m_registry->view<island>(exclude_sleeping_disabled);

    for (auto [entity, island] : island_view.each()) {
        if (could_go_to_sleep(entity)) {
            if (!island.sleep_timestamp) {
                island.sleep_timestamp = m_last_time;
            } else {
                auto sleep_dt = m_last_time - *island.sleep_timestamp;
                if (sleep_dt > island_time_to_sleep) {
                    put_to_sleep(entity);
                    island.sleep_timestamp.reset();
                }
            }
        } else {
            island.sleep_timestamp.reset();
        }
    }
}

void island_manager::set_procedural(entt::entity entity, bool is_procedural) {
    if (is_procedural) {
        if (auto *resident = m_registry->try_get<multi_island_resident>(entity)) {
            entt::entity merged_island_entity;

            if (resident->island_entities.empty()) {
                m_registry->remove<multi_island_resident>(entity);
                m_registry->emplace<island_resident>(entity);

                auto island_entity = create_island();
                auto node_entities = std::vector<entt::entity>{};
                node_entities.push_back(entity);

                insert_to_island(island_entity, node_entities, {});
            } else {
                if (resident->island_entities.size() > 1) {
                    auto island_entities = std::vector(resident->island_entities.begin(), resident->island_entities.end());
                    merged_island_entity = merge_islands(island_entities, {}, {});
                } else {
                    merged_island_entity = *resident->island_entities.begin();
                }

                resident->island_entities.clear(); // Must clear because `on_destroy_multi_island_resident` will be called.
                m_registry->remove<multi_island_resident>(entity);
                m_registry->emplace<island_resident>(entity, merged_island_entity);
            }
        }
    } else {
        if (auto *resident = m_registry->try_get<island_resident>(entity)) {
            entt::sparse_set island_entities;
            island_entities.push(resident->island_entity);
            resident->island_entity = entt::null;

            m_registry->emplace<multi_island_resident>(entity, std::move(island_entities));
            m_registry->remove<island_resident>(entity);
        }
    }
}

}
