#ifndef EDYN_EDYN_HPP
#define EDYN_EDYN_HPP

#include "edyn/build_settings.h"
#include "comp/shared_comp.hpp"
#include "comp/dirty.hpp"
#include "comp/graph_node.hpp"
#include "comp/graph_edge.hpp"
#include "comp/present_position.hpp"
#include "comp/present_orientation.hpp"
#include "math/constants.hpp"
#include "math/scalar.hpp"
#include "math/vector3.hpp"
#include "math/vector2.hpp"
#include "math/quaternion.hpp"
#include "math/matrix3x3.hpp"
#include "math/transform.hpp"
#include "math/math.hpp"
#include "math/geom.hpp"
#include "time/time.hpp"
#include "util/rigidbody.hpp"
#include "util/ragdoll.hpp"
#include "util/constraint_util.hpp"
#include "util/shape_util.hpp"
#include "util/shape_volume.hpp"
#include "util/tuple_util.hpp"
#include "util/exclude_collision.hpp"
#include "collision/contact_manifold.hpp"
#include "collision/contact_point.hpp"
#include "shapes/create_paged_triangle_mesh.hpp"
#include "serialization/s11n.hpp"
#include "parallel/job_dispatcher.hpp"
#include "parallel/parallel_for.hpp"
#include "parallel/parallel_for_async.hpp"
#include "parallel/message_queue.hpp"
#include "parallel/island_coordinator.hpp"
#include "util/moment_of_inertia.hpp"
#include "util/registry_operation_builder.hpp"
#include "collision/contact_manifold_map.hpp"
#include "context/settings.hpp"
#include "collision/raycast.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

/**
 * @brief Initialization configuration.
 */
struct init_config {
    // Number of worker threads to spawn. If zero, value will be taken from
    // `std::thread::hardware_concurrency`.
    size_t num_worker_threads {0};
};

/**
 * @brief Initializes Edyn's internals such as its thread pool and job system.
 * Call it before using Edyn.
 */
void init(const init_config &config = {});

/**
 * @brief Undoes what was done by `init()`. Call it when Edyn is not needed anymore.
 */
void deinit();

/**
 * @brief Attaches Edyn to an EnTT registry.
 * @param registry The registry to be setup to run Edyn.
 */
void attach(entt::registry &registry);

/**
 * @brief Detaches Edyn from an EnTT registry.
 * @param registry The registry to be freed from Edyn's context.
 */
void detach(entt::registry &registry);

/**
 * @brief Get the fixed simulation delta time for each step.
 * @param registry Data source.
 * @return Fixed delta time in seconds.
 */
scalar get_fixed_dt(const entt::registry &registry);

/**
 * @brief Set the fixed simulation delta time for each step.
 * @param registry Data source.
 * @param dt Delta time in seconds.
 */
void set_fixed_dt(entt::registry &registry, scalar dt);

/**
 * @brief Checks if simulation is paused.
 * @param registry Data source.
 * @return Whether simulation is paused.
 */
bool is_paused(const entt::registry &registry);

/**
 * @brief Pauses simulation.
 * @param registry Data source.
 */
void set_paused(entt::registry &registry, bool paused);

/**
 * @brief Updates the simulation. Call it regularly.
 * The actual physics simulation runs in other threads. This function only
 * does coordination of background simulation jobs. It's expected to be a
 * lightweight call.
 * @param registry Data source.
 */
void update(entt::registry &registry);

/**
 * @brief Runs a single step for a paused simulation.
 * @param registry Data source.
 */
void step_simulation(entt::registry &registry);

/**
 * @brief Get index of a component type among all shared components within the
 * library. Also supports any registered external component.
 * @tparam Component The component type.
 * @param registry Data source.
 * @return Component index.
 */
template<typename Component>
size_t get_component_index(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    return settings.index_source->index_of<Component>();
}

/**
 * @brief Get indices of a sequence of component types among all shared
 * components in the library. Also supports any registered external component.
 * @tparam IndexType Desired integral index type.
 * @tparam Component The component type.
 * @param registry Data source.
 * @return Component index.
 */
template<typename IndexType, typename... Component>
auto get_component_indices(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();
    return settings.index_source->indices_of<IndexType, Component...>();
}

/**
 * @brief Registers external components to be shared between island coordinator
 * and island workers.
 * @remark If you need a component of your own to be available in an external
 * system, it must be registered using this function. That will ensure the
 * component is sent to the island workers and is inserted in their private
 * registry.
 * @tparam Component External component types.
 */
template<typename... Component>
void register_external_components(entt::registry &registry) {
    auto &settings = registry.ctx<edyn::settings>();

    settings.make_reg_op_builder = [] () {
        auto external = std::tuple<Component...>{};
        auto all_components = std::tuple_cat(shared_components, external);
        return std::unique_ptr<registry_operation_builder>(
            new registry_operation_builder_impl(all_components));
    };

    auto external = std::tuple<Component...>{};
    auto all_components = std::tuple_cat(shared_components, external);
    settings.index_source.reset(new component_index_source_impl(all_components));

    registry.ctx<island_coordinator>().settings_changed();
}

template<typename... Component>
void register_external_components(entt::registry &registry, [[maybe_unused]] std::tuple<Component...>) {
    register_external_components<Component...>(registry);
}

/**
 * @brief Removes registered external components and resets to defaults.
 */
void remove_external_components(entt::registry &registry);

/**
 * @brief Assigns a function to be called once after a new island worker is
 * created and initialized in a worker thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_external_system_init(entt::registry &registry, external_system_func_t func);

/**
 * @brief Assigns a function to be called before each simulation step in each
 * island worker in a worker thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_external_system_pre_step(entt::registry &registry, external_system_func_t func);

/**
 * @brief Assigns a function to be called after each simulation step in each
 * island worker in a worker thread.
 * @param registry Data source.
 * @param func The function.
 */
void set_external_system_post_step(entt::registry &registry, external_system_func_t func);

/**
 * @brief Set all external system functions in one call. See individual setters
 * for details.
 * @param registry Data source.
 * @param init_func Initialization function called on island worker start up.
 * @param pre_step_func Called before each step.
 * @param post_step_func Called after each step.
 */
void set_external_system_functions(entt::registry &registry,
                                   external_system_func_t init_func,
                                   external_system_func_t pre_step_func,
                                   external_system_func_t post_step_func);

/**
 * @brief Remove all external system functions.
 * @param registry Data source.
 */
void remove_external_systems(entt::registry &registry);

/**
 * @brief Assigns an `external_tag` to this entity and inserts it as a node
 * into the entity graph.
 * This makes it possible to tie this entity and its components to another
 * node such as a rigid body, which means that it will be moved into the
 * island where the rigid body resides.
 * @param registry Data source.
 * @param entity The entity to be tagged.
 * @param procedural If true, the entity will reside exclusively in one island.
 */
void tag_external_entity(entt::registry &registry, entt::entity entity, bool procedural = true);

/**
 * @brief Overrides the default collision filtering function, which checks
 * collision groups and masks. Remember to return false if both entities
 * are the same.
 * @param registry Data source.
 * @param func The function.
 */
void set_should_collide(entt::registry &registry, should_collide_func_t func);

/**
 * @brief Propagates changes to a component to the island worker where the
 * entity currently resides.
 * @tparam Component Component type.
 * @param registry Data source.
 * @param entity The entity that owns the component.
 */
template<typename... Component>
void refresh(entt::registry &registry, entt::entity entity) {
    if (auto *coordinator = registry.try_ctx<island_coordinator>(); coordinator) {
        coordinator->refresh<Component...>(entity);
    }
}

/**
 * @brief Checks whether there is a contact manifold connecting the two entities.
 * @param registry Data source.
 * @param first One entity.
 * @param second Another entity.
 * @return Whether a contact manifold exists between the two entities.
 */
bool manifold_exists(entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc manifold_exists */
bool manifold_exists(entt::registry &registry, entity_pair entities);

/**
 * @brief Get contact manifold entity for a pair of entities.
 * Asserts if the manifold does not exist.
 * @param registry Data source.
 * @param first One entity.
 * @param second Another entity.
 * @return Contact manifold entity.
 */
entt::entity get_manifold_entity(const entt::registry &registry, entt::entity first, entt::entity second);

/*! @copydoc get_manifold_entity */
entt::entity get_manifold_entity(const entt::registry &registry, entity_pair entities);

/**
 * @brief Signal triggered when a contact starts.
 * A contact is considered to start when the first contact point is added to a
 * manifold, i.e. when the number of points in a manifold becomes greater than
 * zero.
 * @param registry Data source.
 * @return Sink to observe contact started events.
 */
entt::sink<void(entt::entity)> on_contact_started(entt::registry &);

/**
 * @brief Signal triggered when a contact ends.
 * A contact ends when the last point is destroyed in a contact manifold, i.e.
 * when the number of points goes to zero, or when a manifold is destroyed due
 * to AABB separation.
 * @param registry Data source.
 * @return Sink to observe contact ended events.
 */
entt::sink<void(entt::entity)> on_contact_ended(entt::registry &);

/**
 * @brief Signal triggered when a contact point is created.
 * This event is also triggered right after a contact started event, for each
 * point that the contact has started with.
 * The signal emits the manifold entity and the contact point id in that manifold.
 * @param registry Data source.
 * @return Sink to observe contact point creation events.
 */
entt::sink<void(entt::entity, contact_manifold::contact_id_type)> on_contact_point_created(entt::registry &);

/**
 * @brief Signal triggered when a contact point is destroyed.
 * This event is also triggered for each contact point before a contact ended
 * event.
 * The signal emits the manifold entity and the contact point id in that manifold.
 * @param registry Data source.
 * @return Sink to observe contact point destruction events.
 */
entt::sink<void(entt::entity, contact_manifold::contact_id_type)> on_contact_point_destroyed(entt::registry &);

/**
 * @brief Visit all edges of a node in the entity graph. This can be used to
 * iterate over all constraints assigned to a rigid body.
 * @remark `contact_constraint`s are not edges in the entity graph. Instead,
 * you're going to find a contact manifold in an edge graph an the contact
 * points and contact constraints can be obtained from it.
 * @tparam Func Visitor function type.
 * @param entity Node entity.
 * @param func Vistor function with signature `void(index_type)`.
 */
template<typename Func>
void visit_edges(entt::registry &registry, entt::entity entity, Func func) {
    auto &node = registry.get<graph_node>(entity);
    auto &graph = registry.ctx<entity_graph>();
    graph.visit_edges(node.node_index, func);
}

/**
 * @brief Get default gravity.
 * This value is assigned as the gravitational acceleration for all new rigid bodies.
 * @param registry Data source.
 * @return Gravity acceleration vector.
 */
vector3 get_gravity(const entt::registry &registry);

/**
 * @brief Changes the default gravity acceleration.
 * This value is assigned as the gravitational acceleration for all new rigid bodies.
 * @param registry Data source.
 * @param gravity The new default gravity acceleration.
 */
void set_gravity(entt::registry &registry, vector3 gravity);

/**
 * @brief Get the number of constraint solver velocity iterations.
 * @param registry Data source.
 * @return Number of solver velocity iterations.
 */
unsigned get_solver_velocity_iterations(const entt::registry &registry);

/**
 * @brief Set the number of constraint solver velocity iterations.
 * @param registry Data source.
 * @param iterations Number of solver velocity iterations.
 */
void set_solver_velocity_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Get the number of constraint solver position iterations.
 * @param registry Data source.
 * @return Number of solver position iterations.
 */
unsigned get_solver_position_iterations(const entt::registry &registry);

/**
 * @brief Set the number of constraint solver position iterations.
 * @param registry Data source.
 * @param iterations Number of solver position iterations.
 */
void set_solver_position_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Get the number of restitution iterations.
 * @param registry Data source.
 * @return Number of restitution iterations.
 */
unsigned get_solver_restitution_iterations(const entt::registry &registry);

/**
 * @brief Set the number of restitution iterations. The restitution solver will
 * stop early once the penetration velocity of all contact points is above a
 * threshold.
 * @param registry Data source.
 * @param iterations Number of restitution iterations.
 */
void set_solver_restitution_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Get the number of individual restitution iterations.
 * @param registry Data source.
 * @return Number of individual restitution iterations.
 */
unsigned get_solver_individual_restitution_iterations(const entt::registry &registry);

/**
 * @brief Set the number of individual restitution iterations. This is the number
 * of iterations used while solving each subset of contact constraints in each
 * iteration of the restitution solver.
 * @param registry Data source.
 * @param iterations Number of individual restitution iterations.
 */
void set_solver_individual_restitution_iterations(entt::registry &registry, unsigned iterations);

/**
 * @brief Use the provided material when two rigid bodies with the given
 * material ids collide.
 * @param registry Data source.
 * @param material_id0 ID of a material.
 * @param material_id1 ID of another material (could be equal to material_id0).
 * @param material Material info.
 */
void insert_material_mixing(entt::registry &registry, material::id_type material_id0,
                            material::id_type material_id1, const material_base &material);

}

#endif // EDYN_EDYN_HPP
