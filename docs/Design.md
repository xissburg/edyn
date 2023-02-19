# Edyn Design Document

This document describes the general engine architecture. It is a bit of a brainstorming document and does not reflect the current state of the library. The ideas presented here are planned to be implemented in the near future.

# Introduction

_Edyn_ (pronounced as _eh-dyin'_) stands for _Entity Dynamics_ and it is a real-time physics engine focused on multi-threaded, networked and distributed simulation of massive dynamic worlds. It is organized as an _entity-component system_ (ECS) using the amazing [EnTT](https://github.com/skypjack/entt) library.

## The ECS approach

Typical physics engines will offer explicit means to create objects such as rigid bodies, whereas in _Edyn_ object creation is implicit due to the entity-component design. A rigid body is created from the bottom up, by associating its parts to a single entity, such as:

```cpp
entt::registry registry;
auto entity = registry.create();
registry.emplace<edyn::dynamic_tag>(entity);
registry.emplace<edyn::position>(entity, 0, 3, 0);
registry.emplace<edyn::orientation>(entity, edyn::quaternion_axis_angle({0, 1, 0}, edyn::to_radians(30)));
registry.emplace<edyn::linvel>(entity, edyn::vector3_zero);
registry.emplace<edyn::angvel>(entity, 0, 0.314, 0);
auto mass = edyn::scalar{50};
registry.emplace<edyn::mass>(entity, mass);
auto &box = registry.emplace<edyn::box_shape>(entity, edyn::vector3{0.5, 0.2, 0.4});
registry.emplace<edyn::inertia>(entity, edyn::moment_of_inertia(box, mass));
registry.emplace<edyn::material>(entity, 0.2, 0.9); // Restitution and friction.
registry.emplace<edyn::gravity>(entity, edyn::gravity_earth);
```

There's no explicit mention of a rigid body in the code, but during the physics update all entities that have a combination of the components assigned above will be treated as a rigid body and their state will be update over time as expected. The update may be carried as follows:

```cpp
// Apply gravity acceleration, increasing linear velocity
auto view = registry.view<edyn::linvel, edyn::gravity, edyn::dynamic_tag>();
for (auto [entity, vel, g] : view.each()) {
  vel += g * dt;
}
// ...
// Move entity with its linear velocity
auto view = registry.view<edyn::position, edyn::linvel, edyn::dynamic_tag>();
for (auto [entity, pos, vel] : view.each()) {
  pos += vel * dt;
}
// ...
// Rotate entity with its angular velocity
auto view = registry.view<edyn::orientation, edyn::angvel, edyn::dynamic_tag>();
for (auto [entity, orn, vel] : view.each()) {
  orn = edyn::integrate(orn, vel, dt);
}
```

Assigning each component to every rigid body entity individually quickly becomes a daunting task which is prone to errors, thus utility functions are provided for common tasks such as creating rigid bodies:

```cpp
// Equivalent to implicit example above.
auto def = edyn::rigidbody_def();
def.kind = edyn::rigidbody_kind::rb_dynamic;
def.position = {0, 3, 0};
def.orientation = edyn::quaternion_axis_angle({0, 1, 0}, edyn::to_radians(30));
def.linvel = edyn::vector3_zero;
def.angvel = {0, 0.314, 0};
def.mass = 50;
def.shape = edyn::box_shape{0.5, 0.2, 0.4}; // Shape is optional.
def.material->restitution = 0.2;
def.material->friction = 0.9;
def.gravity = edyn::gravity_earth;
auto entity = edyn::make_rigidbody(registry, def);
```

It is not necessary to assign a shape to a rigid body. That enables the simulation to contain amorphous rigid bodies which are not visually present in the simulation and don't participate in collision detection, but instead are connected to other bodies via constraints and are used to generate forces that affect the primary entities that users interact with. As an example, this can be useful to simulate drivetrain components in a vehicle.

# The Physics Step

This is the order of major updates during one simulation step:

1. Process messages and registry operations, thus synchronizing state with the main registry, which might create new entities in the local registry.
2. Run broad-phase collision detection. Contact manifolds are created and destroyed in this step.
3. Run narrow-phase collision detection. Closest point calculations are performed and contact points are created.
4. Solve restitution. This is done before applying gravity to prevent resting bodies from bouncing.
5. Apply external forces such as gravity.
6. Solve constraints.
7. Apply constraint impulses.
8. Integrate velocities to obtain new positions and orientations.
9. Asynchronous execution only: Send registry operations accumulated during step to main thread.

# Foundation

The library can be built with single- or double-precision floating point. `edyn::scalar` is simply a `using` declaration equals to `float` or `double` which is set according to the `EDYN_DOUBLE_PRECISION` compilation option. _build_settings.hpp_ is generated during build from _cmake/build_settings.h.in_ so that invocations are linked to the correct definition.

`edyn::vector3` is the vector type for positions and directions.

`edyn::vector2` is the vector type for 2D positions and directions, which is used in some of the intersection algorithms for collision detection when the problem is projected onto a plane.

`edyn::quaternion` is the quaternion type for orientations.

`edyn::matrix3x3` is the 3x3 matrix also used for orientations and orthonormal bases.

_SIMD_ implementations of the above are planned.

# Constraints

Constraints create a relationship between degrees of freedom of rigid bodies, preventing them from moving beyond the allowed range. Constraints are defined as simple structs which hold the `entt::entity` of the bodies it connects and any other specific data such as pivot points in object space. A function that prepares constraints must be provided for each type. These preparation functions configure one or more constraint rows for each constraint. The constraint row corresponds to one constraint equation or inequation in the system of constraints. It contains the Jacobian and impulse limits. These functions are called by the constraint solver right before the solver iterations.

There is no flexibility when it comes to adding new constraints to the library. If that's needed, it'll be necessary to fork the project and add the new constraint internally.

A traditional Sequential Impulse constraint solver is used.

# Collision detection and response

Collision detection is split in two phases: broad-phase and narrow-phase. In broad-phase potential collision pairs are found by checking if the AABBs of different entities are intersecting. Later, in the narrow-phase, closest points are calculated for these pairs.

During broad-phase, intersections between the AABBs of all entities are found using a _dynamic bounding volume tree_, according to [Dynamic Bounding Volume Hierarchies, Erin Catto, GDC 2019](https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf) and [Box2D](https://github.com/erincatto/box2d/blob/master/include/box2d/b2_dynamic_tree.h) [b2DynamicTree](https://github.com/erincatto/box2d/blob/master/src/collision/b2_dynamic_tree.cpp). The AABBs are inflated by a threshold so that contact pairs can be generated before the bodies start to penetrate thus generating contact points in advance. For any new intersection, an entity is created and a `edyn::contact_manifold` component is assigned to it. For any AABB intersection that ceased to exist, the entity is destroyed thus also destroying all components associated with it. The AABB is inflated a bit more when looking for separation to avoid a situation where they'd join and separate repeatedly. This is sometimes called _hysteresis_.

In narrow-phase, closest point calculation is performed for the rigid body pair in all `edyn::contact_manifold`s. The _Separating-Axis Theorem (SAT)_ is employed. A _GJK_ implementation is planned but _SAT_ is preferred due to greater control and precision and better ability to debug and reason about the code.

The `edyn::contact_manifold` component holds information of all contact points and if the rigid body has a material, a `edyn::contact_constraint` is assigned to the same entity. In the constraint preparation function, the `edyn::contact_constraint` gets information from the `edyn::contact_manifold` to set up constraint rows.

Contact point persistence is important for stability and to preserve continuity over time. New contact points that are near existing points get merged together, thus extending the lifetime of an existing contact and reusing the previously applied impulse for _warm starting_ later in the constraint solver. Contact points that are separating (in either tangential or normal directions) are destroyed.

Entities that don't have a shape also don't have a `edyn::AABB` assigned to them and thus are ignored in broad-phase which leads to no collisions ever happening. Rigid bodies without a shape are termed _amorphous_.

To enable collision response for an entity, a `edyn::material` component must be assigned which basically contains the _restitution_ and _friction coefficient_. Otherwise, the entity behaves as a _sensor_, i.e. collision detection is performed but there's no collision response, i.e. no impulses are applied. Intersection events can still be observed.

When rigid bodies collide, the material properties must be merged into one for the collision response, since a single coefficient of friction and restitution are required by the `edyn::contact_constraint`. The individual properties of the material of each rigid body are combined by default mixing functions, which for example, could multiply the values or take their minimum. To satisfy more advanced requirements, materials can have a numerical id and together with a _material mixing table_, separate material properties can be inserted to be used when rigid bodies made of a specific type of material collide. This table simply maps a pair of material ids into a single material. When a collision happens, if there is an entry in the table for the material pair, the values in that entry will be assigned to the `edyn::contact_constraint`. Otherwise, the values of each material will be combined using the mixing functions.

## Separating-Axis Theorem and Implementation

The _Separating-Axis Theorem (SAT)_ states that if there is one axis where the intervals resulted from the projection of two convex shapes on this axis do not intersect, then the shapes also do not intersect. The projections on the axis can be used to determine the signed distance along that axis. The axis with largest signed distance gives us the _minimum translation vector_, which is a minimal displacement that can be applied to either shape to bring them into contact if they're not intersecting or separate them if they're intersecting. Using this axis, the closest features can be found on each shape and a contact manifold can be assembled.

The general structure of a _SAT_ implementation initially searches for the axis that gives the largest signed distance between the projections of the two shapes _A_ and _B_. The direction is always chosen to point outside of _B_ and towards _A_. Then the projection of _A_ is taken as the negative of the largest projection onto the opposite direction and the projection of _B_ is taken as the largest projection onto the axis. This gives us the maximal projection along the axis for both shapes and the distance is simply the difference between the projection of _A_ and the projection of _B_. If the largest distance is bigger than a threshold (usually 2cm), the collision is ignored and no contact points are generated. Otherwise, the _support features_ of each shape are found along that direction. Then the closest points between the support features are calculated as the contact points.

The support feature is a _feature_ that's located furthest along a direction. A _feature_ is a simpler element of a shape, such as vertex, edge and face on a box or polyhedron, cap face, side edge and cap edge on a cylinder. This concept allows the collision detection to be split in two steps: first the closest features are found, then the closest points between the two features are found. Features are simpler and allows for the exact contact points to be calculated in a more manageable way.

The features are intersected on a case-by-case basis and contact points are inserted into the resulting manifold which holds a limited number of points. If the manifold is full, it has to replace an existing point by the new, or leave it as is. The deciding factor is the surface area of the convex hull of the contact points, which should be maximized. Four points is the default maximum number of points, thus the convex hull is at most a tetrahedron, which is always convex, thus making it easy to calculate the surface area. This metric selects the larger manifolds while keeping points that are deeper than most.

## Collision Events

Contact points are part of the `edyn::contact_manifold` component and they're added and removed to its list of points during the collision detection step. To allow users to watch contact points come and go, a set of signals are provided.

To generate these signals, the id of all contact points that were created and destroyed are stored in a `edyn::contact_manifold_events` component during collision detection. This makes it possible to store all contact events that happened in one step for later consumption.

## Restitution

A non-zero coefficient of restitution allows rigid bodies to bounce off one another after a collision. In other words, it establishes the ratio between the relative velocity after and before a collision. A value of zero will make the relative velocity go to zero. A value of one will cause a perfectly elastic collision and the new relative velocity will be equal and opposite to the initial relative velocity.

In impulse based dynamics, restitution can be obtained using a velocity bias in the right-hand side (RHS) of the constraint equation. That will cause the solver to apply impulses to make the relative velocity attain that value. The issue is that only the constraints that connect to the fast moving bodies will receive the extra impulse, since others will initially have a zero relative velocity at the contact points and thus will not have a velocity bias on the RHS. That will lead to inelastic collisions despite the non-zero coefficient of restitution.

Given that the sequential impulses constraint solver produces correct results on the first level of colliding bodies with non-zero restitution, one solution is to first solve these contact constraints in small groups, these groups being the contacts originating at the rigid bodies that are initially moving, starting at the contacts that have the highest penetration velocity. The velocity deltas must be applied to the velocity of the rigid bodies before moving onto the next group of constraints so the relative velocity will be correctly assigned to the RHS of the constraint equation in the next group. The entity graph is used to navigate through the constraints, starting a graph traversal at the faster body and solving all contact constraints on its edges and then repeating the process in the neighboring nodes successively. One iteration is often enough for a sequence of connected rigid bodies. More complex configurations might require more iterations to ensure a complete shock propagation. This step is _solved_ when the relative velocity at the contact points of all contact manifolds with non-zero restitution is above zero.

The contact constraints can be solved normally after the special restitution step. This will produce correct results for chains of rigid bodies in scenarios such as the Newton's Cradle and billiards.

# Shapes

The physical shape of a rigid body can be any of the `edyn::*_shape` components, which are assigned directly to the rigid body entity. Along with the shape of specific type, a `edyn::shape_index` is assigned which can be used to read the shape an entity contains using the `edyn::visit_shape` function.

It's necessary to fork the project and modify the code to add custom shapes. It is also necessary to provide a `edyn::collide` function for every permutation of the custom shape with all existing shapes.

## Polyhedrons and rotated mesh optimization

To avoid having to rotate every vertex position and face normal when doing closest point calculation involving polyhedrons, they are rotated only once after the simulation step and are cached in a `edyn::rotated_mesh`. These rotated values can be reused in multiple collision tests in a single step (note that not all collision tests use these values since most of them are done in the polyhedron's object space).

Unlike the `edyn::convex_mesh` held by a polyhedron, the `edyn::rotated_mesh` is mutable and is only meaningful to the entity it is assigned to, whereas the `edyn::convex_mesh` is immutable and thread-safe and can be shared among multiple polyhedrons. Thus, a new `edyn::rotated_mesh` is created for every new polyhedron in the `edyn::island_worker`. Having the same instance being shared with other workers would not be a problem for dynamic entities, since they can only be present in one worker at a time. However, that's not true for kinematic objects, which can hold a polyhedron shape and be presented in multiple threads.

The polyhedron keeps a weak reference to the `edyn::rotated_mesh` thus the `edyn::island_worker` actually owns the rotated meshes and is responsible for keeping them alive until the polyhedron is destroyed. They are stored in `edyn::rotated_mesh_list` components because `edyn::compound_shape`s can hold multiple polyhedrons, thus it is necessary to be able to store a list of `edyn::rotated_mesh`es for them. The first `edyn::rotated_mesh_list` is assigned to the entity holding the shape itself. New entities are created for the next ones and are linked to the previous. When the original entity is deleted, all linked `edyn::rotated_mesh_list` are deleted in succession.

Furthermore, an array of unique face normals and edge directions are stored in the `edyn::convex_mesh` and their rotated state in a `edyn::rotated_mesh` to avoid testing the same axis multiple times in SAT implementations involving polyhedron shapes. E.g. in a box shaped polyhedron, only 3 edge directions will be considered instead of all 12 edges. They are termed the _relevant_ face normals and edge directions.

## Triangle mesh shape

The `edyn::triangle_mesh` represents a (usually large) concave mesh of triangles. It contains a static bounding volume tree which provides a quicker way to find all triangles that intersect a given AABB. The `edyn::mesh_shape` holds a `std::shared_ptr` to a `edyn::triangle_mesh` which allows it to be present in multiple registries without duplicating the `edyn::triangle_mesh`, which generally contains a lot of data.

The concept of Voronoi regions is used to prevent internal edge collisions. The normal vector of all three adjacent triangles is stored for each triangle. Using the adjacent normal, it is possible to tell whether a direction (separating axis or minimum translation vector) lies in a valid region. If the axis is not in the voronoi region of the closest triangle feature, it is projected onto it so a valid direction is used.

Triangle meshes can be set up with a list of vertices and indices and then it calculates everything that's needed with an invocation of `edyn::triangle_mesh::initialize()`. The list of vertices and indices can be loaded from an `*.obj` file using `edyn::load_tri_mesh_from_obj`. Loading from an `*.obj` can be slow because of parsing and recalculation of all internal properties of a triangle mesh, such as triangle normals, edge normals and vertex tangents. To speed things up, the triangle mesh can be written into a binary file using an output archive:

```cpp
auto output = edyn::file_output_archive("trimesh.bin");
edyn::serialize(output, trimesh);
```

And then loaded quickly using an input archive:

```cpp
auto input = edyn::file_input_archive("trimesh.bin");
edyn::serialize(input, trimesh);
```

## Paged triangle mesh shape

For the shape of the world's terrain, a triangle mesh shape is usually the best choice. For larger worlds, it is interesting to split up this terrain in smaller chunks and load them in and out of the world as needed. The `edyn::paged_triangle_mesh` offers a deferred loading mechanism that will load chunks of a concave triangle mesh as dynamic objects enter their bounding boxes. It keeps a static bounding volume tree with one `edyn::triangle_mesh` on each leaf node and loads them on demand. The `edyn::paged_mesh_shape` holds a `std::shared_ptr` to a `edyn::paged_triangle_mesh` which allows it to be shared among multiples registries without duplicating the data.

It can be created from a list of vertices and indices using the `edyn::create_paged_triangle_mesh` function, which will split the large mesh into smaller chunks. Right after the call, all submeshes will be loaded into the cache which allows it to be fully written to a binary file using a `edyn::paged_triangle_mesh_file_output_archive`. The cache can be cleared afterwards calling `edyn::paged_triangle_mesh::clear_cache()`. Now the mesh can be loaded quickly from file using a `edyn::paged_triangle_mesh_file_input_archive`.

As dynamic entities move into the AABB of the submeshes, it will ask the loader to load the triangle mesh for that region if it's not available yet. It uses a `edyn::triangle_mesh_page_loader_base` to load the required triangle mesh (usually asynchronously) and then will assign a `edyn::triangle_mesh` to the node when done. Since it might take time to load the mesh from file and deserialize it, the query AABB should be inflated to prevent collisions from being missed.

When there are no dynamic entities in the AABB of the submesh, it becomes a candidate for unloading.

In the creation process of a `edyn::paged_triangle_mesh`, the whole mesh is loaded into a single `edyn::triangle_mesh`. Then, it's split up into smaller chunks during the construction of the static bounding volume tree of submeshes, which is configured to continue splitting until the number of triangles in a node is under a certain threshold. For each leaf node, a new `edyn::triangle_mesh` is created containing only the triangles in that node. The submeshes require a special initialization procedure so that adjacency with other submeshes can be accounted for. This part will take already calculated information from the global triangle mesh and assign that directly into the submesh, particularly adjacent triangle normals, which are crucial to prevent internal edge collisions at the submesh boundaries.

## Per-vertex material properties

The `edyn::mesh_shape` and `edyn::paged_mesh_shape` support per-vertex material properties, which allow friction and restitution coefficients to be assigned to each vertex and then the coefficient for each contact point is interpolated over the triangle where the point is located. These coefficients can be assigned using `edyn::triangle_mesh::insert_friction_coefficients` and they can also be loaded from the vertex colors of an _*.obj_ file via `edyn::load_tri_mesh_from_obj` and passed to `edyn::create_paged_triangle_mesh` in the last parameter.

When creating and updating contact points, if a triangle mesh with per-vertex materials is involved, the coefficients will be obtained from the mesh and assigned to the point. The closest feature present in the contact point is used to interpolate the coefficient among the vertices of that feature and obtain a value for the contact point location. If the closest feature is a vertex, the vertex value is used; if it's an edge, the values are linearly interpolated between the two vertices of the edge; if it's a triangle, barycentric coordinates are used to interpolate the values among the three vertices.

# Simulation Islands

Dynamic entities that cannot immediately affect the motion of others can be simulated in isolation. More precisely, two dynamic entities _A_ and _B_ which are not connected via constraints are not capable of immediately affecting the motion of each other. That means, the motion of _A_ and _B_ is independent and thus could be performed in two separate threads.

An _island_ is a set of entities that are connected via constraints, directly or indirectly. The motion of one dynamic entity in an island will likely have an effect on the motion of all other dynamic entities in the island, thus the constraints in one island have to be solved together, and multiple islands can be solved in parallel.

## Merging and Splitting Islands

An island is a connected component in the entity graph, thus whenever a node or edge is added or removed, the connected components must be recalculated and the islands could be merged or split in the process. The islands where these changes happen are selected for later evaluation.

When an edge is created it could merge islands, thus the `edyn::entity_graph::reach` function is used to find out which islands are reachable from the nodes of the new edges. If more than one island is reachable, they must be merged into one. The bigger on is selected as destination and all nodes and edges from the others are moved into it. The other islands are destroyed.

To determine whether an island has been split, the graph is traversed starting from one of its nodes and if the resulting connected component does not contain all nodes that the island owns, it was split in two or more. The graph is then traversed for all of the island's nodes and all smaller islands are calculated this way. The bigger connected component is kept in the original island and new islands are created for the rest and island residents are reassigned.

## Sleeping

Another function of islands is to allow entities to _sleep_ when they're inactive (not moving, or barely moving). As stated before, an island is a set of entities where the motion of one can immediately affect all others, thus when none of these entities are moving, nothing is going to move, so it's wasteful to do motion integration and constraint resolution for an island in this state. In that case the island is put to sleep by assigning a `edyn::sleeping_tag` to all entities in the island. Entities that have a sleeping tag assigned to them are excluded from the physics calculations.

# The Entity Graph

Islands are modeled as a graph, where the rigid bodies are nodes and the constraints and contact manifolds are edges. The graph is stored in a data structure outside of the ECS, `edyn::entity_graph`, where nodes and edges have a numerical id, i.e. `edyn::entity_graph::index_type`. This is an undirected, non-weighted graph which allows multiple edges between nodes. Node entities are assigned a `edyn::graph_node` and edges are assigned a `edyn::graph_edge` which hold the id of the node or edge in the graph. This allows a conversion from node/edge index to entity and vice-versa.

Individual islands can be found using the concept of _connected components_ from graph theory. Islands are represented by an entity with a `edyn::island` component and all node and edge entities have a `edyn::island_resident` component (or `edyn::multi_island_resident` for non-procedural entities) which holds the entity id of the island where they're located at the moment. As nodes are created, destroyed or modified, islands can split into two or more islands, and they can also merge with other islands.

Nodes are categorized as _connecting_ and _non-connecting_. When traversing the graph to calculate the connected components, a node is visited first and then its neighbors that haven't been visited yet are added to a list of nodes to be visited next. If the node is _non-connecting_, the neighbors aren't added to the list of nodes to be visited. The non-procedural entities have their corresponding graph nodes marked as non-connecting because a procedural entity cannot affect the state of another procedural entity _through_ a non-procedural entity, so during graph traversal, the code doesn't walk _through_ a non-connecting node. For example, if there are multiple dynamic entities laying on a static floor, there shouldn't be a single connected component but instead, there should be one connected component for each set of procedural nodes that are touching each other and the static floor should be present in all of them.

## Procedural Nodes

Nodes that have their state calculated by the physics simulation are characterized as _procedural_ using the `edyn::procedural_tag`. These nodes can only be present in one island, which means that if a connection is created between two procedural nodes that reside in different islands, the islands have to be merged into one. Later, if this connection is destroyed, the island can be again split into two. Dynamic rigid bodies and constraints must have a `edyn::procedural_tag` assigned to them. Non-procedural nodes can be present in multiple islands at the same time, since they are effectively read-only from the island's perspective. These are usually the static and kinematic entities, which are not affected by the physics simulation.

Entities that are part of an island need a way to tell what island they're in. Procedural entities can only be present in one island at any given moment, thus they have an `edyn::island_resident` assigned to them, whereas non-procedural entities can be in multiple islands simultaneously, thus they have a `edyn::multi_island_resident` instead.

# Execution modes

_Edyn_ can be executed in sequential or asynchronous mode, more specifically:

- Sequential: runs the simulation in the calling thread. The call to `edyn::update` does everything that's needed for the update cycle before returning. It will only dispatch jobs to run in a background thread for non-blocking tasks such as loading pages in a `edyn::paged_triangle_mesh`.
- Sequential multi-threaded: identical to the sequential mode except that it will conditionally parallelize parts of the update cycle, mostly using `edyn::parallel_for` in tasks such as narrowphase collision detection and solving constraints per island.
- Asynchronous: offloads as much work as possible to background threads to make the call to `edyn::update` as lightweight as possible. This is the highest performing execution mode since it frees up the main thread which generally has a lot more to do than just physics simulation. Using the engine in this mode requires extra steps due to the asynchronous nature of many operations.

# Multi-threading

Multi-threaded execution is modeled as a series of _map-reduce_ operations. For example, during broadphase collision detection, the goal is to query the AABB tree for each rigid body, which can be done with parallel-for. This is the _map_. The results are inserted into a pre-allocated array and when the parallel-for is done, the entries in this array which are not empty become contact manifolds. This is the _reduce_. Next, narrowphase collision detection is performed, where closest point calculation is performed for each contact manifold. Again, this can be done using parallel-for. The collision results are collected and are merged into the contact manifolds, making the contact constraints ready for the constraint solver.

The constraint solver can be parallelized by splitting up the simulation into independent chunks that can be run in parallel, i.e. simulation islands. The process can be further parallelized by partitioning the connected component of each island, generating smaller subsets that can be solved in parallel in each iteration, and at the end the last partition which connect them all is solved and the next iteration repeats the process. A graph partition algorithm must be employed, such as Kernighan-Lin.

In asynchronous execution mode, a _simulation worker_ runs in a dedicated thread and performs all the physics simulation logic. It uses a message queue to communicate and repeatedly sends the physics simulation state back to the main thread to be merged into the registry. The simulation worker has its own registry which holds the simulation data and to merge data back and forth between the main registry and the simulation registry, an _entity-map_ is used to map entities from one registry to their counterpart in the other. Entities contained in components are also mapped. This allows content to be replicated between registries.

Everything that changes during an update is collected into a set of _registry operations_ which are sent to the other end when the update is done. These operations can be executed to replicate changes that happened in the other registry. This is done both in the worker and main thread. Changes to shared components are observed using EnTT signals. The `edyn::registry_operation_builder` provides an interface to build a `edyn::registry_operation`. The `edyn::registry_operation_observer` subscribes to the EnTT signals and add components that have changed to a builder.

## Message Dispatcher

A message system is used for communication among systems running in different threads. The `edyn::message_dispatcher` provides the means to create a named message queue which belongs to one system. Any other system can post messages in that queue by referencing it by its name. All messages come with a sender identifier which can be used to write a response if needed.

The main thread and simulation worker use the global `edyn::message_dispatcher` to communicate among themselves. The main thread creates a queue for itself and the worker does the same, and then they can post messages to the queue of the system they want to communicate with.

## Job System

_Edyn_ has its own job system it uses for parallelizing tasks and running background jobs. The `edyn::job_dispatcher` manages a set of workers which are each associated with a background thread. When a job is scheduled it pushes it into the queue of the least busy worker.

Job queues can also exist in any other thread. This allows scheduling tasks to run in specific threads which is particularly useful in asynchronous invocations that need to return a response in the thread that initiated the asynchronous task. To schedule a job to run in a specific thread, the `std::thread::id` or the queue index must be passed as the first argument of `edyn::job_dispatcher::async`. It is necessary to allocate a queue for the thread by calling `edyn::job_dispatcher::assure_current_queue` and then also call `edyn::job_dispatcher::once_current_queue` periodically to execute the pending jobs scheduled to run in the current thread.

Jobs are a central part of the multi-threaded aspects of the engine and thus are expected to be small and quick to run, and they should **never** wait or sleep. The goal is to keep the worker queues always moving at a fast pace and avoid hogging the queue thus making any subsequent job wait for too long, or having a situation where one queue is backed up by a couple jobs while others are empty (though, _job stealing_ is a possibility in this case using a _Chase-Lev_ lock free queue). Thus, if a job has to perform too much work, it should split it up and use a technique where the job stores its progress state and reschedules itself and then continues execution in the next run. If a job needs to run a for-loop, it should invoke `edyn::parallel_for_async`, where one of the parameters is a job to be dispatched once the for loop is done, which can be the calling job itself, and then immediately return, allowing the next job in the queue to run. When the job is executed again, it's important to know where it was left at thus it's necessary to store a progress state and continue from there.

A job is comprised of a fixed size data buffer and a function pointer that takes that buffer as its single parameter. The worker simply calls the job's function with the data buffer as a parameter. It is responsibility of the job's function to deserialize the buffer into the expected data format and then execute the actual logic. This is to keep things simple and lightweight and to support lock-free queues in the future. If the job data does not fit into the fixed size buffer, it should allocate it dynamically and write the address of the data into the buffer. In this case, manual memory management is necessary and it's important to remember to deallocate the data after the job is done.

## Entity Mapping

Since each `edyn::island_worker` has its own registry where entities from the main registry are replicated, it is necessary to map `entt::entity` (i.e. entity identifiers) from one registry to another, since entities cannot just be the same in different registries. This is called _entity mapping_ and is done using an `edyn::entity_map`, which allows entities to be converted from _remote_ to _local_ and vice-versa.

Registry operations are always written with the local entities and then have to be applied into another registry to replicate changes. In the context of the recipient, the entities in the operations are considered _remote_ and have to be mapped into _local_ before applying changes, including those that are properties of components.

Of special consideration are components that have `entt::entity`s in them, such as the `edyn::contact_manifold`. These `entt::entity`s must be mapped from _remote_ to _local_ before being imported. Static reflection provided by `entt::meta` is used to iterate over the members of the component and finding which ones contain `entt::entity` values and these are then mapped into local space using the entity map.

If the worker creates a new entity (e.g. when a new contact point is created), it won't yet have an entity mapping for it, since there's no corresponding entity in the main registry yet. It will be added to the current set of registry operations as a created entity and when received on the main thread, a new entity will be instantiated and a mapping will be created. The worker needs to know into which remote entity its local entity was mapped, so that it can make the connection later when the main thread sends an operation containing that entity. The entity mapping is added to the current set of registry operations and sent to the worker later, which when executed, will add the mapping to the worker's entity map.

## Parallelizing constraint solver iterations

The constraint solver iterations are rather expensive and more difficult to parallelize since there's a dependency among some of the constraints. For example, in a chain of rigid bodies connected by a simple joint such as `A-(α)-B-(β)-C`, the constraints `α` and `β` cannot be solved in parallel because both depend on body `B`. However, in a chain such as `A-(α)-B-(β)-C-(γ)-D`, the constraints `α` and `γ` can be solved in parallel because they don't have any rigid body in common, and then constraint `β` can be solved in a subsequent step (this is in one iteration of the solver, where 10 iterations is the default). That means the constraint graph, where each rigid body is a node and each constraint is an edge connecting two rigid bodies, can be split into a number of connected subsets which can be solved in parallel and the remaining constraints that connect bodies in different subsets can be solved afterwards. The picture below illustrates the concept:

![ConstraintGraphPartition](https://xissburg.com/images/ConstraintGraphPartition.svg)

The constraints contained within the subsets A and B (i.e. constraints connecting two nodes that are both in the same subset, blue edges) can be solved as a group in two separate threads (that is one iteration). Later, the constraints in the _seam_ that connect A and B together (i.e. constraints connecting two nodes that lie in different subsets, red edges) can be solved once the jobs that solve A and B are completed. The process is then repeated for each solver iteration.

## Parallel-for

The `edyn::parallel_for` and `edyn::parallel_for_async` functions split a range into sub-ranges and invoke the provided callable for these sub-ranges in different worker threads. It is used internally to parallelize computations such as collision detection between distinct pairs of rigid bodies. Users of the library are also free to use these functions to accelerate their for loops.

The difference between `edyn::parallel_for` and `edyn::parallel_for_async` is that the former blocks the current thread until all the work is done and the latter returns immediately and it takes a _completion job_ as parameter which will be dispatched when the work is done. `edyn::parallel_for` also runs a portion of the for loop in the calling thread.

Each instance of a parallel for job increments an atomic integer with the chunk size and if that's still within valid range, it proceeds to run a for loop for that chunk. It then repeats this process until the whole range is covered. This ensures that, even if one of the jobs is very far behind in a work queue, the for loop continues making progress. Thus it's possible that by the time a job is executed, the loop had already been completed, and in the async case the only thing it does is to decrement the atomic reference counter which when it reaches zero, it deallocates the context object. Also in the async case, when a chunk completes, a _completed_ atomic is incremented and when it reaches the total size of the loop, it dispatches the completion job.

## Parallel-reduce

_TODO_

# Presentation

The simulation is always updated in fixed time steps which means the physics state is not synchronized with the current time. That means presenting the physics state to an observer is inadequate as it will yield choppy results. Also, when running the simulation in asynchronous mode, the physics state is what was sent last by the simulation worker, which is also not synchronized with the current time and should not be expected to be a steady sequence of updates. To make presentation consistent, interpolation must be employed to ensure steady and smooth animation. The `edyn::present_position` and `edyn::present_orientation` components are an interpolated version of the physics transform which provide a stable value for presentation at the current time.

# Raycasting

Raycasting queries can be done with two points of a segment `p0` and `p1`. The main broadphase tree is queried with this segment and the island AABBs are tested against it. Multiple rays can be queried in bulk for better performance.

In sequential execution mode, the `edyn::raycast` function must be called and it returns the result immediately. In asynchronous mode, the `edyn::raycast_async` function must be called and it takes an `entt::delegate` which will be invoked later with the result.

In asynchronous mode, a message is sent to the simulation worker which accumulates rays to be queried into a `edyn::raycast_service`. The raycast starts with a query to the broadphase tree for each ray, which can be run in parallel for multiple rays. Potential entities are collected for all rays and then shape raycasts are performed for each, which can be run in parallel for each shape. The result is sent back to the main thread later.

When doing raycasts in a pre/post-step-callback, always call `edyn::raycast`. It's safe to do so in asynchronous execution mode as well. It's just important to remember that the function is being called in a background thread using the simulation worker registry.

# Networking

To allow different instances of an application to interact with the same simulation, it is necessary to synchronize their components across a network. Edyn provides an implementation of networked physics which essentially requires users to receive edyn packets and inject them into the library and to send packets that are generated by the library.

The network model follows a client-server architecture with an authoritative server which holds the valid state of the simulation and the clients display an approximation of what the simulation currently looks like. Relevant entities and components are sent over the network regularly and their values are used to make the local simulation match the remote simulation. The client runs physics simulation normally and overwrites the local state with values received from the server.

In the server side, each client has an AABB of interest, which determines which entities the server will send to that client. As entities come in and out of this AABB, the server will notify the client. When an entity comes in, the server will send enough information for the client to instantiate an equivalent local copy. When it goes out, it just sends the entity id. Upon receiving these packets, the client will create or destroy corresponding entities locally.

In the client side, all entities owned by the client will have their state included in the packet sent to the server regularly. This state includes user inputs (which are going to be external components registered by the user of this library) and procedural simulation state such as transforms and velocities. The latter are sent because the server will apply procedural state from the client in certain situations.

Rigid body state is never sent in isolation, the entire island where a relevant body resides must be shared because it functions as one unit of simulation, since the state of all rigid bodies in an island affects each other.

When a local entity is created to correspond with a remote entity, an entity mapping is created, i.e. a pair of associative containers store the remote-to-local and local-to-remote entity mappings, which is used to find out which local entity a remote packet is referring to. All packets are sent with local entities which are interpreted as remote entities in the other end, thus they have to be mapped into the local registry space using the entity map. When new entity mappings are created, they have to be sent back to the source, so it knows into which entity their entities have been mapped, so that when it receives a packet containing that remote entity, it can be mapped to local.

Components that contain entities must have them mapped to local as well when importing remote state. These components must be properly registered with `entt::meta` so the members that hold `entt::entity` values can be transformed into local space.

Entity-component data is shared via registry snapshots which are packets that store their information efficiently with the goal of minimizing bandwitdh usage. These packets are supposed to be sent through an unreliable UDP channel, since packet loss is acceptable. They are generated by _Edyn_ regularly, a couple of times per second, and include components that have changed and need to be updated in the other end. All components that have _recently changed_ are included in the snapshot. By sending the data repeatedly over the next few packets, it decreases the probability that the data change will not reach the other end due to packet loss.

## Assets

In a real application, rigid bodies are usually part of a group, e.g. a rag doll or a multi-body vehicle. The group most likely represents something greater than just a bunch of rigid bodies and constraints. It might be associated with a graphical representation (e.g. mesh, textures, skeleton, animations...), sound effects and logic. That means rigid bodies that are part of a group or have additional elements associated with them, must be handled as a unit. This is called an _asset_ which is represented by a globally unique id.

Each asset can be instantiated in its own way, by creating entities and assigning whatever components to them. Relevant entities must have an internal id which is unique within the asset and identical in all machines. This id will be used to map internal ids to entity ids of rigid bodies and constraints that already exist in the other end when replicating assets in remote clients.

All entities in an asset are expected to be a single connected component in the entity graph.

When one body that's part of a group enters the AABB of interest of a client, the client is supposed to instantiate not only the rigid bodies in that group, but also all the associated assets. Thus when bodies are assigned to an asset reference, instead of notifying the client of the bodies that just entered their AABB of interest, the server notifies the client with the asset reference information. With the asset id, the client can obtain the asset data locally, from a cache, or download it from a data server. Then the asset can be instantiated and, via the common ids inside the asset, the local entities can be linked to the remote entities by adding these mappings to the client's entity map. With the mappings in place, the code that imports the registry snapshots will apply the remote state onto the local simulation for the new entities.

Instantiating an asset will always yield the same results, at any time, anywhere. This frees the game server from having to send large amounts of information to clients to replicate a set of entities, particularly things such as polyhedron shapes.

It is important to assign the initial state of an asset right after instantiation. For that reason, an _asset sync_ is performed when the client has the asset ready to be instantiated. This will ask the server to send back the values of relevant components which represent the entities' state. Which components have to be synchronized is part of the asset definition. All the other components are assumed to have the exact same values since they were instantiated from the same asset. Only after receiving an _asset sync response_ can the client instantiate the asset and immediately apply the received registry snapshot which will override the asset state with the latest values. This prevents the asset from being instantiated in an inappropriate state, such as wrong location.

When all bodies of an asset leaves the AABB of interest, the client is notified that the asset left. Since the client knows all entities that belong to that asset, it can destroy all of them with only this bit of information. The process above will be repeated if they come back into the AABB of interest.

## Playout delay

Due to network jitter, the state received from clients cannot be applied immediately on the server, or else the timing of user actions will not match thus resulting in different behavior, which can lead to significant disparities over time. For that reason, a _playout delay buffer_ is employed, which simply stores packets received from the client for later execution. Each packet comes with a remote timestamp which has to be transformed into local time using a time delta calculated by the [clock synchronization](#clock-synchronization) process. This means the values in the packet represent the state of the components at that point in time. To apply the state of all packets with the same relative time, they're queued up sorted by timestamp, and in every server update, all packets that have a timestamp that's before the current time minus the playout delay are processed. That means, as long as the playout delay is greater than the latency plus some change, all packets should be processed with the same relative timing.

This means the server simulation runs _in the past_ with respect to the client, as the client state is applied with a delay.

The playout delay of a client must be greater than the biggest latency among all clients present in its AABB of interest. This ensures the time of all clients will be in accordance.

## Temporary ownership

Despite being authoritative, the server allows clients to set the procedural state (i.e. transforms and velocities) of all entities in the islands where entities owned by the client reside, unless there is one or more entities owned by another client also present in that island. This is called _temporary ownership_ of entities not officially owned by any client.

For example, if a vehicle owned by client A is touching a rigid body not owned by any client (e.g. a prop), the client will send the procedural state of that body to the server in addition to the state of the vehicle, and the server will accept and apply the state. Now, if another vehicle owned by client B were to be touching the same rigid body, the server will stop accepting the procedural state of all entities in that island for both clients. Only inputs will be accepted and applied. This allows clients to experience a better simulation most of the time since it is common for a client to be alone in an island and when that's the case, there's no interference coming from the server since the server will not send the procedural state of these entities back to the client, which also means no extrapolation will be done.

The incoming procedural state must be verified by the server, since it could've been tampered with by the client. Positional changes must be within the limits of what would make sense given the logic of the simulation. Velocities must also make sense.

Temporary ownership is optional and can be disabled if security must not be traded with client-side simulation quality.

## Inputs

The user can register external components and additionally tag them as _inputs_, which are components that will be handled differently by the server. The state of input components is always accepted by the server, as long as the client setting it owns the target entity. The state of these components must be then broadcasted to other clients when they change, so they know what the other client is doing. Input components must derive from `edyn::network_input`.

Not all inputs are idempotent. A steady steering input of a vehicle can be applied multiple times without changing the outcome. Now a _shift up_ input, which changes gear in a vehicle, is an input that will change the outcome if applied multiple times. Such inputs must be applied individually at the right time, as they're a form of _action_, not a state. Actions are registered alongside external networked components. Actions are not components though. Instead, they are accumulated in `edyn::action_list` component in every update, and consumed at the end (i.e. the registry clears their pools).

Actions are sent over the network packaged in an `edyn::action_history`, which is a timestamped list of action packs. This history is kept at a maximum size in the client, which always removes old actions, and sent with every following registry snapshot, so that in the event of packet loss, it's likely that the following packet will contain the actions that happened in the lost packet.

Actions are handled separately in the server. When a registry snapshot containing actions arrive, all actions are taken from it and merged into the entity's `edyn::action_history` for later execution. Only actions with a newer timestamp are added to the history and in every update, actions with a timestamp that's before the current time minus the playout delay are executed.

All actions in the current update are added to the input history of the client, so they can be replayed during extrapolation.

When received in the other end over the network, the timestamps of all actions must be converted into local using the time delta from the clock synchronization process.

## Packet validation

Packets might contain invalid data, especially when it comes to packets received from clients, which may contain literally anything, since hackers will attempt to tamper with them. For that reason, packet validation is necessary. A validation function exists for every component type which will check if the data makes sense. They will for example, check if floating point values are valid (e.g. not a `NaN` or `Inf`) and within range, if entities are valid, etc. If validation of any component fails, the whole packet is rejected. Values can also be transformed in the validation function, such as clamping a value within the valid range.

## Clock synchronization

To tell which point in time the state contained in a packet refers to, it is not enough to simply subtract the latency from the local time, because packets are not delivered at the exact same rate they're sent due to network jitter, e.g. one end might send 1 packet every 10ms and the other end might receive none of these packets in a span of over 100ms and then suddenly receive 10 packets at once. Instead, it is required that the packet contains the timestamp of the source at the time the packet was sent and the destination needs to have a time delta which when added to the packet timestamp results in the corresponding local time. The difference between the local time and the calculated packet local timestamp must not be smaller than the latency (within a threshold). The age of the packet can now be calculated and then its data can be inserted in the right place in the playout delay buffer in the server and in the client it can be extrapolated for the right amount of time.

To calculate the time delta, a process involving a sequence of time requests is made, where `edyn::packet::time_req` and `edyn::packet::time_res` are exchanged as unreliable packets. For each `time_req` received, a `time_res` must be sent containing the local timestamp. A `time_req` contains a random int id which must be sent back in the `time_res`. This ensures responses are matched to a request and prevents handling of responses which were not requested. These packets must be sent unreliably via UDP, so the id allows correct handling of lost packets since when a request is sent again because the response is taking too long to come, the stored id is updated, thus if the response arrives too late, it will be ignored.

First, a `time_req` is sent from A to B, as an unreliable packet. The time it was sent is stored as `tr0`. If no response is received within the average RTT plus some margin, it is sent again. When B gets the time request, it replies with a `time_res` containing its local timestamp. Upon receiving the time response, A takes note of B's timestamp `tB0` and sends a time request again at time `tr1`, to which B should respond with another time response, just like before. When A gets the second time response it stores the current time in `tr2` and the second timestamp in `tB1`. If A fails to get a timely response for the second `time_res`, the process has to start again. Now A has all the data that's needed to do the first time delta calculation.

Considering the latency as the average of the request-response delay, i.e. `lat = ((tr2 - tr1) + (tr1 - tr0)) / 2`, the delta between B's timestamps `dtB = tB1 - tB0` should be similar to the latency, otherwise this data point is dropped. The A-to-B time delta can be calculated as `dtA2B = tA - (tB1 + lat)`, where `tA` is the current local time in A. To convert a packet timestamp to local time, just add `dtA2B` to it.

These steps should be repeated a couple of times (e.g. 5 times) and the final value of `dtA2B` should be the average of the intermediate ones.

This process should be repeated once in a while to account for clock drift.

This is done both in the client and server, independently.

## Client-side extrapolation

When receiving data from the server, the received state will be in the past due to network latency and playout delay, thus the client must _extrapolate_ it before merging it onto the local simulation. The remote timestamp of the packet is converted into a local timestamp and the entities present in the packet have all their components assembled together and given to an extrapolation worker which runs a separate simulation in a dedicated thread starting at the calculated local time and going until the current time and then it gives back the final state as a result which can be applied onto the local simulation.

Besides all entities present in the transient snapshot, the edges connecting them in the entity graph are also included in the extrapolation. That's necessary or else constraints between these entities would be ignored since constraints are usually not present in the transient snapshot.

The extrapolated result is likely to not match the local simulation. To avoid having objects move suddenly when the extrapolation result is applied, a _discontinuity_ factor is calculated and it decays over time. The discontinuity holds a position and orientation offset, which are calculated as the difference between the current state and the extrapolated state. This offset is then added to the present position and orientation to generate a smooth decay towards the extrapolated state. Offsets are accumulated in the same discontinuity as new extrapolation results are applied.

An input history is needed during extrapolation so that user inputs (especially the local user's input) can be replayed during extrapolation. A snapshot of input components of entities owned by the client is taken in every update and added to the list of inputs. Input components from other clients are also added to the list as they arrive. They're assigned a timestamp which will allow the extrapolation worker to replay them with the same timing. Since the extrapolation happens while the simulation is still running, just giving the current list of inputs to the extrapolation worker might not be enough, as it would miss new inputs applied after the job starts, which may cause significant differences in the result. Thus, a shared thread-safe input history is used.

The extrapolation worker runs one extrapolation at a time, meaning that extrapolation requests that are sent to it while there's one already taking place might be dropped. It will only process the last request received after it's done with the current.

Starting at the estimated transient snapshot timestamp, an attempt is made to extrapolate until the current time, which is a moving target. It is possible that the time it takes to run one simulation step is greater than the fixed delta time, which means that the extrapolation would never finish, since after each step is completed, the current time has moved further away than the simulation delta time. Thus, an execution time limit is set for each extrapolation and it should be terminated early in case that duration is reached. The extrapolation result will be applied either way.

Users that don't interact with the simulation do not need extrapolation (i.e. spectators).

## Extrapolation level-of-detail

The appearance of the simulation of entities that are further away from the user might not need to have the same level of detail as the entities that are nearby. Thus, it can be beneficial to not extrapolate the state of entities that are far away. Entities and their islands are considered for extrapolation based on their overall distance to the center of the AABB of interest, a.k.a. point of interest, around of which three zones exist:

1. Extrapolation zone: islands in this radius are extrapolated. The amount of extrapolation depends on how close they are to the point of interest. Near the center the extrapolation is performed until the current time. As they move further away towards the perimeter, the length of the extrapolation decays to be a percentage close to the current time, until it gets to zero at the perimeter where no extrapolation is done and they move into the next zone.

2. Snapping zone: islands in this zone have the state of their entities snapped to the values in the transient snapshot. Discontinuities are calculated to smooth out visual disturbances.

3. No simulation zone: entities in this zone have simulation disabled (i.e. `edyn::disabled_tag` is assigned to them). The state from the transient snapshot is applied directly and the velocity is used to do a basic linear extrapolation of the transforms over time. Discontinuities are again used to smooth out the snapping effect.

## Hierarchical interpolation

Transforms are interpolated linearly when discontinuities are introduced and when entities fall into the _no simulation zone_.If the extent of the interpolation is significant, entities that are linked together will likely have their relative transforms diverge in noticeable ways, e.g. a vehicle made of a chassis and 4 wheels which is spinning in place will have its wheels move away from the chassis.

One way to improve the quality of interpolation of linked entities, is to perform it in the space of a parent entity. In the case of a vehicle, the transform of the wheels should be converted to chassis-space for the initial and final transforms in world space, then the chassis-space transforms should be interpolated and then converted back into world space. In the case of a vehicle spinning in place, the chassis-space transform of each wheel is the same in the initial and final timestamps, thus no divergence will be observed.

Entities can have a `edyn::interpolation_parent` component which refers to the parent entity and then that will be considered when performing interpolations.

## Snapshot packet serialization

The snapshot packets contain an array of entities and an array of component pools. The pools are type-erased and rely on a virtual function that will do any type-specific operation such as serialization and importing the data into a registry. The pool has an array of entity indices and an array of components if the component type is not empty (according to `std::is_empty_v`). The indices are with respect to the array of entities that's included in the beginning of the packet. The array of components has a 1-to-1 relationship with the array of indices, where the i-th component is assigned to the entity in the entity array located at the index stored in the i-th element of the array of entity indices. That also means the array of entity indices and the array of components have the same size.

This ensures entities are specified only once in the array of entities and the entity indices can be of much smaller data type, requiring fewer bits. Delta encoding is used to decrease the size of the packet even further, where only the first value is serialized fully, followed by a delta that can be added to the previous to get the next value. Only the entity identification bits are used (the number of bits being the value of `entt::entt_traits<entt::entity>::entity_shift`, which is 20 bits for 32-bit entity type and 32 bits for 64-bit entity type) and the entities are sorted to decrease the difference between subsequent values.

Entity index arrays are also sorted, together with the components array to keep the 1-to-1 relationship.

To delta-encode and array of integers, the biggest difference between subsequent values is calculated and that determines the number of bits required to store the delta. The first value is the number of elements in the array. Then comes 4 bits containing the size in bits for delta values, i.e. a maximum of 16 bits for delta values. Then comes the first value, followed by a sequence of delta values.

Each component has it's serialization function with specific optimizations, such as delta encoding and quantization.

Position can be limited within a range such as `[-2000, 2000]` in the X and Z axes and `[-1000, 1000]` in the Y axis and the precision can be `0.001`, which means 4M possible values in the X and Z axes and 2M values in the Y axes. That would require 22 bits for the X and Z axes and 20 bits for the Y axis, totaling 64 bits, which saves 32 bits over using 3 floats, which would total 96 bits.

The user must provide serialization functions for external networked components, which can use the facilities of the `edyn::bitpack_archive` to optimize the data size.

# Clusters

Multiple server instances can run in different machines in the same local area network (LAN) and balance load. The principles of distributing work among all machine are similar to that of multi-threading.

# Distributed simulation

Several server clusters can exist in different geographical locations to allow for better network performance for local clients. Clusters communicate among themselves to synchronize the redundant simulation of the persistent world.
