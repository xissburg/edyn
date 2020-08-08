# Edyn Design Document

This document describes the general engine architecture. It is a bit of a brainstorming document and does not reflect the current state of the library. The ideas presented here are planned to be implemented in the near future.

# Introduction

_Edyn_ (pronounced like "eh-dyin'") stands for _Entity Dynamics_ and it is a real-time physics engine focused on multi-threaded, networked and distributed simulation of massive dynamic worlds. It is organized as an _entity-component system_ (ECS) using the amazing [EnTT](https://github.com/skypjack/entt) library.

**Warning**: This library is still in an early stage of development and is not yet ready for most use cases. The information provided below is more of a design document and does not reflect the current state of the library.

# The ECS approach

Typical physics engines will offer explicit means to create objects such as rigid bodies, whereas in _Edyn_ object creation is implicit due to the entity-component design. A rigid body is created from the bottom up, by associating its parts to a single entity, such as:

```cpp
entt::registry registry;
auto entity = registry.create();
registry.assign<edyn::dynamic_tag>(entity);
registry.assign<edyn::position>(entity, 0, 3, 0);
registry.assign<edyn::orientation>(entity, edyn::quaternion_axis_angle({0, 1, 0}, edyn::to_radians(30)));
registry.assign<edyn::linvel>(entity, edyn::vector3_zero);
registry.assign<edyn::angvel>(entity, 0, 0.314, 0);
auto mass = edyn::scalar{50};
registry.assign<edyn::mass>(entity, mass);
auto &shape = registry.assign<edyn::shape>(entity, edyn::box_shape{0.5, 0.2, 0.4});
registry.assign<edyn::inertia>(entity, shape.inertia(mass));
registry.assign<edyn::material>(entity, 0.2, 0.9); // Restitution and friction.
registry.assign<edyn::linacc>(entity, edyn::gravity_earth);
```

There's no explicit mention of a rigid body in the code, but during the physics update all entities that have a combination of the components assigned above will be treated as a rigid body and their state will be update over time as expected. The update may be carried as follows:

```cpp
// Apply gravity acceleration, increasing linear velocity
auto view = registry.view<edyn::linvel, const edyn::linacc, const edyn::dynamic_tag>();
view.each([&dt] (auto entity, edyn::linvel &vel, const edyn::linacc &acc, [[maybe_unused]] auto) {
  vel += acc * dt;
});
// ...
// Move entity with its linear velocity
auto view = registry.view<edyn::position, const edyn::linvel, const edyn::dynamic_tag>();
view.each([&dt] (auto entity, edyn::position &pos, const edyn::linvel &vel, [[maybe_unused]] auto) {
  pos += vel * dt;
});
// ...
// Rotate entity with its angular velocity
auto view = registry.view<edyn::orientation, const edyn::angvel, const edyn::dynamic_tag>();
view.each([&dt] (auto entity, edyn::orientation &orn, const edyn::angvel &vel, [[maybe_unused]] auto) {
  orn = edyn::integrate(orn, vel, dt);
});
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
def.shape_opt = {edyn::box_shape{0.5, 0.2, 0.4}}; // Shape is optional.
def.update_inertia();
def.restitution = 0.2;
def.friction = 0.9;
def.gravity = edyn::gravity_earth;
auto entity = edyn::make_rigidbody(registry, def);
```

It is not necessary to assign a shape to a rigid body. That enables the simulation to contain implicit rigid bodies (not to be confused with the meaning of implicit from above) which are not visually present in the simulation and don't participate in collision detection, but instead are connected to other bodies via constraints and are used to generate forces that affect the primary entities that users interact with. As an example, this can be useful to simulate drivetrain components in a vehicle.

# Foundation

The library can be built with single- or double-precision floating point. `edyn::scalar` is simply a `using` declaration equals to `float` or `double` which is set according to the `EDYN_DOUBLE_PRECISION` compilation option. _build_settings.hpp_ is generated during build from _cmake/build_settings.h.in_ so that invocations are linked to the correct definition.

`edyn::vector3` is the vector type for positions and directions.

`edyn::quaternion` is the quaternion type for orientations.

`edyn::matrix3x3` is the 3x3 matrix also used for orientations and orthonormal bases.

_SIMD_ implementations are planned.

# Relations

If one entity could have its motion immediately influenced by another entity, a `edyn::relation` must be setup in a separate entity, which stores the entity ids of the related entities. This will ensure both will be in the same _island_ (more on that later) and the solver will apply impulses between them.

# Constraints

Constraints are components of the type `edyn::constraint` which stores the actual constraint in `std::variant`. Different constraint types have different data and logic, thus the `std::variant`, which by default contains all the fundamental constraint implementations provided by the library. To add custom constraints to the library it is necessary to fork the project and modify the code to insert these into the variant.

A `edyn::constraint` depends on a `edyn::relation` which must be assigned to the same entity. The entities that will be affected by the `edyn::constraint` will be obtained from the corresponding `edyn::relation`.

It's important to note that the size of a `std::variant` is the size of the largest element in its template parameter pack, thus in the situation where one constraint has much more data than all others, it is worth considering storing much of this data somewhere else and keep a pointer to it (e.g. using a `std::unique_ptr`) to avoid wasting too much space.

Constraints are derived from `edyn::constraint_base` and can implement some of the functions that are called in different solver stages to set up and modify its constraint rows.

A Sequential Impulse constraint solver is used.

# Collision detection and response

Many physics engines give special treatment to _contact constraints_ (aka _non-penetration constraints_). In _Edyn_ they are no different of other types of constraints. The `edyn::contact_constraint` is simply part of the `std::variant` in the `edyn::constraint` component.

During broad-phase in  the AABB of all entities is updated and intersections are found. For any new intersection, an entity is created and a `edyn::relation` and a `edyn::contact_manifold` components are assigned to it. For any AABB intersection that ceased to exist, the entity is destroyed thus also destroying all components associated with it.

In narrow-phase, closest point calculation is performed for all entities in a `edyn::relation` that also has a `edyn::contact_manifold`. For each new contact point, an entity is created with the same relation and a `edyn::contact_constraint`. It's important to note that there can be more than one relation referring to the same pair of entities. New contact points that are near existing points get merged together, and contact points that are separating (in either tangential or normal directions) are destroyed.

Entities that don't have a `edyn::shape` also don't have a `edyn::AABB` associated with them and thus are ignored in broad-phase.

After a broad-phase collision starts the AABBs are inflated slightly and then after they separate they go back to normal to avoid a situation where they'd join and separate repeatedly. This is sometimes called _hysteresis_.

To enable collision response for an entity, a `edyn::material` component must be assigned which basically contains the _restitution_ and _friction coefficient_. Otherwise, the entity behaves as a _sensor_, i.e. collision detection is performed but no impulses are applied thus the entity is allowed to penetrate others.

Collision events can be observed by listening to `on_construct<entt::contact_point>` or `on_destroy<entt::contact_point>` in the `entt::registry`. It's important to note that when the `on_construct<entt::contact_point>` event is triggered, the contact was just created and thus its associated constraint has not yet been solved thus it has a zero impulse. If the impulse is required (e.g. to play sounds based on the intensity of the collision), it is possible to observe step signals in `edyn::world::step_sink` and look for `entt::contact_point`s that have `lifetime == 0`, which is only true for new contacts, and at this point they will have the collision impulse assigned.

# Shapes

Similarly to constraints, `edyn::shape` also holds a `std::variant` with all shape types in its parameter pack. It's necessary to fork the project and modify the code to add custom shapes. It is also necessary to provide a `edyn::collide` function for every permutation of the custom shape with all existing shapes.

## Paged triangle mesh shape

For the shape of the world's terrain, a triangle mesh shape is usually the best choice. For larger worlds, it is interesting to split up this terrain in smaller chunks and load them in and out of the world as needed. The `edyn::paged_triangle_mesh` offers a deferred loading mechanism that will load chunks of a concave triangle mesh as dynamic objects enter their bounding boxes. It must be initialized with a set of entities with a `edyn::AABB` assigned and an `Archive` (which is templated).

As dynamic entities move into the `edyn::AABB`s (which should be immutable), it will ask the archive to load the triangle mesh for that region if the entity does not have a `edyn::triangle_mesh` assigned. The `Archive` object must implement `void operator()(entt::entity)` which will load the required triangle mesh (usually asynchronously) and then will assign a `edyn::triangle_mesh` to the given entity when done. Since it might take time to load the mesh from file and deserialize it, the AABBs should be inflated to prevent collisions from being missed.

When there are no dynamic entities in the AABB of the chunk, it becomes a candidate for unloading.

Edge adjacency and Voronoi regions are used to prevent internal edge collisions.

# Multi-threading

## Job System

_Edyn_ has its own job system it uses for parallelizing tasks and running background jobs. The `edyn::job_dispatcher` manages a set of workers which are each associated with a background thread. When a job is scheduled it pushes it into the queue of the least busy worker.

External workers can also exist in non-worker threads. This allows scheduling tasks to run in specific threads which is particularly useful in asynchronous invocations that need to return a response in the thread that initiated the asynchronous task. To schedule a job to run in a specific thread, the `std::thread::id` must be passed as the first argument of `edyn::job_dispatcher::async`. It is necessary to allocate a worker for the thread by calling `edyn::job_dispatcher::assure_current_worker` and then also call `edyn::job_dispatcher::once_current_worker` periodically to execute the pending jobs scheduled to run in the current thread.

Jobs are a central part of the multi-threaded aspects of the engine and thus are expected to be small and quick to run, and they should **never** wait or sleep. The goal is to keep the worker queues always moving at a fast pace and avoid hogging the queue thus making any subsequent job wait for too long, or having a situation where one queue is backed up by a couple jobs while others are empty (_job stealing_ is a possibility in this case but it introduces additional complexity which makes it unfeasible to use a lockfree queue). Thus, if a job has to perform too much work, it should split it up and use a technique where the job stores its progress state and reschedules itself and then continues execution in the next run. If a job needs to run a for loop, it should invoke `edyn::parallel_for_async` and set itself to be rescheduled to run when the returned `edyn::atomic_counter` reaches zero and then immediately return, allowing the next job in the queue to run. When the `edyn::atomic_counter` reaches zero, the same job instance will be scheduled and when it runs again, its important to know where it was left at thus it's necessary to store a progress state and continue from where the work was left at.

A job is comprised of a fixed size data buffer and a function pointer that takes that buffer as its single parameter. The worker simply calls the job's function with the data buffer as a parameter. It is responsibility of the job's function to deserialize the buffer into the expected data format and then execute the actual logic. This is to keep things simple and lightweight and to support lockfree queues in the future.

Using `edyn::timed_job_dispatcher` it is possible to schedule a job to run at a specific timestamp. The `edyn::timed_job_dispatcher` keeps a queue of waiting jobs sorted by desired execution time and it uses a `std::condition_variable` to block execution until the next timed job is ready invoking `std::condition_variable::wait_for` and then schedules it using `edyn::job_dispatcher`. To create a repeating job that is executed every _t_ seconds, it's necessary to have the job reschedule itself to run again at a later time once it finishes processing. This technique is used for running periodic tasks such as the physics simulation updates, which are executed in isolated simulation islands.

## Simulation Islands

Dynamic entities that cannot immediately affect the motion of others can be simulated in isolation. More precisely, two dynamic entities _A_ and _B_ which are not connected via constraints are not capable of immediately affecting the motion of each other. That means, the motion of _A_ and _B_ is independent and thus could be performed in two separate threads.

An _island_ is a set of dynamic entities that are connected via constraints, directly or indirectly. The motion of one dynamic entity in an island will likely have an effect on the motion of all other dynamic entities in the island, thus the constraints in one island have to be solved together. An island is analogous to a _connected component_ in a graph where the rigid bodies are the nodes and the constraints are the edges. Creating or destroying rigid bodies or constraints modify the topology of this graph and require islands to be split or merged.

Static and kinematic entities are not affected by the motion of dynamic entities, thus they can be present simultaneously in every island. Having static and kinematic entities present in multiple islands, thus possibly multiple-threads, means that their data (i.e. their components) will have to be duplicated. In general this is not much data so duplication shouldn't be a concern. One case where it can be desirable to not duplicate data due to its size, is where the static entity is linked to a triangle mesh shape containing thousands of triangles. In this case, the `edyn::triangle_mesh` keeps a `std::shared_ptr` to its triangle data so when it's duplicated, the bulk of its data is reused. This data is going to be accessed from multiple threads thus it must be immutable.

Using the job system, each island can be scheduled to run the simulation in the background in separate threads taking advantage of the job system's load balancing. The `edyn::island_job` has its own registry containing the entities of an island. In its function, it performs the number of steps necessary to bring the simulation to the current time using a `edyn::solver`. After each update, it serializes a snapshot of its registry into a shared and thread-safe buffer, using `edyn::buffer_sync`.

The `edyn::world` in the main physics thread manages the main registry which contains all entities in the simulation. It acts as a coordinator of `edyn::island_job`s, creating new jobs, and merging and splitting existing ones. For every island created, it spawns a new `edyn::island_job`, which receives as initial input a snapshot of the entities in that island and then instantiates them into its local registry when it first runs. It has to create new entities in its private registry and assign the same components to them and create a mapping from the local registry to the main registry. After every step it serializes its registry into the shared `edyn::registry_sync`, mapping from local to main entity ids, since the main registry only accepts entities with their main ids, similar to how an authoritative server (main registry) and a client (island registry) would operate.

If the coordinator wants to create a new entity in that island, it just needs to create it locally first and then ask the island to replicate it by sending a message with the serialized entity and components. The island job will then deserialize it, instantiate it and create an entity id mapping.

If the island creates a new entity (e.g. when a new contact point is created), it won't yet have an entity id mapping for it, so it will put it in a separate buffer in the `edyn::buffer_sync` which is reserved for not yet mapped entities. The coordinator will then grab the entities from this buffer and handle them differently: it will create a new entity in the main registry, assign the same components, and notify the island job what the main entity id for that entity actually is. Upon receiving this message, the island job will create an entity id mapping from local to main and thus will stop sending that entity in the separate buffer and will start to include it in the main serialized buffer since it now knows what the real entity id is.

The island job has a message queue (single producer, single consumer) where it receives commands from the coordinator.

When a physics component is modified, the changes have to be propagated to its associated `edyn::island_job`. For that reason, components must not be modified directly in the main registry, instead the `edyn::world::edit_entity` function must be called with a lambda that performs the desired changes. The lambda will be added as a message in the island job message queue. When the registries are synchronized the change will be visible. That means changes to the physics components are always asynchronous.

Running the simulation in parallel means that the state of entities in the main registry will not be at the same point in time, thus each entity has a `edyn::time_info` component which tells at which point in time the entity was in that state. Then for presentation the state must be interpolated/extrapolated according to the time in that component. That is done internally using `edyn::present_position` and `edyn::present_orientation` which are updated in every `edyn::world::update`.

Splitting the simulation into islands is not enough to maximize resource utilization in certain cases, such as a big pyramid of boxes, which is a single island and would be run in a single island job, thus turning it into a single-threaded solution. It is advantageous to further split the island step into substeps which can run in parallel, one of which is collision detection, which can be fully parallelized using a `edyn::parallel_for_async` and then set itself to be rescheduled once the `edyn::atomic_counter` hits zero. The constraint solver iterations are also rather expensive in this case though more difficult to parallelize since there's a dependency among _some_ of them. For example, in a chain of rigid bodies connected by a simple joint such as `A-(α)-B-(β)-C`, the constraints `α` and `β` cannot be solved in parallel because both depend on body `B`. However, in a chain such as `A-(α)-B-(β)-C-(γ)-D`, the constraints `α` and `γ` can be solved in parallel because they don't have any rigid body in common, and then constraint `β` can be solved in a subsequent step (this is one iteration of the solver). The separation of independent constraints into groups that can be solved in parallel is an _edge coloring_ of the graph. Given a valid edge coloring of the constraint graph, each set of constraints of the same color can be solved in parallel, one after the other. This is possible because no pair of constraints of the same color touches the same rigid bodies. Thus, for each iteration of the solver, it would, for each color, dispatch a set of solver iteration jobs for the constraints of that color using `edyn::parallel_for_async` and wait for them to finish using the returned `edyn::atomic_counter` to reschedule the island job and then continue by dispatching the next color and so on, and then after all colors are done, move over to the next iteration and repeat.

To synchronize registries among different threads, `edyn::buffer_sync` uses roughly the technique described [here](https://codereview.stackexchange.com/questions/163810/lock-free-zero-copy-triple-buffer), that is, a lockfree triple buffer. It simply holds a data buffer where the registry is serialized into.

Broad-phase collision detection is done using a dynamic bounding volume tree (dbvt) using the entities' AABBs inside `edyn::broadphase`. All it does is find intersecting AABBs and then create/destroy entities with a `edyn::contact_constraint` component thus connecting rigid bodies together.

The _step number_ is an unsigned integer which starts at zero and is incremented in every step of the simulation in each island. It corresponds to what point in time the simulation is at. In the single-threaded case all entities will have their state correspond to the same step number at all times. In the multi-threaded scenario entities could be at a different point in time, thus a `edyn::timestamp` component is used to store the step and global time of the entity's state so that when it's synchronized with the main registry, it's possible to correctly perform interpolation to display the entity's transform smoothly on screen, that is, during the update of `edyn::present_position` and `edyn::present_orientation`.

When merging the registries of `edyn::island_job`s onto the main registry their step numbers might differ slightly. This might lead to broadphase pairs not initiating at the exact right time but it does not look like a problem in principle. Slightly inflating AABBs could help mitigate the issue.

Another function of islands is to allow entities to _sleep_ when they're inactive (not moving, or barely moving). As stated before, an island is a set of entities where the motion of one can immediately affect all others, thus when none of these entities are moving, nothing is going to move, so it's wasteful to do motion integration and constraint resolution for an island in this state. In that case the island is put to sleep by assigning a `edyn::sleeping_tag` to all entities in the island. Removing it will wake the entities up.

Changes that happen in the main registry must be propagated to the islands. Those could be changes introduced by users of the library, such as applying a force to an entity or changing a constraint parameter. To perform changes to components in the multi-threaded setup, it's necessary to _replace_ components instead of modifying them (using `entt::registry::replace`) so that the coordinator can delegate these changes to the right island in its `on_replace` delegate {or, maybe, there could be a manual _notify_ kind of function that forces the coordinator to push that entity/component to the island}. In the single-threaded setup, components can be modified directly.

# Networking

The networking model follows a client-server architecture where the server is authoritative but gives the client freedom to directly set the state of the entities it owns in some situations. The goal is to synchronize the simulation on both ends, accounting for network latency and jitter.

The server is the source of truth, containing the actual physics simulation encompassing all entities and components including extra external components used by systems provided by the user of this library. The server has to frequently broadcast to all clients a snapshot of the dynamic components which change very often such as position and velocity. Steady components, which change casually, have to broadcast only when updated.

The server also receives data from the clients, which can be commands to be applied to the entities owned by that client and also component state to be set onto the entities in the server, which the server will have to decide whether to apply them or not.

The client will send commands generated by user inputs and also the dynamic state of the entities it controls/owns.

When receiving data from the server, the received state will be in the past due to network latency, thus the client must _extrapolate_ it before merging it onto the local simulation.

All data shared between client and server is generated using registry snapshots from _EnTT_. Thus, all that's being sent are entities and components which are put into a snapshot and serialized for network use.

The issues above will be discussed in greater detail in the following subsections.

## Server receives data from client

The server expects users to send the dynamic state of the entities the client owns frequently. This state will be applied to the server registry at the right time if the client has permission do so at the moment.

The server simulation runs in the past with respect to the client. This allows the state received from the client to be applied at the right time in the server despite network jitter. More precisely, the state received from the client is accumulated in a _jitter buffer_ (aka _playout delay buffer_) and is applied onto the server registry when the time comes. To match the timing, a step number_is used, instead of a timestamp. This requires the simulation to run with fixed timesteps which must be equal in both ends. This helps maintaining a decent degree of determinism. Ideally, the client states are applied on the server side exactly at the same rate they're applied in the client side, thus leading to the same result on both ends.

The step number is obviously different for each client and server but they increase at the same rate. Using the _round trip time_ (RTT) one can calculate a _step delta_ which can be added to the local step number to convert it to the remote step number.

If the length of the jitter buffer in unit of time is smaller than half the RTT, it will happen that the client state will be applied as soon as it arrives in the server because in that case it should have been applied before it had arrived in the server (which is obviously not possible). That means, the amount of time in the past the server simulation must run has to be bigger than half the maximum RTT among all clients.

If the jitter buffer is too long, that means a bigger problem for the clients because the data they'll get from the server will be older and they'll have to extrapolate more to bring it to the current time which is more costly and errors in prediction are increased. More on extrapolation on the next sections.

To avoid having a jitter buffer that's big enough for all clients, clients are split into groups that are close enough and could potentially interact with one another through collisions or perhaps other custom constraints created on the fly. The size of the jitter buffer has to be adjusted based on the maximum half RTT of the clients who own an entity in that group. Could be something like 120% of the maximum half RTT. The groups are calculated exactly like in the broad-phase collision detection, though using much larger AABBs. The jitter buffer is kept in the island where the client's entities are contained in its length in time is adjusted by the coordinator as the RTT changes.

The reason why groups are used instead of islands is that islands are only merged when they are too close to one another. In the case where a client with small RTT is near another client with a large RTT, in the server world their location could be off significantly (e.g. two vehicles driving side by side) due to the different jitter buffer lengths so it's possible collisions would be missed. Creating groups by using larger AABBs prepares the world for those situations.

The concept of _ownership_ is central to decentralized control and conflict resolution in the server simulation. Dynamic entities can have an owning client, which means that the client has exclusive control over that entity and can directly modify its components. The entities permanently owned by a client are usually the ones created by it or created for it. Those are the entities that are directly controlled by the client, such as a character or a vehicle.

The client will send snapshots of its owned entities to the server which will verify these and possibly apply them to its simulation, in the corresponding group. In the case where there are no other entities owned by another client in the same group, the snapshot will go through a verification process where the components are compared to their current value and they're checked for hacking attempts where the values would be unrealistic. These components will be discarded.

This direct state application means that the client can run the simulation locally without the need to apply and extrapolate state from the server most of the time (for the entities owned by the client). Extrapolation is only necessary when there are other entities owned by another client in the same island.

The client has temporary ownership of all entities in its island, except when an entity owned by another client is also in this island, where in this case the server takes ownership of all entities to avoid conflicting simulation results. In other words, an island can only be owned by one client at a time. If two clients try to take ownership of the same island, the server intervenes and takes ownership to itself.

The client will send state updates to the server for all entities in its island so that the temporarily owned entities will also be synchronized without disrupting the local client simulation.

In the case where the server takes ownership of an island, none of the physics components in the client snapshots will be accepted. Only external components assigned by the external application which is using _Edyn_ will be applied, such as inputs, which will then be handled by the custom systems that can be attached to _Edyn_.

The server will hand ownership back to the client when the dynamic state sent by the client converges towards that of the server {this sounds complex}.

## Server sends data to client

The server will periodically capture snapshots of relevant islands and send them out to clients. Which islands will be sent and how often is determined by the client's _points of interest_.

The point of interest is simply an entity that has a `edyn::position` component. This means a dynamic or kinematic object can be set as a point of interest, such as the character or vehicle controlled by the user which is the only one point of interest that will exist in most cases. Extra points of interest are necessary in case the user can operate a free-look camera and move around to see the world somewhere else.

All entities in a radius around the point of interest will be included in the snapshot. Not all entities are necessarily included in one snapshot. If the server is multi-threaded, this will be done separately per island, which will result in a bunch of smaller snapshot packets being sent for the same step number. These packets don't need to be reliably delivered.

## Client sends data to server

Clients will periodically send snapshots of its owned entities to the server. Dynamic components will be sent often in a stream (unreliable packets). Steady components will be sent when they change (reliable packets).

One important piece that has to be managed by the user of the library are user inputs which will be also applied in the server. It's important that an input history is sent so that the chance of an input not reaching the server is lowered in case of packet loss.

## Client receives data from server

Snapshots sent by the server in the snapshot stream do not contain all components of each entity in it. These snapshots only contain the dynamic components that change very often, such as position and velocity. However, entities often have more components which are necessary to correctly simulate them. Thus, the client must check whether it already has obtained the full information of those entities and if not, query the server for the full thing. The server should respond with a snapshot containing the requested data and this can be loaded into the local registry which will instantiate the full entity and now it's ready to be properly updated.

Upon receiving snapshots from the server, the client is required to extrapolate these to the future before merging them onto its local simulation since the server simulation stays in the past plus there's latency to compensate for. The amount of time to extrapolate forward is calculated based on the snapshot step number converted to the local time frame.

Extrapolation is performed in a background worker thread. It is the same as a typical island task except that it is not synchronized with the global timer. Once it's done, the result comes back to the world/coordinator which has to perhaps step it forward once or twice to make the step number match and then it overrides its components with the extrapolated data. Islands now have to be recalculated. The islands that contain the entities that were extrapolated must be updated with the new data.

In case the server had taken ownership of an entity that's owned by the client, this entity will be contained in the snapshot. In this case it is necessary to also apply all inputs during extrapolation, thus a local input history has to be kept to be used here.

The server snapshot will usually contain a single island. If the extrapolation is too large, the entities in it would possibly collide with an existing object from another local island and the collision would be missed or detected only when merging. A solution would be to keep a history of all objects so that islands nearby the server snapshot island can be rolled back and extrapolated with it, but that seems to be an extra amount of processing and storage to be done for little benefit. As long as the extrapolations are not too large (which usually shouldn't be), this won't be a significant problem. This might be interesting to do for the client's entities though.

Extrapolation will introduce visible errors and disturbances when the extrapolate result doesn't match the previous local state very closely. A discontinuity error is calculated before the components are replaced by the extrapolated version and that is provided in a `edyn::discontinuity` component which can be used by the graphics engine to smooth out these errors by adding them to the next future states. The discontinuity decays towards zero over time.

# Clusters

Multiple server instances can run in different machines in the same local area network (LAN) and balance load. The principles of distributing work among all machine are similar to that of multi-threading.

# Distributed simulation

Several server clusters can exist in different geographical locations to allow for better network performance for local users. Clusters communicate among themselves to synchronize the redundant simulation of the persistent world.
