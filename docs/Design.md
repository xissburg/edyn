# Edyn Design Document

This document describes the general engine architecture. It is a bit of a brainstorming document and does not reflect the current state of the library. The ideas presented here are planned to be implemented in the near future.

## Introduction

_Edyn_ (pronounced like "eh-dyin'") stands for _Entity Dynamics_ and it is a real-time physics engine focused on multi-threaded, networked and distributed simulation of massive dynamic worlds. It is organized as an _entity-component system_ (ECS) using the amazing [EnTT](https://github.com/skypjack/entt) library.

### The ECS approach

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

## Foundation

The library can be built with single- or double-precision floating point. `edyn::scalar` is simply a `using` declaration equals to `float` or `double` which is set according to the `EDYN_DOUBLE_PRECISION` compilation option. _build_settings.hpp_ is generated during build from _cmake/build_settings.h.in_ so that invocations are linked to the correct definition.

`edyn::vector3` is the vector type for positions and directions.

`edyn::quaternion` is the quaternion type for orientations.

`edyn::matrix3x3` is the 3x3 matrix also used for orientations and orthonormal bases.

_SIMD_ implementations are planned.

## Relations

If one entity could have its motion immediately influenced by another entity, a `edyn::relation` must be setup in a separate entity, which stores the entity ids of the related entities. This will ensure both will be in the same _island_ (more on that later) and the solver will apply impulses between them.

## Constraints

Constraints are components of the type `edyn::constraint` which stores the actual constraint in `std::variant`. Different constraint types have different data and logic, thus the `std::variant`, which by default contains all the fundamental constraint implementations provided by the library. To add custom constraints to the library it is necessary to fork the project and modify the code to insert these into the variant.

A `edyn::constraint` depends on a `edyn::relation` which must be assigned to the same entity. The entities that will be affected by the `edyn::constraint` will be obtained from the corresponding `edyn::relation`.

It's important to note that the size of a `std::variant` is the size of the largest element in its template parameter pack, thus in the situation where one constraint has much more data than all others, it is worth considering storing much of this data somewhere else and keep a pointer to it (e.g. using a `std::unique_ptr`) to avoid wasting too much space.

Constraints are derived from `edyn::constraint_base` and can implement some of the functions that are called in different solver stages to set up and modify its constraint rows.

A Sequential Impulse constraint solver is used.

## Collision detection and response

Many physics engines give special treatment to _contact constraints_ (aka _non-penetration constraints_). In _Edyn_ they are no different of other types of constraints. The `edyn::contact_constraint` is simply part of the `std::variant` in the `edyn::constraint` component.

During broad-phase in  the AABB of all entities is updated and intersections are found. For any new intersection, an entity is created and a `edyn::relation` and a `edyn::contact_manifold` components are assigned to it. For any AABB intersection that ceased to exist, the entity is destroyed thus also destroying all components associated with it.

In narrow-phase, closest point calculation is performed for all entities in a `edyn::relation` that also has a `edyn::contact_manifold`. For each new contact point, an entity is created with the same relation and a `edyn::contact_constraint`. It's important to note that there can be more than one relation referring to the same pair of entities. New contact points that are near existing points get merged together, and contact points that are separating (in either tangential or normal directions) are destroyed.

Entities that don't have a `edyn::shape` also don't have a `edyn::AABB` associated with them and thus are ignored in broad-phase.

After a broad-phase collision starts the AABBs are inflated slightly and then after they separate they go back to normal to avoid a situation where they'd join and separate repeatedly. This is sometimes called _hysteresis_.

To enable collision response for an entity, a `edyn::material` component must be assigned which basically contains the _restitution_ and _friction coefficient_. Otherwise, the entity behaves as a _sensor_, i.e. collision detection is performed but no impulses are applied thus the entity is allowed to penetrate others.

Collision events can be observed by listening to `on_construct<entt::contact_point>` or `on_destroy<entt::contact_point>` in the `entt::registry`. It's important to note that when the `on_construct<entt::contact_point>` event is triggered, the contact was just created and thus its associated constraint has not yet been solved thus it has a zero impulse. If the impulse is required (e.g. to play sounds based on the intensity of the collision), it is possible to observe step signals in `edyn::world::step_sink` and look for `entt::contact_point`s that have `lifetime == 0`, which is only true for new contacts, and at this point they will have the collision impulse assigned.

## Shapes

Similarly to constraints, `edyn::shape` also holds a `std::variant` with all shape types in its parameter pack. It's necessary to fork the project and modify the code to add custom shapes. It is also necessary to provide a `edyn::collide` function for every permutation of the custom shape with all existing shapes.

### Paged triangle mesh shape

For the shape of the world's terrain, a triangle mesh shape is usually the best choice. For larger worlds, it is interesting to split up this terrain in smaller chunks and load them in and out of the world as needed. The `edyn::paged_triangle_mesh` offers a deferred loading mechanism that will load chunks of a concave triangle mesh as dynamic objects enter their bounding boxes. It must be initialized with a set of entities with a `edyn::AABB` assigned and an `Archive` (which is templated).

As dynamic entities move into the `edyn::AABB`s (which should be immutable), it will ask the archive to load the triangle mesh for that region if the entity does not have a `edyn::triangle_mesh` assigned. The `Archive` object must implement `void operator()(entt::entity)` which will load the required triangle mesh (usually asynchronously) and then will assign a `edyn::triangle_mesh` to the given entity when done. Since it might take time to load the mesh from file and deserialize it, the AABBs should be inflated to prevent collisions from being missed.

When there are no dynamic entities in the AABB of the chunk, it becomes a candidate for unloading.

Edge adjacency and Voronoi regions are used to prevent internal edge collisions.

## Multi-threading

Multi-threading is implemented on top of a job system where jobs are scheduled to be run in worker threads. As such, unlike a typical physics engine, there's no main loop where the simulation steps are performed. Instead, the simulation steps are performed in a number of jobs that are scheduled to run in any of the worker threads, and after stepping the simulation to the current time they reschedule themselves to be run again at a later time and perform more steps. The results are synchronized into the main registry in every iteration of the application's main loop, usually in the graphics thread.

### Job System

_Edyn_ has its own job system it uses for parallelizing tasks and running background jobs. The `edyn::job_dispatcher` manages a set of workers which are each associated with a background thread. When a job is scheduled it pushes it into the queue of the least busy worker.

Job queues can also exist in any other thread. This allows scheduling tasks to run in specific threads which is particularly useful in asynchronous invocations that need to return a response in the thread that initiated the asynchronous task. To schedule a job to run in a specific thread, the `std::thread::id` must be passed as the first argument of `edyn::job_dispatcher::async`. It is necessary to allocate a queue for the thread by calling `edyn::job_dispatcher::assure_current_queue` and then also call `edyn::job_dispatcher::once_current_queue` periodically to execute the pending jobs scheduled to run in the current thread.

Jobs are a central part of the multi-threaded aspects of the engine and thus are expected to be small and quick to run, and they should **never** wait or sleep. The goal is to keep the worker queues always moving at a fast pace and avoid hogging the queue thus making any subsequent job wait for too long, or having a situation where one queue is backed up by a couple jobs while others are empty. _Job stealing_ is a possibility in this case but it introduces additional complexity which makes it too difficult to model it as a lockfree queue. Thus, if a job has to perform too much work, it should split it up and use a technique where the job stores its progress state and reschedules itself and then continues execution in the next run. If a job needs to run a for-loop, it should invoke `edyn::parallel_for_async` and set itself to be rescheduled to run when the returned `edyn::atomic_counter` reaches zero and then immediately return, allowing the next job in the queue to run. When the `edyn::atomic_counter` reaches zero, the same job will be scheduled and when it runs again, it's important to know where it was left at thus it's necessary to store a progress state and continue from there.

A job is comprised of a fixed size data buffer and a function pointer that takes that buffer as its single parameter. The worker simply calls the job's function with the data buffer as a parameter. It is responsibility of the job's function to deserialize the buffer into the expected data format and then execute the actual logic. This is to keep things simple and lightweight and to support lockfree queues in the future. If the job data does not fit into the fixed size buffer, it should allocate it dynamically and write the address of the data into the buffer. In this case, manual memory management is necessary thus, it's important to remember to deallocate the data after the job is done.

Using `edyn::timed_job_dispatcher` it is possible to schedule a job to run at a specific timestamp. The `edyn::timed_job_dispatcher` keeps a queue of waiting jobs sorted by desired execution time and it uses a `std::condition_variable` to block execution until the next timed job is ready invoking `std::condition_variable::wait_for` and then schedules it using `edyn::job_dispatcher`. To create a repeating job that is executed every _t_ seconds, it's necessary to have the job reschedule itself to run again at a later time once it finishes processing. This technique is used for running periodic tasks such as the physics simulation updates, which are executed in isolated simulation islands.

### Simulation Islands

Dynamic entities that cannot immediately affect the motion of others can be simulated in isolation. More precisely, two dynamic entities _A_ and _B_ which are not connected via constraints are not capable of immediately affecting the motion of each other. That means, the motion of _A_ and _B_ is independent and thus could be performed in two separate threads.

An _island_ is a set of dynamic entities that are connected via constraints, directly or indirectly. The motion of one dynamic entity in an island will likely have an effect on the motion of all other dynamic entities in the island, thus the constraints in one island have to be solved together. An island is analogous to a _connected component_ in a graph where the rigid bodies are the nodes and the constraints are the edges. Creating or destroying rigid bodies or constraints modify the topology of this graph and require islands to be split or merged. Islands have an `edyn::island` component which contains a list of entities that are located in that island. Entities that could be in one or more islands have a `edyn::island_node` component which contains a list of island entities which are the islands where this entity is located.

Dynamic entities can only be present in one island, since their motion is procedurally calculated. That means if two dynamic entities that reside in different islands need to interact (e.g. because of a collision), their islands must be merged into one.

Static and kinematic entities are not affected by the motion of dynamic entities, thus they can be present simultaneously in multiple islands. That means that their data (i.e. their components) will have to be duplicated. In general this is not much data so duplication shouldn't be a concern. One case where it can be desirable to not duplicate data due to its size, is where the static entity is linked to a triangle mesh shape containing thousands of triangles. In this case, the `edyn::triangle_mesh` keeps a `std::shared_ptr` to its triangle data so when it's duplicated, the bulk of its data is reused. This data is going to be accessed from multiple threads thus it must be thread-safe. For a `edyn::triangle_mesh` the data is immutable. A `edyn::paged_triangle_mesh` has a dynamic page loading mechanism which must be secured using mutexes.

Using the job system, each island can be scheduled to run the simulation in the background in separate threads taking advantage of the job system's load balancing. The `edyn::island_worker` has its own registry containing the entities of an island. In its function, it performs the number of steps necessary to bring the simulation to the current time using a `edyn::solver`. The island worker has a message queue (single producer, single consumer) where it receives commands from the coordinator. After each update, it takes a snapshot of its registry (`edyn::registry_snapshot`) and appends it to its message queue, and then reschedules itself to run again at a later time and thus continue stepping the island forward.

The `edyn::world` in the main physics thread manages the main registry which contains all entities in the simulation. It acts as a coordinator of `edyn::island_worker`s, creating new jobs, and merging and splitting existing ones. For every dynamic entity created, a new island is also created and it spawns a new `edyn::island_worker`, which receives as initial input a snapshot of the entities in that island and which will then instantiate them into its local registry when it first runs. It has to create new entities in its private registry and assign the same components to them and create a mapping from the local registry to the main registry. After every step it takes a snapshot of its registry and appends it to its message queue, mapping from local to main entity ids, similar to how an authoritative server (main registry) and a client (island registry) would operate.

The `edyn::world` runs a simple broadphase to detect AABB intersections only. When two entities intersect, they have to be moved into a single island. If they're two dynamic entities and they are in the same island, nothing needs to be done. If they're in separate islands, the islands must be merged. If one of the entities is dynamic and the other is either static or kinematic, the static/kinematic entity must be added to the island where dynamic entity is. In that case, add the static/kinematic to the list of entities of that island and then put it into a snapshot and send it over to the `edyn::island_worker`. 

When creating a `edyn::relation` between two dynamic entities, it is required to replicate the relation in the respective island. If both entities are in the same island, that is just a matter of sending this relation in a snapshot to the island via its message queue. If the dynamic entities are in separate islands, it's necessary to merge them into one island first and then send the same snapshot. When the island worker gets the snapshot, it will instantiate the relation and create an entity id mapping for it.

To merge two islands _A_ and _B_, the coordinator has to take all entities and components from _B_, insert them into a snapshot and send to _A_ and then terminate _B_ by calling `edyn::island_worker::finish()`.

If the island creates a new entity (e.g. when a new contact point is created), it won't yet have an entity id mapping for it, so it will put it in a separate section in the snapshot. In the main registry, it will first read the entries that are adequately mapped into its local entity space and then it will handle the entities that are not mapped yet right after. In this case it will create a local entity and store a mapping from this entity to the island entity and it will send this mapping over to the island worker. With this mapping available, the island worker does not have to place this entity in the separate section of the snapshot anymore.

When a physics component is modified, the changes have to be propagated to its respective `edyn::island_worker`. The `edyn::world` observes `entt::registry::on_update` to propagate changes to the islands, however, if the component is modified without using `entt::registry::replace()` or `entt::registry::patch()`, the `edyn::world` will not be able to know whether a component has changed. In that case, it's necessary to call `edyn::world::refresh()` with the entity and components that need to be updated in their islands.

Running the simulation in parallel means that the state of entities in the main registry will not be at the same point in time. However, all entities in one island are synchronized at the same point in time, which is given away by the timestamp property of an island. Then, for presentation the state must be interpolated/extrapolated according to the time in that island. That is done internally using `edyn::present_position` and `edyn::present_orientation` which are updated in every `edyn::world::update` and those should be used for presentation instead of `edyn::position` or `edyn::orientation`.

When merging the registries of `edyn::island_worker`s onto the main registry their timestamp might differ slightly. This might lead to broadphase pairs not initiating at the exact right time but it does not look like a problem in principle. Slightly inflating AABBs could help mitigate the issue.

Splitting the simulation into islands is not enough to maximize resource utilization in certain cases, such as a big pyramid of boxes, which is a single island and would be run in a single island worker, thus turning it into a single-threaded solution. It is advantageous to further split the island step into substeps which can run in parallel, one of which is collision detection, which can be fully parallelized using a `edyn::parallel_for_async` and then set itself to be rescheduled once the `edyn::atomic_counter` hits zero. The constraint solver iterations are also rather expensive in this case though more difficult to parallelize since there's a dependency among _some_ of them. For example, in a chain of rigid bodies connected by a simple joint such as `A-(α)-B-(β)-C`, the constraints `α` and `β` cannot be solved in parallel because both depend on body `B`. However, in a chain such as `A-(α)-B-(β)-C-(γ)-D`, the constraints `α` and `γ` can be solved in parallel because they don't have any rigid body in common, and then constraint `β` can be solved in a subsequent step (this is in one iteration of the solver, where 10 iterations is the default). The separation of independent constraints into groups that can be solved in parallel is an _edge coloring_ of the graph. Given a valid edge coloring of the constraint graph, each set of constraints of the same color can be solved in parallel. This is possible because no pair of constraints of the same color touches the same rigid bodies. Thus, for each iteration of the solver, it would, for each color, dispatch a set of solver iteration jobs for the constraints of that color using `edyn::parallel_for_async` and wait for them to finish using the returned `edyn::atomic_counter` to reschedule the island job and then continue by dispatching the next color and so on, and then after all colors are done, move over to the next iteration and repeat.

Broad-phase collision detection is done using a dynamic bounding volume tree using the entities' AABBs inside `edyn::broadphase`. All it does is find intersecting/separating AABBs and then create/destroy entities with a `edyn::relation` component thus connecting rigid bodies together.

The _step number_ is an unsigned integer which starts at zero and is incremented in every step of the simulation in each island. It corresponds to what point in time the simulation is at. All entities in an island will have their state correspond to the same step number at all times.

Another function of islands is to allow entities to _sleep_ when they're inactive (not moving, or barely moving). As stated before, an island is a set of entities where the motion of one can immediately affect all others, thus when none of these entities are moving, nothing is going to move, so it's wasteful to do motion integration and constraint resolution for an island in this state. In that case the island is put to sleep by assigning a `edyn::sleeping_tag` to all entities in the island. The `edyn::island_worker` stops rescheduling itself when it is set to sleep. It must be waken up by the coordinator.

### Parallel-for

A naïve parallel-for implementation would split the range of work into _N_ nearly equally sized subranges (where _N_ is the number of cores or workers) and dispatch these into simple jobs that iterate over that subrange, calling a function on each iteration which takes the index as its single parameter. The `edyn::parallel_for` behaves quite differently. It initially dispatches a specialized parallel-for job and then starts running the for-loop right after, in the calling thread. The parallel-for job has a reference to a structure that holds the state of the for-loop that's currently running in the calling thread. It first checks if there's enough work left to do and then proceeds to _steal_ a subrange of that loop while it's still running, by subtracting a value from its upper bound, which is an atomic integer. It then repeats the same initial steps where it schedules another parallel-for job and starts iterating of the range it has taken for itself. When the next parallel-for job runs, it will do the same, though now it has to choose which subrange to steal from, which should be the one with the largest amount of work left. This splitting continues until the number of parallel-for jobs reaches a limit, usually the number of workers available. 

This technique is superior in that it's naturally adaptive to the current conditions where in the naïve implementation, the call to `edyn::parallel_for` could block waiting for a subrange that is waiting in a busy queue to be actually run. It ensures that the for-loop is always making progress.

In the asynchronous form, `edyn::parallel_for_async` does not execute a subrange of the for-loop in the calling thread. It instead dispatches a parallel-for job and immediatelly returns an `edyn::atomic_counter` which allows the caller to assign a job to be scheduled when the for-loop is done, i.e. when the counter reaches zero. **Never** use `edyn::parallel_for` inside a job, that is, when running in a worker thread, to avoid piling up work in the respective queue. Always use `edyn::parallel_for_async` in that case and use the `edyn::atomic_counter` to continue the work later when the for-loop is done.

Really big for-loops could still cause the worker thread to be blocked for too long executing a long range of work. In this case it's possible to specify a maximum number of iterations per execution. The parallel-for job will reschedule itself and return once the maximum number of iterations is reached and when it's picked for execution again, it continues from where the work was left at.

### Parallel-reduce

_TODO_

## Networking

The networking model follows a client-server architecture where the server is authoritative but gives the client freedom to directly set the state of the entities it owns in some situations. The goal is to synchronize the simulation on both ends, accounting for network latency and jitter.

The server is the source of truth, containing the actual physics simulation encompassing all entities and components including extra external components used by systems provided by the user of this library. The server has to frequently broadcast to all clients a snapshot of the dynamic components which change very often such as position and velocity. Steady components, which change casually, have to broadcast only when updated.

The server also receives data from the clients, which can be commands to be applied to the entities owned by that client and also component state to be set onto the entities in the server, which the server will have to decide whether to apply them or not.

The client will send commands generated by user inputs and also the dynamic state of the entities it controls/owns.

When receiving data from the server, the received state will be in the past due to network latency, thus the client must _extrapolate_ it before merging it onto the local simulation.

All data shared between client and server is generated using registry snapshots from _EnTT_. Thus, all that's being sent are entities and components which are put into a snapshot and serialized for network use.

The issues above will be discussed in greater detail in the following subsections.

### Server receives data from client

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

### Server sends data to client

The server will periodically capture snapshots of relevant islands and send them out to clients. Which islands will be sent and how often is determined by the client's _points of interest_.

The point of interest is simply an entity that has a `edyn::position` component. This means a dynamic or kinematic object can be set as a point of interest, such as the character or vehicle controlled by the user which is the only one point of interest that will exist in most cases. Extra points of interest are necessary in case the user can operate a free-look camera and move around to see the world somewhere else.

All entities in a radius around the point of interest will be included in the snapshot. Not all entities are necessarily included in one snapshot. If the server is multi-threaded, this will be done separately per island, which will result in a bunch of smaller snapshot packets being sent for the same step number. These packets don't need to be reliably delivered.

### Client sends data to server

Clients will periodically send snapshots of its owned entities to the server. Dynamic components will be sent often in a stream (unreliable packets). Steady components will be sent when they change (reliable packets).

One important piece that has to be managed by the user of the library are user inputs which will be also applied in the server. It's important that an input history is sent so that the chance of an input not reaching the server is lowered in case of packet loss.

### Client receives data from server

Snapshots sent by the server in the snapshot stream do not contain all components of each entity in it. These snapshots only contain the dynamic components that change very often, such as position and velocity. However, entities often have more components which are necessary to correctly simulate them. Thus, the client must check whether it already has obtained the full information of those entities and if not, query the server for the full thing. The server should respond with a snapshot containing the requested data and this can be loaded into the local registry which will instantiate the full entity and now it's ready to be properly updated.

Upon receiving snapshots from the server, the client is required to extrapolate these to the future before merging them onto its local simulation since the server simulation stays in the past plus there's latency to compensate for. The amount of time to extrapolate forward is calculated based on the snapshot step number converted to the local time frame.

Extrapolation is performed in a background worker thread. It is the same as a typical island task except that it is not synchronized with the global timer. Once it's done, the result comes back to the world/coordinator which has to perhaps step it forward once or twice to make the step number match and then it overrides its components with the extrapolated data. Islands now have to be recalculated. The islands that contain the entities that were extrapolated must be updated with the new data.

In case the server had taken ownership of an entity that's owned by the client, this entity will be contained in the snapshot. In this case it is necessary to also apply all inputs during extrapolation, thus a local input history has to be kept to be used here.

The server snapshot will usually contain a single island. If the extrapolation is too large, the entities in it would possibly collide with an existing object from another local island and the collision would be missed or detected only when merging. A solution would be to keep a history of all objects so that islands nearby the server snapshot island can be rolled back and extrapolated with it, but that seems to be an extra amount of processing and storage to be done for little benefit. As long as the extrapolations are not too large (which usually shouldn't be), this won't be a significant problem. This might be interesting to do for the client's entities though.

Extrapolation will introduce visible errors and disturbances when the extrapolate result doesn't match the previous local state very closely. A discontinuity error is calculated before the components are replaced by the extrapolated version and that is provided in a `edyn::discontinuity` component which can be used by the graphics engine to smooth out these errors by adding them to the next future states. The discontinuity decays towards zero over time.

## Clusters

Multiple server instances can run in different machines in the same local area network (LAN) and balance load. The principles of distributing work among all machine are similar to that of multi-threading.

## Distributed simulation

Several server clusters can exist in different geographical locations to allow for better network performance for local clients. Clusters communicate among themselves to synchronize the redundant simulation of the persistent world.
