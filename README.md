![EdynLogo](https://user-images.githubusercontent.com/762769/211650462-3ad6dab2-5e47-4b62-993c-ac7fc7650cde.svg)

_Edyn_ (pronunciation: "eh-dyin'") stands for _Entity Dynamics_ and it is a real-time physics engine organized as an ECS (Entity-Component System) using the amazing [EnTT](https://github.com/skypjack/entt) library. The main goals of this library is to be multi-threaded and to support networked and distributed physics simulation of large dynamic worlds.

Examples are located in a separate repository: [Edyn Testbed](https://github.com/xissburg/edyn-testbed)

# Build Instructions

_Edyn_ is a compiled library.

## Requirements

A compiler with C++17 support, `CMake` version 3.23.0 or above and [Conan 2.0](https://conan.io/).

Dependencies:
- [EnTT](https://github.com/skypjack/entt) (installed via [Conan](https://conan.io/))

## Steps

In the terminal, `cd` into the _Edyn_ directory and run:

```
$ conan install .
$ conan build .
```

Then you should find the library under `build/Release/lib/`. Conan builds the optimized release version of the library by default. To build the debug version with assertions enabled, specify the build type and options:
```
$ conan install . -s build_type=Debug
$ conan build . -s build_type=Debug -o '&:enable_assert=True'
```

## Windows and Visual Studio 2019

After cloning the repo using [Git Bash](https://git-scm.com/downloads/win) and assuming [Conan 2.x](https://conan.io/) is installed, enter the following commands:
```
$ conan install .
$ cmake -S . -B build/generators -G "Visual Studio 16 2019" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
```

The _Edyn.sln_ solution should be in the _build/generators_ directory. Open it and it should be ready to build the library. It's important to note whether you want to build it as a static or dynamic library. It's is set to dynamic by default in VS2019. If you want to build it as a static library, you'll have to open the project properties (`Alt Enter`) and under `Configuration Properties > C/C++ > Code Generation > Runtime Library` select `Multi-threaded Debug (/MTd)` for debug builds and `Multi-thread (/MT)` for release builds.

When linking your application against _Edyn_ you'll also have to link `winmm.lib` because it needs the `timeGetTime()` function.

## Integration

To use _Edyn_ in your project you'll have to link it with `libEdyn` (or `libEdyn_d` for debug builds) which can be found in `edyn/build/lib/` after it's built. On Linux, you'll also have to link `pthread`. On Windows, you'll additionally have to link `winmm.lib`.

The paths to `edyn/include` and  `edyn/build/include` must be added to your include paths.

# The ECS approach

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
auto &shape = registry.emplace<edyn::box_shape>(entity, 0.5, 0.2, 0.4); // Box half-extents.
registry.emplace<edyn::inertia>(entity, edyn::moment_of_inertia(shape, mass));
registry.emplace<edyn::material>(entity, 0.2, 0.9); // Restitution and friction.
registry.emplace<edyn::gravity>(entity, edyn::gravity_earth);
```

There's no explicit mention of a rigid body in the code, but during the physics update all entities that have a combination of the components assigned above will be treated as a rigid body and their state will be updated over time as expected. Then, the rigid body motion may be updated as follows:

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
// Equivalent to example above.
auto def = edyn::rigidbody_def();
def.kind = edyn::rigidbody_kind::rb_dynamic;
def.position = {0, 3, 0};
def.orientation = edyn::quaternion_axis_angle({0, 1, 0}, edyn::to_radians(30));
def.linvel = edyn::vector3_zero;
def.angvel = {0, 0.314, 0};
def.mass = 50;
def.shape = edyn::box_shape{0.5, 0.2, 0.4}; // Shape is optional.
def.material->restitution = 0.2; // Material is also optional.
def.material->friction = 0.9;
def.gravity = edyn::gravity_earth;
auto entity = edyn::make_rigidbody(registry, def);
```

You are free to assign other components of your own to the returned entity. An existing entity can be passed to `edyn::make_rigidbody` as well, such as:

```cpp
auto entity = registry.create();
registry.emplace<MyCustomComponent>(entity, ...); // Assign your own components to it.
edyn::make_rigidbody(entity, registry, def);
```

# Basic setup

_Edyn_ is built as a multi-threaded library from the ground up which requires initializing its worker threads on start-up invoking `edyn::init()`, and then it must be attached to an `entt::registry` before setting up the scene:

```cpp
#include <entt/entt.hpp>
#include <edyn/edyn.hpp>

entt::registry registry;
edyn::init();
edyn::attach(registry);

// Create rigid bodies as shown above...

// Call `edyn::update()` periodically in your main loop somewhere.
for (;;) {
  edyn::update(registry);
  // Do something with the results, e.g. render scene.
  // ...
}
```

When `edyn::update()` is called, it processes any pending changes, creates/destroys workers if needed, dispatches messages to workers, reads and processes messages from workers which are merged into the `entt::registry`, preparing the entities and components to be rendered right after.

Due to its multi-threaded nature, all changes to relevant components in the main `entt::registry` need to be propagated to the worker threads. _Edyn_ doesn't automatically pick up these changes, thus it's necessary to notify it either by calling `edyn::refresh()` or assigning a `edyn::dirty` component to the entity and calling some of its functions such as `edyn::dirty::updated()` (e.g. `registry.emplace<edyn::dirty>(entity).updated<edyn::position, edyn::linvel>()`).

# Features

Following is a list containing the current major features:
- Dynamic, static and kinematic rigid bodies.
- Broad-phase collision detection using a dynamic bounding volume tree.
- Narrow-phase collision detection using specialized code for each permutation of shape types. Most closest point calculation is done using SAT.
- Closest point calculation can be done explicitly using the `edyn::collide` function and a pair of shapes, outside of a physics simulation.
- Shapes: sphere, cylinder, capsule, box, convex polyhedron, compound, plane, concave triangle mesh, paged triangle mesh which loads chunks in the background on demand.
- Constraints: distance, soft distance, hinge, point, gravity, generic, CV-joint, cone.
- Hinge constraint limits beyond the `[-π, π]` range, limit restitution, bump stops, friction, damping and spring.
- Ability to easily change the center of mass, without affecting constraint pivot points.
- Amorphous rigid bodies, i.e. rigid bodies without a shape which do not take part in collision detection and can be connected to other bodies via constraints.
- Rigid bodies which do not collide yet generate collision events, aka sensors.
- Raycasting.
- Ragdolls.
- Collision filtering using the typical group and mask bitsets and the more advanced _collision exclusion list_ for greater control.
- Collision events.
- Material ids and material properties to be used when a specific pair of materials interact.
- Per-vertex material properties in triangle mesh shapes.
- Rolling and spinning friction.
- Internal edge collision avoidance in triangle mesh shapes using Voronoi regions.
- Sequential impulses constraint solver with warm-starting.
- Proper restitution propagation using a specialized restitution solver.
- Parallel simulation using islands.
- Networked physics using a client-server model. _Edyn_ performs client state application in the server-side using a jitter buffer and does extrapolation on the client-side in a background thread. It generates packets containing physics state which just needs to be sent over the network and once received in the other side it must be fed into the engine. It handles serialization, deserialization and processing of all packets.
- Use your own existing task system to run _Edyn_'s background/parallel tasks without spawning additional threads.

# Games

- [Exhibition of Speed](https://store.steampowered.com/app/2947450/Exhibition_of_Speed): Build your own car and go racing.

# Documentation and Support

Check out the [wiki](https://github.com/xissburg/edyn/wiki) where the user manual can be found.

Check out [Edyn Testbed](https://github.com/xissburg/edyn-testbed) for concrete examples.

See the [design document](https://github.com/xissburg/edyn/blob/master/docs/Design.md) for information about the internals and planned features.

Questions can be asked on [discussions](https://github.com/xissburg/edyn/discussions) and the author can also be reached on the [EnTT Discord](https://discord.gg/5BjPWBd) under the same username and avatar.

Follow progress on [Trello](https://trello.com/b/5RQdZ7e1/edyn) where you can see the to-do list and what's being currently worked on.
