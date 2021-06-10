![EdynLogo](https://xissburg.com/images/EdynLogo.svg)

_Edyn_ (pronunciation: "eh-dyin'") stands for _Entity Dynamics_ and it is a real-time physics engine organized as an ECS (Entity-Component System) using the amazing [EnTT](https://github.com/skypjack/entt) library. The main goals of this library is to be multi-threaded and to support networked and distributed physics simulation of large dynamic worlds.

It is still in an early stage of development and is not yet ready for use. Feel free to explore and contribute meanwhile.

Examples are located in a separate repository: [Edyn Testbed](https://github.com/xissburg/edyn-testbed)

# Build Instructions

_Edyn_ is a compiled library.

## Requirements

A compiler with C++17 support is required, along with `CMake` version 3.12.4 or above.

Dependencies:
- [EnTT](https://github.com/skypjack/entt) (installed via [Conan](https://conan.io/))

## Steps

In the terminal, go into the _Edyn_ directory and do:

```
$ mkdir build
$ cd build
$ conan install ../conanfile.txt
$ cmake ..
$ make
```

Then you should find the library under `edyn/build/lib/`.

## Windows and Visual Studio 2019

After running `cmake ..`, the _Edyn.sln_ solution should be in the _build_ directory. Open it and it should be ready to build the library. It's important to note whether you want to build it as a static or dynamic library. It's is set to dynamic by default in VS2019. If you want to build it as a static library, you'll have to open the project properties (`Alt Enter`) and under `Configuration Properties > C/C++ > Code Generation > Runtime Library` select `Multi-threaded Debug (/MTd)` for debug builds and `Multi-thread (/MT)` for release builds.

When linking your application against _Edyn_ you'll also have to link `winmm.lib` because of the `timeGetTime()` function.

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
auto &shape = registry.emplace<edyn::shape>(entity, edyn::box_shape{0.5, 0.2, 0.4});
registry.emplace<edyn::inertia>(entity, edyn::moment_of_inertia(shape, mass));
registry.emplace<edyn::material>(entity, 0.2, 0.9); // Restitution and friction.
registry.emplace<edyn::linacc>(entity, edyn::gravity_earth);
```

There's no explicit mention of a rigid body in the code, but during the physics update all entities that have a combination of the components assigned above will be treated as a rigid body and their state will be updated over time as expected. The update may be carried as follows:

```cpp
// Apply gravity acceleration, increasing linear velocity.
auto view = registry.view<edyn::linvel, edyn::linacc, edyn::dynamic_tag>();
view.each([dt] (edyn::linvel &vel, edyn::linacc &acc) {
  vel += acc * dt;
});
// ...
// Move entity with its linear velocity.
auto view = registry.view<edyn::position, edyn::linvel, edyn::dynamic_tag>();
view.each([dt] (edyn::position &pos, edyn::linvel &vel) {
  pos += vel * dt;
});
// ...
// Rotate entity with its angular velocity.
auto view = registry.view<edyn::orientation, edyn::angvel, edyn::dynamic_tag>();
view.each([dt] (edyn::orientation &orn, edyn::angvel &vel) {
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
def.shape = edyn::box_shape{0.5, 0.2, 0.4}; // Shape is optional.
def.update_inertia();
def.restitution = 0.2;
def.friction = 0.9;
def.gravity = edyn::gravity_earth;
auto entity = edyn::make_rigidbody(registry, def);
```

# Basics

_Edyn_ is built as a multi-threaded library from the ground up which requires initializing its worker threads on start-up invoking `edyn::init()`, and then it must be attached to an `entt::registry` before setting up the scene:

```cpp
#include <entt/entt.hpp>
#include <edyn/edyn.hpp>

entt::registry registry;
edyn::init();
edyn::attach(registry);

// Create rigid bodies and constraints...

// Call `edyn::update()` periodically in your main loop somewhere.
for (;;) {
  edyn::update(registry);
  // Do something with the results, e.g. render scene.
  // ...
}
```

When `edyn::update()` is called, it processes any pending changes, creates/destroys workers if needed, dispatches messages to workers, reads and processes messages from workers which are merged into the `entt::registry`, preparing the entities and components to be rendered right after.

Due to its multi-threaded nature, all changes to relevant components in the main `entt::registry` need to be propagated to the worker threads. _Edyn_ doesn't automatically pick up these changes, thus it's necessary to notify it either by calling `edyn::refresh()` or assigning a `edyn::dirty` component to the entity and calling some of its functions such as `edyn::dirty::updated()` (e.g. `registry.emplace<edyn::dirty>(entity).updated<edyn::position, edyn::linvel>()`).

# Design Philosophy

The code style and architecture is largely inspired by _EnTT_ which seeks to leverage modern C++ features. It steers away from typical object-oriented patterns making minimal use of inheritance and polymorphism, thus maintaining a more flat structure throughout. Data-oriented design principles are applied wherever feasible.

Code readability and maintainability are prioritized over the highest performance possible. This library does not intend to break world records. Some techniques might be superior in terms of performance but they also bring in a lot of pain due to the extra complexity, which also has the potential to result in more bugs which can be even harder to fix. They also make the code less welcoming to potential contributors. No proper benchmark has been done yet, but the basic performance tests already show similar and sometimes better performance than Bullet Physics, and there are still a lot of optimizations queued in the backlog.

See [docs/Design.md](https://github.com/xissburg/edyn/blob/master/docs/Design.md) to learn more about the engine's planned architecture.
