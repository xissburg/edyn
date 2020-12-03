![EdynLogo](https://xissburg.com/images/EdynLogo.svg)

_Edyn_ (pronounced like "eh-dyin'") stands for _Entity Dynamics_ and it is a real-time physics engine organized as an ECS (Entity-Component System) using the amazing [EnTT](https://github.com/skypjack/entt) library. The main goals of this library is to be multi-threaded and to support networked and distributed physics simulation of large dynamic worlds.

It is still in an early stage of development and is not yet ready for use. Feel free to explore and contribute meanwhile.

Examples are located in a separate repository: [Edyn Testbed](https://github.com/xissburg/edyn-testbed)

## Build Instructions

### Requirements

A compiler with C++17 support is required, along with `CMake` version 3.12.4 or above.

### Steps

In the _Edyn_ directory:

```
$ mkdir build
$ cd build
$ conan install ../conanfile.txt
$ cmake ..
$ make
```

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
auto &shape = registry.emplace<edyn::shape>(entity, edyn::box_shape{0.5, 0.2, 0.4});
registry.emplace<edyn::inertia>(entity, shape.inertia(mass));
registry.emplace<edyn::material>(entity, 0.2, 0.9); // Restitution and friction.
registry.emplace<edyn::linacc>(entity, edyn::gravity_earth);
```

There's no explicit mention of a rigid body in the code, but during the physics update all entities that have a combination of the components assigned above will be treated as a rigid body and their state will be updated over time as expected. The update may be carried as follows:

```cpp
// Apply gravity acceleration, increasing linear velocity.
auto view = registry.view<edyn::linvel, const edyn::linacc, const edyn::dynamic_tag>();
view.each([dt] (auto entity, edyn::linvel &vel, const edyn::linacc &acc, [[maybe_unused]] auto) {
  vel += acc * dt;
});
// ...
// Move entity with its linear velocity.
auto view = registry.view<edyn::position, const edyn::linvel, const edyn::dynamic_tag>();
view.each([dt] (auto entity, edyn::position &pos, const edyn::linvel &vel, [[maybe_unused]] auto) {
  pos += vel * dt;
});
// ...
// Rotate entity with its angular velocity.
auto view = registry.view<edyn::orientation, const edyn::angvel, const edyn::dynamic_tag>();
view.each([dt] (auto entity, edyn::orientation &orn, const edyn::angvel &vel, [[maybe_unused]] auto) {
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

## Documentation

See [docs/Design.md](https://github.com/xissburg/edyn/blob/master/docs/Design.md) to learn more about the engine's planned architecture.
