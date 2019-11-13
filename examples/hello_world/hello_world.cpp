#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

void print_entities(entt::registry& registry, edyn::scalar dt) {
    auto& world = registry.ctx<edyn::world>();
    auto step = world.current_step();   
    auto time = step * world.fixed_dt;

    printf("================================\n");
    printf("step %lu, dt %.6f, time %.2f\n", step, dt, time);

    auto view = registry.view<const edyn::current_position, const edyn::linvel>();
    view.each([] (auto ent, const auto &pos, const auto &vel) {
        printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        printf("vel (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), vel.x, vel.y, vel.z);
    });

    if (time >= 10) {
        world.quit();
    }                                                                                                                                
}

int main(int argc, char** argv) {
    entt::registry registry;

    const auto ent = registry.create();
    // This entity has aposition in space.
    registry.assign<edyn::position>(ent, 0, 3, 0);
    // Current position used for presentation. See `current_pos.cpp` for details.
    registry.assign<edyn::current_position>(ent);
    // Linear velocity.
    registry.assign<edyn::linvel>(ent, 0, 10, 0);
    // Gravity linear acceleration.
    registry.assign<edyn::linacc>(ent, edyn::gravity_earth);

    // Create an `edyn::world` into the registry's context.
    auto& world = registry.set<edyn::world>(registry);
    world.update_sink().connect<&print_entities>(registry);
    // Run physics main loop. Could also simply call `world.update(dt)` in
    // your own main loop.
    world.run();

    return 0;
}