#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

/**
 * Demonstration of the importance of interpolation for smooth real-time
 * presentation on screen using `edyn::present_position`. Using the raw
 * `edyn::position` will generally cause jitter. The value in 
 * `edyn::present_position` is delayed by `world.fixed_dt` with respect
 * to the current global time. It's calculated by translating the physics
 * position backwards using `linvel`.
 */
void print_entities(entt::registry& registry, edyn::scalar dt) {
    auto& world = registry.ctx<edyn::world>();
    auto step = world.current_step();
    auto time = step * world.fixed_dt;

    printf("================================\n");
    printf("step %lu, dt %.6f, time %.2f\n", world.current_step(), dt, time);

    auto view = registry.view<const edyn::position, const edyn::present_position>();
    view.each([] (auto ent, const auto& pos, const auto& curpos) {
        // Compare the physics position to the presentation position and notice how
        // the physics position `pos` does not change uniformly after each update 
        // because the number of steps vary per update since the number of steps 
        // taken is `floor(dt / fixed_dt)`, which could be zero sometimes as well. The
        // presentation position `curpos` should change linearly with dt (because
        // `linvel` is constant in this example).
        printf("pos    (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        printf("curpos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), curpos.x, curpos.y, curpos.z);
    });

    if (time >= 10) {
        world.quit();
    }
}

int main(int argc, char** argv) {
    entt::registry registry;

    const auto ent = registry.create();
    registry.assign<edyn::dynamic_tag>(ent);
    registry.assign<edyn::position>(ent, 0, 0, 0);
    registry.assign<edyn::present_position>(ent);
    // Set a constant speed that will move the entity 1 unit per step.
    registry.assign<edyn::linvel>(ent, 0, 1/0.041, 0);

    auto& world = registry.set<edyn::world>(registry);
    world.fixed_dt = 0.041;
    world.update_sink().connect<&print_entities>(registry);
    world.run();

    return 0;
}