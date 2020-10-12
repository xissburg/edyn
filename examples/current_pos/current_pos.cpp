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
void print_entities(entt::registry& registry) {
    auto& world = registry.ctx<edyn::world>();

    printf("================================\n");

    auto view = registry.view<const edyn::position, const edyn::present_position>();
    view.each([] (auto ent, const auto& pos, const auto& curpos) {
        // Compare the physics position to the presentation position and notice how
        // the physics position `pos` does not change uniformly after each update 
        // because the number of steps vary per update since the number of steps 
        // taken is `floor(dt / fixed_dt)`, which could be zero sometimes as well. The
        // presentation position `curpos` should change linearly with dt (because
        // `linvel` is constant in this example).
        printf("pos    (%d): %.3f, %.3f, %.3f\n", entt::to_integral(ent), pos.x, pos.y, pos.z);
        printf("curpos (%d): %.3f, %.3f, %.3f\n", entt::to_integral(ent), curpos.x, curpos.y, curpos.z);
    });
}

int main(int argc, char** argv) {
    entt::registry registry;

    const auto ent = registry.create();
    registry.emplace<edyn::dynamic_tag>(ent);
    registry.emplace<edyn::position>(ent, 0, 0, 0);
    registry.emplace<edyn::present_position>(ent);
    // Set a constant speed that will move the entity 1 unit per step.
    registry.emplace<edyn::linvel>(ent, 0, 1/0.041, 0);

    auto& world = registry.set<edyn::world>(registry);
    world.m_fixed_dt = 0.041;

    for (;;) {
        world.update();
        print_entities(registry);
        edyn::delay(100);
    }

    return 0;
}