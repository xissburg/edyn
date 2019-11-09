#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

/**
 * Demonstration of the importance of interpolation for smooth real-time
 * presentation on screen using `edyn::current_position`. Using the raw
 * `edyn::position` will generally cause jitter.
 */
int main(int argc, char** argv) {
    entt::registry registry;
    edyn::world world(registry);

    // Use a fixed dt that will result in large residual dt in the fixed
    // time step updates for the purposes of this demonstration. A prime
    // number works well.
    world.fixed_dt = 0.041;

    const auto ent = registry.create();
    registry.assign<edyn::position>(ent, 0, 0, 0);
    registry.assign<edyn::current_position>(ent);
    // Set a speed that will move the entity 1 unit per update.
    registry.assign<edyn::linvel>(ent, 0, 1/0.4, 0);

    const auto freq = edyn::performance_frequency();
    const auto timescale = 1.0 / freq;
    const auto t0 = edyn::performance_counter();
    auto ti = t0;

    while (ti < t0 + 10 * freq) {
        const auto t = edyn::performance_counter();
        const auto dt = (t - ti) * timescale;
        world.update(dt);
        ti = t;

        printf("===============================\n");
        printf("step %lu, dt %.6f\n", world.current_step(), dt);

        auto view = registry.view<const edyn::position, const edyn::current_position>();
        view.each([] (auto ent, const auto& pos, const auto& curpos) {
            // Compare the physics position to the presentation positon and notice how
            // the physics position `pos` does not change uniformly after each update due the 
            // different number of steps taken each time. The presentation position `curpos`
            // should change 1 unit per update (it also grows a thousandth every once in while
            // since dt is not exactly the same every time).
            printf("pos    (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
            printf("curpos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), curpos.x, curpos.y, curpos.z);
        });

        edyn::delay(400);
    }

    return 0;
}