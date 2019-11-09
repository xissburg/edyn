#include <edyn/edyn.hpp>
#include <edyn/time/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

int main(int argc, char** argv) {
    entt::registry registry;
    edyn::world world(registry);

    const auto ent = registry.create();
    registry.assign<edyn::position>(ent, 0, 3, 0);
    registry.assign<edyn::current_position>(ent);
    registry.assign<edyn::linvel>(ent);
    registry.assign<edyn::linacc>(ent, edyn::gravity_earth);

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

        auto view = registry.view<const edyn::current_position>();
        view.each([] (auto ent, const auto& pos) {
            printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        });

        edyn::delay(300);
    }

    return 0;
}