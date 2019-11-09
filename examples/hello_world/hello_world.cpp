#include <edyn/edyn.hpp>
#include <edyn/time.hpp>
#include <entt/entt.hpp>
#include <cstdio>

int main(int argc, char** argv) {
    entt::registry registry;

    auto ent = registry.create();
    registry.assign<edyn::position>(ent, 0, 3, 0);
    registry.assign<edyn::linvel>(ent);
    registry.assign<edyn::linacc>(ent, edyn::gravity_earth);

    const auto freq = edyn::performance_frequency();
    const auto t0 = edyn::performance_counter();
    auto ti = t0;
    auto timescale = 1.0 / freq;

    while (ti < t0 + 10 * freq) {
        const auto t = edyn::performance_counter();
        const auto dt = (t - ti) * timescale;
        edyn::update(registry, dt);
        ti = t;

        auto view = registry.view<const edyn::position>();
        view.each([] (auto ent, const auto& pos) {
            printf("pos (%d): %.3f, %.3f, %.3f\n", entt::to_integer(ent), pos.x, pos.y, pos.z);
        });
    }

    return 0;
}