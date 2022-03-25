#ifndef EDYN_NETWORKING_PACKET_SERVER_SETTINGS_HPP
#define EDYN_NETWORKING_PACKET_SERVER_SETTINGS_HPP

#include <cstdint>
#include "edyn/math/vector3.hpp"
#include "edyn/context/settings.hpp"

namespace edyn::packet {

struct server_settings {
    scalar fixed_dt;
    vector3 gravity;
    uint8_t num_solver_velocity_iterations;
    uint8_t num_solver_position_iterations;
    uint8_t num_restitution_iterations;
    uint8_t num_individual_restitution_iterations;
    bool allow_full_ownership;

    server_settings() = default;

    server_settings(settings &settings, bool allow_full_ownership)
        : fixed_dt(settings.fixed_dt)
        , gravity(settings.gravity)
        , num_solver_velocity_iterations(settings.num_solver_velocity_iterations)
        , num_solver_position_iterations(settings.num_solver_position_iterations)
        , num_restitution_iterations(settings.num_restitution_iterations)
        , num_individual_restitution_iterations(settings.num_individual_restitution_iterations)
        , allow_full_ownership(allow_full_ownership)
    {}
};

template<typename Archive>
void serialize(Archive &archive, server_settings &settings) {
    archive(settings.fixed_dt);
    archive(settings.gravity);
    archive(settings.num_solver_velocity_iterations);
    archive(settings.num_solver_position_iterations);
    archive(settings.num_restitution_iterations);
    archive(settings.num_individual_restitution_iterations);
    archive(settings.allow_full_ownership);
}

}

#endif // EDYN_NETWORKING_PACKET_SERVER_SETTINGS_HPP
