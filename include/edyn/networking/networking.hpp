#ifndef EDYN_NETWORKING_NETWORKING_HPP
#define EDYN_NETWORKING_NETWORKING_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/networking/packet/edyn_packet.hpp"
#include "edyn/networking/sys/client_side.hpp"
#include "edyn/networking/sys/server_side.hpp"
#include "edyn/networking/context/client_network_context.hpp"
#include "edyn/networking/context/server_network_context.hpp"
#include "edyn/networking/comp/remote_client.hpp"
#include "edyn/networking/networking_external.hpp"
#include <entt/entity/fwd.hpp>

namespace edyn {

/**
 * @brief Set client output registry snapshot rate per second, i.e. every
 * 1/rate seconds a registry snapshot will be taken and sent to the server.
 * The snapshot will contain all relevant entities and components that have
 * changed recently.
 * @param registry Data source.
 * @param rate Snapshot rate.
 */
void set_network_client_snapshot_rate(entt::registry &, double rate);

/**
 * @brief Get client output registry snapshot rate.
 * @param registry Data source.
 * @return Snapshot rate.
 */
double get_network_client_snapshot_rate(entt::registry &);

/**
 * @brief Set client network round-trip time (RTT).
 * @param registry Data source.
 * @param rtt The round-trip time in seconds.
 */
void set_network_client_round_trip_time(entt::registry &, double rtt);

/**
 * @brief Get client network round-trip time (RTT).
 * @param registry Data source.
 * @return The round-trip time in seconds.
 */
double get_network_client_round_trip_time(entt::registry &);

/**
 * @brief Enable or disable client-side extrapolation.
 * @remark When registry snapshots are received, due to network delay and
 * server-side playout delay, the state contained in them is from an earlier
 * point in time. Thus extrapolation is needed to create an approximation of
 * what the state should be at the current time. If disabled, the state is
 * applied immediately.
 * @param registry Data source.
 * @param enabled Whether extrapolation should be enabled.
 */
void set_network_client_extrapolation_enabled(entt::registry &, bool);

/**
 * @brief Toggle extrapolation enabled state.
 * @param registry Data source.
 * @return Whether extrapolation is enabled.
 */
bool toggle_network_client_extrapolation_enabled(entt::registry &);

/**
 * @brief Get extrapolation enabled state.
 * @param registry Data source.
 * @return Whether extrapolation is enabled.
 */
bool get_network_client_extrapolation_enabled(entt::registry &);

/**
 * @brief Set discontinuity decay rate, which is a value in [0, 1) that is
 * multiplied by the discontinuity offset after every simulation step.
 * @remark This is applied in every step of the simulation thus making this
 * value dependent on the fixed delta-time. If the fixed delta-time is halved,
 * there will be more steps performed per second thus doubling the rate of
 * decay per unit of time. To maintain a similar rate in that case, the
 * discontinuity decay rate should be the square root of the original value.
 * @remark When an extrapolation finishes, the result will usually not exactly
 * match the current state of the simulation, which means entities will
 * suddenly snap to a new position. To avoid this undesired visual disturbance,
 * the difference between the new and the old state is calculated and is decayed
 * over time. This difference can be added to the current transform to smooth
 * out these discontinuity effects. The `present_position` and
 * `present_orientation` components have this offset applied to them, thus
 * making a smooth visualization readily possible.
 * @param registry Data source.
 * @param rate Discontinuity decay rate per simulation step.
 */
void set_network_client_discontinuity_decay_rate(entt::registry &, scalar);

/**
 * @brief Get discontinuity decay rate per simulation step.
 * @param registry Data source.
 * @return Discontinuity decay rate per simulation step.
 */
scalar get_network_client_discontinuity_decay_rate(entt::registry &);

/**
 * In case the timestamp of a registry snapshot lies right after the time an
 * action happenend, it is possible that the action wasn't still applied in the
 * server side at the time the snapshot was generated. Perhaps the action was
 * applied at the same time the snapshot was generated and then its effects
 * were only visible in the next update, which will cause a glitch on client-side
 * extrapolation because the action will not be applied initially and the initial
 * state does not include the effects of the action because it wasn't applied in
 * the server at the time the snapshot was generated. All actions that happened
 * before the snapshot time within this threshold will be applied at the start
 * of an extrapolation.
 * @param registry Data source.
 * @param threshold Actions in the input history which lie before the registry
 * snapshot timestamp within this threshold (specified in seconds) will be
 * applied before the extrapolation steps start.
 */
void set_network_client_action_time_threshold(entt::registry &, double);

/**
 * @brief Get client-side extrapolation action threshold.
 * @param registry Data source.
 * @return Action threshold in seconds.
 */
double get_network_client_action_time_threshold(entt::registry &);

/**
 * @brief Get client packet sink. This sink must be observed and the packets
 * that are published into it should be sent over the network immediately.
 * @param registry Data source.
 * @return Packet sink.
 */
entt::sink<entt::sigh<void(const packet::edyn_packet &)>>
network_client_packet_sink(entt::registry &);

/**
 * @brief Triggered when the client entity is set by the server and replicated
 * locally.
 * @param registry Data source.
 * @return Client assignment sink.
 */
entt::sink<entt::sigh<void(entt::entity)>>
network_client_assigned_sink(entt::registry &);

/**
 * @brief Triggered when an extrapolation job times out due to it taking more
 * than the extrapolation time limit to complete. This will happen when the
 * simulation to be extrapolated is quite large and/or the length of time to be
 * extrapolated is quite long.
 * @param registry Data source.
 * @return Timeout warning sink.
 */
entt::sink<entt::sigh<void(void)>>
network_client_extrapolation_timeout_sink(entt::registry &);

/**
 * @brief Get server packet sink. This sink must be observed and the packets
 * that are published into it should be sent over the network immediately.
 * @param registry Data source.
 * @return Packet sink where the observers take the client entity and packet
 * as arguments.
 */
entt::sink<entt::sigh<void(entt::entity, const packet::edyn_packet &)>>
network_server_packet_sink(entt::registry &);

}

#endif // EDYN_NETWORKING_NETWORKING_HPP
