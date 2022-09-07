#ifndef EDYN_NETWORKING_UTIL_PROCESS_EXTRAPOLATION_RESULT_HPP
#define EDYN_NETWORKING_UTIL_PROCESS_EXTRAPOLATION_RESULT_HPP

#include <entt/entity/fwd.hpp>

namespace edyn {

class entity_map;
struct extrapolation_result;

void process_extrapolation_result(entt::registry &registry, entity_map &emap,
                                  const extrapolation_result &result);

void process_extrapolation_result(entt::registry &registry,
                                  const extrapolation_result &result);

}

#endif // EDYN_NETWORKING_UTIL_PROCESS_EXTRAPOLATION_RESULT_HPP
