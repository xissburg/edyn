#include "edyn/networking/util/process_extrapolation_result.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include "edyn/networking/util/accumulate_discontinuities.hpp"
#include "edyn/networking/util/assign_previous_transforms.hpp"
#include "edyn/networking/util/import_contact_manifolds.hpp"

namespace edyn {

void process_extrapolation_result(entt::registry &registry, entity_map &emap,
                                  const extrapolation_result &result) {
    EDYN_ASSERT(!result.ops.empty());

    // Assign current transforms to previous before importing pools into registry.
    assign_previous_transforms(registry);

    result.ops.execute(registry, emap);

    accumulate_discontinuities(registry);
    import_contact_manifolds(registry, emap, result.manifolds);
}

}
