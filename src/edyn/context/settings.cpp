#include "edyn/context/settings.hpp"
#include "edyn/comp/shared_comp.hpp"
#include "edyn/replication/component_index_source.hpp"

namespace edyn {

settings::settings()
    : index_source(new component_index_source_impl(shared_components_t{}))
{}

}
