#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

polyhedron_shape::polyhedron_shape(std::shared_ptr<convex_mesh> mesh)
    : mesh(mesh)
{}

}
