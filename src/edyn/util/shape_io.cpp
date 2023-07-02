#include "edyn/util/shape_io.hpp"
#include "edyn/shapes/compound_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
#include <fstream>
#include <sstream>
#include <numeric>

namespace edyn {

static vector3 read_vector3(std::istringstream &iss) {
    edyn::vector3 v;
    iss >> v.x >> v.y >> v.z;
    return v;
}

static void read_face_indices(std::istringstream &iss,
                              std::vector<uint32_t> &indices,
                              uint32_t offset, bool triangulate) {
    std::string idx_str;
    auto count = size_t{};
    uint32_t first_idx;
    uint32_t prev_idx;

    while (true) {
        iss >> idx_str;
        if (iss.fail()) break;

        // Remove everything after the first slash to keep only the first
        // element in the "v/vt/vn" sequence.
        auto pos = idx_str.find_first_of('/');
        if (pos != std::string::npos) {
            idx_str.erase(pos);
        }

        auto idx = static_cast<uint32_t>(std::stoi(idx_str));
        EDYN_ASSERT(idx >= 1 + offset);

        if (triangulate && count >= 3) {
            indices.push_back(first_idx);
            indices.push_back(prev_idx);
        }

        indices.push_back(idx - 1 - offset);

        if (count == 0) {
            first_idx = indices.back();
        }

        prev_idx = indices.back();
        ++count;
    }
}

static void read_face(std::istringstream &iss,
                      std::vector<uint32_t> &indices,
                      std::vector<uint32_t> &faces,
                      uint32_t offset, bool triangulate) {
    // Store where this face starts in the `indices` array.
    faces.push_back(indices.size());

    read_face_indices(iss, indices, offset, triangulate);

    // Store the number of vertices in this face.
    auto count = indices.size() - faces.back();
    faces.push_back(count);
}

template<typename Stream>
void load_meshes_from_obj_stream(Stream &stream,
                                 std::vector<obj_mesh> &meshes,
                                 vector3 pos,
                                 quaternion orn,
                                 vector3 scale) {
    std::string line;
    auto mesh = obj_mesh{};
    uint32_t index_offset = 0;

    while (std::getline(stream, line)) {
        auto pos_space = line.find(" ");

        if (pos_space == std::string::npos) {
            continue;
        }

        auto cmd = line.substr(0, pos_space);

        if (cmd == "o") {
            if (!mesh.vertices.empty()) {
                index_offset += mesh.vertices.size();
                meshes.emplace_back(std::move(mesh));
            }
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            mesh.name = iss.str();
        } else if (cmd == "v") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            auto v = read_vector3(iss);

            if (scale != vector3_one) {
                v *= scale;
            }

            if (orn != quaternion_identity) {
                v = rotate(orn, v);
            }

            if (pos != vector3_zero ) {
                v += pos;
            }

            mesh.vertices.push_back(v);

            if (!iss.eof()) {
                // Try reading vertex color.
                auto color = read_vector3(iss);

                if (!iss.fail()) {
                    mesh.colors.push_back(color);
                }
            }
        } else if (cmd == "f") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            read_face(iss, mesh.indices, mesh.faces, index_offset, false);
        }
    }

    if (!mesh.vertices.empty()) {
        meshes.emplace_back(std::move(mesh));
    }
}

bool load_meshes_from_obj(const std::string &path,
                          std::vector<obj_mesh> &meshes,
                          vector3 pos,
                          quaternion orn,
                          vector3 scale) {
    auto file = std::ifstream(path);

    if (!file.is_open()) {
        return false;
    }

    load_meshes_from_obj_stream(file, meshes, pos, orn, scale);

    return true;
}

void load_meshes_from_obj(std::stringstream &ss,
                          std::vector<obj_mesh> &meshes,
                          vector3 pos,
                          quaternion orn,
                          vector3 scale) {
    load_meshes_from_obj_stream(ss, meshes, pos, orn, scale);
}

template<typename Stream>
void load_tri_mesh_from_obj_stream(Stream &stream,
                                   std::vector<vector3> &vertices,
                                   std::vector<uint32_t> &indices,
                                   std::vector<vector3> *colors,
                                   vector3 pos,
                                   quaternion orn,
                                   vector3 scale) {
    std::string line;

    while (std::getline(stream, line)) {
        auto pos_space = line.find(" ");

        if (pos_space == std::string::npos) {
            continue;
        }

        auto cmd = line.substr(0, pos_space);

        if (cmd == "v") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            auto v = read_vector3(iss);

            if (scale != vector3_one) {
                v *= scale;
            }

            if (orn != quaternion_identity) {
                v = rotate(orn, v);
            }

            if (pos != vector3_zero ) {
                v += pos;
            }

            vertices.push_back(v);

            if (colors != nullptr && !iss.eof()) {
                // Try reading vertex color.
                auto color = read_vector3(iss);

                if (!iss.fail()) {
                    colors->push_back(color);
                }
            }
        } else if (cmd == "f") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            read_face_indices(iss, indices, 0, true);
        }
    }
}

bool load_tri_mesh_from_obj(const std::string &path,
                            std::vector<vector3> &vertices,
                            std::vector<uint32_t> &indices,
                            std::vector<vector3> *colors,
                            vector3 pos,
                            quaternion orn,
                            vector3 scale) {
    auto file = std::ifstream(path);

    if (!file.is_open()) {
        return false;
    }

    load_tri_mesh_from_obj_stream(file, vertices, indices, colors, pos, orn, scale);
    return true;
}

void load_tri_mesh_from_obj(std::stringstream &stream,
                            std::vector<vector3> &vertices,
                            std::vector<uint32_t> &indices,
                            std::vector<vector3> *colors,
                            vector3 pos,
                            quaternion orn,
                            vector3 scale) {
    load_tri_mesh_from_obj_stream(stream, vertices, indices, colors, pos, orn, scale);
}

template<typename Stream>
std::vector<polyhedron_with_center> load_convex_polyhedrons_from_obj_stream(
    Stream &stream,
    const vector3 &pos,
    const quaternion &orn,
    const vector3 &scale) {

    auto meshes = std::vector<obj_mesh>{};

    load_meshes_from_obj_stream(stream, meshes, pos, orn, scale);

    EDYN_ASSERT(!meshes.empty());
    auto polyhedrons = std::vector<polyhedron_with_center>{};
    polyhedrons.reserve(meshes.size());

    // Create a polyhedron shape for each mesh.
    for (auto &mesh : meshes) {
        auto center = mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

        // Make all vertices to be positioned with respect to the centroid.
        // This is important for correct moment of inertia calculation using
        // the parallel axis theorem, which requires the moment of inertia
        // about the center of mass as input for compound shapes.
        for (auto &v : mesh.vertices) {
            v -= center;
        }

        auto &poly = polyhedrons.emplace_back();
        poly.center = center;
        poly.shape.mesh = std::make_shared<convex_mesh>();
        poly.shape.mesh->vertices = std::move(mesh.vertices);
        poly.shape.mesh->indices = std::move(mesh.indices);
        poly.shape.mesh->faces = std::move(mesh.faces);
        poly.shape.mesh->update_calculated_properties();

    #ifdef EDYN_DEBUG
        poly.shape.mesh->validate();
    #endif
    }

    return polyhedrons;
}

std::vector<polyhedron_with_center> load_convex_polyhedrons_from_obj(
    const std::string &path_to_obj,
    const vector3 &pos,
    const quaternion &orn,
    const vector3 &scale) {

    auto file = std::ifstream(path_to_obj);

    if (!file.is_open()) {
        return {};
    }

    return load_convex_polyhedrons_from_obj_stream(file, pos, orn, scale);
}

std::vector<polyhedron_with_center> load_convex_polyhedrons_from_obj(
    std::stringstream &ss,
    const vector3 &pos,
    const quaternion &orn,
    const vector3 &scale) {
    return load_convex_polyhedrons_from_obj_stream(ss, pos, orn, scale);
}

template<typename Stream>
compound_shape load_compound_shape_from_obj_stream(
    Stream &stream,
    const vector3 &pos,
    const quaternion &orn,
    const vector3 &scale) {

    auto polyhedrons = load_convex_polyhedrons_from_obj_stream(stream, pos, orn, scale);
    EDYN_ASSERT(!polyhedrons.empty());

    auto compound = compound_shape{};

    // Create a polyhedron shape for each mesh.
    for (auto &poly : polyhedrons) {
        compound.add_shape(poly.shape, poly.center, quaternion_identity);
    }

    compound.finish();
    return compound;
}

compound_shape load_compound_shape_from_obj(
    const std::string &path_to_obj,
    const vector3 &pos,
    const quaternion &orn,
    const vector3 &scale) {

    auto file = std::ifstream(path_to_obj);

    if (!file.is_open()) {
        return {};
    }

    return load_compound_shape_from_obj_stream(file, pos, orn, scale);
}

compound_shape load_compound_shape_from_obj(
    std::stringstream &ss,
    const vector3 &pos,
    const quaternion &orn,
    const vector3 &scale) {
    return load_compound_shape_from_obj_stream(ss, pos, orn, scale);
}

}
