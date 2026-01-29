#include "edyn/networking/util/import_contact_manifolds.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/collision/contact_manifold_map.hpp"
#include "edyn/collision/contact_point.hpp"
#include "edyn/comp/tag.hpp"
#include "edyn/comp/transient.hpp"
#include "edyn/context/settings.hpp"
#include "edyn/networking/extrapolation/extrapolation_result.hpp"
#include "edyn/util/collision_util.hpp"
#include "edyn/util/constraint_util.hpp"
#include "edyn/collision/contact_manifold.hpp"
#include "edyn/replication/entity_map.hpp"
#include "edyn/util/contact_manifold_util.hpp"

namespace edyn {

void flip_contact_point(contact_point &cp) {
    std::swap(cp.pivotA, cp.pivotB);
    cp.normal *= -1;
}

void flip_contact_point_geom(contact_point_geometry &cp_geom) {
    std::swap(cp_geom.featureA, cp_geom.featureB);
    auto att = cp_geom.normal_attachment;
    cp_geom.normal_attachment = att == contact_normal_attachment::normal_on_A ? contact_normal_attachment::normal_on_B : contact_normal_attachment::normal_on_A;
}

void replace_manifold(entt::registry &registry,
                      const extrapolation_result::contact_manifold_info &manifold_info,
                      const entt::storage<extrapolation_result::contact_point_info> &contacts,
                      const contact_manifold_map &manifold_map) {
    EDYN_ASSERT(registry.all_of<rigidbody_tag>(manifold_info.body[0]));
    EDYN_ASSERT(registry.all_of<rigidbody_tag>(manifold_info.body[1]));
    entt::entity manifold_entity;

    // Find a matching manifold and replace it...
    if (manifold_map.contains(manifold_info.body[0], manifold_info.body[1])) {
        manifold_entity = manifold_map.get(manifold_info.body[0], manifold_info.body[1]);
    } else {
        // ...or create a new one and assign a new value to it.
        // Important remark: `make_contact_manifold` does not necessarily
        // create the `contact_manifold` with the bodies in the same order
        // that's passed in the arguments.
        auto separation_threshold = contact_breaking_threshold * scalar(1.3);
        manifold_entity = make_contact_manifold(registry,
                                                manifold_info.body[0], manifold_info.body[1],
                                                separation_threshold);
    }

    auto &manifold = registry.get<contact_manifold>(manifold_entity);
    const bool is_swapped = manifold_info.body[0] != manifold.body[0];

    auto cp_view = registry.view<contact_point, contact_point_geometry, contact_point_impulse>();
    auto merged_contacts = entt::sparse_set{};

    contact_manifold_each_point(registry, manifold_entity,
        [&](entt::entity contact_entity) {
            auto &cp = cp_view.get<contact_point>(contact_entity);
            auto info_entity = manifold_info.contact_entity;
            auto shortest_dist_sqr = square(contact_caching_threshold);
            auto nearest_contact = entt::entity{entt::null};

            while (info_entity != entt::null) {
                auto &contact = contacts.get(info_entity);
                auto contact_pivotA = is_swapped ? contact.pt.pivotB : contact.pt.pivotA;
                auto contact_pivotB = is_swapped ? contact.pt.pivotA : contact.pt.pivotB;

                auto dA = length_sqr(cp.pivotA - contact_pivotA);
                auto dB = length_sqr(cp.pivotB - contact_pivotB);

                if (dA < shortest_dist_sqr) {
                    shortest_dist_sqr = dA;
                    nearest_contact = info_entity;
                }

                if (dB < shortest_dist_sqr) {
                    shortest_dist_sqr = dB;
                    nearest_contact = info_entity;
                }

                info_entity = contact.list.next;
            }

            if (nearest_contact != entt::null && !merged_contacts.contains(nearest_contact)) {
                auto &contact = contacts.get(nearest_contact);

                if (is_swapped) {
                    auto pt = contact.pt;
                    flip_contact_point(pt);
                    cp = pt;

                    auto geom = contact.geom;
                    flip_contact_point_geom(geom);
                    cp_view.get<contact_point_geometry>(contact_entity) = geom;
                } else {
                    cp = contact.pt;
                    cp_view.get<contact_point_geometry>(contact_entity) = contact.geom;
                }

                cp_view.get<contact_point_impulse>(contact_entity) = contact.imp;
                merged_contacts.push(nearest_contact);
            } else {
                destroy_contact_point(registry, contact_entity);
            }
        });

    // Create contact points for the unmerged.
    auto info_entity = manifold_info.contact_entity;
    auto &manifold_state = registry.get<contact_manifold_state>(manifold_entity);
    auto &async_settings = registry.ctx().get<const settings>().async_settings;
    const auto &transient = async_settings->contact_points_transient;

    while (info_entity != entt::null) {
        auto &contact = contacts.get(info_entity);

        if (!merged_contacts.contains(info_entity)) {
            auto pt = collision_result::collision_point{};
            pt.pivotA = contact.pt.pivotA;
            pt.pivotB = contact.pt.pivotB;
            pt.normal = contact.pt.normal;
            pt.distance = contact.geom.distance;
            pt.normal_attachment = contact.geom.normal_attachment;
            pt.featureA = contact.geom.featureA;
            pt.featureB = contact.geom.featureB;

            if (is_swapped) {
                pt.swap();
            }

            create_contact_point(registry, manifold_entity, manifold, manifold_state, pt, transient);
        }

        info_entity = contact.list.next;
    }
}

void import_contact_manifolds(entt::registry &registry, const entity_map &emap,
                              const std::vector<extrapolation_result::contact_manifold_info> &manifolds,
                              const entt::storage<extrapolation_result::contact_point_info> &contacts) {
    const auto &manifold_map = registry.ctx().get<contact_manifold_map>();

    for (auto manifold : manifolds) {
        if (!emap.contains(manifold.body[0]) || !emap.contains(manifold.body[1])) {
            continue;
        }

        manifold.body[0] = emap.at(manifold.body[0]);
        manifold.body[1] = emap.at(manifold.body[1]);

        if (!registry.valid(manifold.body[0]) || !registry.valid(manifold.body[1])) {
            continue;
        }

        replace_manifold(registry, manifold, contacts, manifold_map);
    }
}

void import_contact_manifolds(entt::registry &registry,
                              const std::vector<extrapolation_result::contact_manifold_info> &manifolds,
                              const entt::storage<extrapolation_result::contact_point_info> &contacts) {
    const auto &manifold_map = registry.ctx().get<contact_manifold_map>();

    for (auto manifold : manifolds) {
        if (!registry.valid(manifold.body[0]) || !registry.valid(manifold.body[1])) {
            continue;
        }

        replace_manifold(registry, manifold, contacts, manifold_map);
    }
}

}
