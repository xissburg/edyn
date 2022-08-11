#ifndef EDYN_COMP_DIRTY_HPP
#define EDYN_COMP_DIRTY_HPP

#include <set>
#include <entt/entity/fwd.hpp>
#include <entt/core/type_info.hpp>

namespace edyn {

/**
 * @brief Marks an entity as dirty, consequently scheduling them for a refresh
 *      in the other end, i.e. from simulation worker to main and
 *      vice versa. These components are consumed by an simulation worker or
 *      main thread in their update, i.e. they get processed and deleted right
 *      after.
 */
struct dirty {
    // If the entity was just created, this flag must be set.
    bool is_new_entity {false};

    using id_set_t = std::set<entt::id_type>;

    id_set_t created_ids;
    id_set_t updated_ids;
    id_set_t destroyed_ids;

    /**
     * @brief Marks the given components as created.
     * @tparam Ts The created component types.
     * @return This object.
     */
    template<typename... Ts>
    dirty & created() {
        return cud<Ts...>(&dirty::created_ids);
    }

    /**
     * @brief Marks the given components as updated.
     * @tparam Ts The updated component types.
     * @return This object.
     */
    template<typename... Ts>
    dirty & updated() {
        return cud<Ts...>(&dirty::updated_ids);
    }

    /**
     * @brief Marks the given components as destroyed.
     * @tparam Ts The destroyed component types.
     * @return This object.
     */
    template<typename... Ts>
    dirty & destroyed() {
        return cud<Ts...>(&dirty::destroyed_ids);
    }

    /**
     * @brief Marks the owner as a newly created entity.
     * @return This object.
     */
    dirty & set_new() {
        is_new_entity = true;
        return *this;
    }

    dirty & merge(const dirty &other) {
        created_ids.insert(other.created_ids.begin(), other.created_ids.end());
        updated_ids.insert(other.updated_ids.begin(), other.updated_ids.end());
        destroyed_ids.insert(other.destroyed_ids.begin(), other.destroyed_ids.end());
        return *this;
    }

private:
    // CUD: Create, Update, Delete.
    template<typename... Ts>
    dirty & cud(id_set_t dirty:: *member) {
        ((this->*member).insert(entt::type_index<Ts>::value()), ...);
        return *this;
    }
};

}

#endif // EDYN_COMP_DIRTY_HPP
