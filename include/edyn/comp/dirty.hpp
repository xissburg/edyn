#ifndef EDYN_COMP_DIRTY_HPP
#define EDYN_COMP_DIRTY_HPP

#include <set>
#include <entt/entity/fwd.hpp>
#include <entt/core/type_info.hpp>

namespace edyn {

/**
 * @brief Marks an entity as dirty, consequently scheduling them for a refresh
 *      in the other end, i.e. from island worker to island coordinator and
 *      vice versa. These components are consumed by an island worker or
 *      coordinator in their update, i.e. they get processed and deleted right
 *      after.
 */
struct dirty {
    // If the entity was just created, this flag must be set.
    bool is_new_entity {false};

    using id_set_t = std::set<entt::id_type>;

    id_set_t created_indexes;
    id_set_t updated_indexes;
    id_set_t destroyed_indexes;

    /**
     * @brief Marks the given components as created.
     * @tparam Ts The created component types.
     * @return This object.
     */
    template<typename... Ts>
    dirty & created() {
        return cud<Ts...>(&dirty::created_indexes);
    }

    /**
     * @brief Marks the given components as updated.
     * @tparam Ts The updated component types.
     * @return This object.
     */
    template<typename... Ts>
    dirty & updated() {
        return cud<Ts...>(&dirty::updated_indexes);
    }

    /**
     * @brief Marks the given components as destroyed.
     * @tparam Ts The destroyed component types.
     * @return This object.
     */
    template<typename... Ts>
    dirty & destroyed() {
        return cud<Ts...>(&dirty::destroyed_indexes);
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
        created_indexes.insert(other.created_indexes.begin(), other.created_indexes.end());
        updated_indexes.insert(other.updated_indexes.begin(), other.updated_indexes.end());
        destroyed_indexes.insert(other.destroyed_indexes.begin(), other.destroyed_indexes.end());
        return *this;
    }

private:
    // CUD: Create, Update, Delete.
    template<typename... Ts>
    dirty & cud(id_set_t dirty:: *member) {
        ((this->*member).insert(entt::type_id<Ts>().seq()), ...);
        return *this;
    }
};

}

#endif // EDYN_COMP_DIRTY_HPP
