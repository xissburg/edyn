#ifndef EDYN_COMP_MERGE_COMPONENT_HPP
#define EDYN_COMP_MERGE_COMPONENT_HPP

namespace edyn {

/**
 * @brief Function which can be specialized to perform custom logic when
 * replacing a component with a new value coming from a remote source, such as
 * importing components from the main registry into a worker and vice versa, and
 * importing remote values for components in client-server communication.
 * It simply assigns the new value by default.
 */
template<typename Component>
void merge_component(Component &component, const Component &new_value) {
    component = new_value;
}

}

#endif // EDYN_COMP_MERGE_COMPONENT_HPP
