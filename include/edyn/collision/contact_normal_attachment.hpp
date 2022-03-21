#ifndef EDYN_COLLISION_CONTACT_NORMAL_ATTACHMENT_HPP
#define EDYN_COLLISION_CONTACT_NORMAL_ATTACHMENT_HPP

namespace edyn {

/**
 * @brief To which body a contact normal is attached in a contact point.
 * This is used while solving position contact constraints where the bodies
 * are translated and rotated in each iteration, which causes the contact
 * points to move. An approximation is made to place the contact points and
 * contact normal in a new location where the closest points could be. If
 * the closest feature on a body is planar, it is selected as the place
 * where the contact normal is attached to, and thus it is rotated with this
 * body in each iteration.
 */
enum class contact_normal_attachment : unsigned char {
    none,
    normal_on_A,
    normal_on_B
};

template<typename Archive>
void serialize(Archive &archive, contact_normal_attachment &attachment) {
    serialize_enum(archive, attachment);
}

}

#endif // EDYN_COLLISION_CONTACT_NORMAL_ATTACHMENT_HPP
