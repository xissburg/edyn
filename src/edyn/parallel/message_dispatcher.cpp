#include "edyn/parallel/message_dispatcher.hpp"

namespace edyn {

message_dispatcher &message_dispatcher::global() {
    static message_dispatcher instance;
    return instance;
}

}
