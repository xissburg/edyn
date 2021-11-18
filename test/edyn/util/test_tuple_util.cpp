#include "../common/common.hpp"
#include <tuple>

TEST(test_tuple_util, test_visit_tuple) {
    auto tuple = std::make_tuple(std::string("str"), 3.14f, 'c', 667);

    for (size_t i = 0; i < std::tuple_size_v<decltype(tuple)>; ++i) {
        edyn::visit_tuple(tuple, i, [] (auto &&value) {
            using ValueType = std::decay_t<decltype(value)>;
            auto count = 0;

            if constexpr(std::is_same_v<ValueType, std::string>) {
                ASSERT_EQ(value, "str");
                ++count;
            }

            if constexpr(std::is_same_v<ValueType, float>) {
                ASSERT_EQ(value, 3.14f);
                ++count;
            }

            if constexpr(std::is_same_v<ValueType, char>) {
                ASSERT_EQ(value, 'c');
                ++count;
            }

            if constexpr(std::is_same_v<ValueType, int>) {
                ASSERT_EQ(value, 667);
                ++count;
            }

            ASSERT_EQ(count, 1);
        });
    }
}
