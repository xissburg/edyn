#include "../common/common.hpp"

TEST(std_serialization_test, test_variant) {
    auto var = std::variant<int, double, std::string>{1.2};
    auto buffer = edyn::memory_output_archive::buffer_type{};
    auto output = edyn::memory_output_archive(buffer);
    serialize(output, var);
    auto input = edyn::memory_input_archive(buffer.data(), buffer.size());
    auto var_in = std::variant<int, double, std::string>{};
    serialize(input, var_in);
    ASSERT_TRUE(std::holds_alternative<double>(var_in));
    ASSERT_DOUBLE_EQ(std::get<double>(var_in), 1.2);
}

TEST(std_serialization_test, test_map) {
    auto map = std::map<std::string, int>{};
    map["one"] = 1;
    map["twelve"] = 12;

    auto buffer = edyn::memory_output_archive::buffer_type{};
    auto output = edyn::memory_output_archive(buffer);
    serialize(output, map);
    auto input = edyn::memory_input_archive(buffer.data(), buffer.size());
    auto map_in = std::map<std::string, int>{};
    serialize(input, map_in);
    ASSERT_EQ(map_in["one"], 1);
    ASSERT_EQ(map_in["twelve"], 12);
}
