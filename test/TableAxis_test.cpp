#define _USE_MATH_DEFINES
#include "control/TableAxis.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace liteaerosim::control;

TEST(TableAxisTest, Instantiation00) {

    TableAxis<float> axis;

    EXPECT_EQ(axis.name.size(), 0);
    EXPECT_EQ(axis.domain.size(), 0);

}

TEST(TableAxisTest, Read00) {

    TableAxis<float> axis;
    std::ifstream fs;

    fs.open("tmp.json");

    json data = json::parse("{\"hello\": \"world\"}");

    EXPECT_EQ(data.is_boolean(), false);

    EXPECT_EQ(axis.name.size(), 0);
    EXPECT_EQ(axis.domain.size(), 0);

    std::stringstream ss1;
    ss1 << "{\"hello\": \"world\"}";

    EXPECT_EQ(axis.readJSON(ss1), 0);

    std::stringstream ss;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        EXPECT_EQ(axis.readJSON(ss), 0);
    }

    EXPECT_EQ(axis.domain.size(), 0);

}
