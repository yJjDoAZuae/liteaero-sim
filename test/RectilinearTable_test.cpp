#define _USE_MATH_DEFINES
#include "control/RectilinearTable.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace liteaerosim::control;

TEST(RectilinearTableTest, Instantiation00) {

    RectilinearTable<float, float, 3> tab;

    EXPECT_EQ(tab.numRecords(), 0);

}

TEST(RectilinearTableTest, Read00) {

    RectilinearTable<float, float, 3> tab;
    std::ifstream fs;

    fs.open("tmp.json");

    json data = json::parse("{\"hello\": \"world\"}");

    EXPECT_EQ(data.is_boolean(), false);

    EXPECT_EQ(tab.numRecords(), 0);
    EXPECT_EQ(tab.readFile("foo.json"), 0);

    std::stringstream ss;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        EXPECT_EQ(tab.readJSON(ss), 0);
    }

    EXPECT_EQ(tab.numRecords(), 0);

}
