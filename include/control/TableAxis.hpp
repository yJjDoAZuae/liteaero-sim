#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace liteaerosim::control {

template <typename Ta>
class TableAxis {

    public:

        std::string name;
        std::vector<Ta> domain;

        int read(std::string filePath);

        int readJSON(std::stringstream& ss);

};

template <typename Ta>
int TableAxis<Ta>::readJSON(std::stringstream& ss) 
{
    json data = json::parse(ss);

    return 0;
}

}
