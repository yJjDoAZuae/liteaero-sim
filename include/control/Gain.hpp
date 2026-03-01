#pragma once

#include "SISOBlock.hpp"
#include "control/RectilinearTable.hpp"

#include <vector>
#include <string>
#include <sstream>
#include <fstream>

namespace liteaerosim::control {

template <typename T, uint32_t NumAxes>    
class Gain {

    public:

        Gain() : _K(0) {}
        Gain(T K) { _K = K; }

        void schedule(std::array<T, NumAxes> u);
        int readJSON(std::stringstream& ss);
        int readFile(std::string filepath);

        T K() const { return _K; }
        void operator=(T K) { _K = K; }
        // void operator=(double K) { _K = T(K); }
        operator double() const { return _K; }

    protected:

        RectilinearTable<T, T, NumAxes> table;
        T _K;

};


// TODO: implement this
template <typename T, uint32_t NumAxes>
int Gain<T,NumAxes>::readJSON(std::stringstream& ss)
{
    json data;
    data = json::parse(ss);

    return 0;

}

template <typename T, uint32_t NumAxes>
int Gain<T,NumAxes>::readFile(std::string filepath)
{
    std::ifstream fs;

    fs.open(filepath);

    std::stringstream ss;

    json data;
    if (fs) {
        ss << fs.rdbuf();
        fs.close();
        readJSON(ss);
    }

    return 0;
}

// template <typename T, uint32_t NumAxes>
// T operator* (const Gain<T,NumAxes>& y, double x)
// {
//     return T(y.K() * x);
// }

// template <typename T, uint32_t NumAxes>
// T operator* (double x, const Gain<T,NumAxes>& y)
// {
//     return T(x * y.K());
// }

// template <typename T, uint32_t NumAxes>
// T operator/ (const Gain<T,NumAxes>& y, double x)
// {
//     return T(y.K() / x);
// }

// template <typename T, uint32_t NumAxes>
// T operator/ (double x, const Gain<T,NumAxes>& y)
// {
//     return T(x / y.K());
// }
// template <typename T, uint32_t NumAxes>
// T operator+ (const Gain<T,NumAxes>& y, double x)
// {
//     return T(y.K() + x);
// }

// template <typename T, uint32_t NumAxes>
// T operator+ (double x, const Gain<T,NumAxes>& y)
// {
//     return T(x + y.K());
// }

// template <typename T, uint32_t NumAxes>
// T operator- (const Gain<T,NumAxes>& y, double x)
// {
//     return T(y.K() - x);
// }

// template <typename T, uint32_t NumAxes>
// T operator- (double x, const Gain<T,NumAxes>& y)
// {
//     return T(x - y.K());
// }

}
