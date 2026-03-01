#include "numerics.hpp"

namespace liteaerosim {

FiltVectorXf left_resize(const FiltVectorXf& in, int len) {
    FiltVectorXf out(len);
    if (in.size() < len) {
        out << FiltVectorXf::Zero(len - in.size()), in;
    } else {
        out << in.tail(len);
    }
    return out;
}

FiltVectorXf right_resize(const FiltVectorXf& in, int len) {
    FiltVectorXf out(len);
    if (in.size() < len) {
        out << in, FiltVectorXf::Zero(len - in.size());
    } else {
        out << in.head(len);
    }
    return out;
}

void roll_buffer(FiltVectorXf& buff, float u) {
    if (buff.size() > 0) {
        buff << u, buff.head(buff.size() - 1);
    }
}

void roll_buffer(Vec3& buff, float u) {
    buff(2) = buff(1);
    buff(1) = buff(0);
    buff(0) = u;
}

}  // namespace liteaerosim
