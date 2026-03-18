#include "SisoElement.hpp"

namespace liteaerosim {

float SisoElement::step(float u) {
    in_  = u;
    out_ = onStep(u);
    if (logger_) {
        onLog(*logger_);
    }
    return out_;
}

void SisoElement::reset() {
    in_  = 0.0f;
    out_ = 0.0f;
    onReset();
}

} // namespace liteaerosim
