#include "SisoElement.hpp"

namespace liteaerosim {

float SisoElement::step(float u) {
    out_ = onStep(u);  // in_ still holds previous value during onStep
    in_  = u;
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
