#pragma once

#include "SisoElement.hpp"

namespace liteaerosim::control {

class Unwrap : public liteaerosim::SisoElement {
public:
    /// Warm-start: set both input and output to u.
    void resetTo(float u) { in_ = u; out_ = u; }

    using SisoElement::step;

    /// Unwrap u relative to an externally provided reference value.
    /// Updates in_ and out_ directly (bypasses NVI step).
    float step(float u, float ref);

protected:
    float onStep(float u) override;
    void  onReset() override {}
    void  onInitialize(const nlohmann::json&) override {}
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Unwrap"; }
};

} // namespace liteaerosim::control
