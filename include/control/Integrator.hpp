#pragma once

#include "control/control.hpp"
#include "SisoElement.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace liteaerosim::control {

class Integrator : public liteaerosim::SisoElement {
public:
    Integrator() :
        _dt(1.0f),
        _method(DiscretizationMethod::FwdEuler)
    {}

    Limit limit;
    std::vector<Antiwindup> aw;

    /// Warm-start: set output (and input) to u.
    void resetTo(float u);

    void setDt(float dt)     { _dt = (dt > 1e-6f) ? dt : 1.0f; }
    void setMethod(DiscretizationMethod method) { _method = method; }
    float dt() const { return _dt; }

protected:
    float onStep(float u) override;
    void  onReset() override;
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Integrator"; }

private:
    float _dt;
    DiscretizationMethod _method;
};

} // namespace liteaerosim::control
