#pragma once

#include "control/control.hpp"
#include "SisoElement.hpp"
#include "control/Limit.hpp"
#include "control/Antiwindup.hpp"
#include <vector>

namespace liteaerosim::control {

class Derivative : public liteaerosim::SisoElement {
public:
    Derivative() :
        _dt(1.0f),
        _Tau(0.0f),
        _method(DiscretizationMethod::FwdEuler)
    {}

    Limit limit;

    /// Warm-start: set input to u, output to uDot.
    void resetTo(float u, float uDot = 0.0f);

    void setDt(float dt)    { _dt = (dt > 1e-6f) ? dt : 1.0f; }
    void setTau(float tau)  { _Tau = tau; }
    void setMethod(DiscretizationMethod method) { _method = method; }
    float dt()  const { return _dt; }
    float Tau() const { return _Tau; }

protected:
    float onStep(float u) override;
    void  onReset() override {}
    void  onInitialize(const nlohmann::json& config) override;
    nlohmann::json onSerializeJson() const override;
    void  onDeserializeJson(const nlohmann::json& state) override;
    int   schemaVersion() const override { return 1; }
    const char* typeName() const override { return "Derivative"; }

private:
    float _dt;
    float _Tau;
    DiscretizationMethod _method;
};

} // namespace liteaerosim::control
