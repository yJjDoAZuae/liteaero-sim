#include "control/Derivative.hpp"

using namespace liteaerosim::control;

float Derivative::onStep(float u)
{
    _Tau = (_Tau > 4.0f * _dt) ? _Tau : 4.0f * _dt;

    float next;
    switch (_method) {
    case DiscretizationMethod::BackEuler:
        next = limit.step(_Tau / (_Tau + _dt) * out_ + (u - in_) / (_Tau + _dt));
        break;
    case DiscretizationMethod::FwdEuler:
        next = limit.step(out_ * (_Tau - _dt) / _Tau + (u - in_) / _Tau);
        break;
    case DiscretizationMethod::Bilinear:
        next = limit.step(out_ * (2.0f * _Tau - _dt) / (2.0f * _Tau + _dt)
                          + 0.5f * (u + in_) * 2.0f / (2.0f * _Tau + _dt));
        break;
    default:
        next = out_;
        break;
    }

    return next;
}

void Derivative::resetTo(float u, float uDot)
{
    out_ = limit.step(uDot);
    in_  = u;
}

void Derivative::onInitialize(const nlohmann::json& config)
{
    _dt     = config.at("dt_s").get<float>();
    _Tau    = config.at("tau_s").get<float>();
    _method = static_cast<DiscretizationMethod>(config.at("method").get<int>());
}

nlohmann::json Derivative::onSerializeJson() const
{
    return {
        {"in",     in_},
        {"out",    out_},
        {"dt_s",   _dt},
        {"tau_s",  _Tau},
        {"method", static_cast<int>(_method)}
    };
}

void Derivative::onDeserializeJson(const nlohmann::json& state)
{
    in_     = state.at("in").get<float>();
    out_    = state.at("out").get<float>();
    _dt     = state.at("dt_s").get<float>();
    _Tau    = state.at("tau_s").get<float>();
    _method = static_cast<DiscretizationMethod>(state.at("method").get<int>());
}
