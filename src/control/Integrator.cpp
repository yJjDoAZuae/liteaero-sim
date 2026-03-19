#include "control/Integrator.hpp"

using namespace liteaerosim::control;

float Integrator::onStep(float u)
{
    bool awActive = false;
    for (size_t k = 0; k < aw.size(); k++) {
        awActive |= aw.at(k).isActive();
    }

    float next;
    if (!awActive) {
        switch (_method) {
        case DiscretizationMethod::BackEuler:
            next = limit.step(out_ + u * _dt);
            break;
        case DiscretizationMethod::FwdEuler:
            next = limit.step(out_ + in_ * _dt);
            break;
        case DiscretizationMethod::Bilinear:
            next = limit.step(out_ + 0.5f * (u + in_) * _dt);
            break;
        default:
            next = out_;
            break;
        }
    } else {
        next = limit.step(out_);
    }

    return next;
}

void Integrator::resetTo(float u)
{
    out_ = limit.step(u);
    in_  = u;
}

void Integrator::onReset()
{
    // in_ and out_ already zeroed by SisoElement::reset()
}

void Integrator::onInitialize(const nlohmann::json& config)
{
    _dt     = config.at("dt_s").get<float>();
    _method = static_cast<DiscretizationMethod>(config.at("method").get<int>());
}

nlohmann::json Integrator::onSerializeJson() const
{
    nlohmann::json aw_array = nlohmann::json::array();
    for (const auto& detector : aw) {
        aw_array.push_back(detector.serializeJson());
    }
    return {
        {"in",         in_},
        {"out",        out_},
        {"dt_s",       _dt},
        {"method",     static_cast<int>(_method)},
        {"antiwindup", aw_array}
    };
}

void Integrator::onDeserializeJson(const nlohmann::json& state)
{
    in_     = state.at("in").get<float>();
    out_    = state.at("out").get<float>();
    _dt     = state.at("dt_s").get<float>();
    _method = static_cast<DiscretizationMethod>(state.at("method").get<int>());

    if (state.contains("antiwindup")) {
        const auto& aw_array = state.at("antiwindup");
        aw.resize(aw_array.size());
        for (size_t k = 0; k < aw_array.size(); ++k) {
            aw[k].deserializeJson(aw_array[k]);
        }
    }
}
