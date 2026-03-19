#include "control/Unwrap.hpp"
#include "math/math_util.hpp"

using namespace liteaerosim::control;

float Unwrap::onStep(float u)
{
    return out_ + MathUtil::wrapToPi(u - out_);
}

float Unwrap::step(float u, float ref)
{
    in_  = u;
    out_ = ref + MathUtil::wrapToPi(u - ref);
    return out_;
}

nlohmann::json Unwrap::onSerializeJson() const
{
    return {
        {"in",  in_},
        {"out", out_}
    };
}

void Unwrap::onDeserializeJson(const nlohmann::json& state)
{
    in_  = state.at("in").get<float>();
    out_ = state.at("out").get<float>();
}
