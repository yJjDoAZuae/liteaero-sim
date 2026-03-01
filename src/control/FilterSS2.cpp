#define _USE_MATH_DEFINES
#include <cmath>
#include <stdexcept>
#include <string>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS2.hpp"

using namespace liteaerosim::control;

namespace liteaerosim::control {

static constexpr float kDcTol               = 1e-6f;
static constexpr float kInvertibleThreshold = 1e-4f;

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

FilterSS2::FilterSS2() {
    phi_.setZero();
    gamma_.setZero();
    h_.setZero();
    j_.setOnes();
    x_.setZero();
}

FilterSS2::FilterSS2(FilterSS2& filt) {
    copy(filt);
}

void FilterSS2::copy(FilterSS2& filt) {
    phi_        = filt.phi_;
    gamma_      = filt.gamma_;
    h_          = filt.h_;
    j_          = filt.j_;
    x_          = filt.x_;
    in_         = filt.in_;
    out_        = filt.out_;
    error_code_ = filt.error_code_;
    order_      = filt.order_;
    params_     = filt.params_;
}

// ---------------------------------------------------------------------------
// DynamicBlock hooks
// ---------------------------------------------------------------------------

void FilterSS2::onInitialize(const nlohmann::json& config) {
    error_code_ = 0;
    params_     = config;

    const float       dt_s   = config.at("dt_s").get<float>();
    const std::string design = config.at("design").get<std::string>();

    if (design == "low_pass_first") {
        designLowPassFirst(dt_s, config.at("tau_s").get<float>());
    } else if (design == "low_pass_second") {
        designLowPassSecond(dt_s,
                            config.at("wn_rad_s").get<float>(),
                            config.at("zeta").get<float>(),
                            config.at("tau_zero_s").get<float>());
    } else if (design == "high_pass_first") {
        designHighPassFirst(dt_s, config.at("tau_s").get<float>());
    } else if (design == "high_pass_second") {
        designHighPassSecond(dt_s,
                             config.at("wn_rad_s").get<float>(),
                             config.at("zeta").get<float>(),
                             config.at("c_zero").get<float>());
    } else if (design == "deriv") {
        designDerivative(dt_s, config.at("tau_s").get<float>());
    } else if (design == "notch_second") {
        designNotchSecond(dt_s,
                          config.at("wn_rad_s").get<float>(),
                          config.at("zeta_den").get<float>(),
                          config.at("zeta_num").get<float>());
    } else {
        throw std::invalid_argument("FilterSS2::onInitialize: unknown design key \"" + design + "\"");
    }
}

void FilterSS2::onReset() {
    x_.setZero();
}

float FilterSS2::onStep(float u) {
    // Output is based on current state (strictly proper or proper).
    const float y = (h_ * x_ + j_ * u).value();
    x_            = phi_ * x_ + gamma_ * u;
    return y;
}

nlohmann::json FilterSS2::onSerialize() const {
    return {
        {"state",  {{"x0", x_(0)}, {"x1", x_(1)}}},
        {"params", params_}
    };
}

void FilterSS2::onDeserialize(const nlohmann::json& state) {
    onInitialize(state.at("params"));
    const nlohmann::json& s = state.at("state");
    x_(0) = s.at("x0").get<float>();
    x_(1) = s.at("x1").get<float>();
}

void FilterSS2::onLog(liteaerosim::ILogger& logger) const {
    logger.log("FilterSS2.in",  in_);
    logger.log("FilterSS2.out", out_);
    logger.log("FilterSS2.x0",  x_(0));
    logger.log("FilterSS2.x1",  x_(1));
}

// ---------------------------------------------------------------------------
// Filter-specific query and reset
// ---------------------------------------------------------------------------

float FilterSS2::dcGain() const {
    Mat22 ImPhiInv;
    bool  invertible = false;

    (Mat22::Identity() - phi_).computeInverseWithCheck(ImPhiInv, invertible, kInvertibleThreshold);

    if (!invertible) {
        return 1.0f;  // integrator-like; caller must handle
    }

    return (h_ * ImPhiInv * gamma_ + j_).value();
}

void FilterSS2::resetToInput(float in_val) {
    x_.setZero();
    in_  = 0.0f;
    out_ = 0.0f;

    if (error_code_ != 0) {
        return;
    }

    Mat22 ImPhiInv;
    bool  invertible = false;

    (Mat22::Identity() - phi_).computeInverseWithCheck(ImPhiInv, invertible, kInvertibleThreshold);

    if (!invertible) {
        error_code_ += FilterError::UNSTABLE;
        return;
    }

    in_  = in_val;
    out_ = dcGain() * in_val;
    x_   = ImPhiInv * gamma_ * in_val;
}

void FilterSS2::resetToOutput(float out_val) {
    x_.setZero();
    in_  = 0.0f;
    out_ = 0.0f;

    const float dc = dcGain();

    if (std::fabs(dc) < kDcTol) {
        error_code_ += FilterError::ZERO_DC_GAIN;
        return;
    }

    if (error_code_ != 0) {
        return;
    }

    Mat22 ImPhiInv;
    bool  invertible = false;

    (Mat22::Identity() - phi_).computeInverseWithCheck(ImPhiInv, invertible, kInvertibleThreshold);

    if (!invertible) {
        error_code_ += FilterError::UNSTABLE;
        return;
    }

    out_ = out_val;
    in_  = out_val / dc;
    x_   = ImPhiInv * gamma_ * in_;
}

// ---------------------------------------------------------------------------
// Grammians
// ---------------------------------------------------------------------------

Mat22 FilterSS2::controlGrammian() const {
    Mat22 C(Mat22::Zero(2, 2));
    for (int k = 0; k < 2; ++k) {
        C(Eigen::all, k) << Mat21(phi_.pow(k) * gamma_);
    }
    return C;
}

Mat22 FilterSS2::observeGrammian() const {
    Mat22 C(Mat22::Zero(2, 2));
    for (int k = 0; k < 2; ++k) {
        C(k, Eigen::all) << Mat12(h_ * phi_.pow(k));
    }
    return C;
}

// ---------------------------------------------------------------------------
// Private design methods
// ---------------------------------------------------------------------------

void FilterSS2::designLowPassFirst(float dt_s, float tau_s) {
    Eigen::Vector3f num_s, den_s, num_z, den_z;
    num_s << 0.0f, 0.0f, 1.0f / tau_s;
    den_s << 0.0f, 1.0f, 1.0f / tau_s;

    error_code_ += tustin_1_tf(num_s, den_s, dt_s, 2.0f * static_cast<float>(M_PI) / tau_s, num_z, den_z);
    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 1;
}

void FilterSS2::designLowPassSecond(float dt_s, float wn_rad_s, float zeta, float tau_zero_s) {
    Eigen::Vector3f num_s, den_s, num_z, den_z;
    num_s << 0.0f, tau_zero_s * wn_rad_s * wn_rad_s, wn_rad_s * wn_rad_s;
    den_s << 1.0f, 2.0f * zeta * wn_rad_s,           wn_rad_s * wn_rad_s;

    error_code_ += tustin_2_tf(num_s, den_s, dt_s, wn_rad_s, num_z, den_z);
    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 2;
}

void FilterSS2::designHighPassFirst(float dt_s, float tau_s) {
    Eigen::Vector3f num_s, den_s, num_z, den_z;
    num_s << 0.0f, 1.0f / tau_s, 0.0f;
    den_s << 0.0f, 1.0f,         1.0f / tau_s;

    error_code_ += tustin_2_tf(num_s, den_s, dt_s, 2.0f * static_cast<float>(M_PI) / tau_s, num_z, den_z);
    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 1;
}

void FilterSS2::designHighPassSecond(float dt_s, float wn_rad_s, float zeta, float c_zero) {
    Eigen::Vector3f num_s, den_s, num_z, den_z;
    num_s << 1.0f, c_zero * 2.0f * zeta * wn_rad_s, 0.0f;
    den_s << 1.0f, 2.0f * zeta * wn_rad_s,           wn_rad_s * wn_rad_s;

    error_code_ += tustin_2_tf(num_s, den_s, dt_s, wn_rad_s, num_z, den_z);
    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 2;
}

void FilterSS2::designDerivative(float dt_s, float tau_s) {
    // s/tau / (s + 1/tau) — same polynomial structure as high-pass first
    Eigen::Vector3f num_s, den_s, num_z, den_z;
    num_s << 0.0f, 1.0f / tau_s, 0.0f;
    den_s << 0.0f, 1.0f,         1.0f / tau_s;

    error_code_ += tustin_2_tf(num_s, den_s, dt_s, 2.0f * static_cast<float>(M_PI) / tau_s, num_z, den_z);
    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 1;
}

void FilterSS2::designNotchSecond(float dt_s, float wn_rad_s, float zeta_den, float zeta_num) {
    Eigen::Vector3f num_s, den_s, num_z, den_z;
    num_s << 1.0f, 2.0f * zeta_num * wn_rad_s, wn_rad_s * wn_rad_s;
    den_s << 1.0f, 2.0f * zeta_den * wn_rad_s, wn_rad_s * wn_rad_s;

    error_code_ += tustin_2_tf(num_s, den_s, dt_s, wn_rad_s, num_z, den_z);
    tf2ss(num_z, den_z, phi_, gamma_, h_, j_);
    order_ = 2;
}

}  // namespace liteaerosim::control
