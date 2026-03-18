#pragma once

#include "control/Filter.hpp"
#include "control/control.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

namespace liteaerosim::control {

/// Second-order (or first-order) state-space IIR filter.
///
/// Configured via initialize() with a JSON design specification.
/// Supports low-pass, high-pass, notch, and derivative filter types.
///
/// Config format:
///   {"dt_s": 0.01, "design": "low_pass_first", "tau_s": 0.5}
///   {"dt_s": 0.01, "design": "low_pass_second", "wn_rad_s": 10.0, "zeta": 0.707, "tau_zero_s": 0.0}
///   {"dt_s": 0.01, "design": "high_pass_first",  "tau_s": 0.5}
///   {"dt_s": 0.01, "design": "high_pass_second", "wn_rad_s": 10.0, "zeta": 0.707, "c_zero": 0.0}
///   {"dt_s": 0.01, "design": "deriv",             "tau_s": 0.05}
///   {"dt_s": 0.01, "design": "notch_second",      "wn_rad_s": 10.0, "zeta_den": 0.05, "zeta_num": 0.7}
class FilterSS2 : public Filter {
public:
    FilterSS2();
    explicit FilterSS2(FilterSS2& filt);
    ~FilterSS2() override = default;

    /// Copy all state and parameters from another FilterSS2.
    void copy(FilterSS2& filt);

    // -----------------------------------------------------------------------
    // Filter interface
    // -----------------------------------------------------------------------
    uint8_t  order()                        const override { return order_; }
    float    dcGain()                       const override;
    void     resetToInput(float in_val)           override;
    void     resetToOutput(float out_val)         override;
    uint16_t errorCode()                    const override { return error_code_; }

    // -----------------------------------------------------------------------
    // State-space matrix accessors (read-only)
    // -----------------------------------------------------------------------
    Mat22 Phi()   const { return phi_; }
    Mat21 Gamma() const { return gamma_; }
    Mat12 H()     const { return h_; }
    Mat11 J()     const { return j_; }
    Mat21 x()     const { return x_; }

    Mat22 controlGrammian() const;
    Mat22 observeGrammian() const;

protected:
    // -----------------------------------------------------------------------
    // DynamicElement / SisoElement hooks
    // -----------------------------------------------------------------------
    void           onInitialize(const nlohmann::json& config)        override;
    void           onReset()                                         override;
    float          onStep(float u)                                   override;
    nlohmann::json onSerializeJson()                           const override;
    void           onDeserializeJson(const nlohmann::json& state)    override;
    void           onLog(liteaerosim::ILogger& logger)         const override;
    int            schemaVersion()                             const override { return 1; }
    const char*    typeName()                                  const override { return "FilterSS2"; }

private:
    // -----------------------------------------------------------------------
    // Filter design methods (called from onInitialize)
    // -----------------------------------------------------------------------
    void designLowPassFirst  (float dt_s, float tau_s);
    void designLowPassSecond (float dt_s, float wn_rad_s, float zeta, float tau_zero_s);
    void designHighPassFirst (float dt_s, float tau_s);
    void designHighPassSecond(float dt_s, float wn_rad_s, float zeta, float c_zero);
    void designDerivative    (float dt_s, float tau_s);
    void designNotchSecond   (float dt_s, float wn_rad_s, float zeta_den, float zeta_num);

    // -----------------------------------------------------------------------
    // State-space matrices and state vector
    // -----------------------------------------------------------------------
    Mat22 phi_;
    Mat21 gamma_;
    Mat12 h_;
    Mat11 j_;
    Mat21 x_;

    uint8_t  order_      = 0;
    uint16_t error_code_ = 0;

    nlohmann::json params_;  ///< Stored config JSON for serialization round-trip.
};

}  // namespace liteaerosim::control
