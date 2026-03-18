#define _USE_MATH_DEFINES
#include "control/FilterSS2.hpp"
#include "control/filter_realizations.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace liteaerosim::control;
using namespace liteaerosim;

// ---------------------------------------------------------------------------
// Default-constructed state
// ---------------------------------------------------------------------------

TEST(FilterSS2Test, Instantiation00) {
    FilterSS2 G;
    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);
    EXPECT_EQ(G.order(), 0);
}

TEST(FilterSS2Test, Update00) {
    FilterSS2 G;
    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);
    EXPECT_EQ(G.order(), 0);
    // Default filter has J = 1, H = 0 — passes input unchanged
    EXPECT_EQ(G.step(1.0f), 1.0f);
}

// ---------------------------------------------------------------------------
// First-order low-pass — stepping and resetToInput
// ---------------------------------------------------------------------------

TEST(FilterSS2Test, FirstOrderLP00) {
    FilterSS2 G;
    float dt  = 0.1f;
    float tau = 10.0f;

    G.initialize({{"dt_s", dt}, {"design", "low_pass_first"}, {"tau_s", tau}});

    EXPECT_EQ(G.errorCode(), 0);
    EXPECT_EQ(G.order(), 1);

    EXPECT_NEAR(G.step(1.0f), 0.00497512437810943f,  1e-3f);
    EXPECT_NEAR(G.step(1.0f), 0.014875869409173084f, 1e-3f);
    EXPECT_NEAR(G.step(1.0f), 0.024678099564305757f, 1e-3f);
    EXPECT_NEAR(G.step(1.0f), 0.03438279509102915f,  1e-3f);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);
    EXPECT_EQ(G.order(), 1);
    EXPECT_EQ(G.in(),  0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    for (int k = 0; k < 100; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 0.6302749995213918f, 1e-3f);
}

// ---------------------------------------------------------------------------
// Second-order low-pass — stepping, matrix inspection, resetToInput
// ---------------------------------------------------------------------------

TEST(FilterSS2Test, SecondOrderLP00) {
    FilterSS2 G;
    float dt       = 0.1f;
    float wn_rps   = 2.0f;
    float zeta     = 1.0f / sqrtf(2.0f);
    float tau_zero = 0.0f;

    Eigen::Vector3f num_s(0.0f, 0.0f, wn_rps * wn_rps);
    Eigen::Vector3f den_s(1.0f, 2.0f * zeta * wn_rps, wn_rps * wn_rps);
    Eigen::Vector3f num_z, den_z;
    tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    G.initialize({{"dt_s", dt}, {"design", "low_pass_second"},
                  {"wn_rad_s", wn_rps}, {"zeta", zeta}, {"tau_zero_s", tau_zero}});

    EXPECT_EQ(G.errorCode(), 0);
    EXPECT_EQ(G.order(), 2);

    Mat22 Phi(G.Phi());
    Mat21 Gamma(G.Gamma());
    Mat12 H(G.H());
    Mat11 J(G.J());

    float           Ginf    = num_z(0) / den_z(0);
    Eigen::Vector3f tmp_num = num_z - Ginf * den_z;

    EXPECT_EQ(Phi(0, 0), 0.0f);
    EXPECT_EQ(Phi(0, 1), 1.0f);
    EXPECT_NEAR(Phi(1, 0), -den_z(2), 1e-8f);
    EXPECT_NEAR(Phi(1, 1), -den_z(1), 1e-8f);

    EXPECT_EQ(Gamma(0, 0), 0.0f);
    EXPECT_EQ(Gamma(1, 0), 1.0f);

    EXPECT_NEAR(H(0, 0), tmp_num(2), 1e-8f);
    EXPECT_NEAR(H(0, 1), tmp_num(1), 1e-8f);

    EXPECT_NEAR(J(0, 0), 0.008739046114517035, 1e-8);

    EXPECT_NEAR(G.step(1.0f), 0.008739046114517035, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.041236855975003976, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.09924343173467982,  1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.1744469218931253,   1e-6);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);
    EXPECT_EQ(G.order(), 2);
    EXPECT_EQ(G.in(),  0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    for (int k = 0; k < 20; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 1.0361699725173787, 1e-6);
}

// ---------------------------------------------------------------------------
// DynamicElement lifecycle — reset()
// ---------------------------------------------------------------------------

TEST(FilterSS2Test, DynamicElementReset) {
    FilterSS2 G;
    G.initialize({{"dt_s", 0.1f}, {"design", "low_pass_first"}, {"tau_s", 10.0f}});
    G.step(1.0f);
    G.step(1.0f);
    G.reset();
    EXPECT_EQ(G.in(),  0.0f);
    EXPECT_EQ(G.out(), 0.0f);
}

// ---------------------------------------------------------------------------
// Serialization — round-trip fidelity
// ---------------------------------------------------------------------------

TEST(FilterSS2Test, SerializeRoundTrip) {
    FilterSS2 G;
    G.initialize({{"dt_s", 0.1f}, {"design", "low_pass_first"}, {"tau_s", 10.0f}});

    G.step(1.0f);
    G.step(1.0f);
    G.step(1.0f);

    const float out_before = G.out();
    const float in_before  = G.in();

    nlohmann::json snapshot = G.serializeJson();

    EXPECT_EQ(snapshot.at("schema_version").get<int>(), 1);
    EXPECT_EQ(snapshot.at("type").get<std::string>(), "FilterSS2");
    EXPECT_FLOAT_EQ(snapshot.at("in").get<float>(),  in_before);
    EXPECT_FLOAT_EQ(snapshot.at("out").get<float>(), out_before);

    FilterSS2 G2;
    G2.deserializeJson(snapshot);

    EXPECT_FLOAT_EQ(G2.in(),  in_before);
    EXPECT_FLOAT_EQ(G2.out(), out_before);

    EXPECT_FLOAT_EQ(G.step(1.0f), G2.step(1.0f));
    EXPECT_FLOAT_EQ(G.step(1.0f), G2.step(1.0f));
    EXPECT_FLOAT_EQ(G.step(1.0f), G2.step(1.0f));
}

// ---------------------------------------------------------------------------
// Serialization — schema version mismatch throws
// ---------------------------------------------------------------------------

TEST(FilterSS2Test, SerializeSchemaVersionReject) {
    FilterSS2 G;
    G.initialize({{"dt_s", 0.1f}, {"design", "low_pass_first"}, {"tau_s", 10.0f}});

    nlohmann::json snapshot = G.serializeJson();
    snapshot["schema_version"] = 999;

    EXPECT_THROW(G.deserializeJson(snapshot), std::runtime_error);
}
