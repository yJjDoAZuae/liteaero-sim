#define _USE_MATH_DEFINES
#include "control/FilterSS2Clip.hpp"
#include "control/filter_realizations.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace liteaerosim::control;
using namespace liteaerosim;

TEST(FilterSS2ClipTest, Instantiation00) {

    FilterSS2Clip G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    EXPECT_EQ(G.order(), 0);

}

TEST(FilterSS2ClipTest, Update00) {

    FilterSS2Clip G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    EXPECT_EQ(G.order(), 0);

    EXPECT_EQ(G.step(1.0f), 1.0f);
}

TEST(FilterSS2ClipTest, FirstOrderLP00) {

    FilterSS2Clip G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float tau = 10;

    G.setLowPassFirstIIR(dt, tau);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 1);

    EXPECT_NEAR(G.step(1.0f), 0.00497512437810943f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.014875869409173084f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.024678099564305757f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.03438279509102915f, 1e-3);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 1);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    ofstream outFile;

    outFile.open("FilterSS2ClipTest_FirstOrderLP00.csv");

    outFile << std::setprecision(6);

    outFile << "step, time, in, out" << endl;

    outFile << 0 << ", " << 0 << ", " << G.in() << ", " << G.out() << endl;
    for (int k = 0; k < 100; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
        EXPECT_LE(G.out(), 1.01);  // check for overshoot
        outFile << k << ", " << (k+1)*dt << ", " << G.in() << ", " << G.out() << endl;
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 0.6302749995213918f, 1e-3);  // check first time constant response

    for (int k = 100; k < 500; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
        EXPECT_LE(G.out(), 1.01);  // check for overshoot
        outFile << k << ", " << (k+1)*dt << ", " << G.in() << ", " << G.out() << endl;
    }

    outFile.close();

}

TEST(FilterSS2ClipTest, SecondOrderLP00) {

    FilterSS2Clip G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float wn_rps = 2.0;
    float zeta = 1.0/sqrt(2.0);
    float tau_zero = 0.0f;

    Eigen::Vector3f num_s(0,0,wn_rps*wn_rps);
    Eigen::Vector3f den_s(1.0,2.0*zeta*wn_rps,wn_rps*wn_rps);

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    G.setLowPassSecondIIR(dt, wn_rps, zeta, tau_zero);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    Mat22 Phi(G.Phi());
    Mat21 Gamma(G.Gamma());
    Mat12 H(G.H());
    Mat11 J(G.J());

    float Ginf = num_z(0)/den_z(0);

    Eigen::Vector3f tmp_num = num_z - Ginf*den_z;

    EXPECT_EQ(Phi(0,0),0.0f);
    EXPECT_EQ(Phi(0,1),1.0f);
    EXPECT_NEAR(Phi(1,0),-den_z(2), 1e-8);
    EXPECT_NEAR(Phi(1,1),-den_z(1), 1e-8);

    EXPECT_EQ(Gamma(0,0),0.0f);
    EXPECT_EQ(Gamma(1,0),1.0f);

    EXPECT_NEAR(H(0,0),tmp_num(2), 1e-8);
    EXPECT_NEAR(H(0,1),tmp_num(1), 1e-8);

    EXPECT_NEAR(J(0,0),0.008739046114517035, 1e-8);

    EXPECT_NEAR(G.step(1.0f), 0.008739046114517035, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.041236855975003976, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.09924343173467982, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.1744469218931253, 1e-6);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);
    
    ofstream outFile;

    outFile.open("FilterSS2ClipTest_SecondOrderLP00.csv");

    outFile << std::setprecision(6);

    outFile << "step, time, in, out" << endl;

    outFile << 0 << ", " << 0 << ", " << G.in() << ", " << G.out() << endl;
    for (int k = 0; k < 20; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
        EXPECT_LE(G.out(), 1.06);  // check for overshoot
        outFile << k << ", " << (k+1)*dt << ", " << G.in() << ", " << G.out() << endl;
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 1.0361699725173787, 1e-6);

    for (int k = 20; k < 50; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
        EXPECT_LE(G.out(), 1.06);  // check for overshoot
        outFile << k << ", " << (k+1)*dt << ", " << G.in() << ", " << G.out() << endl;
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 1.0, 1e-2);

    outFile.close();
    


}


TEST(FilterSS2ClipTest, FirstOrderLP01) {

    FilterSS2Clip G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float tau = 10;

    G.setLowPassFirstIIR(dt, tau);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 1);

    EXPECT_NEAR(G.step(1.0f), 0.00497512437810943f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.014875869409173084f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.024678099564305757f, 1e-3);
    EXPECT_NEAR(G.step(1.0f), 0.03438279509102915f, 1e-3);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 1);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    G.valLimit.setUpper(0.9);
    G.valLimit.enable();
    G.rateLimit.setUpper(0.05f);
    G.rateLimit.enable();
    
    ofstream outFile;

    outFile.open("FilterSS2ClipTest_FirstOrderLP01.csv");

    outFile << std::setprecision(6);

    outFile << "step, time, in, out" << endl;

    outFile << 0 << ", " << 0 << ", " << G.in() << ", " << G.out() << endl;
    for (int k = 0; k < 500; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
        EXPECT_LE(G.out(), 1.0);  // check for overshoot
        outFile << k << ", " << (k+1)*dt << ", " << G.in() << ", " << G.out() << endl;
    }
    
    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 0.9, 1e-2);

    outFile.close();

}

TEST(FilterSS2ClipTest, SecondOrderLP01) {

    FilterSS2Clip G;

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    float dt = 0.1;
    float wn_rps = 2.0;
    float zeta = 1.0/sqrt(2.0);
    float tau_zero = 0.0f;

    Eigen::Vector3f num_s(0,0,wn_rps*wn_rps);
    Eigen::Vector3f den_s(1.0,2.0*zeta*wn_rps,wn_rps*wn_rps);

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    G.setLowPassSecondIIR(dt, wn_rps, zeta, tau_zero);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    Mat22 Phi(G.Phi());
    Mat21 Gamma(G.Gamma());
    Mat12 H(G.H());
    Mat11 J(G.J());

    float Ginf = num_z(0)/den_z(0);

    Eigen::Vector3f tmp_num = num_z - Ginf*den_z;

    EXPECT_EQ(Phi(0,0),0.0f);
    EXPECT_EQ(Phi(0,1),1.0f);
    EXPECT_NEAR(Phi(1,0),-den_z(2), 1e-8);
    EXPECT_NEAR(Phi(1,1),-den_z(1), 1e-8);

    EXPECT_EQ(Gamma(0,0),0.0f);
    EXPECT_EQ(Gamma(1,0),1.0f);

    EXPECT_NEAR(H(0,0),tmp_num(2), 1e-8);
    EXPECT_NEAR(H(0,1),tmp_num(1), 1e-8);

    EXPECT_NEAR(J(0,0),0.008739046114517035, 1e-8);

    EXPECT_NEAR(G.step(1.0f), 0.008739046114517035, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.041236855975003976, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.09924343173467982, 1e-6);
    EXPECT_NEAR(G.step(1.0f), 0.1744469218931253, 1e-6);

    G.resetToInput(0.0f);

    EXPECT_EQ(G.errorCode(), 0);

    EXPECT_EQ(G.order(), 2);

    EXPECT_EQ(G.in(), 0.0f);
    EXPECT_EQ(G.out(), 0.0f);

    G.valLimit.setUpper(0.9);
    G.valLimit.enable();
    G.rateLimit.setUpper(0.5f);
    G.rateLimit.enable();

    ofstream outFile;

    outFile.open("FilterSS2ClipTest_SecondOrderLP01.csv");

    outFile << std::setprecision(6);

    outFile << "step, time, in, out" << endl;

    outFile << 0 << ", " << 0 << ", " << G.in() << ", " << G.out() << endl;
    for (int k = 0; k < 50; k++) {
        G.step(1.0f);
        EXPECT_EQ(G.errorCode(), 0);
        EXPECT_LE(G.out(), 1.0);  // check for overshoot
        outFile << k << ", " << (k+1)*dt << ", " << G.in() << ", " << G.out() << endl;
    }

    EXPECT_EQ(G.in(), 1.0f);
    EXPECT_NEAR(G.out(), 0.9, 1e-2);

    outFile.close();
    
}

