#define _USE_MATH_DEFINES
#include "control/filter_realizations.hpp"
#include <gtest/gtest.h>

using namespace liteaerosim::control;
using namespace liteaerosim;

TEST(FilterRealizationsTest, Tustin2TF200) {

    Vec3 num_s(0,0,1);
    Vec3 den_s(0,10,1);

    Vec3 num_z;
    Vec3 den_z;

    float dt = 0.1;

    tustin_1_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.00497512437810943f, 1e-8);
    EXPECT_NEAR(num_z(1),0.00497512437810943f, 1e-8);
    EXPECT_EQ(num_z(2),0.0f);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-0.9900497512437811f, 1e-8);
    EXPECT_EQ(den_z(2),0.0f);

}

TEST(FilterRealizationsTest, Tustin2TF201) {

    Vec3 num_s(0,0,4.0);
    Vec3 den_s(1.0,2*1.0/sqrt(2)*2.0,4.0);

    Vec3 num_z;
    Vec3 den_z;

    float dt = 0.1;

    tustin_2_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.008684917945832371f, 1e-8);
    EXPECT_NEAR(num_z(1),0.017369835891664742f, 1e-8);
    EXPECT_NEAR(num_z(2),0.00868491794583226f, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-1.7196137532748001f, 1e-8);
    EXPECT_NEAR(den_z(2),0.7543534250581294f, 1e-8);

} 
TEST(FilterRealizationsTest, Tustin2TF202) {

    Vec3 num_s(0,0,1);
    Vec3 den_s(0,10,1);

    Vec3 num_z;
    Vec3 den_z;

    float dt = 0.1;

    tustin_1_tf(num_s, den_s, dt, 2*M_PI/10.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.004976753625925978, 1e-8);
    EXPECT_NEAR(num_z(1),0.004976753625925978, 1e-8);
    EXPECT_EQ(num_z(2),0.0f);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-0.9900464927481482, 1e-6);
    EXPECT_EQ(den_z(2),0.0f);

}

TEST(FilterRealizationsTest, Tustin2TF203) {

    Vec3 num_s(0,0,4.0);
    Vec3 den_s(1.0,2*1.0/sqrt(2)*2.0,4.0);

    Vec3 num_z;
    Vec3 den_z;

    float dt = 0.1;

    tustin_2_tf(num_s, den_s, dt, 2.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.008739046114517035, 1e-8);
    EXPECT_NEAR(num_z(1),0.017478092229033848, 1e-8);
    EXPECT_NEAR(num_z(2),0.00873904611451748, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-1.7186907397711062, 1e-6);
    EXPECT_NEAR(den_z(2),0.7536469242291746, 1e-6);

}

TEST(FilterRealizationsTest, Tustin2TF00) {

    FiltVectorXf num_s;
    FiltVectorXf den_s;
    num_s.resize(1);
    den_s.resize(2);
    num_s << 1;
    den_s << 10, 1;

    FiltVectorXf num_z;
    FiltVectorXf den_z;

    float dt = 0.1;

    tustin_1_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    EXPECT_EQ(num_z.size(),2);
    EXPECT_EQ(den_z.size(),2);
    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.00497512437810943f, 1e-8);
    EXPECT_NEAR(num_z(1),0.00497512437810943f, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-0.9900497512437811f, 1e-8);

}

TEST(FilterRealizationsTest, Tustin2TF01) {

    FiltVectorXf num_s;
    FiltVectorXf den_s;
    num_s.resize(1);
    den_s.resize(3);
    num_s << 4.0;
    den_s << 1.0,2*1.0/sqrt(2)*2.0,4.0;

    FiltVectorXf num_z;
    FiltVectorXf den_z;

    float dt = 0.1;

    tustin_2_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.008684917945832371f, 1e-8);
    EXPECT_NEAR(num_z(1),0.017369835891664742f, 1e-8);
    EXPECT_NEAR(num_z(2),0.00868491794583226f, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-1.7196137532748001f, 1e-8);
    EXPECT_NEAR(den_z(2),0.7543534250581294f, 1e-8);

}

TEST(FilterRealizationsTest, TF2SS200) {

    Vec3 num_s(0,0,1);
    Vec3 den_s(0,10,1);

    Vec3 num_z;
    Vec3 den_z;

    float dt = 0.1;

    tustin_1_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.00497512437810943f, 1e-8);
    EXPECT_NEAR(num_z(1),0.00497512437810943f, 1e-8);
    EXPECT_EQ(num_z(2),0.0f);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-0.9900497512437811f, 1e-8);
    EXPECT_EQ(den_z(2),0.0f);

    Mat22 Phi;
    Mat21 Gamma;
    Mat12 H;
    Mat11 J;

    tf2ss(num_z, den_z, Phi, Gamma, H, J);

    EXPECT_EQ(Phi(0,0),0.0f);
    EXPECT_EQ(Phi(0,1),1.0f);
    EXPECT_EQ(Phi(1,0),0.0f);
    EXPECT_NEAR(Phi(1,1),0.9900497512437811f, 1e-8);

    EXPECT_EQ(Gamma(0,0),0.0f);
    EXPECT_EQ(Gamma(1,0),1.0f);

    EXPECT_EQ(H(0,0),0.0f);
    EXPECT_NEAR(H(0,1),0.00497512437810943f + 0.00497512437810943f*0.9900497512437811f, 1e-8);

    EXPECT_NEAR(J(0,0),0.00497512437810943f, 1e-8);

}

TEST(FilterRealizationsTest, TF2SS201) {

    Vec3 num_s(0,0,4.0);
    Vec3 den_s(1.0,2*1.0/sqrt(2)*2.0,4.0);

    Vec3 num_z;
    Vec3 den_z;

    float dt = 0.1;

    tustin_2_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.008684917945832371f, 1e-8);
    EXPECT_NEAR(num_z(1),0.017369835891664742f, 1e-8);
    EXPECT_NEAR(num_z(2),0.00868491794583226f, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-1.7196137532748001f, 1e-8);
    EXPECT_NEAR(den_z(2),0.7543534250581294f, 1e-8);

    Mat22 Phi;
    Mat21 Gamma;
    Mat12 H;
    Mat11 J;

    tf2ss(num_z, den_z, Phi, Gamma, H, J);

    EXPECT_EQ(Phi(0,0),0.0f);
    EXPECT_EQ(Phi(0,1),1.0f);
    EXPECT_NEAR(Phi(1,0),-0.7543534250581294f, 1e-8);
    EXPECT_NEAR(Phi(1,1),1.7196137532748001f, 1e-8);

    float Ginf = num_z(0)/den_z(0);

    Vec3 tmp_num = num_z - Ginf*den_z;

    EXPECT_EQ(Gamma(0,0),0.0f);
    EXPECT_EQ(Gamma(1,0),1.0f);

    EXPECT_NEAR(H(0,0),tmp_num(2), 1e-8);
    EXPECT_NEAR(H(0,1),tmp_num(1), 1e-8);

    EXPECT_NEAR(J(0,0),0.008684917945832371f, 1e-8);

}

TEST(FilterRealizationsTest, TF2SS00) {

    FiltVectorXf num_s;
    FiltVectorXf den_s;
    num_s.resize(1);
    den_s.resize(2);
    num_s << 1;
    den_s << 10, 1;

    FiltVectorXf num_z;
    FiltVectorXf den_z;

    float dt = 0.1;

    tustin_1_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    EXPECT_EQ(num_z.size(),2);
    EXPECT_EQ(den_z.size(),2);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.00497512437810943f, 1e-8);
    EXPECT_NEAR(num_z(1),0.00497512437810943f, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-0.9900497512437811f, 1e-8);

    MatNN Phi;
    MatN1 Gamma;
    Mat1N H;
    Mat11 J;

    tf2ss(num_z, den_z, Phi, Gamma, H, J);

    EXPECT_EQ(Phi.rows(),1);
    EXPECT_EQ(Phi.cols(),1);

    EXPECT_EQ(Gamma.rows(),1);
    EXPECT_EQ(Gamma.cols(),1);

    EXPECT_EQ(H.rows(),1);
    EXPECT_EQ(H.cols(),1);

    EXPECT_EQ(J.rows(),1);
    EXPECT_EQ(J.cols(),1);

    EXPECT_NEAR(Phi(0,0),0.9900497512437811f, 1e-8);

    EXPECT_EQ(Gamma(0,0),1.0f);

    EXPECT_NEAR(H(0,0),0.00497512437810943f + 0.00497512437810943f*0.9900497512437811f, 1e-8);

    EXPECT_NEAR(J(0,0),0.00497512437810943f, 1e-8);

}

TEST(FilterRealizationsTest, TF2SS01) {

    FiltVectorXf num_s;
    FiltVectorXf den_s;
    num_s.resize(1);
    den_s.resize(3);
    num_s << 4.0;
    den_s << 1.0,2*1.0/sqrt(2)*2.0,4.0;

    FiltVectorXf num_z;
    FiltVectorXf den_z;

    float dt = 0.1;

    tustin_2_tf(num_s, den_s, dt, 0.0f, num_z, den_z);

    EXPECT_EQ(num_z.size(),3);
    EXPECT_EQ(den_z.size(),3);

    // values from python control toolbox
    EXPECT_NEAR(num_z(0),0.008684917945832371f, 1e-8);
    EXPECT_NEAR(num_z(1),0.017369835891664742f, 1e-8);
    EXPECT_NEAR(num_z(2),0.00868491794583226f, 1e-8);
    EXPECT_EQ(den_z(0),1.0f);
    EXPECT_NEAR(den_z(1),-1.7196137532748001f, 1e-8);
    EXPECT_NEAR(den_z(2),0.7543534250581294f, 1e-8);

    MatNN Phi;
    MatN1 Gamma;
    Mat1N H;
    Mat11 J;

    tf2ss(num_z, den_z, Phi, Gamma, H, J);

    EXPECT_EQ(Phi.rows(),2);
    EXPECT_EQ(Phi.cols(),2);

    EXPECT_EQ(Gamma.rows(),2);
    EXPECT_EQ(Gamma.cols(),1);

    EXPECT_EQ(H.rows(),1);
    EXPECT_EQ(H.cols(),2);

    EXPECT_EQ(J.rows(),1);
    EXPECT_EQ(J.cols(),1);

    EXPECT_EQ(Phi(0,0),0.0f);
    EXPECT_EQ(Phi(0,1),1.0f);
    EXPECT_NEAR(Phi(1,0),-0.7543534250581294f, 1e-8);
    EXPECT_NEAR(Phi(1,1),1.7196137532748001f, 1e-8);

    float Ginf = num_z(0)/den_z(0);

    FiltVectorXf tmp_num = num_z - Ginf*den_z;

    EXPECT_EQ(Gamma(0,0),0.0f);
    EXPECT_EQ(Gamma(1,0),1.0f);

    EXPECT_NEAR(H(0,0),tmp_num(2), 1e-8);
    EXPECT_NEAR(H(0,1),tmp_num(1), 1e-8);

    EXPECT_NEAR(J(0,0),0.008684917945832371f, 1e-8);

}

// TEST(FilterSS2Test, SecondOrderLP00) {

//     FilterSS2 G;

//     EXPECT_EQ(G.in(), 0.0f);
//     EXPECT_EQ(G.out(), 0.0f);

//     float dt = 0.1;
//     float wn_rps = 2.0;
//     float zeta = 1.0/sqrt(2.0);
//     float tau_zero = 0.0f;

//     Eigen::Vector3f num_s(0,0,wn_rps*wn_rps);
//     Eigen::Vector3f den_s(1.0,2.0*zeta*wn_rps,wn_rps*wn_rps);

//     Eigen::Vector3f num_z;
//     Eigen::Vector3f den_z;

//     tustin_2_tf(num_s, den_s, dt, num_z, den_z);

//     G.setLowPassSecondIIR(dt, wn_rps, zeta, tau_zero);

//     EXPECT_EQ(G.errorCode(), 0);

//     EXPECT_EQ(G.order(), 2);

//     Mat22 Phi(G.Phi());
//     Mat21 Gamma(G.Gamma());
//     Mat12 H(G.H());
//     Mat11 J(G.J());

//     float Ginf = num_z(0)/den_z(0);

//     Eigen::Vector3f tmp_num = num_z - Ginf*den_z;

//     EXPECT_EQ(Phi(0,0),0.0f);
//     EXPECT_EQ(Phi(0,1),1.0f);
//     EXPECT_NEAR(Phi(1,0),-den_z(2), 1e-8);
//     EXPECT_NEAR(Phi(1,1),-den_z(1), 1e-8);

//     EXPECT_EQ(Gamma(0,0),0.0f);
//     EXPECT_EQ(Gamma(1,0),1.0f);

//     EXPECT_NEAR(H(0,0),tmp_num(2), 1e-8);
//     EXPECT_NEAR(H(0,1),tmp_num(1), 1e-8);

//     EXPECT_NEAR(J(0,0),0.008684917945832371f, 1e-8);

//     EXPECT_NEAR(G.step(1.0f), 0.008684917945832371f, 1e-6);
//     EXPECT_NEAR(G.step(1.0f), 0.04098945818321358f, 1e-6);
//     EXPECT_NEAR(G.step(1.0f), 0.09867421021567828f, 1e-6);
//     EXPECT_NEAR(G.step(1.0f), 0.1735006625919544f, 1e-6);

//     G.resetInput(0.0f);

//     EXPECT_EQ(G.errorCode(), 0);

//     EXPECT_EQ(G.order(), 2);

//     EXPECT_EQ(G.in(), 0.0f);
//     EXPECT_EQ(G.out(), 0.0f);

//     for (int k = 0; k < 20; k++) {
//         G.step(1.0f);
//         EXPECT_EQ(G.errorCode(), 0);
//     }

//     EXPECT_EQ(G.in(), 1.0f);
//     EXPECT_NEAR(G.out(), 1.035730817247945f, 1e-5);
    
// }

TEST(FilterRealizationsTest, TustinNSS00) {

    float dt = 0.1;
    float wn_rps = 2.0;
    float zeta = 1.0/sqrt(2.0);
    float tau_zero = 0.0f;

    FiltVectorXf num_s;
    FiltVectorXf den_s;
    num_s.resize(1);
    den_s.resize(3);
    num_s << wn_rps*wn_rps;
    den_s << 1.0,2.0*zeta*wn_rps,wn_rps*wn_rps;

    MatNN A;
    MatN1 B;
    Mat1N C;
    Mat11 D;

    tf2ss(num_s, den_s, A, B, C, D);

    MatNN Phi;
    MatN1 Gamma;
    Mat1N H;
    Mat11 J;

    tustin_n_ss(A, B, C, D, dt, wn_rps, Phi, Gamma, H, J);

    EXPECT_EQ(Phi.rows(),2);
    EXPECT_EQ(Phi.cols(),2);

    EXPECT_EQ(Gamma.rows(),2);
    EXPECT_EQ(Gamma.cols(),1);

    EXPECT_EQ(H.rows(),1);
    EXPECT_EQ(H.cols(),2);

    EXPECT_EQ(J.rows(),1);
    EXPECT_EQ(J.cols(),1);

    EXPECT_NEAR(Phi(0,0),0.982521907770966, 1e-6);
    EXPECT_NEAR(Phi(0,1),0.087098965221857, 1e-6);
    EXPECT_NEAR(Phi(1,0),-0.348395860887428, 1e-6);
    EXPECT_NEAR(Phi(1,1),0.736168832000141, 1e-6);

    EXPECT_NEAR(Gamma(0,0),0.004369523057259, 1e-8);
    EXPECT_NEAR(Gamma(1,0),0.087098965221857, 1e-8);

    EXPECT_NEAR(H(0,0),3.965043815541932, 1e-6);
    EXPECT_NEAR(H(0,1),0.174197930443714, 1e-8);

    MatNN ImPhi = MatNN::Identity(2,2)-Phi;
    MatNN ImPhiInv = ImPhi.inverse();
 
    EXPECT_NEAR(ImPhiInv(0,0),7.547481857361692, 1e-5);
    EXPECT_NEAR(ImPhiInv(0,1),2.491661105814809, 1e-5);
    EXPECT_NEAR(ImPhiInv(1,0),-9.966644423259234, 1e-5);
    EXPECT_NEAR(ImPhiInv(1,1),0.500000000000001, 1e-5);
 
    EXPECT_NEAR(H.norm() * Gamma.norm(),0.3461190709161926, 1e-5);

    EXPECT_NEAR(J(0,0),0.008739046114517035, 1e-6);

    EXPECT_NEAR((H*ImPhiInv*Gamma).value(),0.991260953885483, 1e-6);

    EXPECT_NEAR(J(0,0),0.008739046114517, 1e-6);
    
}

TEST(FilterRealizationsTest, Tustin2SS00) {

    float dt = 0.1;
    float wn_rps = 2.0;
    float zeta = 1.0/sqrt(2.0);
    float tau_zero = 0.0f;

    Vec3 num_s;
    Vec3 den_s;
    num_s << 0,0,wn_rps*wn_rps;
    den_s << 1.0,2.0*zeta*wn_rps,wn_rps*wn_rps;

    Mat22 A;
    Mat21 B;
    Mat12 C;
    Mat11 D;

    tf2ss(num_s, den_s, A, B, C, D);

    Mat22 Phi;
    Mat21 Gamma;
    Mat12 H;
    Mat11 J;

    tustin_2_ss(A, B, C, D, dt, wn_rps, Phi, Gamma, H, J);

    EXPECT_NEAR(Phi(0,0),0.982521907770966, 1e-6);
    EXPECT_NEAR(Phi(0,1),0.087098965221857, 1e-6);
    EXPECT_NEAR(Phi(1,0),-0.348395860887428, 1e-6);
    EXPECT_NEAR(Phi(1,1),0.736168832000141, 1e-6);

    MatNN ImPhi = MatNN::Identity(2,2)-Phi;
    MatNN ImPhiInv = ImPhi.inverse();
 
    EXPECT_NEAR(ImPhiInv(0,0),7.547481857361692, 1e-5);
    EXPECT_NEAR(ImPhiInv(0,1),2.491661105814809, 1e-5);
    EXPECT_NEAR(ImPhiInv(1,0),-9.966644423259234, 1e-5);
    EXPECT_NEAR(ImPhiInv(1,1),0.500000000000001, 1e-5);
 
    EXPECT_NEAR(H.norm() * Gamma.norm(),0.3461190709161926, 1e-5);

    EXPECT_NEAR(J(0,0),0.008739046114517035, 1e-6);

    EXPECT_NEAR((H*ImPhiInv*Gamma).value(),0.991260953885483, 1e-6);

    EXPECT_NEAR(J(0,0),0.008739046114517, 1e-6);
    
}
