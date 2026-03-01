#define _USE_MATH_DEFINES

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include <cmath>

using namespace liteaerosim::control;

namespace liteaerosim::control {

// https://en.wikipedia.org/wiki/Butterworth_filter#Normalized_Butterworth_polynomials
FilterError butter(uint8_t order, float wn_rps, FiltVectorXf& num_s, FiltVectorXf& den_s)
{
    if (order < 1 || order > 10 || order > kFilterMaxStates) {
        return FilterError::INVALID_DIMENSION;
    }

    num_s.resize(1);
    den_s.resize(order+1);

    num_s << 1;
    float a = 1/wn_rps;
    switch (order) {
        case 0:
            // DC pass through
            den_s << 1;
            break;

        case 1:
            den_s << 1, 1;
            break;
        case 2:
            den_s << 1, 2.0/sqrt(2.0), 1;
            break;
        case 3:
            den_s << 1, 2, 2, 1;
            break;
        case 4:
            den_s << 1, 2.6131, 3.4142, 2.6131, 1;
            break;
        case 5:
            den_s << 1, 3.2361, 5.2361, 5.2361, 3.2361, 1;
            break;
        case 6:
            den_s << 1, 3.8637, 7.4641, 9.1416, 7.4641, 3.8637, 1;
            break;
        case 7:
            den_s << 1, 4.4940, 10.0978, 14.5918, 14.5918, 10.0978, 4.4940, 1;
            break;
        case 8:
            den_s << 1, 5.1258, 13.1371, 21.8462, 25.6884, 21.8462, 13.1371, 5.1258, 1;
            break;
        case 9:
            den_s << 1, 5.7588, 16.5817, 31.1634, 41.9864, 41.9864, 31.1634, 16.5817, 5.7588, 1;
            break;
        case 10:
            den_s << 1, 6.3925, 20.4317, 42.8021, 64.8824, 74.2334, 64.8824, 42.8021, 20.4317, 6.3925, 1;
            break;
    }

    // update the coefficients for wn_rps
    for (int k = 0; k < den_s.size(); k++) {
        den_s(k) *= 1/pow(wn_rps, order-k);
    }

    return FilterError::NONE;
}

void butterz_pzk(uint8_t order, uint8_t k, float dt, float wn_rps, float& pz_Re, float& pz_Im, float& zz_Re, float& zz_Im, bool& is_conjugate)
{

    float f_c = wn_rps/(2.0f*M_PI);
    float f_s = 1.0f/dt;

    float theta_c = 2.0f*M_PI*(f_c/f_s);

    // prewarping parameter
    float c = 1.0f/std::tan(theta_c/2.0f);

    // If the filter has an odd number of poles, then 
    // there will be a single real axis pole
    // and all the others will be complex conjugate pairs.
    // With the pole indexing in Nguyen's paper this real pole will be
    // at k = (n-1)/2
    // If the filter has an even number of poles, then
    // the last pole we need to define (given that they are all conjugate pairs)
    // is k = n/2 - 1
    // With either even or odd pole count, there will be
    // n/2 conjugate pairs (with integer division) and n%2 real poles.
    bool isOdd = order % 2 > 0;

    // number of conjugate pairs
    uint8_t n_conj = order/2;

    is_conjugate = (k == n_conj + 1 && isOdd);

    // k = 0...n-1 for stable poles
    float theta_k = M_PI/2.0f + M_PI/(2.0*order) + k*M_PI/order;

    // continuous time pole locations (no continuous time zeros)
    float s_k_Re = cos(theta_k);
    float s_k_Im = sin(theta_k);

    // kth pole locations in the z domain
    pz_Re = cos(theta_c) / (1-cos(theta_k) * sin(theta_c));
    pz_Im = sin(theta_k)*sin(theta_c) / (1-cos(theta_k) * sin(theta_c));

    // kth zero location in the z domain
    zz_Re = -1;
    zz_Im = 0;

}

// https://en.wikipedia.org/wiki/Butterworth_filter#Normalized_Butterworth_polynomials
// https://www.researchgate.net/publication/358716710_The_Transfer_Function_of_the_nth-Order_Digital_Butterworth_Low_Pass_Filter
// NOTE: excessive state dimension in a single transfer function can lead to numerical problems
FilterError butterz(uint8_t order, float dt, float wn_rps, FiltVectorXf& num_s, FiltVectorXf& den_s)
{
    if (order < 1 || order > kFilterMaxStates) {
        return FilterError::INVALID_DIMENSION;
    }

    return FilterError::NONE;
}

FilterError tustin_1_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, float wc_rad, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz.resize(1);
    denz.resize(1);
    numz << 1.0f;
    denz << 1.0f;

    if (num.size() < 1 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;
    if (wc_rad > 0.0f) {
        K = wc_rad/std::tan(wc_rad*dt/2.0f);
    }

    float coeff_denom = 1.0f;
    coeff_denom = den(0) * K + den(1);

    // printf("tustin_1: den->k[0] = %0.3f, den->k[1] = %0.3f, K = %0.3f\n", den->k[0], den->k[1], K);

    if (coeff_denom < tol)
    {
        return FilterError::UNSTABLE;
    }

    // left pad or left truncate the numerator if num.size() != 2
    FiltVectorXf tmp_num = left_resize(num, 2);

    numz.resize(2);
    numz(0) = (tmp_num(0) * K + tmp_num(1)) / coeff_denom;
    numz(1) = (-tmp_num(0) * K + tmp_num(1)) / coeff_denom;

    denz.resize(2);
    denz(0) = 1.0f;
    denz(1) = (-den(0) * K + den(1)) / coeff_denom;

    return FilterError::NONE;
}

FilterError tustin_2_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, float wc_rad, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;

    numz.resize(1);
    denz.resize(1);
    numz << 1.0f;
    denz << 1.0f;

    if (num.size() < 1 || den.size() < 3) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;
    if (wc_rad > 0.0f) {
        K = wc_rad/std::tan(wc_rad*dt/2.0f);
    }
    float coeff_denom = den(0) * K * K + den(1) * K + den(2);

    if (coeff_denom < tol)
    {
        return FilterError::UNSTABLE;
    }

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, 3);
    numz.resize(3);
    numz(0) = (tmp_num(0)*K*K + tmp_num(1)*K + tmp_num(2)) / coeff_denom;
    numz(1) = (-2.0*tmp_num(0)*K*K       + 2.0*tmp_num(2)) / coeff_denom;
    numz(2) = (tmp_num(0)*K*K - tmp_num(1)*K + tmp_num(2)) / coeff_denom;

    denz.resize(3);
    denz(0) = 1.0f;
    denz(1) = (-2.0*den(0)*K*K       + 2.0*den(2)) / coeff_denom;
    denz(2) = (den(0)*K*K - den(1)*K + den(2)) / coeff_denom;

    return FilterError::NONE;
}

// TODO: unimplemented
// https://www.researchgate.net/publication/358716710_The_Transfer_Function_of_the_nth-Order_Digital_Butterworth_Low_Pass_Filter
FilterError tustin_n_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, float wc_rad, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;
    numz.resize(1);
    denz.resize(1);
    numz << 1.0f;
    denz << 1.0f;

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, n+1);

    numz.resize(n+1);
    denz.resize(n+1);
    numz.setZero();
    denz.setZero();
    numz(0) = 1.0f;
    denz(0) = 1.0f;

    // TODO: convert from state space realization to discrete transfer function
    // this will require factorizing into a polynomial form
    // probably need to solve for the generalized eigenvalues/eigenvectors,
    // convert to Jordan form and then to a second order sections form

    return FilterError::NONE;
}

FilterError tustin_n_ss(const MatNN &A,
                        const MatN1 &B,
                        const Mat1N &C,
                        const Mat11 &D,
                        float dt,
                        float wc_rad,
                        MatNN &Phi,
                        MatN1 &Gamma,
                        Mat1N &H,
                        Mat11 &J )
{
    const float tol = 1.0e-6;

    if (A.size() == 0 || B.size() == 0 || C.size() == 0 || D.size() == 0) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    char n = A.rows();

    Phi.resize(n,n);
    Gamma.resize(n,1);
    H.resize(1,n);
    J.resize(1,1);

    Phi.setZero();
    Gamma.setZero();
    H.setZero();
    J.setZero();

    // https://ocw.mit.edu/courses/6-245-multivariable-control-systems-spring-2004/e7aeed6b7a0d508ad3632c9a46b9a21d_lec11_6245_2004.pdf
    float w0 = 2.0f / dt;
    if (wc_rad > 0.0f) {
        w0 = wc_rad/std::tan(wc_rad*dt/2.0f);
    }

    // (I*w0 - A)^-1 // from MIT 6-245 notes
    MatNN invIw0mA = Eigen::Inverse(MatNN::Identity(n,n)*w0 - A);

    // using FPW's notation here
    Phi = (MatNN::Identity(n,n)*w0 + A)*invIw0mA;
    Gamma =invIw0mA*B*2.0f;
    H = w0*C*invIw0mA;
    J = D + C*invIw0mA*B;

    return FilterError::NONE;
}

// Forward Euler discrete transform -- only use when poles are << than nyquist
// TODO: unimplemented
FilterError forward_n_tf(const FiltVectorXf &num, const FiltVectorXf &den, float dt, FiltVectorXf &numz, FiltVectorXf &denz)
{
    const float tol = 1.0e-6;
    numz.resize(1);
    denz.resize(1);
    numz << 1.0f;
    denz << 1.0f;

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION;
    }

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() != 3
    FiltVectorXf tmp_num = left_resize(num, n+1);

    numz.resize(n+1);
    denz.resize(n+1);

    // TODO: convert from state space realization to discrete transfer function
    // this will require factorizing into a polynomial form
    // probably need to solve for the generalized eigenvalues/eigenvectors,
    // convert to Jordan form and then to a second order sections form

    return FilterError::NONE;
}


FilterError tf2ss( const FiltVectorXf &num, 
            const FiltVectorXf &den, 
            MatNN &A, 
            MatN1 &B, 
            Mat1N &C, 
            Mat11 &D)
{

    static const float tol = 1e-9;

    if (num.size() == 0 || den.size() < 2) {
        return FilterError::INVALID_DIMENSION; // invalid vector dimension
    }

    float d0 = den(0);
    if (fabs(d0) < tol) {
        return FilterError::INVALID_POLYNOMIAL;
    }

    char n = den.size() - 1;

     // left pad or left truncate the numerator if num.size() < den.size()
     // normalize both num and den so that den(0) = 1
    FiltVectorXf tmp_num = left_resize(num/d0, n+1);
    FiltVectorXf tmp_den = den/d0;

    // s->inf initial value subtracted out so that remaining num is relative degree one or greater
    float Ginf = tmp_num(0);
    tmp_num -= Ginf * tmp_den; // tmp_num(0) = 0 after this

    A.resize(n, n);
    B.resize(n, 1);
    C.resize(1, n);
    D.resize(1, 1);

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    if (n > 1) {

        A.block(0,0,n-1,1).setZero();
        A.block(0,1,n-1,n-1) << MatNN::Identity(n-1,n-1);
        A.block(n-1,0,1,n) = Mat1N(-tmp_den(Eigen::seqN(n,n,Eigen::fix<-1>)));

        B(n - 1) = 1;
        C = Mat1N(tmp_num(Eigen::seqN(n, n, Eigen::fix<-1>)));
        D << Ginf;
    } else {
        A << -tmp_den(Eigen::last);
        B << 1;
        C << tmp_num(Eigen::last);
        D << Ginf;
    }

    return FilterError::NONE;
}

FilterError tf2ss(const Vec3 &num, 
                  const Vec3 &den,
                  Mat22 &A,
                  Mat21 &B,
                  Mat12 &C,
                  Mat11 &D)
{

    // s->inf initial value subtracted out so that remaining num is relative degree one or greater
    float Ginf = num(0)/den(0);

    Vec3 tmp_num = num - Ginf*den;

    A.setZero();
    B.setZero();
    C.setZero();
    D.setZero();

    A << 0.0f, 1.0f, -den(2), -den(1);
    B(1) = 1.0f;
    C << tmp_num(2), tmp_num(1);
    D << Ginf;

    return FilterError::NONE;
}

FilterError tustin_1_tf(const Vec3 &num, const Vec3 &den, float dt, float wc_rad, Vec3 &numz, Vec3 &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f, 0.0f, 0.0f;
    denz << 1.0f, 0.0f, 0.0f;

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;
    if (wc_rad > 0.0f) {
        K = wc_rad/std::tan(wc_rad*dt/2.0f);
    }

    float coeff_denom = den(1) * K + den(2);

    // printf("tustin_1: den->k[0] = %0.3f, den->k[1] = %0.3f, K = %0.3f\n", den->k[0], den->k[1], K);

    if (fabs(coeff_denom) < tol)
    {
        return FilterError::UNSTABLE;
    }

    numz(0) = (num(1) * K + num(2)) / coeff_denom;
    numz(1) = (-num(1) * K + num(2)) / coeff_denom;

    denz(0) = 1.0f;
    denz(1) = (-den(1) * K + den(2)) / coeff_denom;

    return FilterError::NONE;
}

FilterError tustin_2_tf(const Vec3 &num, const Vec3 &den, float dt, float wc_rad, Vec3 &numz, Vec3 &denz)
{
    const float tol = 1.0e-6;

    numz << 1.0f, 0.0f, 0.0f;
    denz << 1.0f, 0.0f, 0.0f;

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    float K = 2.0f / dt;
    if (wc_rad > 0.0f) {
        K = wc_rad/std::tan(wc_rad*dt/2.0f);
    }
    float coeff_denom = den(0) * K * K + den(1) * K + den(2);

    if (fabs(coeff_denom) < tol)
    {
        return FilterError::UNSTABLE;
    }

    numz(0) = (num(0)*K*K + num(1)*K + num(2)) / coeff_denom;
    numz(1) = (-2.0*num(0)*K*K       + 2.0*num(2)) / coeff_denom;
    numz(2) = (num(0)*K*K - num(1)*K + num(2)) / coeff_denom;

    denz(0) = 1.0f;
    denz(1) = (-2.0*den(0)*K*K       + 2.0*den(2)) / coeff_denom;
    denz(2) = (den(0)*K*K - den(1)*K + den(2)) / coeff_denom;

    return FilterError::NONE;
}


FilterError tustin_2_ss(const Mat22 &A,
                        const Mat21 &B,
                        const Mat12 &C,
                        const Mat11 &D,
                        float dt,
                        float wc_rad,
                        Mat22 &Phi,
                        Mat21 &Gamma,
                        Mat12 &H,
                        Mat11 &J )
{
    const float tol = 1.0e-6;

    if (dt < tol) {
        return FilterError::INVALID_TIMESTEP;
    }

    Phi.setZero();
    Gamma.setZero();
    H.setZero();
    J.setZero();

    // https://ocw.mit.edu/courses/6-245-multivariable-control-systems-spring-2004/e7aeed6b7a0d508ad3632c9a46b9a21d_lec11_6245_2004.pdf
    float w0 = 2.0f / dt;
    if (wc_rad > 0.0f) {
        w0 = wc_rad/std::tan(wc_rad*dt/2.0f);
    }

    // (I*w0 - A)^-1 // from MIT 6-245 notes
    Mat22 invIw0mA = Eigen::Inverse(Mat22::Identity(2,2)*w0 - A);

    // using FPW's notation here
    Phi = (Mat22::Identity(2,2)*w0 + A)*invIw0mA;
    Gamma =invIw0mA*B*2.0f;
    H = w0*C*invIw0mA;
    J = D + C*invIw0mA*B;

    return FilterError::NONE;
}

FilterError zpk2tf2(Vec2c zeros, Vec2c poles, float K, uint8_t nz, uint8_t np, Vec3 &num, Vec3 &den)
{

    const float tol = 1e-6;

    num << K, 0, 0;
    den << 1, 0, 0;

    if (nz > 2 || np > 2 || nz > np) {
        return FilterError::INVALID_DIMENSION;
    }

    if (np == 0) {
        return FilterError::NONE;
    }

    if (np == 1) {
        den << 1, -poles(0).real(), 0;
        if (nz == 1) {
            num << 1, -zeros(0).real(), 0;
        }
    } else {
        den << 1, -(poles(0).real() + poles(1).real()), (poles(0)*poles(1)).real();
        if (nz == 1) {
            num << 0, 1, -zeros(0).real();
        } else if (nz == 2) {
            num << 1, -(zeros(0).real() + zeros(1).real()), (zeros(0)*zeros(1)).real();
        }
    }

    // TODO: handle zero den sum (integrator case)
    // This assumes a discrete filter?

    // forward euler integrator example
    // y_k+1 = y_k + u_k
    // H(z) = 1/(z - 1) <-- den.sum() == 0

    float dcGain=1.0f;
    if (fabs(den.sum()) > tol) {
        dcGain = num.sum()/den.sum();
    } else {
        // TODO: how to set gain for a filter with a pure integrator pole?
    }

    

    // TODO: handle zero dcgain (derivative case)
    num = num*K/dcGain;

    return FilterError::NONE;
}

}
