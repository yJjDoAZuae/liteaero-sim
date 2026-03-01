#define _USE_MATH_DEFINES
#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS2Clip.hpp"

static float dcTol = 1e-6;

using namespace liteaerosim::control;
using namespace liteaerosim;

// copy implementation
// template <char NUM_STATES=FILTER_MAX_STATES>
// void SISOFilter<NUM_STATES>::copy(SISOFilter &filt)
void FilterSS2Clip::copy(FilterSS2Clip &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
    
    _order = filt.order();
    _dt = filt.dt();
}

void FilterSS2Clip::setLowPassFirstIIR(float dt, float tau)
{
    // HACK: zero order hold realization
    // ydot = -1/tau y + 1/tau u
    // yk+1 - yk = (-1/tau yk + 1/tau u) * dt
    // yk+1 = (1 - dt/tau) xk + dt/tau u

    // numz.k[0] = dt/tau;
    // numz.k[1] = 0.0f;
    // denz.k[0] = 1.0f;
    // denz.k[1] = -(1-dt/tau);

    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;
    num_s(1) = 0.0f;
    num_s(2) = 1.0f / tau;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_1_tf(num_s, den_s, dt, 2.0f*M_PI / tau, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 1;
    _dt = dt;
}

void FilterSS2Clip::setLowPassSecondIIR(float dt, float wn_rps, float zeta, float tau_zero)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;  // s^2
    num_s(1) = tau_zero * wn_rps * wn_rps; // s
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 2;
    _dt = dt;
}

void FilterSS2Clip::setNotchSecondIIR(float dt, float wn_rps, float zeta_den, float zeta_num)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 1.0f;
    num_s(1) = 2.0f * zeta_num * wn_rps;
    num_s(2) = wn_rps * wn_rps;
    den_s(0) = 1.0f;
    den_s(1) = 2.0f * zeta_den * wn_rps;
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 2;
    _dt = dt;
}

void FilterSS2Clip::setHighPassFirstIIR(float dt, float tau)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode = tustin_2_tf(num_s, den_s, dt, 2.0f*M_PI / tau, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 1;
    _dt = dt;
}

void FilterSS2Clip::setHighPassSecondIIR(float dt, float wn_rps, float zeta, float c_zero)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 1.0f;  // s^2
    num_s(1) = c_zero * 2.0f * zeta * wn_rps;  // s
    num_s(2) = 0.0f;
    den_s(0) = 1.0f;  // s^2
    den_s(1) = 2.0f * zeta * wn_rps;  // s
    den_s(2) = wn_rps * wn_rps;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, wn_rps, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 2;
    _dt = dt;
}

void FilterSS2Clip::setDerivIIR(float dt, float tau)
{
    Eigen::Vector3f num_s;
    Eigen::Vector3f den_s;

    num_s(0) = 0.0f;
    num_s(1) = 1.0f / tau;
    num_s(2) = 0.0f;
    den_s(0) = 0.0f;
    den_s(1) = 1.0f;
    den_s(2) = 1.0f / tau;

    Eigen::Vector3f num_z;
    Eigen::Vector3f den_z;

    _errorCode += tustin_2_tf(num_s, den_s, dt, 2.0f*M_PI / tau, num_z, den_z);

    tf2ss(num_z, den_z, _Phi, _Gamma, _H, _J);

    _order = 1;
    _dt = dt;
}

void FilterSS2Clip::resetInput(float in)
{
    const float tol = 1e-6;

    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        Mat22 ImPhiInv;
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        (Mat22::Identity() - _Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        _in = in;
        _out = valLimit.step(dcGain * in);

        if (fabs(dcGain) > tol) {
            _in = _out/dcGain;
        }

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

void FilterSS2Clip::resetOutput(float out)
{
    const float tol = 1e-6;

    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (fabs(dcGain) < tol) {
        _errorCode += FilterError::ZERO_DC_GAIN;
        return;
    }

    if (errorCode() == 0)
    {

        Mat22 ImPhiInv;
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        (Mat22::Identity() - _Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        // Assumption that rateLimit limits interval includes 0
        // otherwise we would need to do a ramp reset rather than DC
        rateLimit.step(0.0f);
        _out = valLimit.step(out);
        _in = out/dcGain;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }

}

float FilterSS2Clip::dcGain() const
{
    // float dcGain = 1.0f;

    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.

    Mat22 ImPhiInv;
    bool invertible = false;
    float absDeterminantThreshold = 1e-4;

    (Mat22::Identity() - _Phi).computeInverseWithCheck(ImPhiInv, invertible, absDeterminantThreshold);

    if (!invertible) {
        // _errorCode += FilterError::INFINITE_DC_GAIN;
        return 1.0f;
    }

    return (_H*ImPhiInv*_Gamma + _J).value();
}

Mat22 FilterSS2Clip::controlGrammian() const
{
    Mat22 C(Mat22::Zero(2,2));

    for (int k = 0; k<2; k++) {
        C(Eigen::all, k) << Mat21(_Phi.pow(k) * _Gamma);
    }

    return C;
}

Mat22 FilterSS2Clip::observeGrammian() const
{
    Mat22 C(Mat22::Zero(2,2));

    for (int k = 0; k<2; k++) {
        C(k, Eigen::all) << Mat12(_H * _Phi.pow(k));
    }

    return C;
}

void FilterSS2Clip::backsolve1(float inPrev, float outPrev)
{
    // Backsolve to correct the state based on the limited values.
    // We are correcting the *previous* iteration's state here
    // to make it consistent with the output value we just calculated.
    bool invertible = false;
    const float absDeterminantThreshold = 1e-12;

    // y_k+1 = H x_k + J u_k
    Mat12 A;
    Mat11 b;

    // Ax = b
    // A will be column rank deficient, use the right pseudoinverse
    // A^+ = A^T (A A^T)^-1

    A << _H;
    b << _out - (_J * _in).value();

    Mat21 ApInv;
    Mat11 AATInv;

    Mat11 AAT;
    AAT = A*A.transpose();

    AAT.computeInverseWithCheck(AATInv, invertible, absDeterminantThreshold);

    if (invertible) {
        ApInv = A.transpose() * AATInv;
        _x = ApInv * b;
    }
}

void FilterSS2Clip::backsolve2(float inPrev, float outPrev)
{
    // Backsolve to correct the state based on the limited values.
    // We are correcting the *previous* iteration's state here
    // to make it consistent with the output value we just calculated.
    Mat22 PhiInv;
    bool invertible = false;
    const float absDeterminantThreshold = 1e-12;

    // _Phi should be invertible for a stable filter, but if it's not
    // we'll just skip backsolving
    _Phi.computeInverseWithCheck(PhiInv, invertible, absDeterminantThreshold);

    if (invertible) {

        // y_k = H Phi^-1 (x_k - Gamma u_k-1) + J u_k-1
        // y_k+1 = H x_k + J u_k
        Mat22 A;
        Mat21 b;

        A << _H*PhiInv,
            _H;
        b << outPrev + (_H * PhiInv * _Gamma * inPrev).value() - (_J * inPrev).value(), 
            _out - (_J * _in).value();

        Mat22 AInv;
        A.computeInverseWithCheck(AInv, invertible, absDeterminantThreshold);

        if (invertible) {
            _x = AInv * b;
        } else {
            // A is uninvertible, try the left psuedoinverse instead
            // A^+ = (A^T A)^-1 A^T
            Mat22 ApInv;
            Mat22 ATAInv;
            (A.transpose()*A).computeInverseWithCheck(ATAInv, invertible, absDeterminantThreshold);

            if (invertible) {
                ApInv = ATAInv * A.transpose();
                _x = ApInv * b;
            }
        }
    }
}

void FilterSS2Clip::backsolve(float inPrev, float outPrev)
{
    // Backsolve to correct the state based on the limited values.
    // We are correcting the *previous* iteration's state here
    // to make it consistent with the output value we just calculated.

    if (_order == 1) {
        backsolve1(inPrev, outPrev);
    } else if (_order == 2) {
        backsolve2(inPrev, outPrev);
    }
}

float FilterSS2Clip::step(float in)
{

    float inPrev = _in;
    _in = in;

    float delU = in - inPrev;

    float outPrev = _out;

    // TRICKY: update the output first
    _out = (_H*_x + _J*_in).value();

    // Euler derivative computation with derivative
    // TODO: use a tustin derivative here?
    float outDot = rateLimit.step((_out - outPrev)/_dt);

    // Apply the rate limit to the output but also reimpose the value limit
    _out = valLimit.step(outPrev + outDot*_dt);

    if (valLimit.isLimited() || rateLimit.isLimited()) {
        backsolve(inPrev, outPrev);
    }

    // state update
    _x = _Phi*_x + _Gamma*_in;

    return this->out();
}
