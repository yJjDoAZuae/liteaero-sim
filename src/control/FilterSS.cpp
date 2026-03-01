#define _USE_MATH_DEFINES
#include <cmath>
// #include <stdio.h>
// #include <string.h>

#include "control/control.hpp"
#include "control/filter_realizations.hpp"
#include "control/FilterSS.hpp"

static float dcTol = 1e-6;

using namespace liteaerosim::control;
using namespace liteaerosim;


void FilterSS::copy(FilterSS &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
}

void FilterSS::copy(liteaerosim::control::FilterSS2 &filt)
{
    _Phi << filt.Phi();
    _Gamma << filt.Gamma();
    _H << filt.H();
    _J << filt.J();
    _x << filt.x();

    _errorCode = filt.errorCode();
}

void FilterSS::setButterworthIIR(uint8_t order, float dt, float wn_rps)
{

    FiltVectorXf num_s;
    FiltVectorXf den_s;

    FilterError rc = butter(order, wn_rps, num_s, den_s);

    if (rc != FilterError::NONE) {
        _errorCode += rc;
        return;
    }

    MatNN A;
    MatN1 B;
    Mat1N C;
    Mat11 D;

    tf2ss(num_s, den_s, A, B, C, D);

    setDimension(order);

    _errorCode += tustin_n_ss(A,B,C,D, dt, wn_rps, _Phi, _Gamma, _H, _J);

    resetInput(0.0f);

}

void FilterSS::resetInput(float in)
{
    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        char n = _Phi.rows();

        MatNN ImPhiInv(MatNN::Zero(n,n));
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        MatNN ImPhi(MatNN::Identity(n,n) - _Phi);
        Eigen::FullPivLU<MatNN> LU(ImPhi);
        invertible = LU.isInvertible();

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        ImPhiInv << LU.inverse();

        _in = in;
        _out = dcGain * in;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

void FilterSS::resetOutput(float out)
{
    _x.setZero();
    _out = 0.0f;
    _in = 0.0f;
    float dcGain = this->dcGain();

    if (errorCode() == 0)
    {

        uint8_t n = _Phi.rows();

        MatNN ImPhiInv(MatNN::Zero(n,n));
        bool invertible = false;
        float absDeterminantThreshold = 1e-4;

        MatNN ImPhi(MatNN::Identity(n,n) - _Phi);
        Eigen::FullPivLU<MatNN> LU(ImPhi);
        invertible = LU.isInvertible();

        if (!invertible) {
            _errorCode += FilterError::UNSTABLE;
            return;
        }

        ImPhiInv << LU.inverse();

        _out = out;
        _in = out/dcGain;

        _x << ImPhiInv * _Gamma * _in;
    } else {
        return;
    }
}

float FilterSS::dcGain() const
{
    // We need an arbitrary finite DC gain to accommodate
    // band pass, high pass, and lead/lag filters filters
    // TODO: Filter stability test?

    // Test for infinite DC gain.
    // Pure integrators should not be implemented
    // using an ARMA filter.

    uint8_t n = _Phi.rows();

    MatNN ImPhiInv(MatNN::Zero(n,n));
    bool invertible = false;
    float absDeterminantThreshold = 1e-4;

    MatNN ImPhi(MatNN::Identity(n,n) - _Phi);
    Eigen::FullPivLU<MatNN> LU(ImPhi);
    invertible = LU.isInvertible();

    if (!invertible) {
        //_errorCode += FilterError::UNSTABLE;
        return 1.0f;
    }

    ImPhiInv << LU.inverse();

    return (_H*ImPhiInv*_Gamma + _J).value();
}

MatNN FilterSS::controlGrammian() const
{

    uint8_t n = _Phi.rows();

    MatNN C(MatNN::Zero(n,n));

    for (uint8_t k = 0; k<n; k++) {
        C(Eigen::all, k) = MatN1(_Phi.pow(k) * _Gamma);
    }

    return C;
}

MatNN FilterSS::observeGrammian() const
{
    uint8_t n = _Phi.rows();

    MatNN C(MatNN::Zero(n,n));

    for (uint8_t k = 0; k<n; k++) {
        C(k, Eigen::all) = Mat1N(_H * _Phi.pow(k));
    }

    return C;
}

uint8_t FilterSS::order() const
{
    uint8_t order = _Phi.rows();

    if (order > 0) {
        Eigen::JacobiSVD<MatNN> ControlSVD;
        Eigen::JacobiSVD<MatNN> ObserveSVD;

        MatNN CC = controlGrammian();
        MatNN CO = observeGrammian();

        ControlSVD.compute(CC);
        ObserveSVD.compute(CO);

        uint8_t crank = ControlSVD.rank();
        order = (crank < order) ? crank : order;
        uint8_t orank = ObserveSVD.rank();
        order = (orank < order) ? orank : order;
    }

    return order;
}

void FilterSS::setDimension(uint8_t dim)
{
    _Phi.resize(dim,dim);
    _Gamma.resize(dim,1);
    _H.resize(1,dim);
    _J.resize(1,1);
    _x.resize(dim,1);
}

float FilterSS::step(float in)
{
    this->_in = in;

    // TRICKY: update the output first
    _out = (_H*_x + _J*in).value();

    // now update the state
    _x = _Phi*_x + _Gamma*in;

    return this->out();
}
