#include "control/Integrator.hpp"

using namespace liteaerosim::control;

float Integrator::step(float in)
{

    bool awActive = false;

    for (int k = 0; k < aw.size(); k++) {
        awActive |= aw.at(k).isActive();
    }

    if (!awActive) {

        switch (_method) {

        case DiscretizationMethod::BackEuler:
            // Backward Euler
            _out = limit.step(_out + in * _dt);
            break;

        case DiscretizationMethod::FwdEuler:
            // Forward Euler
            _out = limit.step(_out + _in * _dt);
            break;

        case DiscretizationMethod::Bilinear:
            // Bilinear (Tustin)
            _out = limit.step(_out + 0.5*(in + _in) * _dt);
            break;
        }
    } else {
        _out = limit.step(_out);
    }

    _in = in;
    return _out;
}

void Integrator::reset(float in)
{

    _out = limit.step(in);
    _in = in;

}
