

#include "control/Antiwindup.hpp"

using namespace liteaerosim::control;

void Antiwindup::operator=(float in)
{
    float diff = in - _in;
    _in = in;

    _activeLower = false;
    _activeUpper = false;
    switch (direction) {
        case AW_Direction::AW_Null:
            break;

        case AW_Direction::AW_Negative:
            _activeLower = (_in < limit) && (!latch_on_direction || diff < 0);
            _activeUpper = false;
            break;

        case AW_Direction::AW_Positive:
            _activeUpper = (_in > limit) && (!latch_on_direction || diff > 0);
            _activeLower = false;
            break;
    }
}
