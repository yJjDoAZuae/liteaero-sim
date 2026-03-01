#pragma once

#include "SISOBlock.hpp"

#include <string>
#include <sstream>
#include <vector>

namespace liteaerosim::control {

typedef enum {
    AW_Negative = -1,
    AW_Null = 0,
    AW_Positive = 1
} AW_Direction;

class Antiwindup {

    public:

        Antiwindup() : 
            name(""), 
            direction(AW_Direction::AW_Null), 
            limit(0), 
            latch_on_direction(false),
            _in(0), 
            _active(false) {}

        std::string name;
        AW_Direction direction;
        float limit;
        bool latch_on_direction;  // TBD naming for this.  True == use direction of diff to determine if the mode is active

        void operator=(float in);

        bool isActive() const { return isActiveLower() || isActiveUpper(); }
        bool isActiveLower() const {return _activeLower; }
        bool isActiveUpper() const {return _activeUpper; }

    protected:

        float _in;
        bool _active;
        bool _activeLower;
        bool _activeUpper;

};

}
