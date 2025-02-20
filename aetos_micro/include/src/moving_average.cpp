#ifndef __MOVING_AVERAGE_CPP__
#define __MOVING_AVERAGE_CPP__

#include "moving_average.hpp"

namespace Helpers
{
    template <typename TYPE, uint16_t COEFF_NB>
    MovingAverage<TYPE, COEFF_NB>::
        MovingAverage(TYPE startingValue_)
    {
        for (uint16_t i = 0u; i < COEFF_NB; i++)
        {
            this->addValue(startingValue_);
        }
    }

    template <typename TYPE, uint16_t COEFF_NB>
    MovingAverage<TYPE, COEFF_NB>::
        ~MovingAverage(void) {}

    template <typename TYPE, uint16_t COEFF_NB>
    float MovingAverage<TYPE, COEFF_NB>::
        addValue(TYPE value_)
    {
        _avg -= (float)_avgTable[_cursor];
        _avg += (float)value_;
        _avgTable[_cursor] = value_;
        _cursor = _cursor + 1 == COEFF_NB ? 0 : _cursor + 1;
        return _avg / (float)COEFF_NB;
    }

    template <typename TYPE, uint16_t COEFF_NB>
    float MovingAverage<TYPE, COEFF_NB>::
        getAverage(void)
    {
        return _avg / (float)COEFF_NB;
    }
}

#endif // __MOVING_AVERAGE_CPP__
