#ifndef __MOVING_AVERAGE_HPP__
#define __MOVING_AVERAGE_HPP__

#include "Arduino.h"

// =============================================================================
// moving_average.hpp defines a class for making moving_average / lowpass filter
// easier in code.
// =============================================================================

namespace Helpers
{
    /// @brief MovingAverage class wrapper with fixed number of coefficients.
    /// @tparam TYPE The type of data stored into the MovingAverage, it's the user
    /// job to make sure the type makes sense for a MovingAverage.
    /// @tparam COEFF_NB The number of elements contained into the moving average.
    /// @example
    /// void setup(void)
    /// {
    ///     RoverHelpers::MovingAverage<float, 10> avg;
    ///
    ///     for (uint8_t i = 0; i < 10; i++)
    ///     {
    ///         LOG(INFO, "currentAvg = %f", avg.addValue(10.0f));
    ///     }
    /// }
    ///
    /// Output: "1.0, 2.0, 3.0, ... 10.0"
    template <typename TYPE, uint16_t COEFF_NB>
    class MovingAverage
    {
    public:
        /// @brief MovingAverage's constructor.
        /// @param startingValue_ The average will start at this specific value,
        /// default is 0.
        MovingAverage(TYPE startingValue_ = static_cast<TYPE>(0));

        /// @brief MovingAverage's destructor
        ~MovingAverage(void);

        /// @brief Adds a value to the movingAverage.
        /// @param value_ value to add to the average.
        /// @return The new average
        float addValue(TYPE value_);

        /// @return Current average.
        float getAverage(void);

    private:
        TYPE _avgTable[COEFF_NB] = {0};
        uint16_t _cursor = 0;
        float _avg = 0.0f;
    };
}

#include "src/moving_average.cpp"

#endif // __MOVING_AVERAGE_HPP__
