#ifndef __CHRONO_HPP__
#define __CHRONO_HPP__

namespace Helpers
{
    /// @brief Chronometer helper class
    /// @tparam TYPE The return type of the clock function used in 2nd template 
    /// argument
    /// @tparam CLOCK_FUNC A function that returns the current clock of the 
    /// processor. In arduino framework: millis or micros
    /// @example 
    /// void setup()
    /// {
    ///     Helpers::Chrono<unsigned long, micros> chrono;
    ///     
    ///     delay(2000);
    ///     Serial.printf("Time: %u", chrono.getTime());
    ///
    ///     Output: "Time: 200010"
    /// } 
    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    class Chrono
    {
    private:
        TYPE _startClock;
        TYPE _accumulatedTime;
        bool _paused;

    public:
        Chrono(void);
        ~Chrono(void);
        void init(void);
        void pause(void);
        void resume(void);
        TYPE getTime(void);
        void restart(void);
    };
}

#include "src/chrono.cpp"

#endif // __CHRONO_HPP__
