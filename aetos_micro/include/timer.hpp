#ifndef __TIMER_H__
#define __TIMER_H__

namespace Helpers
{
    /// @brief Timer helper class. A wrapper to do block of code after a certain
    /// time or at a certain frequency.
    /// @tparam TYPE The return type of the clock function used in 2nd template
    /// argument
    /// @tparam CLOCK_FUNC A function that returns the current clock of the
    /// processor. In arduino framework: millis or micros
    /// @example Executing a block of code at 1 Hz (each 1000 milliseconds)
    /// void setup()
    /// {
    ///     Helpers::Timer<unsigned long, millis> timer(1000);
    ///
    ///     for (EVER)
    ///     {
    ///         if (timer.isDone())
    ///         {
    ///             /* Do something at 1 Hz*/
    ///         }
    ///     }
    /// }
    template <typename T, T (*clockFunc)(void)>
    class Timer
    {
    private:
        T _prevClock;
        T _interval;

        void setInterval(T interval);

    public:
        Timer(void);
        Timer(T interval_);
        ~Timer(void);
        void init(T interval_);
        bool isDone(bool reset_ = 1);
        void updateInterval(T newInterval_);
        T getInterval(void);
        void reset(void);
    };
}

#include "src/timer.cpp"

#endif //__TIMER_H__
