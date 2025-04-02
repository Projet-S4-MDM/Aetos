
#ifndef __POSITION_PID_HPP__
#define __POSITION_PID_HPP__

#include <array>

class PositionPid
{
public:
    PositionPid(float initialX_, float initialY_, float initialZ_)
    {
        _initialPosition[0] = initialX_;
        _initialPosition[1] = initialY_;
        _initialPosition[2] = initialZ_;
    };

private:
    std::array<float, 3> _initialPosition;
    

};

#endif