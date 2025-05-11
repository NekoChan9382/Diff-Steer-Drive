#include "coordinate.hpp"
#include "steer_drive.hpp"
#include "PID_new.hpp"
#include <iostream>
#include <cmath>
#include "mbed.h"

class DiffSteer;

constexpr int wheel_amount = 4;

int main()
{
    bit::Coordinate p{0.0f, 1.0f, 0.0f};
    bit::Coordinate q{1.0f, 0.0f, 0.0f};

    bit::SteerDrive<wheel_amount>(1);

    // bit::CoordinatePolar a = p;
    // q *= 2;
    bit::convert_ang(p, M_PI / 2);
    const float d = bit::distance(p, q);
    printf("%.2f\n", d);
    // std::cout << p.x << " " << p.y << " " << p.ang << " " << p.axis_ang << std::endl;
    printf("%.2f %.2f %.2f %.2f\n", p.x, p.y, p.ang, p.axis_ang);
}

class DiffSteer //* Concept: SteerValueからCANMessageの返却まで
{
    public:
    DiffSteer()
    {
        for (int i = 0; i < wheel_amount * 2; ++i)
        {
            pid_[i] = Pid(param_);
        } 
    }


    private:

    const PidGain gain_ = {1, 0, 0};
    const PidParameter param_ = {gain_, -100, 100};
    std::array<Pid, wheel_amount * 2> pid_;
};