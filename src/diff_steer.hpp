#ifndef DIFF_STEER_HPP
#define DIFF_STEER_HPP

// #include "mbed.h"
#include "coordinate.hpp"
#include <cmath>
#include <array>

namespace bit {

struct DiffSteerValue
{
    float vel;
    float theta;

    DiffSteerValue(const float& vel, const float& theta)
    : vel(vel), theta(theta) {}

    explicit DiffSteerValue(const CoordinatePolar& other) noexcept
    {
        vel = other.r;
        theta = other.theta;
    }

    explicit DiffSteerValue(const Coordinate& other) noexcept
    {
        DiffSteerValue(static_cast<CoordinatePolar>(other));
    }
};

template<int N>
class DiffSteer
{
    static_assert(N > 0, "N must be positive");
public:
    DiffSteer(const float& robot_radius) : robot_radius_(robot_radius)
    {
        for (int i = 0; i < N; ++i)
        {
            constexpr float k = 2 * M_PI / N;
            constexpr float ofs = k / 2;
            wheel_pos_[i] = Coordinate(robot_radius_, 0, 0, i * k + ofs);
        }
    }

    DiffSteer(const std::array<Coordinate, N>& wheel_pos)
    {
        for (int i = 0; i < N; ++i)
        {
            wheel_pos[i] = wheel_pos[i];
        }
    }

    std::array<DiffSteerValue, N> calc_vel(const Velocity& vel)
    {
        std::array<DiffSteerValue, N> value;
        for (int i; i<N; ++i)
        {
            CoordinatePolar pos = static_cast<CoordinatePolar>(wheel_pos_[i]);
            convert_ang(pos, 0);
            const Velocity tmp = {vel.x + pos.r * vel.ang * cos(pos.ang),
                                    vel.y + pos.r * vel.ang * cos(pos.ang)};
            value[i] = static_cast<DiffSteerValue>(tmp);
        }
        return value;
    }

    
private:
    float robot_radius_;
    std::array<Coordinate, N> wheel_pos_;
};

}  // namespace bit

#endif // DIFF_STEER_HPP
