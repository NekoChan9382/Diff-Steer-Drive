#include "coordinate.hpp"
#include "diff_steer.hpp"
#include <iostream>
#include <cmath>
#include "mbed.h"

int main()
{
    bit::Coordinate p{0.0f, 1.0f, 0.0f};
    bit::Coordinate q{1.0f, 0.0f, 0.0f};

    bit::DiffSteer<4>(1);

    // bit::CoordinatePolar a = p;
    // q *= 2;
    bit::convert_ang(p, M_PI / 2);
    const float d = bit::distance(p, q);
    printf("%.2f\n", d);
    // std::cout << p.x << " " << p.y << " " << p.ang << " " << p.axis_ang << std::endl;
    printf("%.2f %.2f %.2f %.2f\n", p.x, p.y, p.ang, p.axis_ang);
}