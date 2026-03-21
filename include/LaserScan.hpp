#ifndef LASERSCAN_HPP
#define LASERSCAN_HPP

#include <vector>

struct LaserScan {
    double timestamp;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    std::vector<double> ranges;
};

#endif