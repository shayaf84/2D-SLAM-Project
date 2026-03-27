#include "ScanUtils.hpp"
#include <cmath>

std::vector<Eigen::Vector2d> scanToPoints(const LaserScan &scan) {
    double angle_min = scan.angle_min;
    double angle_max = scan.angle_max;
    double angle_increment = scan.angle_increment;
    double range_min = scan.range_min;
    double range_max = scan.range_max;
    std::vector<double> ranges = scan.ranges;
    std::vector<Eigen::Vector2d> result;

    for (int i = 0; i < ranges.size(); i++) {
        double theta = angle_min + i * angle_increment;

        if (!std::isfinite(ranges[i])) {
            continue;
        }
        
        if (ranges[i] < range_min || ranges[i] > range_max) {
            continue;
        }

        double x = ranges[i] * std::cos(theta);
        double y = ranges[i] * std::sin(theta);
    

        Eigen::Vector2d vector = Eigen::Vector2d(x, y);
        result.push_back(vector);
    }
    return result;
    
}