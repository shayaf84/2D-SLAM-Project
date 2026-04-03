#ifndef SCANUTILS_HPP
#define SCANUTILS_HPP


#include "LaserScan.hpp"
#include <vector>
#include <Eigen/Dense>


std::vector<Eigen::Vector2d> scanToPoints(const LaserScan &scan);

#endif