#include "ScanLogLoader.hpp"
#include "ScanUtils.hpp"

#include <iostream>
#include <vector>

int main() {
    ScanLogLoader loader;
    std::vector<LaserScan> scans = loader.loadFromFile("2D-SLAM-Project/data/base_scan.txt");
    std::vector<std::vector<Eigen::Vector2d>> points;
    for (int i = 0; i < scans.size(); i++) {
        points.push_back(scanToPoints(scans[i]));
    }
    return 0;
}