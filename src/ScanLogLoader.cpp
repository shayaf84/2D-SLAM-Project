#include "ScanLogLoader.hpp"


#include <fstream>
#include <sstream>
#include <stdexcept>


std::vector<LaserScan> ScanLogLoader::loadFromFile(const std::string &path) const {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }

    std::vector<LaserScan> scanList;
    std::string line;

    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        std::istringstream line_stream(line);
        LaserScan scan;
        int num_ranges = 0;

        if (!(line_stream >> scan.timestamp
                          >> scan.angle_min
                          >> scan.angle_max
                          >> scan.angle_increment
                          >> scan.range_min
                          >> scan.range_max
                          >> num_ranges)) {
                            throw std::runtime_error("Failed to parse scan");
        }

        scan.ranges.resize(num_ranges);
        for (int i = 0; i < num_ranges; i++) {
            if (!(line_stream >> scan.ranges[i])) {
                throw std::runtime_error("Failed to parse range value");
            }
        }
        scanList.push_back(scan);
        
    }

    return scanList;
}