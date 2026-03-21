#ifndef SCANLOGLOADER_HPP
#define SCANLOGLOADER_HPP

#include <string>
#include <vector>
#include "LaserScan.hpp"

class ScanLogLoader {
    public:
        std::vector<LaserScan> loadFromFile(const std::string &path) const;
};

#endif