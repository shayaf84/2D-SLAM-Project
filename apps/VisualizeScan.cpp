#include "ScanLogLoader.hpp"

#include <iostream>
#include <vector>

int main() {
    ScanLogLoader loader;
    std::vector<LaserScan> scans = loader.loadFromFile("data/base_scan.txt");

    return 0;
}