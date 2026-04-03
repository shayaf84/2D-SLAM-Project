#ifndef POSEGRAPH_HPP
#define POSEGRAPH_HPP

#include <vector>
#include <Eigen/Dense>
#include "Pose2.hpp"

enum class FactorType {
    Odometry,
    LoopClosure
};

struct PoseGraphNode {
    int id;
    Pose2 estimate;
};

struct PoseGraphEdge {
    int from_id;
    int to_id;
    Pose2 measurement;
    Eigen::Matrix3d information;
    FactorType type;
};

struct PriorFactor {
    int node_id;
    Pose2 measurement;
    Eigen::Matrix3d information;
};



#endif