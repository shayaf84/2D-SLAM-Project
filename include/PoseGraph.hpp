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

class PoseGraph {
    public:
        PoseGraph();
        int addNode(const Pose2 &pose_estimate);
        void addPriorFactor(int node_id, const Pose2 &measurement, const Eigen::Matrix3d &information);
        void addOdometryFactor(int from_id, int to_id, const Pose2 &measurement, const Eigen::Matrix3d &information);
        void addLoopClosureFactor(int from_id, int to_id, const Pose2 &measurement, const Eigen::Matrix3d &information);


        const std::vector<PoseGraphNode>& nodes() const;
        const std::vector<PoseGraphEdge>& edges() const;
        const std::vector<PriorFactor>& priors() const;
    
    private:
        std::vector<PoseGraphNode> nodes_;
        std::vector<PoseGraphEdge> edges_;
        std::vector<PriorFactor> priors_;
};

#endif