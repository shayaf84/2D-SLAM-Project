#include "PoseGraph.hpp"

PoseGraph::PoseGraph() {

}

int PoseGraph::addNode(const Pose2 &pose_estimate) {
    int id = nodes_.size();
    PoseGraphNode toAdd = PoseGraphNode{id, pose_estimate};
    nodes_.push_back(toAdd);
    return id;
}

void PoseGraph::addPriorFactor(int node_id, const Pose2 &measurement, const Eigen::Matrix3d &information) {
    PriorFactor toAdd = PriorFactor{node_id, measurement, information};
    priors_.push_back(toAdd);
}

void PoseGraph::addOdometryFactor(int from_id, int to_id, const Pose2 &measurement, const Eigen::Matrix3d &information) {
    PoseGraphEdge toAdd = PoseGraphEdge{from_id, to_id, measurement, information, FactorType::Odometry};
    edges_.push_back(toAdd);
}

void PoseGraph::addLoopClosureFactor(int from_id, int to_id, const Pose2 &measurement, const Eigen::Matrix3d &information) {
    PoseGraphEdge toAdd = PoseGraphEdge{from_id, to_id, measurement, information, FactorType::LoopClosure};
    edges_.push_back(toAdd);
}

const std::vector<PoseGraphNode>& PoseGraph::nodes() const {
    return nodes_;
}

const std::vector<PoseGraphEdge>& PoseGraph::edges() const {
    return edges_;
}

const std::vector<PriorFactor>& PoseGraph::priors() const {
    return priors_;
}