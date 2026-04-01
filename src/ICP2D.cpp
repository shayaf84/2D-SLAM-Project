#include "ICP2D.hpp"
#include "nanoflann.hpp"

using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd>;

std::vector<Eigen::Vector2d> transformPoints(const std::vector<Eigen::Vector2d> &points, const Pose2 &pose) {
    std::vector<Eigen::Vector2d> transformedPoints;
    for (int i = 0; i < points.size(); i++) {
        transformedPoints.push_back(pose.transformPoint(points[i]));
    }
    return transformedPoints;
}

Eigen::MatrixXd pointsToMatrix(const std::vector <Eigen::Vector2d> &points) {
    Eigen::MatrixXd matrix(points.size(), 2);
    for (int i = 0; i < points.size(); i++) {
        matrix(i, 0) = points[i].x();
        matrix(i, 1) = points[i].y();
    }
    return matrix;
}

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> findCorrespondences(
    const std::vector<Eigen::Vector2d> &source,
    const std::vector<Eigen::Vector2d> &target
) {

    Eigen::MatrixXd target_matrix = pointsToMatrix(target);
    KDTree target_tree(2, std::cref(target_matrix), 10);
    target_tree.index_->buildIndex();

    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences;

    for (int i = 0; i < source.size(); i++) {
        double query_pt[2] = {source[i].x(), source[i].y()};
        Eigen::Index nearest_index = 0;
        double dist_squared = 0.0;

        int num_found = target_tree.index_->knnSearch(query_pt, 1, &nearest_index, &dist_squared);
        if (!num_found) {
            continue;
        }
        correspondences.push_back({source[i], target[nearest_index]});
    }
    return correspondences;
}
