#include "ICP2D.hpp"
#include "nanoflann.hpp"
#include <cmath>

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

Pose2 estimateRigidTransform(
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &correspondences
) {
    if (correspondences.empty()) {
        return Pose2();
    }
    Eigen::Vector2d source_centroid = Eigen::Vector2d(0.0, 0.0);
    Eigen::Vector2d target_centroid = Eigen::Vector2d(0.0, 0.0);
    Eigen::Matrix2d covariance_matrix = Eigen::Matrix2d::Zero();

    for (const auto &pair_element : correspondences) {
        source_centroid += pair_element.first;
        target_centroid += pair_element.second;
    }
    source_centroid /= correspondences.size();
    target_centroid /= correspondences.size();

    for (const auto &pair_element : correspondences) {
        Eigen::Vector2d source_delta = pair_element.first - source_centroid;
        Eigen::Vector2d target_delta = pair_element.second - target_centroid;
        covariance_matrix += source_delta * target_delta.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        covariance_matrix,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );
    
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d rot = V * U.transpose();
    
    if (rot.determinant() < 0) {
        V.col(1) *= -1.0;
        rot = V * U.transpose();
    }

    Eigen::Vector2d trans = target_centroid - rot * source_centroid;
    double theta = std::atan2(rot(1,0), rot(0,0));
    return Pose2(trans.x(), trans.y(), theta);
}

Pose2 runICP(const std::vector<Eigen::Vector2d> &source, const std::vector<Eigen::Vector2d> &target) {
    Pose2 estimate;
    const int MAX_ITER = 20;
    
    for (int i = 0; i < MAX_ITER; i++) {
        std::vector<Eigen::Vector2d> transformedSource = transformPoints(source, estimate);
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences = findCorrespondences(transformedSource, target);
        Pose2 delta = estimateRigidTransform(correspondences);
        estimate = delta.compose(estimate);
        if (delta.translationNorm() < 1e-3 && delta.rotationMagnitude() < 1e-3) {
            break;
        }
    }
    return estimate;
}