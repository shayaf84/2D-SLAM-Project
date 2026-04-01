#ifndef ICP2D_HPP
#define ICP2D_HPP

#include <vector>
#include <Eigen/Dense>
#include "Pose2.hpp"

std::vector<Eigen::Vector2d> transformPoints(const std::vector<Eigen::Vector2d> &points, const Pose2 &pose);

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> findCorrespondences(
    const std::vector<Eigen::Vector2d> &source,
    const std::vector<Eigen::Vector2d> &target
);

Eigen::MatrixXd pointsToMatrix(const std::vector<Eigen::Vector2d> &points);


#endif