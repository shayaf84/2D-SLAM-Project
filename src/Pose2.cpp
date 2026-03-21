#include "Pose2.hpp"
#include <cmath>

Pose2::Pose2() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
}

Pose2::Pose2(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
}

double Pose2::x() const {
    return x_;
}

double Pose2::y() const {
    return y_;
}

double Pose2::theta() const {
    return theta_;
}

Pose2::Vector2 Pose2::translation() const {
    return Vector2(x_, y_);
}

Pose2::Matrix2 Pose2::rotation() const {
    Matrix2 rot;
    rot << std::cos(theta_), -1 * std::sin(theta_),
        std::sin(theta_), std::cos(theta_);
    return rot;
}

Pose2::Matrix3 Pose2::homogeneousMatrix() const {
    Matrix3 transform = Matrix3::Identity();
    Matrix2 rot = rotation();

    transform.block<2,2>(0,0) = rot;
    transform(0,2) = x_;
    transform(1,2) = y_;
    return transform;
}

Pose2 Pose2::inverse() const {
    double c = std::cos(theta_);
    double s = std::sin(theta_);
    double x_inv = -(c * x_ + s * y_);
    double y_inv = s * x_ - c * y_;
    return Pose2(x_inv, y_inv, -theta_);
}

Pose2 Pose2::compose(const Pose2 &other) const {
    double c = std::cos(theta_);
    double s = std::sin(theta_);

    double composed_x = c * other.x_ - s * other.y_ + x_;
    double composed_y = s * other.x_ + c * other.y_ + y_;
    double composed_theta = theta_ + other.theta_;

    return Pose2(composed_x, composed_y, composed_theta);
} 

Pose2::Vector2 Pose2::transformPoint(const Vector2 &point) const {
    double c = std::cos(theta_);
    double s = std::sin(theta_);

    double new_x = c * point.x() - s * point.y() + x_;
    double new_y = s * point.x() + c * point.y() + y_;

    return Vector2(new_x, new_y);
}
