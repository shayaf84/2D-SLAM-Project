#ifndef POSE2
#define POSE2


#include <Eigen/Dense>

class Pose2 {
    public:
        using Vector2 = Eigen::Vector2d;
        using Matrix2 = Eigen::Matrix2d;
        using Matrix3 = Eigen::Matrix3d;

        Pose2();
        Pose2(double x, double y, double theta);

        double x() const;
        double y() const;
        double theta() const;

        Vector2 translation() const;
        Matrix2 rotation() const;
        Matrix3 homogeneousMatrix() const;


        double translationNorm() const;
        double rotationMagnitude() const;


        Pose2 inverse() const;

        Vector2 transformPoint(const Vector2 &point) const;
        
        Pose2 compose(const Pose2 &other) const;

    private:
        double x_;
        double y_;
        double theta_;
};

#endif