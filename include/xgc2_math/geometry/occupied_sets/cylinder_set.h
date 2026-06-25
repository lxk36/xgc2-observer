#ifndef XGC2_MATH_GEOMETRY_OCCUPIED_SETS_CYLINDER_SET_H
#define XGC2_MATH_GEOMETRY_OCCUPIED_SETS_CYLINDER_SET_H

#include "xgc2_math/geometry/occupied_sets/convex_set_base.h"
#include <Eigen/Dense>
#include <cmath>

namespace xgc2_math {

class CylinderSet final : public ConvexSet3D {
  public:
    struct Options {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double radius = 0.0;
        double height = 0.0;
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    };

    CylinderSet() = default;

    explicit CylinderSet(const Options& options)
        : center_value_(options.center), radius_(options.radius), height_(options.height),
          orientation_(options.orientation.normalized()) {}

    CylinderSet(const Eigen::Vector3d& center_value, double radius, double height,
                const Eigen::Quaterniond& orientation)
        : CylinderSet(Options{center_value, radius, height, orientation}) {}

    Eigen::Vector3d center() const override { return center_value_; }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        const Eigen::Vector3d local_direction = orientation_.conjugate() * direction;
        Eigen::Vector3d local_support = Eigen::Vector3d::Zero();
        const double radial_norm = std::hypot(local_direction.x(), local_direction.y());
        if (radial_norm > 1e-9) {
            local_support.x() = radius_ * local_direction.x() / radial_norm;
            local_support.y() = radius_ * local_direction.y() / radial_norm;
        }
        local_support.z() = local_direction.z() >= 0.0 ? 0.5 * height_ : -0.5 * height_;
        return makeSupportQueryResult(center_value_ + orientation_ * local_support, direction);
    }

  private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    double radius_ = 0.0;
    double height_ = 0.0;
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

} // namespace xgc2_math

#endif // XGC2_MATH_GEOMETRY_OCCUPIED_SETS_CYLINDER_SET_H
