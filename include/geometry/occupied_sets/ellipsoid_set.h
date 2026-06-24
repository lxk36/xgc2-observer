#ifndef XGC2_MATH_GEOMETRY_OCCUPIED_SETS_ELLIPSOID_SET_H
#define XGC2_MATH_GEOMETRY_OCCUPIED_SETS_ELLIPSOID_SET_H

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "geometry/occupied_sets/convex_set_base.h"

namespace xgc2_math {

class EllipsoidSet final : public ConvexSet3D {
  public:
    struct Options {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d radii = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    };

    EllipsoidSet() = default;

    explicit EllipsoidSet(const Options& options)
        : center_value_(options.center), radii_(options.radii.cwiseAbs()),
          orientation_(options.orientation.normalized()) {}

    EllipsoidSet(const Eigen::Vector3d& center_value, const Eigen::Vector3d& radii,
                 const Eigen::Quaterniond& orientation)
        : EllipsoidSet(Options{center_value, radii, orientation}) {}

    Eigen::Vector3d center() const override { return center_value_; }

    SupportQueryResult support(const Eigen::Vector3d& direction) const override {
        if (direction.squaredNorm() <= 1e-18) {
            return makeSupportQueryResult(center_value_, direction);
        }

        const Eigen::Vector3d local_direction = orientation_.conjugate() * direction;
        const Eigen::Vector3d squared_radii = radii_.cwiseProduct(radii_);
        const Eigen::Vector3d numerator = squared_radii.cwiseProduct(local_direction);
        const double denominator = std::sqrt(std::max(0.0, local_direction.dot(numerator)));
        if (denominator <= 1e-18) {
            return makeSupportQueryResult(center_value_, direction);
        }

        const Eigen::Vector3d local_support = numerator / denominator;
        return makeSupportQueryResult(center_value_ + orientation_ * local_support, direction);
    }

  private:
    Eigen::Vector3d center_value_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d radii_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

} // namespace xgc2_math

#endif // XGC2_MATH_GEOMETRY_OCCUPIED_SETS_ELLIPSOID_SET_H
