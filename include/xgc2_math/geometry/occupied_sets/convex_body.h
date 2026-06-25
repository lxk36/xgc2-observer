#ifndef XGC2_MATH_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_H
#define XGC2_MATH_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_H

#include "xgc2_math/geometry/occupied_sets/convex_set_base.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace xgc2_math {

struct ConvexBody {
    int id = -1;
    std::string name;
    std::string geometry_type;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    std::shared_ptr<const ConvexSet3D> shape;

    bool isRegistered() const { return static_cast<bool>(shape); }

    Eigen::Vector3d center() const { return shape ? shape->center() : position; }

    SupportQueryResult support(const Eigen::Vector3d& direction) const {
        return shape ? shape->support(direction) : makeSupportQueryResult(position, direction);
    }
};

} // namespace xgc2_math

#endif // XGC2_MATH_GEOMETRY_OCCUPIED_SETS_CONVEX_BODY_H
