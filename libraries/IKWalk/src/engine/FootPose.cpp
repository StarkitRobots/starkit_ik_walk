#include "FootPose.hpp"

namespace rhoban
{
    Eigen::Affine3d FootPose::toTrunk() const
    {
        auto transform = Eigen::Affine3d::Identity();

        transform.translation().x() = x;
        transform.translation().y() = y;

        transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                .toRotationMatrix();

        return transform;
    }
    
} // namespace rhoban
