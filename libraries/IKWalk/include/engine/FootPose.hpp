#pragma once

#include <Eigen/Dense>

namespace rhoban
{
    struct FootPose
    {
    public:

        Eigen::Affine3d toTrunk() const;

    public:
        double x;
        double y;
        double z;

        double yaw;

    }; //struct FootPose

} // namespace rhoban
