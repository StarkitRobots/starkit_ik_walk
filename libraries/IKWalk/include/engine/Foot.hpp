#pragma once

#include "utils/spline/PolySpline.hpp"

#include "FootPose.hpp"

namespace rhoban
{
    struct Foot
    {
    public:

        void clearSplines();

    public:

        inline const FootPose position(const double time) const
        {
            return FootPose{
                xSpline.get(time),
                ySpline.get(time),
                zSpline.get(time),
                yawSpline.get(time)
            };
        }

    public:

        utils::PolySpline xSpline;
        utils::PolySpline ySpline;
        utils::PolySpline zSpline;

        utils::PolySpline yawSpline;

        double trunkYOffset;

    }; // struct Foot

} // namespace rhoban