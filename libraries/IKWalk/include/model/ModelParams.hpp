#pragma once

namespace rhoban
{
    struct ModelParams
    {
        double distFootYOffset;

        /**
         * Model leg typical length between
         * each rotation axis
         */
        double distHipToKnee;
        double distKneeToAnkle;
        double distAnkleToGround;
        
        /**
         * Distance between the two feet in lateral
         * axis while in zero position
         */
        double distFeetLateral;

    }; // struct ModelParams

} // namespace rhoban
