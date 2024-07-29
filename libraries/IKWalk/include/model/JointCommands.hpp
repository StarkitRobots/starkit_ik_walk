#pragma once

namespace rhoban
{
    struct JointCommands
    {
        struct Leg
        {
            double hip_yaw;
            double hip_roll;
            double hip_pitch;
            double knee;
            double ankle_pitch;
            double ankle_roll;

        };

        struct Arm
        {
            double shoulder_roll;
            double shoulder_pitch;
            double elbow;

        };

        Arm leftArm;
        Arm rightArm;

        Leg leftLeg;
        Leg rightLeg;

    }; // JointCommands

} // namespace rhoban
