#pragma once

namespace rhoban
{
    struct WalkEngineParams
    {
        /**
         * Complete (two legs) walk cycle frequency
         * in Hertz
         */
        double frequency;

        /**
         * Length of double support phase
         * in phase time
         * (between 0 and 1)
         * 0 is null double support and full single support
         * 1 is full double support and null single support
         */
        double doubleSupportRatio;

        double maxStepSizeX;
        double maxStepSizeXBackward;
        double maxDiffStepSizeX;
        
        double maxStepSizeY;
        double maxDiffStepSizeY;

        double maxStepSizeYaw;
        double maxDiffStepSizeYaw;

        /**
         * Forward length of each foot step
         * in meters
         * >0 goes forward
         * <0 goes backward
         * (dynamic parameter)
         */
        // double stepGain;

        /**
         * Vertical rise height of each foot
         * in meters (positive)
         */
        double riseGain;
        double riseDuration;

        /**
         * Angular yaw rotation of each 
         * foot for each step in radian.
         * 0 does not turn
         * >0 turns left
         * <0 turns right
         * (dynamic parameter)
         */
        // double turnGain;

        /**
         * Lateral length of each foot step
         * in meters.
         * >0 goes left
         * <0 goes right
         * (dynamic parameter)
         */
        // double lateralGain;

        /**
         * Vertical foot offset from trunk 
         * in meters (positive)
         * 0 is in init position
         * > 0 set the robot lower to the ground
         */
        double trunkZOffset;
        double trunkHeight;

        /**
         * Lateral trunk oscillation amplitude
         * in meters (positive)
         */
        double swingGain;

        /**
         * Lateral angular oscillation amplitude
         * of swing trunkRoll in radian
         */
        double swingRollGain;

        /**
         * Phase shift of lateral trunk oscillation
         * between 0 and 1
         */
        double swingPhase;

        /**
         * Foot X-Z spline velocities
         * at ground take off and ground landing.
         * Step stands for X and rise stands for Z
         * velocities.
         * Typical values ranges within 0 and 5.
         * >0 for DownVel is having the foot touching the
         * ground with backward velocity.
         * >0 for UpVel is having the foot going back
         * forward with non perpendicular tangent.
         */
        double stepUpVel;
        double stepDownVel;
        double riseUpVel;
        double riseDownVel;

        /**
         * Time length in phase time
         * where swing lateral oscillation
         * remains on the same side
         * between 0 and 0.5
         */
        double swingPause;

        /**
         * Swing lateral spline velocity (positive).
         * Control the "smoothness" of swing trajectory.
         * Typical values are between 0 and 5.
         */
        double swingVel;

        /**
         * Forward trunk-foot offset 
         * with respect to foot in meters
         * >0 moves the trunk forward
         * <0 moves the trunk backward
         */
        double trunkXOffset;

        /**
         * Lateral trunk-foot offset
         * with respect to foot in meters
         * >0 moves the trunk on the left
         * <0 moves the trunk on the right
         */
        double trunkYOffset;

        /**
         * Trunk angular rotation
         * around Y in radian
         * >0 bends the trunk forward
         * <0 bends the trunk backward
         */
        double trunkPitch;
        double trunkPitchOut;

        double trunkPitchNeutral;
        double trunkPitchForward;
        double trunkPitchBackward;

        /**
         * Trunk angular rotation
         * around X in radian
         * >0 bends the trunk on the right
         * <0 bends the trunk on the left
         */
        double trunkRoll;
        double trunkRollOut;

        double footYOffset;
        double footYOffsetPerStepSizeY;

    }; // struct WalkEngineParams

    struct WalkEngineCommands
    {
        double stepSizeX;
        double stepSizeY;
        double stepSizeYaw;

        bool enableWalk;
        bool enableCircular;

    }; //struct WalkEngineCommands

} // namespace rhoban