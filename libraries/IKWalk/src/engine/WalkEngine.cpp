#include "WalkEngine.hpp"

namespace rhoban
{
    bool WalkEngine::computeAngles(JointCommands& angles, const double elapsed)
    {
        phase += elapsed / stepDuration;

        if (stepCount == 0 || phase > 1.0)
        {
            newStep();
            phase = 0.0;

            if (commands.enableWalk)
            {
                stepCount += 1;
            }
        }

        phase = utils::coerce(phase, 0.0, 1.0);

        auto leftPose   = left.position(phase * stepDuration);
        auto rightPose  = right.position(phase * stepDuration);
        
        // TODO: add swing class maybe
          // Swing calculations
        // double swing = swingSpline.get(timeSinceLastStep);
        auto swingP     = (isLeftSupport ? M_PI : 0.0) + M_PI * 2.0 * params.swingPhase;
        auto swing      = params.swingGain      * sin(M_PI * phase + swingP);
        auto rollSwing  = params.swingRollGain  * sin(M_PI * phase + swingP);

        Eigen::Affine3d trunkPitchRot   (Eigen::AngleAxisd(-params.trunkPitch,     Eigen::Vector3d::UnitY()));
        Eigen::Affine3d trunkPitchRotPID(Eigen::AngleAxisd(-params.trunkPitchOut,  Eigen::Vector3d::UnitY()));

        //Default pitch values (no PID)
        Eigen::Affine3d trunkPitchRotLeft = trunkPitchRot;
        Eigen::Affine3d trunkPitchRotRight = trunkPitchRot;

        //Adding PID to pitch values. If disabled, everything stays the same
        if(isLeftSupport) 
        {
            trunkPitchRotRight = trunkPitchRotPID;
        } 
        else 
        {
            trunkPitchRotLeft = trunkPitchRotPID; 
        }

        Eigen::Affine3d swingRollRot(Eigen::AngleAxisd(rollSwing, Eigen::Vector3d::UnitX()));

        Eigen::Affine3d trunkRollRot    (Eigen::AngleAxisd(params.trunkRoll,    Eigen::Vector3d::UnitX()));
        Eigen::Affine3d trunkRollRotPID (Eigen::AngleAxisd(params.trunkRollOut, Eigen::Vector3d::UnitX()));

        auto trunkRollRotInverted     = trunkRollRot    .inverse() * swingRollRot;
        auto trunkRollRotPIDInverted  = trunkRollRotPID .inverse() * swingRollRot;

        trunkRollRot    = trunkRollRot    * swingRollRot;
        trunkRollRotPID = trunkRollRotPID * swingRollRot;

        Eigen::Affine3d trunkRollRotRightFoot = trunkRollRotInverted;
        Eigen::Affine3d trunkRollRotRightHip  = trunkRollRotInverted;

        Eigen::Affine3d trunkRollRotLeftFoot  = trunkRollRot;
        Eigen::Affine3d trunkRollRotLeftHip   = trunkRollRot;
        //Default roll values (no PID)
        if (isLeftSupport)
        {
            trunkRollRotRightFoot = trunkRollRotPIDInverted;
            trunkRollRotRightHip  = trunkRollRotPIDInverted;
        }
        else
        {
            trunkRollRotLeftFoot  = trunkRollRotPID;
            trunkRollRotLeftHip   = trunkRollRotPID;
        }

        return model.computeLegIK(
                angles, Side::Left,
                trunkPitchRotLeft * trunkRollRotLeftHip * Eigen::Vector3d(leftPose.x, leftPose.y + swing, -model.legsLength() + params.trunkZOffset + leftPose.z),
                Eigen::AngleAxisd(-leftPose.yaw, 
                Eigen::Vector3d::UnitZ()) * trunkPitchRotLeft.linear().inverse() * trunkRollRotLeftFoot.linear().inverse()
            ) && model.computeLegIK(
                angles, Side::Right,
                trunkPitchRotRight * trunkRollRotRightHip * Eigen::Vector3d(rightPose.x, rightPose.y + swing, -model.legsLength() + params.trunkZOffset + rightPose.z),
                Eigen::AngleAxisd(-rightPose.yaw, 
                Eigen::Vector3d::UnitZ()) * trunkPitchRotRight.linear().inverse() * trunkRollRotRightFoot.linear().inverse() 
            );
    }

    void WalkEngine::reset()
    {
        commands.stepSizeX = 0.0;
        commands.stepSizeY = 0.0;
        commands.stepSizeYaw = 0.0;

        phase = 0.0;

        stepCount = 0;
    }

    void WalkEngine::setCommands(const WalkEngineCommands& newCommands)
    {
        utils::VariationBound< double >::update(commands.stepSizeX, newCommands.stepSizeX, params.maxDiffStepSizeX, 1.0);
        utils::VariationBound< double >::update(commands.stepSizeY, newCommands.stepSizeY, params.maxDiffStepSizeY, 1.0);
        utils::VariationBound< double >::update(commands.stepSizeYaw, utils::deg2rad(newCommands.stepSizeYaw), utils::deg2rad(params.maxDiffStepSizeYaw), 1.0);

        commands.stepSizeX = utils::coerce(commands.stepSizeX, -params.maxStepSizeXBackward, params.maxStepSizeX);
        commands.stepSizeY = utils::coerce(commands.stepSizeY, -params.maxStepSizeY, params.maxStepSizeY);
        commands.stepSizeYaw = utils::coerce(commands.stepSizeYaw, -utils::deg2rad(params.maxStepSizeYaw), utils::deg2rad(params.maxStepSizeYaw));

        commands.enableWalk = newCommands.enableWalk;
        commands.enableCircular = newCommands.enableCircular;
    }

    void WalkEngine::newStep()
    {
        auto previousStepDuration = stepDuration;

        auto stepDuration = 1.0 / (2.0 * params.frequency);

        auto oldLeft = left;
        auto oldRight = right;

        left.clearSplines();
        right.clearSplines();

        left.trunkYOffset   = model.footYOffset + params.footYOffset + params.footYOffsetPerStepSizeY * std::abs(commands.stepSizeY);
        right.trunkYOffset  = -left.trunkYOffset;

        // Defining begining of splines according to previous feet position
        left.xSpline.addPoint(0.0, oldLeft.xSpline.get(previousStepDuration), oldLeft.xSpline.getVel(previousStepDuration));
        left.ySpline.addPoint(0.0, oldLeft.ySpline.get(previousStepDuration), oldLeft.ySpline.getVel(previousStepDuration));

        right.xSpline.addPoint(0.0, oldRight.xSpline.get(previousStepDuration), oldRight.xSpline.getVel(previousStepDuration));
        right.ySpline.addPoint(0.0, oldRight.ySpline.get(previousStepDuration), oldRight.ySpline.getVel(previousStepDuration));

        left    .yawSpline.addPoint(0.0, oldLeft  .yawSpline.get(previousStepDuration), oldLeft   .yawSpline.getVel(previousStepDuration));
        right   .yawSpline.addPoint(0.0, oldRight .yawSpline.get(previousStepDuration), oldRight  .yawSpline.getVel(previousStepDuration));


        if (params.doubleSupportRatio > 0.0) 
        {
            left.xSpline.addPoint(stepDuration * params.doubleSupportRatio * 0.5, oldLeft.xSpline.get(previousStepDuration), oldLeft.xSpline.getVel(previousStepDuration)); 
            right.xSpline.addPoint(stepDuration * params.doubleSupportRatio * 0.5, oldRight.xSpline.get(previousStepDuration), oldRight.xSpline.getVel(previousStepDuration)); 
        }

        // Changing support foot
        isLeftSupport = !isLeftSupport;

        std::cout << "GOOD" << std::endl;
        
        // Support foot is on the ground
        supportFoot().zSpline.addPoint(0.0, 0.0, 0.0);
        supportFoot().zSpline.addPoint(stepDuration, 0.0, 0.0);

        /*
            flyingFoot foot Z profile:
        
            | riseDuration |
            |______________|     ________...  _
            /              \    /             |
        /                \  /              | riseGain
        /                  \/               |
        |   stepDuration   |                ^
        
        Rise/fall slopes will be smoothed by splines

        */

        std::cout << stepDuration << std::endl;

        // Flying foot will rise
        flyingFoot().zSpline.addPoint(0.0, 0.0, 0.0);

        if (params.doubleSupportRatio > 0.0)
        {  
            flyingFoot().zSpline.addPoint(stepDuration * params.doubleSupportRatio * 0.5, 0.0, 0.0);
        }

        if (params.riseDuration > 0.0)
        {
            flyingFoot().zSpline.addPoint(stepDuration * (1.0 - params.riseDuration) * 0.5, params.riseGain, 0.0);
            flyingFoot().zSpline.addPoint(stepDuration * (1.0 + params.riseDuration) * 0.5, params.riseGain, 0.0);
        }
        else
        {
            flyingFoot().zSpline.addPoint(stepDuration * 0.5, params.riseGain, 0.0);
        }

        if (params.doubleSupportRatio > 0.0)
        {  
            flyingFoot().zSpline.addPoint(stepDuration * (1.0 - params.doubleSupportRatio * 0.5), 0.0, 0.0);
        }

        flyingFoot().zSpline.addPoint(stepDuration, 0.0, 0.0);

        std::cout << "GOOD" << std::endl;

        if (fabs(commands.stepSizeYaw) > 0.01 && commands.enableCircular)
        {
            // Speed vector
            utils::Point speed(commands.stepSizeX, commands.stepSizeY);
            // Center of rotation
            auto center = speed.perpendicular() / commands.stepSizeYaw;

            // For both feet, computing the new position in the
            utils::Point sFoot(params.trunkXOffset, supportFoot().trunkYOffset);
            supportFoot().xSpline.addPoint(stepDuration, sFoot.x, 0.0);
            supportFoot().ySpline.addPoint(stepDuration, sFoot.y, 0.0);

            utils::Point fFoot(params.trunkXOffset, flyingFoot().trunkYOffset);
            fFoot = (fFoot - center).rotation(utils::rad2deg(commands.stepSizeYaw * 0.5)) + center;
            flyingFoot().xSpline.addPoint(stepDuration, fFoot.x, 0.0);
            flyingFoot().ySpline.addPoint(stepDuration, fFoot.y, 0.0);
        }
        else
        {
            if (params.doubleSupportRatio > 0.0) {      
                supportFoot().xSpline.addPoint(stepDuration * (1.0 - params.doubleSupportRatio * 0.5), params.trunkXOffset - commands.stepSizeX * 0.5, 0);
                flyingFoot().xSpline.addPoint(stepDuration * (1.0 - params.doubleSupportRatio * 0.5), params.trunkXOffset + commands.stepSizeX * 0.5, 0);      
            }

            supportFoot().xSpline.addPoint(stepDuration, params.trunkXOffset - commands.stepSizeX * 0.5, 0);
            supportFoot().ySpline.addPoint(stepDuration, supportFoot().trunkYOffset - commands.stepSizeY * 0.5, 0);


            flyingFoot().xSpline.addPoint(stepDuration, params.trunkXOffset + commands.stepSizeX * 0.5, 0);
            flyingFoot().ySpline.addPoint(stepDuration, flyingFoot().trunkYOffset + commands.stepSizeY * 0.5, 0);
        }

        // Yaw spline
        if (params.doubleSupportRatio > 0.0) 
        {      
            supportFoot().yawSpline.addPoint(stepDuration * (1.0 - params.doubleSupportRatio * 0.5), -commands.stepSizeYaw * 0.5, 0.0);
            flyingFoot().yawSpline.addPoint(stepDuration * (1.0 - params.doubleSupportRatio * 0.5), commands.stepSizeYaw * 0.5, 0.0);
        }

        supportFoot().yawSpline.addPoint(stepDuration, -commands.stepSizeYaw * 0.5, 0);
        flyingFoot().yawSpline.addPoint(stepDuration, commands.stepSizeYaw * 0.5, 0);
        
    }

} // namespace rhoban
