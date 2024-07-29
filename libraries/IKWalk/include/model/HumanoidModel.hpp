#pragma once

#include <Eigen/Dense>

#include "JointCommands.hpp"
#include "LegIK.hpp"
#include "ModelParams.hpp"

namespace rhoban 
{
    enum class Side
    {
        Left,
        Right
    };
    
    /**
     * HumanoidModel
     */
    class HumanoidModel
    {
    public:

        /**
         * Initialize the model with given
         * typical leg length
         */
        explicit HumanoidModel(const ModelParams params) : 
            footYOffset         (params.distFootYOffset),
            legHipToKnee        (params.distHipToKnee),
            legKneeToAnkle      (params.distKneeToAnkle),
            legAnkleToGround    (params.distAnkleToGround),
            legLateral          (params.distFeetLateral),
            legIK(params.distHipToKnee, params.distKneeToAnkle, params.distAnkleToGround)
        {}

    public:

        /**
         * Run analytical inverse kinematics LegIK and 
         * assign outputs motor positions.
         * Target footPos and angles are given in
         * special frame when all leg motors are in zero position.
         * Frame center is located on left or right foot tip.
         * True is returned if angles are updated and inverse
         * kinematics is sucessful, else false is returned.
         */
        bool computeLegIK(
            JointCommands& outputs,
            Side side,
            const Eigen::Vector3d& footPos,
            const Eigen::Matrix3d& footRotation = Eigen::Matrix3d::Identity()
        );

    public:
        /**
         * Return the initial vertical distance
         * from trunk frame to foot tip frame (Z)
         */
        double legsLength() const;

        /**
         * Return the initial lateral distance
         * between each feet
         */
        double feetDistance() const;
    

    private:
        /**
         * Compute and return the IK position reference
         * vector and orientation reference matrix
         * in LegIK specifics structures
         */
        LegIK::Vector3D buildTargetPos(const Eigen::Vector3d& footPos, const Side side) const;
            
        LegIK::Frame3D buildTargetOrientation(const Eigen::Matrix3d& rotation) const;

    private:
        /**
         * Check inverse kinematics computed value
         * and throw an error in case of NaN
         */
        void checkNaN(const LegIK::Position& result, const LegIK::Vector3D& pos,const LegIK::Frame3D& orientation) const;

        /**
         * Assign model leg DOF to given IK results
         */
        void setIKResult(const LegIK::Position& result, const Side, JointCommands& commands) const;
        void setIKResult(const LegIK::Position& result,     JointCommands::Leg& legCommands) const;

    public:

        /**
         * Leg segments lengths used by
         * inverse kinematics
         */
        double footYOffset;
        double legHipToKnee;
        double legKneeToAnkle;
        double legAnkleToGround;
        double legLateral;

        LegIK::IK legIK;

    }; // class HumanoidModel

} // namespace rhoban
