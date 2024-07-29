#include "HumanoidModel.hpp"

namespace rhoban {
        
bool HumanoidModel::computeLegIK(
    JointCommands& outputs,
    Side side,
    const Eigen::Vector3d& footPos,
    const Eigen::Matrix3d& footRotation
)
{

    //Convert foot position from given 
    //target to LegIK base
    auto legIKTarget = buildTargetPos(footPos, side);

    //Convert orientation from given frame
    //to LegIK base
    auto legIKMatrix = buildTargetOrientation(footRotation);
    
    //Run inverse kinematics
    LegIK::Position result;
    auto isSuccess = legIK.compute(legIKTarget, legIKMatrix, result);

    //Update degrees of freedom on success
    if (isSuccess) 
    {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, side, outputs);
    } 

    return isSuccess;
}
        
double HumanoidModel::legsLength() const
{
    return legHipToKnee + legKneeToAnkle + legAnkleToGround;
}
        
double HumanoidModel::feetDistance() const
{
    return legLateral;
}

LegIK::Vector3D HumanoidModel::buildTargetPos(const Eigen::Vector3d& footPos, const Side side) const
{
    //Building LegIK input target position 
    //data structure
    LegIK::Vector3D legIKTarget{footPos.x(), footPos.y(), footPos.z()};

    switch (side)
    {

    case Side::Left:
        legIKTarget[1] -= footYOffset;
        break;

    case Side::Right:
        legIKTarget[1] += footYOffset;
        break;

    default:
        break;
    }

    return legIKTarget;
}

LegIK::Frame3D HumanoidModel::buildTargetOrientation(const Eigen::Matrix3d& rotation) const
{
    LegIK::Frame3D legIKMatrix;

    for (auto a = 0; a < 3; a++)
    {
        for (auto b = 0; b < 3; b++)
        {
            legIKMatrix[a][b] = rotation(a, b);
        }
    }
    return legIKMatrix;
}
        
void HumanoidModel::setIKResult(const LegIK::Position& result, const Side side, JointCommands& commands) const
{
    switch (side)
    {
    case Side::Left:
        setIKResult(result, commands.leftLeg);
        break;

    case Side::Right:
        setIKResult(result, commands.rightLeg);
        break;

    default:
        break;
    }
}

void HumanoidModel::setIKResult(const LegIK::Position& result, JointCommands::Leg& legCommands) const
{
        legCommands.hip_yaw     =   result.theta[0];
        legCommands.hip_roll    =   result.theta[1];
        legCommands.hip_pitch   = - result.theta[2];
        legCommands.knee        =   result.theta[3];
        legCommands.ankle_pitch = - result.theta[4];
        legCommands.ankle_roll  =   result.theta[5];
}

void HumanoidModel::checkNaN(
    const LegIK::Position& result, 
    const LegIK::Vector3D& pos,
    const LegIK::Frame3D& orientation) const
{
    //Check if Nan is returned
    if (
        std::isnan(result.theta[0]) ||
        std::isnan(result.theta[1]) ||
        std::isnan(result.theta[2]) ||
        std::isnan(result.theta[3]) ||
        std::isnan(result.theta[4]) ||
        std::isnan(result.theta[5])
    ) {
        throw std::logic_error("LegIK NaN invalid result. "
            + std::string("theta0=") 
            + std::to_string(result.theta[0]) 
            + std::string(" ")
            + std::string("theta1=") 
            + std::to_string(result.theta[1]) 
            + std::string(" ")
            + std::string("theta2=") 
            + std::to_string(result.theta[2]) 
            + std::string(" ")
            + std::string("theta3=") 
            + std::to_string(result.theta[3]) 
            + std::string(" ")
            + std::string("theta4=") 
            + std::to_string(result.theta[4]) 
            + std::string(" ")
            + std::string("theta5=") 
            + std::to_string(result.theta[5]) 
            + std::string(" pos=")
            + pos.pp()
            + std::string(" orientation=")
            + orientation.pp()
        );
    }
}

}

