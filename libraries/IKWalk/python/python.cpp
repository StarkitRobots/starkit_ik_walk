#include <iostream>
#include "IKWalk.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_ik_walk_parameters(py::module &m)
{
    
    py::class_<Rhoban::IKWalkParameters>(m, "IKWalkParameters",
        R"pbdoc(
        Walk parameters

        Attributes:
            distHipToKnee (float): Model leg typical length between each rotation axis
            distKneeToAnkle (float): Model leg typical length between each rotation axis
            distAnkleToGround (float): Model leg typical length between each rotation axis
            distFeetLateral (float): Distance between the two feet in the lateral axis while in zero position
            freq (float): Complete (two legs) walk cycle frequency in Hertz
            enabledGain (float): Global gain multiplying all time-dependent movement between 0 and 1. Control walk enabled/disabled smoothing. 0 is walk disabled. 1 is walk fully enabled
            supportPhaseRatio (float): Length of double support phase in phase time (between 0 and 1). 0 is null double support and full single support. 1 is full double support and null single support
            footYOffset (float): Lateral offset on the default foot position in meters (foot lateral distance). 0 is default. > 0 is both feet external offset
            stepGain (float): Forward length of each foot step in meters. >0 goes forward. <0 goes backward (dynamic parameter)
            riseGain (float): Vertical rise height of each foot in meters (positive)
            turnGain (float): Angular yaw rotation of each foot for each step in radian. 0 does not turn. >0 turns left. <0 turns right (dynamic parameter)
            lateralGain (float): Lateral length of each foot step in meters. >0 goes left. <0 goes right (dynamic parameter)
            trunkZOffset (float): Vertical foot offset from trunk in meters (positive). 0 is in the init position. > 0 sets the robot lower to the ground
            swingGain (float): Lateral trunk oscillation amplitude in meters (positive)
            swingRollGain (float): Lateral angular oscillation amplitude of swing trunkRoll in radian
            swingPhase (float): Phase shift of lateral trunk oscillation between 0 and 1
            stepUpVel (float): Foot X-Z spline velocities at ground take off and ground landing. Step stands for X and rise stands for Z velocities. Typical values range within 0 and 5. >0 for DownVel is having the foot touching the ground with backward velocity. >0 for UpVel is having the foot going back forward with a non-perpendicular tangent
            stepDownVel (float): Foot X-Z spline velocities at ground take off and ground landing. Step stands for X and rise stands for Z velocities. Typical values range within 0 and 5. >0 for DownVel is having the foot touching the ground with backward velocity. >0 for UpVel is having the foot going back forward with a non-perpendicular tangent
            riseUpVel (float): Foot X-Z spline velocities at ground take off and ground landing. Step stands for X and rise stands for Z velocities. Typical values range within 0 and 5. >0 for DownVel is having the foot touching the ground with backward velocity. >0 for UpVel is having the foot going back forward with a non-perpendicular tangent
            riseDownVel (float): Foot X-Z spline velocities at ground take off and ground landing. Step stands for X and rise stands for Z velocities. Typical values range within 0 and 5. >0 for DownVel is having the foot touching the ground with backward velocity. >0 for UpVel is having the foot going back forward with a non-perpendicular tangent
            swingPause (float): Time length in phase time where swing lateral oscillation remains on the same side between 0 and 0.5
            swingVel (float): Swing lateral spline velocity (positive). Control the 'smoothness' of the swing trajectory. Typical values are between 0 and 5
            trunkXOffset (float): Forward trunk-foot offset with respect to the foot in meters. >0 moves the trunk forward. <0 moves the trunk backward
            trunkYOffset (float): Lateral trunk-foot offset with respect to the foot in meters. >0 moves the trunk on the left. <0 moves the trunk on the right
            trunkPitch (float): Trunk angular rotation around Y in radian. >0 bends the trunk forward. <0 bends the trunk backward
            trunkRoll (float): Trunk angular rotation around X in radian. >0 bends the trunk on the right. <0 bends the trunk on the left
            extraLeftX (float): Add extra offset on the X direction of the left foot in meters
            extraLeftY (float): Add extra offset on the Y direction of the left foot in meters
            extraLeftZ (float): Add extra offset on the Z direction of the left foot in meters
            extraRightX (float): Add extra offset on the X direction of the right foot in meters
            extraRightY (float): Add extra offset on the Y direction of the right foot in meters
            extraRightZ (float): Add extra offset on the Z direction of the right foot in meters
            extraLeftYaw (float): Add extra angular offset on the Yaw rotation of the left foot in radians
            extraLeftPitch (float): Add extra angular offset on the Pitch rotation of the left foot in radians
            extraLeftRoll (float): Add extra angular offset on the Roll rotation of the left foot in radians
            extraRightYaw (float): Add extra angular offset on the Yaw rotation of the right foot in radians
            extraRightPitch (float): Add extra angular offset on the Pitch rotation of the right foot in radians
            extraRightRoll (float): Add extra angular offset on the Roll rotation of the right foot in radians
        )pbdoc")
        .def(py::init<>())
        .def_readwrite("distHipToKnee", &Rhoban::IKWalkParameters::distHipToKnee)
        .def_readwrite("distKneeToAnkle", &Rhoban::IKWalkParameters::distKneeToAnkle)
        .def_readwrite("distAnkleToGround", &Rhoban::IKWalkParameters::distAnkleToGround)
        .def_readwrite("distFeetLateral", &Rhoban::IKWalkParameters::distFeetLateral)
        .def_readwrite("freq", &Rhoban::IKWalkParameters::freq)
        .def_readwrite("enabledGain", &Rhoban::IKWalkParameters::enabledGain)
        .def_readwrite("supportPhaseRatio", &Rhoban::IKWalkParameters::supportPhaseRatio)
        .def_readwrite("footYOffset", &Rhoban::IKWalkParameters::footYOffset)
        .def_readwrite("stepGain", &Rhoban::IKWalkParameters::stepGain)
        .def_readwrite("riseGain", &Rhoban::IKWalkParameters::riseGain)
        .def_readwrite("turnGain", &Rhoban::IKWalkParameters::turnGain)
        .def_readwrite("lateralGain", &Rhoban::IKWalkParameters::lateralGain)
        .def_readwrite("trunkZOffset", &Rhoban::IKWalkParameters::trunkZOffset)
        .def_readwrite("swingGain", &Rhoban::IKWalkParameters::swingGain)
        .def_readwrite("swingRollGain", &Rhoban::IKWalkParameters::swingRollGain)
        .def_readwrite("swingPhase", &Rhoban::IKWalkParameters::swingPhase)
        .def_readwrite("stepUpVel", &Rhoban::IKWalkParameters::stepUpVel)
        .def_readwrite("stepDownVel", &Rhoban::IKWalkParameters::stepDownVel)
        .def_readwrite("riseUpVel", &Rhoban::IKWalkParameters::riseUpVel)
        .def_readwrite("riseDownVel", &Rhoban::IKWalkParameters::riseDownVel)
        .def_readwrite("swingPause", &Rhoban::IKWalkParameters::swingPause)
        .def_readwrite("swingVel", &Rhoban::IKWalkParameters::swingVel)
        .def_readwrite("trunkXOffset", &Rhoban::IKWalkParameters::trunkXOffset)
        .def_readwrite("trunkYOffset", &Rhoban::IKWalkParameters::trunkYOffset)
        .def_readwrite("trunkPitch", &Rhoban::IKWalkParameters::trunkPitch)
        .def_readwrite("trunkRoll", &Rhoban::IKWalkParameters::trunkRoll)
        .def_readwrite("extraLeftX", &Rhoban::IKWalkParameters::extraLeftX)
        .def_readwrite("extraLeftY", &Rhoban::IKWalkParameters::extraLeftY)
        .def_readwrite("extraLeftZ", &Rhoban::IKWalkParameters::extraLeftZ)
        .def_readwrite("extraRightX", &Rhoban::IKWalkParameters::extraRightX)
        .def_readwrite("extraRightY", &Rhoban::IKWalkParameters::extraRightY)
        .def_readwrite("extraRightZ", &Rhoban::IKWalkParameters::extraRightZ)
        .def_readwrite("extraLeftYaw", &Rhoban::IKWalkParameters::extraLeftYaw)
        .def_readwrite("extraLeftPitch", &Rhoban::IKWalkParameters::extraLeftPitch)
        .def_readwrite("extraLeftRoll", &Rhoban::IKWalkParameters::extraLeftRoll)
        .def_readwrite("extraRightYaw", &Rhoban::IKWalkParameters::extraRightYaw)
        .def_readwrite("extraRightPitch", &Rhoban::IKWalkParameters::extraRightPitch)
        .def_readwrite("extraRightRoll", &Rhoban::IKWalkParameters::extraRightRoll)
        .def("__repr__", [](const Rhoban::IKWalkParameters& self) {
        return "<IKWalkParameters:\n"
            "  distHipToKnee: " + std::to_string(self.distHipToKnee) + "\n"
            "  distKneeToAnkle: " + std::to_string(self.distKneeToAnkle) + "\n"
            "  distAnkleToGround: " + std::to_string(self.distAnkleToGround) + "\n"
            "  distFeetLateral: " + std::to_string(self.distFeetLateral) + "\n"
            "  freq: " + std::to_string(self.freq) + "\n"
            "  enabledGain: " + std::to_string(self.enabledGain) + "\n"
            "  supportPhaseRatio: " + std::to_string(self.supportPhaseRatio) + "\n"
            "  footYOffset: " + std::to_string(self.footYOffset) + "\n"
            "  stepGain: " + std::to_string(self.stepGain) + "\n"
            "  riseGain: " + std::to_string(self.riseGain) + "\n"
            "  turnGain: " + std::to_string(self.turnGain) + "\n"
            "  lateralGain: " + std::to_string(self.lateralGain) + "\n"
            "  trunkZOffset: " + std::to_string(self.trunkZOffset) + "\n"
            "  swingGain: " + std::to_string(self.swingGain) + "\n"
            "  swingRollGain: " + std::to_string(self.swingRollGain) + "\n"
            "  swingPhase: " + std::to_string(self.swingPhase) + "\n"
            "  stepUpVel: " + std::to_string(self.stepUpVel) + "\n"
            "  stepDownVel: " + std::to_string(self.stepDownVel) + "\n"
            "  riseUpVel: " + std::to_string(self.riseUpVel) + "\n"
            "  riseDownVel: " + std::to_string(self.riseDownVel) + "\n"
            "  swingPause: " + std::to_string(self.swingPause) + "\n"
            "  swingVel: " + std::to_string(self.swingVel) + "\n"
            "  trunkXOffset: " + std::to_string(self.trunkXOffset) + "\n"
            "  trunkYOffset: " + std::to_string(self.trunkYOffset) + "\n"
            "  trunkPitch: " + std::to_string(self.trunkPitch) + "\n"
            "  trunkRoll: " + std::to_string(self.trunkRoll) + "\n"
            "  extraLeftX: " + std::to_string(self.extraLeftX) + "\n"
            "  extraLeftY: " + std::to_string(self.extraLeftY) + "\n"
            "  extraLeftZ: " + std::to_string(self.extraLeftZ) + "\n"
            "  extraRightX: " + std::to_string(self.extraRightX) + "\n"
            "  extraRightY: " + std::to_string(self.extraRightY) + "\n"
            "  extraRightZ: " + std::to_string(self.extraRightZ) + "\n"
            "  extraLeftYaw: " + std::to_string(self.extraLeftYaw) + "\n"
            "  extraLeftPitch: " + std::to_string(self.extraLeftPitch) + "\n"
            "  extraLeftRoll: " + std::to_string(self.extraLeftRoll) + "\n"
            "  extraRightYaw: " + std::to_string(self.extraRightYaw) + "\n"
            "  extraRightPitch: " + std::to_string(self.extraRightPitch) + "\n"
            "  extraRightRoll: " + std::to_string(self.extraRightRoll) + "\n"
            ">";
            })
            .def("__getattr__", [](const Rhoban::IKWalkParameters& self, const std::string& name) {
                // Custom getattr function to access struct attributes through the __dict__ attribute
                if (name == "__dict__") {
                    py::dict dict;
                    dict["distHipToKnee"] = self.distHipToKnee;
                    dict["distKneeToAnkle"] = self.distKneeToAnkle;
                    dict["distAnkleToGround"] = self.distAnkleToGround;
                    dict["distFeetLateral"] = self.distFeetLateral;
                    dict["freq"] = self.freq;
                    dict["enabledGain"] = self.enabledGain;
                    dict["supportPhaseRatio"] = self.supportPhaseRatio;
                    dict["footYOffset"] = self.footYOffset;
                    dict["stepGain"] = self.stepGain;
                    dict["riseGain"] = self.riseGain;
                    dict["turnGain"] = self.turnGain;
                    dict["lateralGain"] = self.lateralGain;
                    dict["trunkZOffset"] = self.trunkZOffset;
                    dict["swingGain"] = self.swingGain;
                    dict["swingRollGain"] = self.swingRollGain;
                    dict["swingPhase"] = self.swingPhase;
                    dict["stepUpVel"] = self.stepUpVel;
                    dict["stepDownVel"] = self.stepDownVel;
                    dict["riseUpVel"] = self.riseUpVel;
                    dict["riseDownVel"] = self.riseDownVel;
                    dict["swingPause"] = self.swingPause;
                    dict["swingVel"] = self.swingVel;
                    dict["trunkXOffset"] = self.trunkXOffset;
                    dict["trunkYOffset"] = self.trunkYOffset;
                    dict["trunkPitch"] = self.trunkPitch;
                    dict["trunkRoll"] = self.trunkRoll;
                    dict["extraLeftX"] = self.extraLeftX;
                    dict["extraLeftY"] = self.extraLeftY;
                    dict["extraLeftZ"] = self.extraLeftZ;
                    dict["extraRightX"] = self.extraRightX;
                    dict["extraRightY"] = self.extraRightY;
                    dict["extraRightZ"] = self.extraRightZ;
                    dict["extraLeftYaw"] = self.extraLeftYaw;
                    dict["extraLeftPitch"] = self.extraLeftPitch;
                    dict["extraLeftRoll"] = self.extraLeftRoll;
                    dict["extraRightYaw"] = self.extraRightYaw;
                    dict["extraRightPitch"] = self.extraRightPitch;
                    dict["extraRightRoll"] = self.extraRightRoll;
                    return dict;
                }
                throw py::attribute_error("attribute not found");
                });
}

void init_ik_walk_outputs(py::module &m)
{
    py::class_<Rhoban::IKWalkOutputs>(m, "IKWalkOutputs")
        .def(py::init<>())
        .def_readwrite("phase", &Rhoban::IKWalkOutputs::phase)
        .def_readwrite("left_hip_yaw", &Rhoban::IKWalkOutputs::left_hip_yaw)
        .def_readwrite("left_hip_roll", &Rhoban::IKWalkOutputs::left_hip_roll)
        .def_readwrite("left_hip_pitch", &Rhoban::IKWalkOutputs::left_hip_pitch)
        .def_readwrite("left_knee", &Rhoban::IKWalkOutputs::left_knee)
        .def_readwrite("left_ankle_pitch", &Rhoban::IKWalkOutputs::left_ankle_pitch)
        .def_readwrite("left_ankle_roll", &Rhoban::IKWalkOutputs::left_ankle_roll)
        .def_readwrite("right_hip_yaw", &Rhoban::IKWalkOutputs::right_hip_yaw)
        .def_readwrite("right_hip_roll", &Rhoban::IKWalkOutputs::right_hip_roll)
        .def_readwrite("right_hip_pitch", &Rhoban::IKWalkOutputs::right_hip_pitch)
        .def_readwrite("right_knee", &Rhoban::IKWalkOutputs::right_knee)
        .def_readwrite("right_ankle_pitch", &Rhoban::IKWalkOutputs::right_ankle_pitch)
        .def_readwrite("right_ankle_roll", &Rhoban::IKWalkOutputs::right_ankle_roll)
        .def_readwrite("left_shoulder_roll", &Rhoban::IKWalkOutputs::left_shoulder_roll)
        .def_readwrite("left_shoulder_pitch", &Rhoban::IKWalkOutputs::left_shoulder_pitch)
        .def_readwrite("right_shoulder_roll", &Rhoban::IKWalkOutputs::right_shoulder_roll)
        .def_readwrite("right_shoulder_pitch", &Rhoban::IKWalkOutputs::right_shoulder_pitch)
        .def_readwrite("left_elbow", &Rhoban::IKWalkOutputs::left_elbow)
        .def_readwrite("right_elbow", &Rhoban::IKWalkOutputs::right_elbow)
        .def("__getattr__", [](const Rhoban::IKWalkOutputs& self, const std::string& name) {
        // Custom getattr function to access struct attributes through the __dict__ attribute
            if (name == "__dict__") {
                std::cout << "GETTING __dict__ attr" << std::endl;
                py::dict dict;
                dict["phase"] = self.phase;
                dict["left_hip_yaw"] = self.left_hip_yaw;
                dict["left_hip_roll"] = self.left_hip_roll;
                dict["left_hip_pitch"] = self.left_hip_pitch;
                dict["left_knee"] = self.left_knee;
                dict["left_ankle_pitch"] = self.left_ankle_pitch;
                dict["left_ankle_roll"] = self.left_ankle_roll;
                dict["right_hip_yaw"] = self.right_hip_yaw;
                dict["right_hip_roll"] = self.right_hip_roll;
                dict["right_hip_pitch"] = self.right_hip_pitch;
                dict["right_knee"] = self.right_knee;
                dict["right_ankle_pitch"] = self.right_ankle_pitch;
                dict["right_ankle_roll"] = self.right_ankle_roll;
                dict["left_shoulder_roll"] = self.left_shoulder_roll;
                dict["left_shoulder_pitch"] = self.left_shoulder_pitch;
                dict["right_shoulder_roll"] = self.right_shoulder_roll;
                dict["right_shoulder_pitch"] = self.right_shoulder_pitch;
                dict["left_elbow"] = self.left_elbow;
                dict["right_elbow"] = self.right_elbow;
                return dict;
            }
            std::cout << "GETTING " << name << " attr" << std::endl;
            throw py::attribute_error("attribute not found");
        })
        .def("__repr__", [](const Rhoban::IKWalkOutputs& outputs) {
            return "<IKWalkOutputs:\n"
                   "phase: " + std::to_string(outputs.phase) + "\n"
                   "  left_hip_yaw: " + std::to_string(outputs.left_hip_yaw) + "\n"
                   "  left_hip_roll: " + std::to_string(outputs.left_hip_roll) + "\n"
                   "  left_hip_pitch: " + std::to_string(outputs.left_hip_pitch) + "\n"
                   "  left_knee: " + std::to_string(outputs.left_knee) + "\n"
                   "  left_ankle_pitch: " + std::to_string(outputs.left_ankle_pitch) + "\n"
                   "  left_ankle_roll: " + std::to_string(outputs.left_ankle_roll) + "\n"
                   "  right_hip_yaw: " + std::to_string(outputs.right_hip_yaw) + "\n"
                   "  right_hip_roll: " + std::to_string(outputs.right_hip_roll) + "\n"
                   "  right_hip_pitch: " + std::to_string(outputs.right_hip_pitch) + "\n"
                   "  right_knee: " + std::to_string(outputs.right_knee) + "\n"
                   "  right_ankle_pitch: " + std::to_string(outputs.right_ankle_pitch) + "\n"
                   "  right_ankle_roll: " + std::to_string(outputs.right_ankle_roll) + "\n"
                   ">";
        });
}

void init_ik_walk_inputs(py::module &m)
{
    py::class_<Rhoban::IKWalkInputs>(m, "IKWalkInputs")
        .def(py::init<>())
        .def_readwrite("fusedErrorParams", &Rhoban::IKWalkInputs::fusedErrorParams)
        .def_readwrite("fusedErrors", &Rhoban::IKWalkInputs::fusedErrors)
        .def_readwrite("armParams", &Rhoban::IKWalkInputs::armParams)
        .def_readwrite("stepTiming", &Rhoban::IKWalkInputs::stepTiming)
        .def("__getattr__", [](const Rhoban::IKWalkInputs& self, const std::string& name) {
        // Custom getattr function to access struct attributes through the __dict__ attribute
            if (name == "__dict__") {
                std::cout << "GETTING __dict__ attr" << std::endl;
                py::dict dict;
                dict["fusedErrorParams"] = self.fusedErrorParams;
                dict["fusedErrors"] = self.fusedErrors;
                dict["armParams"] = self.armParams;
                dict["stepTiming"] = self.stepTiming;
                return dict;
            }
            std::cout << "GETTING " << name << " attr" << std::endl;
            throw py::attribute_error("attribute not found");
        });
        // .def("__repr__", [](const Rhoban::IKWalkInputs& inputs) {
        //     return "<IKWalkInputs:\n"
        //            "fusedErrorParams: " + std::to_string(inputs.fusedErrorParams) + "\n"
        //            "  fusedErrors: " + std::to_string(inputs.fusedErrors) + "\n"
        //            "  armParams: " + std::to_string(inputs.armParams) + "\n"
        //            ">";
        // });

    py::class_<Rhoban::IKWalkInputs::FeedbackErrorParams>(m, "FeedbackErrorParams")
        .def(py::init<>())
        .def_readwrite("fusedFeedbackX", &Rhoban::IKWalkInputs::FeedbackErrorParams::fusedFeedbackX)
        .def_readwrite("fusedFeedbackY", &Rhoban::IKWalkInputs::FeedbackErrorParams::fusedFeedbackY)
        .def_readwrite("fusedDerivativeFeedbackX", &Rhoban::IKWalkInputs::FeedbackErrorParams::fusedDerivativeFeedbackX)
        .def_readwrite("fusedDerivativeFeedbackY", &Rhoban::IKWalkInputs::FeedbackErrorParams::fusedDerivativeFeedbackY)
        .def_readwrite("fusedIntegralFeedbackX", &Rhoban::IKWalkInputs::FeedbackErrorParams::fusedIntegralFeedbackX)
        .def_readwrite("fusedIntegralFeedbackY", &Rhoban::IKWalkInputs::FeedbackErrorParams::fusedIntegralFeedbackY)
        .def("__getattr__", [](const Rhoban::IKWalkInputs::FeedbackErrorParams& self, const std::string& name) {
        // Custom getattr function to access struct attributes through the __dict__ attribute
            if (name == "__dict__") {
                std::cout << "GETTING __dict__ attr" << std::endl;
                py::dict dict;
                dict["fusedFeedbackX"] = self.fusedFeedbackX;
                dict["fusedFeedbackY"] = self.fusedFeedbackY;
                dict["fusedDerivativeFeedbackX"] = self.fusedDerivativeFeedbackX;
                dict["fusedDerivativeFeedbackY"] = self.fusedDerivativeFeedbackY;
                dict["fusedIntegralFeedbackX"] = self.fusedIntegralFeedbackX;
                dict["fusedIntegralFeedbackY"] = self.fusedIntegralFeedbackY;
                return dict;
            }
            std::cout << "GETTING " << name << " attr" << std::endl;
            throw py::attribute_error("attribute not found");
        });
        // .def("__repr__", [](const Rhoban::IKWalkInputs::FeedbackErrorParams& errorParams) {
        //     return "<FeedbackErrorParams:\n"
        //            "fusedFeedbackX: " + std::to_string(errorParams.fusedFeedbackX) + "\n"
        //            "fusedFeedbackY: " + std::to_string(errorParams.fusedFeedbackY) + "\n"
        //            "fusedDerivativeFeedbackX: " + std::to_string(errorParams.fusedDerivativeFeedbackX) + "\n"
        //            "fusedDerivativeFeedbackY: " + std::to_string(errorParams.fusedDerivativeFeedbackY) + "\n"
        //            "fusedIntegralFeedbackX: " + std::to_string(errorParams.fusedIntegralFeedbackX) + "\n"
        //            "fusedIntegralFeedbackY: " + std::to_string(errorParams.fusedIntegralFeedbackY) + "\n"
        //            ">";
        // });

    py::class_<Rhoban::IKWalkInputs::FusedError>(m, "FusedError")
        .def(py::init<>())
        .def_readwrite("fusedErrorX", &Rhoban::IKWalkInputs::FusedError::fusedErrorX)
        .def_readwrite("fusedErrorY", &Rhoban::IKWalkInputs::FusedError::fusedErrorY)
        .def_readwrite("fusedDerivativeErrorX", &Rhoban::IKWalkInputs::FusedError::fusedDerivativeErrorX)
        .def_readwrite("fusedDerivativeErrorY", &Rhoban::IKWalkInputs::FusedError::fusedDerivativeErrorY)
        .def_readwrite("fusedIntegralErrorX", &Rhoban::IKWalkInputs::FusedError::fusedIntegralErrorX)
        .def_readwrite("fusedIntegralErrorY", &Rhoban::IKWalkInputs::FusedError::fusedIntegralErrorY)
        .def("__getattr__", [](const Rhoban::IKWalkInputs::FusedError& self, const std::string& name) {
        // Custom getattr function to access struct attributes through the __dict__ attribute
            if (name == "__dict__") {
                std::cout << "GETTING __dict__ attr" << std::endl;
                py::dict dict;
                dict["fusedErrorX"] = self.fusedErrorX;
                dict["fusedErrorY"] = self.fusedErrorY;
                dict["fusedDerivativeErrorX"] = self.fusedDerivativeErrorX;
                dict["fusedDerivativeErrorY"] = self.fusedDerivativeErrorY;
                dict["fusedIntegralErrorX"] = self.fusedIntegralErrorX;
                dict["fusedIntegralErrorY"] = self.fusedIntegralErrorY;
                return dict;
            }
            std::cout << "GETTING " << name << " attr" << std::endl;
            throw py::attribute_error("attribute not found");
        });
        // .def("__repr__", [](const Rhoban::IKWalkInputs::FusedError& errorParams) {
        //     return "<FusedError:\n"
        //            "fusedErrorX: " + std::to_string(errorParams.fusedErrorX) + "\n"
        //            "fusedErrorY: " + std::to_string(errorParams.fusedErrorY) + "\n"
        //            "fusedDerivativeErrorX: " + std::to_string(errorParams.fusedDerivativeErrorX) + "\n"
        //            "fusedDerivativeErrorY: " + std::to_string(errorParams.fusedDerivativeErrorY) + "\n"
        //            "fusedIntegralErrorX: " + std::to_string(errorParams.fusedIntegralErrorX) + "\n"
        //            "fusedIntegralErrorY: " + std::to_string(errorParams.fusedIntegralErrorY) + "\n"
        //            ">";
        // });

    py::class_<Rhoban::IKWalkInputs::FeedbackArmParams>(m, "FeedbackArmParams")
        .def(py::init<>())
        .def_readwrite("biasArmAngleX", &Rhoban::IKWalkInputs::FeedbackArmParams::biasArmAngleX)
        .def_readwrite("biasArmAngleY", &Rhoban::IKWalkInputs::FeedbackArmParams::biasArmAngleY)
        .def_readwrite("fusedFeedbackArmAngleX", &Rhoban::IKWalkInputs::FeedbackArmParams::fusedFeedbackArmAngleX)
        .def_readwrite("fusedFeedbackArmAngleY", &Rhoban::IKWalkInputs::FeedbackArmParams::fusedFeedbackArmAngleY)
        .def_readwrite("fusedDerivativeFeedbackArmAngleX", &Rhoban::IKWalkInputs::FeedbackArmParams::fusedDerivativeFeedbackArmAngleX)
        .def_readwrite("fusedDerivativeFeedbackArmAngleY", &Rhoban::IKWalkInputs::FeedbackArmParams::fusedDerivativeFeedbackArmAngleY)
        .def_readwrite("fusedIntegralFeedbackArmAngleX", &Rhoban::IKWalkInputs::FeedbackArmParams::fusedIntegralFeedbackArmAngleX)
        .def_readwrite("fusedIntegralFeedbackArmAngleY", &Rhoban::IKWalkInputs::FeedbackArmParams::fusedIntegralFeedbackArmAngleY)
        .def_readwrite("extension", &Rhoban::IKWalkInputs::FeedbackArmParams::extension)
        .def("__getattr__", [](const Rhoban::IKWalkInputs::FeedbackArmParams& self, const std::string& name) {
        // Custom getattr function to access struct attributes through the __dict__ attribute
            if (name == "__dict__") {
                std::cout << "GETTING __dict__ attr" << std::endl;
                py::dict dict;
                dict["biasArmAngleX"] = self.biasArmAngleX;
                dict["biasArmAngleY"] = self.biasArmAngleY;
                dict["fusedFeedbackArmAngleX"] = self.fusedFeedbackArmAngleX;
                dict["fusedFeedbackArmAngleY"] = self.fusedFeedbackArmAngleY;
                dict["fusedDerivativeFeedbackArmAngleX"] = self.fusedDerivativeFeedbackArmAngleX;
                dict["fusedDerivativeFeedbackArmAngleY"] = self.fusedDerivativeFeedbackArmAngleY;
                dict["fusedIntegralFeedbackArmAngleX"] = self.fusedIntegralFeedbackArmAngleX;
                dict["fusedIntegralFeedbackArmAngleY"] = self.fusedIntegralFeedbackArmAngleY;
                dict["extension"] = self.extension;
                return dict;
            }
            std::cout << "GETTING " << name << " attr" << std::endl;
            throw py::attribute_error("attribute not found");
        });
        // .def("__repr__", [](const Rhoban::IKWalkInputs::FeedbackArmParams& armParams) {
        //     return "<FusedError:\n"
        //            "biasArmAngleX: " + std::to_string(armParams.biasArmAngleX) + "\n"
        //            "biasArmAngleY: " + std::to_string(armParams.biasArmAngleY) + "\n"
        //            "fusedFeedbackArmAngleX: " + std::to_string(armParams.fusedFeedbackArmAngleX) + "\n"
        //            "fusedFeedbackArmAngleY: " + std::to_string(armParams.fusedFeedbackArmAngleY) + "\n"
        //            "fusedDerivativeFeedbackArmAngleX: " + std::to_string(armParams.fusedDerivativeFeedbackArmAngleX) + "\n"
        //            "fusedDerivativeFeedbackArmAngleY: " + std::to_string(armParams.fusedDerivativeFeedbackArmAngleY) + "\n"
        //            "fusedIntegralFeedbackArmAngleX: " + std::to_string(armParams.fusedIntegralFeedbackArmAngleX) + "\n"
        //            "fusedIntegralFeedbackArmAngleY: " + std::to_string(armParams.fusedIntegralFeedbackArmAngleY) + "\n"
        //            ">";
        // });

        py::class_<Rhoban::IKWalkInputs::StepTimingParams>(m, "StepTimingParams")
        .def(py::init<>())
        .def_readwrite("deadbandRadius", &Rhoban::IKWalkInputs::StepTimingParams::deadbandRadius)
        .def_readwrite("gainSpeedUp", &Rhoban::IKWalkInputs::StepTimingParams::gainSpeedUp)
        .def_readwrite("gainSlowDown", &Rhoban::IKWalkInputs::StepTimingParams::gainSlowDown)
        .def_readwrite("weightFactor", &Rhoban::IKWalkInputs::StepTimingParams::weightFactor)
        .def_readwrite("doubleSupportPhaseLen", &Rhoban::IKWalkInputs::StepTimingParams::doubleSupportPhaseLen)
        .def_readwrite("gaitFrequencyMax", &Rhoban::IKWalkInputs::StepTimingParams::gaitFrequencyMax)
        .def("__getattr__", [](const Rhoban::IKWalkInputs::StepTimingParams& self, const std::string& name) {
        // Custom getattr function to access struct attributes through the __dict__ attribute
            if (name == "__dict__") {
                std::cout << "GETTING __dict__ attr" << std::endl;
                py::dict dict;
                dict["deadbandRadius"] = self.deadbandRadius;
                dict["gainSpeedUp"] = self.gainSpeedUp;
                dict["gainSlowDown"] = self.gainSlowDown;
                dict["weightFactor"] = self.weightFactor;
                dict["doubleSupportPhaseLen"] = self.doubleSupportPhaseLen;
                dict["gaitFrequencyMax"] = self.gaitFrequencyMax;
                return dict;
            }
            std::cout << "GETTING " << name << " attr" << std::endl;
            throw py::attribute_error("attribute not found");
        });
        // .def("__repr__", [](const Rhoban::IKWalkInputs::FeedbackArmParams& armParams) {
        //     return "<FusedError:\n"
        //            "biasArmAngleX: " + std::to_string(armParams.biasArmAngleX) + "\n"
        //            "biasArmAngleY: " + std::to_string(armParams.biasArmAngleY) + "\n"
        //            "fusedFeedbackArmAngleX: " + std::to_string(armParams.fusedFeedbackArmAngleX) + "\n"
        //            "fusedFeedbackArmAngleY: " + std::to_string(armParams.fusedFeedbackArmAngleY) + "\n"
        //            "fusedDerivativeFeedbackArmAngleX: " + std::to_string(armParams.fusedDerivativeFeedbackArmAngleX) + "\n"
        //            "fusedDerivativeFeedbackArmAngleY: " + std::to_string(armParams.fusedDerivativeFeedbackArmAngleY) + "\n"
        //            "fusedIntegralFeedbackArmAngleX: " + std::to_string(armParams.fusedIntegralFeedbackArmAngleX) + "\n"
        //            "fusedIntegralFeedbackArmAngleY: " + std::to_string(armParams.fusedIntegralFeedbackArmAngleY) + "\n"
        //            ">";
        // });
}

void init_ik_walk(py::module &m)
{
    py::class_<Rhoban::IKWalk>(m, "IKWalk")
        .def_static("walk", &Rhoban::IKWalk::walk,
            R"pbdoc(
            Compute and return target motor reference positions using given walk parameters at the given phase (between 0 and 1).
            The phase is updated according to the frequency parameter and the given time step dt.
            If inverse kinematics fail, false is returned and neither the phase nor the output is updated.

            Args:
                params (IKWalkParameters): IKWalkParameters object representing the walk parameters.
                dt (float): Time step for the walk computation.
                phase (float): Phase value between 0 and 1.
                outputs (IKWalkOutputs): IKWalkOutputs object to store the computed motor reference positions.

            Returns:
                bool: True if the walk computation is successful, False otherwise.
            )pbdoc")
        .def_static("boundPhase", &Rhoban::IKWalk::boundPhase, py::arg("phase"),
            R"pbdoc(
            Cycle the given phase between 0 and 1.

            Args:
                phase (float): Phase value to be cycled.
            )pbdoc");
}

PYBIND11_MODULE(starkit_ik_walk, m) {
    // Optional docstring
    m.doc() = "Simple Walk Engine library";
    
    init_ik_walk_parameters(m);
    init_ik_walk_outputs(m);
    init_ik_walk_inputs(m);
    init_ik_walk(m);

}

/**
 * Run the walk for given among of time and update
 * phase and time state
 */
void runWalk(
    Rhoban::IKWalkParameters& params, 
    double timeLength, 
    double& phase, 
    double& time)
{
    //Leg motor computed positions
    struct Rhoban::IKWalkOutputs outputs;

    struct Rhoban::IKWalkInputs inputs;
    
    //Walk engine frequency
    double engineFrequency = 50.0;

    for (double t=0.0;t<=timeLength;t+=1.0/engineFrequency) {
        time += 1.0/engineFrequency;
        bool success = Rhoban::IKWalk::walk(
            params, //Walk parameters
            1.0/engineFrequency, //Time step
            phase, //Current walk phase -will be updated)
            outputs, //Result target position (updated)
            inputs
        );
        if (!success) {
            //The requested position for left or right foot is not feasible
            //(phase is not updated)
            std::cout << time << " Inverse Kinematics error. Position not reachable." << std::endl;
        } else {
            std::cout << time << " ";
            std::cout << phase << " ";
            std::cout << outputs.left_hip_yaw << " ";
            std::cout << outputs.left_hip_pitch << " ";
            std::cout << outputs.left_hip_roll << " ";
            std::cout << outputs.left_knee << " ";
            std::cout << outputs.left_ankle_pitch << " ";
            std::cout << outputs.left_ankle_roll << " ";
            std::cout << outputs.right_hip_yaw << " ";
            std::cout << outputs.right_hip_pitch << " ";
            std::cout << outputs.right_hip_roll << " ";
            std::cout << outputs.right_knee << " ";
            std::cout << outputs.right_ankle_pitch << " ";
            std::cout << outputs.right_ankle_roll << " ";
            std::cout << std::endl;
        }
    } 
}

