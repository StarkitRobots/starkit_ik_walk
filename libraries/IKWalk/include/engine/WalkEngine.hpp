#pragma once

#include <Eigen/Dense>

#include "model/HumanoidModel.hpp"
#include "model/JointCommands.hpp"
#include "model/ModelParams.hpp"

#include "utils/control/VariationBound.hpp"
#include "utils/geometry/Point.hpp"
#include "utils/math_functions.hpp"

#include "Foot.hpp"
#include "WalkEngineParams.hpp"

namespace rhoban
{
    class WalkEngine
    {
    public:

        explicit WalkEngine(const ModelParams modelParams, const WalkEngineParams params) : 
            model(modelParams), params(params), isLeftSupport(true), commands{0.0, 0.0, 0.0, false, false}, stepDuration(1.0 / (2.0 * params.frequency)), stepCount(0)
        {}

    public:

        bool computeAngles(JointCommands& angles, const double elapsed);

        void reset();

    public:

        inline constexpr auto defaultTrunkPitch() const
        {
            return utils::deg2rad(commands.stepSizeX > 0.0 
                ? params.trunkPitchNeutral + (params.trunkPitchForward - params.trunkPitchNeutral) * commands.stepSizeX / params.maxStepSizeX
                : params.trunkPitchNeutral + (params.trunkPitchBackward - params.trunkPitchNeutral) * commands.stepSizeX / params.maxStepSizeXBackward
            );
        }

        inline constexpr auto walkStarted() const noexcept
        {
            return stepCount >= 2;
        }

    public:

        void setCommands(const WalkEngineCommands& newCommands);
    
    private:

        void newStep();

    private:

        inline Foot& supportFoot()
        {
            return (isLeftSupport ? left : right);
        }

        inline Foot& flyingFoot()
        {
            return (isLeftSupport ? right : left);
        }


    private:
        
        bool isLeftSupport;

        std::size_t stepCount;

        double swingPhase;
        double _swingGain;

        double _manualSwing;

        double swingRollGain;

        double phase;
        double stepDuration;

        Foot left;
        Foot right;

        HumanoidModel model;

    public:

        WalkEngineParams params;
        WalkEngineCommands commands;

    }; // class WalkEngine

} // namespace rhoban
