#ifndef LEPH_IKWALKINPUTS_HPP
#define LEPH_IKWALKINPUTS_HPP

#include <iostream>
#include <vector>

namespace Rhoban
{
    struct IKWalkInputs // TODO: deal with name
    {

        struct FeedbackErrorParams
        {
            double fusedFeedbackX;
            double fusedFeedbackY;
            double fusedDerivativeFeedbackX;
            double fusedDerivativeFeedbackY;
            double fusedIntegralFeedbackX;
            double fusedIntegralFeedbackY;

        } fusedErrorParams;

        struct FusedError
        {
            double fusedErrorX;
            double fusedErrorY;
            double fusedDerivativeErrorX;
            double fusedDerivativeErrorY;
            double fusedIntegralErrorX;
            double fusedIntegralErrorY;
            
        } fusedErrors;
        
        struct FeedbackArmParams
        {
            double biasArmAngleX;
            double biasArmAngleY;
            double fusedFeedbackArmAngleX;
            double fusedFeedbackArmAngleY;
            double fusedDerivativeFeedbackArmAngleX;
            double fusedDerivativeFeedbackArmAngleY;
            double fusedIntegralFeedbackArmAngleX;
            double fusedIntegralFeedbackArmAngleY;

            double extension;

        } armParams;

        struct StepTimingParams
        {
            double deadbandRadius;
            double gainSpeedUp;
            double gainSlowDown;
            double weightFactor;
            double doubleSupportPhaseLen;
            double gaitFrequencyMax;

        } stepTiming;
        
    }; // struct IKWalkInputs
    
} // namespace Rhoban

#endif
