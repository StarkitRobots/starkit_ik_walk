import numpy as np

from feedback.mean_filter       import MeanFilter
from feedback.wlbf_filter       import WLBFFilter

from feedback.ew_integrator     import EWIntegrator

from feedback.smooth_deadband   import SmoothDeadband
from feedback.matrix_multiplier import MatrixMultiplier

from feedback.pipeline              import Pipeline
from feedback.step_timing_pipeline  import StepTimingPipeline

from setup.robot_params import robot, inputs

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# filters
fusedFeedFilter     = MeanFilter((9,    2))
dFusedFeedFilter    = WLBFFilter((30,   2), np.array([timestep/ 1000, timestep / 1000]))
iFusedFeedFilter    = EWIntegrator(timestep / 1000, 0.5)

# integrator for iFused Pipeline

# smooth deadband
fusedFeedSmoothDead     = SmoothDeadband(np.array([1,   0.035]),    np.array([0, 0]))
dFusedFeedSmoothDead    = SmoothDeadband(np.array([1,   0.08]),     np.array([0, 0]))
iFusedFeedSmoothDead    = SmoothDeadband(np.array([0.1, 0.1]),      np.array([0, 0]))

stepTimingSmoothDead    = SmoothDeadband(np.array([inputs.stepTiming.deadbandRadius]), np.array([0]))

# matricies
fusedFeedMatrix = MatrixMultiplier(
    np.array([[1, 0], 
              [0, 1]])
)

dFusedFeedMatrix = MatrixMultiplier(
    np.array([[1, 0  ], 
              [0, 0.1]])
)

iFusedFeedMatrix = MatrixMultiplier(
    np.array([[1, 0], 
              [0, 1]])
)

# pipelines
fusedPipeline = Pipeline(
    [
        fusedFeedFilter,
        fusedFeedSmoothDead,
        fusedFeedMatrix,
    ]
)

dFusedPipeline = Pipeline(
    [
        dFusedFeedFilter,
        dFusedFeedSmoothDead,
        dFusedFeedMatrix,
    ]
)

iFusedPipeline = Pipeline(
    [
        iFusedFeedFilter,
        iFusedFeedSmoothDead,
        iFusedFeedMatrix,
    ]
)

stepTimingPipeline = StepTimingPipeline(fusedFeedFilter, stepTimingSmoothDead)

def handle(deviations):
    global fusedPipeline, dFusedPipeline, iFusedPipeline
    global inputs

    (
        inputs.fusedErrors.fusedErrorX,             inputs.fusedErrors.fusedErrorY,
        inputs.fusedErrors.fusedDerivativeErrorX,   inputs.fusedErrors.fusedDerivativeErrorY,
        inputs.fusedErrors.fusedIntegralErrorX,     inputs.fusedErrors.fusedIntegralErrorY,
    ) = (
        *fusedPipeline.handle(deviations), 
        *dFusedPipeline.handle(deviations), 
        *(iFusedPipeline.handle(deviations)),
    )

    return inputs.fusedErrors

