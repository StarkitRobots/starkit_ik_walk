import copy
import json
import logging

from enum       import Enum
from pathlib    import Path
from sys        import stderr
from typing     import Any

import starkit_ik_walk as sk

from setup.robot_params import send_command

class WalkController:

    ENGINE_PARAMS_FILEPATH  = Path("engine_params.json")
    MODEL_PARAMS_FILEPATH   = Path("model_params.json")

    class State(Enum):
        NotWalking      = 0
        WalkStarting    = 1
        Walking         = 2

    def __init__(self) -> None:
        self.logger = logging.getLogger("WalkController")

        modelParams, engineParams = self.__loadParametersFromJson()

        self.engine     = sk.IKWalk(modelParams, engineParams)
        self.commands   = sk.IKWalkCommands()
        self.angles     = sk.IKWalkOutputs()

        self.state = self.State.NotWalking

    def __loadObjectFromJson(self, filepath, object) -> None:
        readParams = dict()
        with open(filepath) as paramsFile:
            readParams = json.loads(paramsFile.read())
        
        for name, value in readParams.items():
            if not hasattr(object, name):
                self.logger.warning(f"No attribute {name} in the object")
                continue
            
            setattr(object, name, value)

    def __loadModelParamsFromJson(self) -> sk.ModelParams:
        params = sk.ModelParams()
        self.__loadObjectFromJson(self.MODEL_PARAMS_FILEPATH, params)
        return params

    def __loadEngineParamsFromJson(self) -> sk.IKWalkParameters:
        params = sk.IKWalkParameters()
        self.__loadObjectFromJson(self.ENGINE_PARAMS_FILEPATH, params)
        return params

    def __loadParametersFromJson(self) -> tuple[sk.ModelParams, sk.IKWalkParameters]:

        return (
            self.__loadModelParamsFromJson(),
            self.__loadEngineParamsFromJson()
        )
    
    def step(self, elapsed: float) -> None:
        self.engine.setCommands(self.commands)

        if self.state == self.State.NotWalking:
            self.engine.reset()
            if self.engine.commands.enableWalk:
                self.state = self.State.WalkStarting

        elif self.state == self.State.WalkStarting:
            if self.engine.walkStarted():
                self.state = self.State.Walking

        else: #Walking
            if not self.engine.commands.enableWalk:
                self.engine.commands.stepSizeX = 0.0
                self.engine.commands.stepSizeY = 0.0
                self.engine.commands.stepSizeYaw = 0.0

                self.state = self.State.NotWalking
        
        self.engine.params.trunkPitch       = self.engine.defaultTrunkPitch()
        self.engine.params.trunkPitchOut    = self.engine.defaultTrunkPitch()

        if self.engine.walk(self.angles, elapsed):
            send_command(self.angles)
        else:
            print(" Inverse Kinematics error. Position not reachable.", file=stderr)

