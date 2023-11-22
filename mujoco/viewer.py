# Copyright 2022 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Interactive GUI viewer for MuJoCo."""

import abc
import atexit
import contextlib
import math
import os
import queue
import sys
import threading
import time
from typing import Callable, Optional, Tuple, Union
import weakref

import tkinter as tk
from tkinter import ttk

import glfw
import mujoco
from mujoco import _simulate
from mujoco import starkit_ik_walk as sk
import numpy as np

if not glfw._glfw:  # pylint: disable=protected-access
  raise RuntimeError('GLFW dynamic library handle is not available')
else:
  _simulate.set_glfw_dlhandle(glfw._glfw._handle)  # pylint: disable=protected-access

# Logarithmically spaced realtime slow-down coefficients (percent).
PERCENT_REALTIME = (
    100, 80, 66, 50, 40, 33, 25, 20, 16, 13,
    10, 8, 6.6, 5, 4, 3.3, 2.5, 2, 1.6, 1.3,
    1, 0.8, 0.66, 0.5, 0.4, 0.33, 0.25, 0.2, 0.16, 0.13,
    0.1
)

# Maximum time mis-alignment before re-sync.
MAX_SYNC_MISALIGN = 0.1

# Fraction of refresh available for simulation.
SIM_REFRESH_FRACTION = 0.7

CallbackType = Callable[[mujoco.MjModel, mujoco.MjData], None]
LoaderType = Callable[[], Tuple[mujoco.MjModel, mujoco.MjData]]
KeyCallbackType = Callable[[int], None]

# Loader function that also returns a file path for the GUI to display.
_LoaderWithPathType = Callable[[], Tuple[mujoco.MjModel, mujoco.MjData, str]]
_InternalLoaderType = Union[LoaderType, _LoaderWithPathType]

_Simulate = _simulate.Simulate

params = sk.IKWalkParameters()
params.distHipToKnee = 0.113# 0.093
params.distKneeToAnkle = 0.082# 0.105
params.distAnkleToGround = 0.024# 0.032
params.distFeetLateral = 0.079
params.freq = 2.25
params.enabledGain = 1.0
params.supportPhaseRatio = 0.0
params.footYOffset = 0.025
params.stepGain = 0.05
params.riseGain = 0.08
params.turnGain = 0.0
params.lateralGain = 0.0
params.trunkZOffset = 0.02
params.swingGain = 0.02
params.swingRollGain = 0.0
params.swingPhase = 0.25
params.stepUpVel = 4.0
params.stepDownVel = 4.0
params.riseUpVel = 4.0
params.riseDownVel = 5.0
params.swingPause = 0.0
params.swingVel = 4.0
params.trunkXOffset = 0.002 # 0.002
params.trunkYOffset = 0.0
params.trunkPitch = 0.12
params.trunkRoll = 0.0
params.extraLeftX = 0.0
params.extraLeftY = 0.0
params.extraLeftZ = 0.0
params.extraRightX = 0.0
params.extraRightY = 0.0
params.extraRightZ = 0.0
params.extraLeftYaw = 0.0
params.extraLeftPitch = 0.0
params.extraLeftRoll = 0.0
params.extraRightYaw = 0.0
params.extraRightPitch = 0.0
params.extraRightRoll = 0.0

phase = 0.0
labels = {}
button_enabled = None

attribute_ranges = {
    "stepGain": (-0.1, 0.1),
    "lateralGain": (-0.06, 0.06),
    "turnGain": (-0.5, 0.5),
    "freq": (0.1, 5.0),
    "supportPhaseRatio": (0.0, 1.0),
    "footYOffset": (-0.2, 0.2),
    "riseGain": (0.0, 0.1),
    "swingGain": (0.0, 0.1),
    "swingRollGain": (-1.0, 1.0),
    "swingPhase": (0.0, 1.0),
    # "stepUpVel": (0.0, 5.0),
    # "stepDownVel": (0.0, 5.0),
    # "riseUpVel": (0.0, 5.0),
    # "riseDownVel": (0.0, 5.0),
    "swingPause": (0.0, 0.5),
    "swingVel": (0.0, 5.0),
    "trunkXOffset": (-0.2, 0.2),
    "trunkYOffset": (-0.2, 0.2),
    "trunkZOffset": (0.01, 0.2),
    "trunkPitch": (-1.0, 1.0),
    "trunkRoll": (-1.0, 1.0)
}

ctrl_naming = {
              "head_pitch" : 0,
              "head_yaw" : 1,
              "right_elbow" : 2,
              "right_elbow_yaw" : 3,
              "right_shoulder_roll" : 4,
              "right_shoulder_pitch": 5,
              "left_elbow" : 6,
              "left_elbow_yaw" : 7,
              "left_shoulder_roll" : 8,
              "left_shoulder_pitch" : 9,
              "waist_yaw" : 10,
              "right_hip_yaw" : 11,
              "right_hip_roll" : 12,
              "right_hip_pitch" : 13,
              "right_upper_knee" : 14,
              "right_knee" : 15,
              "right_ankle_pitch" : 16,
              "right_ankle_roll" : 17,
              "left_hip_yaw" : 18,
              "left_hip_roll" : 19,
              "left_hip_pitch" : 20,
              "left_upper_knee" : 21,
              "left_knee" : 22,
              "left_ankle_pitch" : 23,
              "left_ankle_roll" : 24 }

dof_names = {
    'left_elbow' : 6,
    'right_elbow' : 2,
    'left_hip_yaw' : 18, 
    'left_hip_roll' : 19, 
    'left_hip_pitch' : 20,
    'left_upper_knee' : 21, 
    'left_knee' : 22,
    'left_ankle_pitch' : 23, 
    'left_ankle_roll' : 24, 
    'right_hip_yaw' : 11, 
    'right_hip_roll' : 12, 
    'right_hip_pitch' : 13, 
    'right_upper_knee' : 14, 
    'right_knee' : 15, 
    'right_ankle_pitch' : 16, 
    'right_ankle_roll' : 17}

def update_value(param_name, value):
    global params
    global labels
    print(float(value))
    setattr(params, param_name, float(value))
    if param_name in labels.keys():
        labels[param_name].configure(text=f"{param_name}: {float(value):.3f}")

def create_window_1():
    global params
    global labels
    global button_enabled
    global attribute_ranges
    window = tk.Tk()
    window.title("Control robot")
    
    button_enabled = ttk.Button(window, text="Enable", command=lambda: toggle_enabled())
    button_enabled.pack(padx=10, pady=10)
    
    for attr_name, (min_value, max_value) in attribute_ranges.items():
        if attr_name in ("stepGain", "lateralGain", "turnGain"):
            label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            labels[attr_name] = label
            label.pack(pady=5)
        
            trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 command=lambda value, param_name=attr_name: update_value(param_name, value))
            trackbar.set(getattr(params, attr_name))
            trackbar.pack(pady=5)
    return window

def create_window_2():
    global params
    global labels
    global attribute_ranges
    window = tk.Tk()
    window.title("Parameter settings")
    
    attribute_names = vars(params).keys()
    
    trackbars_per_row = 4
    current_row = 0
    current_column = 0

    for attr_name, (min_value, max_value) in attribute_ranges.items():
        if attr_name not in ("stepGain", "lateralGain", "turnGain"):
            label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            label.grid(row=current_row, column=current_column, padx=5, pady=5)
            labels[attr_name] = label

            trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 command=lambda value, param_name=attr_name: update_value(param_name, value))
            trackbar.set(getattr(params, attr_name))
            trackbar.grid(row=current_row, column=current_column+1, padx=5, pady=5)

            current_column += 2
            if current_column >= trackbars_per_row * 2:
                current_row += 1
                current_column = 0
    return window

def toggle_enabled():
    global params
    global labels
    params.enabledGain = 1.0 if params.enabledGain == 0.0 else 0.0
    button_enabled.configure(text="Disable" if params.enabledGain == 1.0 else "Enable")

w1 = create_window_1()
w1.geometry("+1200+200")
w2 = create_window_2()
w2.geometry("+100+550")

class Handle:
  """A handle for interacting with a MuJoCo viewer."""

  def __init__(
      self,
      sim: _Simulate,
      cam: mujoco.MjvCamera,
      opt: mujoco.MjvOption,
      pert: mujoco.MjvPerturb,
      user_scn: Optional[mujoco.MjvScene],
  ):
    self._sim = weakref.ref(sim)
    self._cam = cam
    self._opt = opt
    self._pert = pert
    self._user_scn = user_scn

  @property
  def cam(self):
    return self._cam

  @property
  def opt(self):
    return self._opt

  @property
  def perturb(self):
    return self._pert

  @property
  def user_scn(self):
    return self._user_scn

  def close(self):
    sim = self._sim()
    if sim is not None:
      sim.exit()

  def _get_sim(self) -> Optional[_Simulate]:
    sim = self._sim()
    if sim is not None:
      try:
        return sim if sim.exitrequest == 0 else None
      except mujoco.UnexpectedError:
        # UnexpectedError is raised when accessing `exitrequest` after the
        # underlying simulate instance has been deleted in C++.
        return None
    return None

  def is_running(self) -> bool:
    return self._get_sim() is not None

  def lock(self):
    sim = self._get_sim()
    if sim is not None:
      return sim.lock()
    return contextlib.nullcontext()

  def sync(self):
    sim = self._get_sim()
    if sim is not None:
      sim.sync()  # locks internally

  def update_hfield(self, hfieldid: int):
    sim = self._get_sim()
    if sim is not None:
      sim.update_hfield(hfieldid)  # locks internally and blocks until done

  def update_mesh(self, meshid: int):
    sim = self._get_sim()
    if sim is not None:
      sim.update_mesh(meshid)  # locks internally and blocks until done

  def update_texture(self, texid: int):
    sim = self._get_sim()
    if sim is not None:
      sim.update_texture(texid)  # locks internally and blocks until done

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.close()


# Abstract base dispatcher class for systems that require UI calls to be made
# on a specific thread (e.g. macOS). This is subclassed by system-specific
# Python launcher (mjpython) to implement the required dispatching mechanism.
class _MjPythonBase(metaclass=abc.ABCMeta):

  def launch_on_ui_thread(
      self,
      model: mujoco.MjModel,
      data: mujoco.MjData,
      handle_return: Optional['queue.Queue[Handle]'],
      key_callback: Optional[KeyCallbackType],
  ):
    pass


# When running under mjpython, the launcher initializes this object.
_MJPYTHON: Optional[_MjPythonBase] = None


def _file_loader(path: str) -> _LoaderWithPathType:
  """Loads an MJCF model from file path."""

  def load(path=path) -> Tuple[mujoco.MjModel, mujoco.MjData, str]:
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    return m, d, path

  return load


def _reload(
    simulate: _Simulate, loader: _InternalLoaderType,
    notify_loaded: Optional[Callable[[], None]] = None
) -> Optional[Tuple[mujoco.MjModel, mujoco.MjData]]:
  """Internal function for reloading a model in the viewer."""
  try:
    simulate.load_message('') # path is unknown at this point
    load_tuple = loader()
  except Exception as e:  # pylint: disable=broad-except
    simulate.load_error = str(e)
    simulate.load_message_clear()
  else:
    m, d = load_tuple[:2]

    # If the loader does not raise an exception then we assume that it
    # successfully created mjModel and mjData. This is specified in the type
    # annotation, but we perform a runtime assertion here as well to prevent
    # possible segmentation faults.
    assert m is not None and d is not None

    path = load_tuple[2] if len(load_tuple) == 3 else ''
    simulate.load(m, d, path)

    if notify_loaded:
      notify_loaded()

    return m, d


def _physics_loop(simulate: _Simulate, loader: Optional[_InternalLoaderType]):
  """Physics loop for the GUI, to be run in a separate thread."""
  m: mujoco.MjModel = None
  d: mujoco.MjData = None
  ctrl_noise = np.array([])
  reload = True
  global phase
  # CPU-sim synchronization point.
  synccpu = 0.0
  syncsim = 0.0

  # Run until asked to exit.
  while not simulate.exitrequest:
    if simulate.droploadrequest:
      simulate.droploadrequest = 0
      loader = _file_loader(simulate.dropfilename)
      reload = True

    if simulate.uiloadrequest:
      simulate.uiloadrequest_decrement()
      reload = True

    if reload and loader is not None:
      result = _reload(simulate, loader)
      if result is not None:
        m, d = result
        ctrl_noise = np.zeros((m.nu,))

    reload = False

    # Sleep for 1 ms or yield, to let main thread run.
    if simulate.run != 0 and simulate.busywait != 0:
      time.sleep(0)
    else:
      time.sleep(0.001)

    with simulate.lock():
      if m is not None:
        assert d is not None
        if simulate.run:
          stepped = False
          # Record CPU time at start of iteration.
          startcpu = time.time()

          elapsedcpu = startcpu - synccpu
          elapsedsim = d.time - syncsim
          
          # print(*m.actuator_acc0)
          # num = d.MjNum
          print(d.ctrl)
          print(f"ELAPSEDSIM {elapsedsim}")
          # for elem in d.ctrl:

          print("-------------------------------")

          # Inject noise.
          if simulate.ctrl_noise_std != 0.0:
            # Convert rate and scale to discrete time (Ornstein–Uhlenbeck).
            rate = math.exp(-m.opt.timestep /
                            max(simulate.ctrl_noise_rate, mujoco.mjMINVAL))
            scale = simulate.ctrl_noise_std * math.sqrt(1 - rate * rate)
            
            for i in range(m.nu):
              # Update noise.
              ctrl_noise[i] = (rate * ctrl_noise[i] +
                               scale * mujoco.mju_standardNormal(None))

              # Apply noise.
              d.ctrl[i] = ctrl_noise[i]

          # Requested slow-down factor.
          slowdown = 100 / PERCENT_REALTIME[simulate.real_time_index]

          # Misalignment: distance from target sim time > MAX_SYNC_MISALIGN.
          misaligned = abs(elapsedcpu / slowdown -
                           elapsedsim) > MAX_SYNC_MISALIGN

          # Out-of-sync (for any reason): reset sync times, step.
          timestep = elapsedsim
          def send_command(command: sk.IKWalkOutputs):
            for name in dof_names:
              if "elbow" in name:
                  # motor.setPosition(-2.5)
                  d.ctrl[dof_names[name]] = 0
              elif "upper_knee" in name:
                  d.ctrl[dof_names[name]] = 0
              else:
                  d.ctrl[dof_names[name]] = getattr(command, name)
          outputs = sk.IKWalkOutputs()
          # if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
          if sk.IKWalk.walk(params, 0.002, phase, outputs):
            send_command(outputs)
            # global phase
            phase = outputs.phase
          if (elapsedsim < 0 or elapsedcpu < 0 or synccpu == 0 or misaligned or
              simulate.speed_changed):
            # Re-sync.
            synccpu = startcpu
            syncsim = d.time
            simulate.speed_changed = False

            # Run single step, let next iteration deal with timing.
            mujoco.mj_step(m, d)
            stepped = True

          # In-sync: step until ahead of cpu.
          else:
            measured = False
            prevsim = d.time
            refreshtime = SIM_REFRESH_FRACTION / simulate.refresh_rate
            # Step while sim lags behind CPU and within refreshtime.
            while (((d.time - syncsim) * slowdown <
                    (time.time() - synccpu)) and
                   ((time.time() - startcpu) < refreshtime)):
              # Measure slowdown before first step.
              if not measured and elapsedsim:
                simulate.measured_slowdown = elapsedcpu / elapsedsim
                measured = True

              # Call mj_step.
              mujoco.mj_step(m, d)
              stepped = True

              # Break if reset.
              if d.time < prevsim:
                break

          # save current state to history buffer
          if (stepped):
            simulate.add_to_history()
        else:  # simulate.run is False: GUI is paused.

          # Run mj_forward, to update rendering and joint sliders.
          mujoco.mj_forward(m, d)
          simulate.speed_changed = True

def tkinter_loop(simulate: _Simulate):
  m: mujoco.MjModel = None
  d: mujoco.MjData = None
  ctrl_noise = np.array([])
  reload = True
  while not simulate.exitrequest:
    w1.update_idletasks()
    w1.update()
    w2.update_idletasks()
    w2.update()

def _launch_internal(
    model: Optional[mujoco.MjModel] = None,
    data: Optional[mujoco.MjData] = None,
    *,
    run_physics_thread: bool,
    loader: Optional[_InternalLoaderType] = None,
    handle_return: Optional['queue.Queue[Handle]'] = None,
    key_callback: Optional[KeyCallbackType] = None,
    show_left_ui: bool = True,
    show_right_ui: bool = True,
) -> None:
  """Internal API, so that the public API has more readable type annotations."""
  if model is None and data is not None:
    raise ValueError('mjData is specified but mjModel is not')
  elif callable(model) and data is not None:
    raise ValueError(
        'mjData should not be specified when an mjModel loader is used')
  elif loader is not None and model is not None:
    raise ValueError('model and loader are both specified')
  elif run_physics_thread and handle_return is not None:
    raise ValueError('run_physics_thread and handle_return are both specified')

  if loader is None and model is not None:

    def _loader(m=model, d=data) -> Tuple[mujoco.MjModel, mujoco.MjData]:
      if d is None:
        d = mujoco.MjData(m)
      return m, d

    loader = _loader

  cam = mujoco.MjvCamera()
  opt = mujoco.MjvOption()
  pert = mujoco.MjvPerturb()
  if model and not run_physics_thread:
    user_scn = mujoco.MjvScene(model, _Simulate.MAX_GEOM)
  else:
    user_scn = None
  simulate = _Simulate(
      cam, opt, pert, user_scn, run_physics_thread, key_callback
  )

  simulate.ui0_enable = show_left_ui
  simulate.ui1_enable = show_right_ui

  # Initialize GLFW if not using mjpython.
  if _MJPYTHON is None:
    if not glfw.init():
      raise mujoco.FatalError('could not initialize GLFW')
    atexit.register(glfw.terminate)

  notify_loaded = None
  if handle_return:
    notify_loaded = lambda: handle_return.put_nowait(
        Handle(simulate, cam, opt, pert, user_scn)
    )

  if run_physics_thread:
    side_thread = threading.Thread(
        target=_physics_loop, args=(simulate, loader))
  else:
    side_thread = threading.Thread(
        target=_reload, args=(simulate, loader, notify_loaded))
  # tkinter_thread = threading.Thread(
  #       target=tkinter_loop(simulate))
  def make_exit(simulate):
    def exit_simulate():
      simulate.exit()
    return exit_simulate

  exit_simulate = make_exit(simulate)
  atexit.register(exit_simulate)

  side_thread.start()
  simulate.render_loop()
  # tkinter_thread.start()
  atexit.unregister(exit_simulate)
  side_thread.join()
  simulate.destroy()

def launch(
    model: Optional[mujoco.MjModel] = None,
    data: Optional[mujoco.MjData] = None,
    *,
    loader: Optional[LoaderType] = None,
    show_left_ui: bool = True,
    show_right_ui: bool = True,
) -> None:
  """Launches the Simulate GUI."""
  _launch_internal(
      model,
      data,
      run_physics_thread=True,
      loader=loader,
      show_left_ui=show_left_ui,
      show_right_ui=show_right_ui,
  )


def launch_from_path(path: str) -> None:
  """Launches the Simulate GUI from file path."""
  _launch_internal(run_physics_thread=True, loader=_file_loader(path))


def launch_passive(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    key_callback: Optional[KeyCallbackType] = None,
    show_left_ui: bool = True,
    show_right_ui: bool = True,
) -> Handle:
  """Launches a passive Simulate GUI without blocking the running thread."""
  if not isinstance(model, mujoco.MjModel):
    raise ValueError(f'`model` is not a mujoco.MjModel: got {model!r}')
  if not isinstance(data, mujoco.MjData):
    raise ValueError(f'`data` is not a mujoco.MjData: got {data!r}')
  if key_callback is not None and not callable(key_callback):
    raise ValueError(
        f'`key_callback` is not callable: got {key_callback!r}')

  mujoco.mj_forward(model, data)
  handle_return = queue.Queue(1)

  if sys.platform != 'darwin':
    thread = threading.Thread(
        target=_launch_internal,
        args=(model, data),
        kwargs=dict(
            run_physics_thread=False,
            handle_return=handle_return,
            key_callback=key_callback,
            show_left_ui=show_left_ui,
            show_right_ui=show_right_ui,
        ),
    )
    thread.daemon = True
    thread.start()
  else:
    if not isinstance(_MJPYTHON, _MjPythonBase):
      raise RuntimeError(
          '`launch_passive` requires that the Python script be run under '
          '`mjpython` on macOS')
    _MJPYTHON.launch_on_ui_thread(
        model,
        data,
        handle_return,
        key_callback,
        show_left_ui,
        show_right_ui,
    )

  return handle_return.get()


if __name__ == '__main__':
  # pylint: disable=g-bad-import-order
  from absl import app  # pylint: disable=g-import-not-at-top
  from absl import flags  # pylint: disable=g-import-not-at-top

  _MJCF_PATH = flags.DEFINE_string('mjcf', None, 'Path to MJCF file.')

  def main(argv) -> None:
    del argv
    if _MJCF_PATH.value is not None:
      launch_from_path(os.path.expanduser(_MJCF_PATH.value))
    else:
      launch()

  app.run(main)
