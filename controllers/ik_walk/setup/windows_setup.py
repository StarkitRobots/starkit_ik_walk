import tkinter as tk
from tkinter import ttk

# from setup.robot_params import attribute_ranges, labels, params, inputs, inputs_ranges

# from setup.stabSetup import PIDStab
# from setup.stabSetup import enableStepTimingName,       stepTimingRanges
# from setup.stabSetup import enableFusedRollCOMShift,    fusedRollCOMShiftRanges
# from setup.stabSetup import enableFusedRollHip,         fusedRollHipRanges
# from setup.stabSetup import enableFusedPitchArm,        fusedPitchArmRanges
# from setup.stabSetup import enableFusedPitchHip,        fusedPitchHipRanges

# from setup.stabSetup import fusedRollStabRanges
# from setup.stabSetup import fusedPitchStabRanges

# from setup.stabSetup import pressStab, pressStabLabels, pressStabRanges
# from setup.pulseSetup import pulseGen, pulseLabels, enableRandomPulse, pulseRanges

class RobotSettingsWindow:

    def __init__(self, object, ranges: dict, name: str, enableName: str=None) -> None:
        self.object = object


        self.window = tk.Tk()
        self.window.title(name)
        
        if not enableName is None:
            self.enableName = enableName
            self.enableButton = self.__configureEnableButton()
        
        self.labels = dict()
        for parameterName, (minValue, maxValue) in ranges.items():
            self.__configureTrackbar(parameterName, minValue, maxValue)

    def __configureEnableButton(self) -> ttk.Button:
        enableButton = ttk.Button(self.window, text="Disable" if bool(getattr(self.object, self.enableName)) else "Enable", command=self.__toggleEnableButton)
        enableButton.pack(padx=10, pady=10)

        return enableButton
    
    def __configureTrackbar(self, name: str, minValue, maxValue) -> None:
        label = ttk.Label(self.window, text=f"{name}: {getattr(self.object, name):.3f}")
        self.labels[name] = label
        label.pack(pady=5)

        trackbar = ttk.Scale(self.window, from_=minValue, to=maxValue, length=200, orient=tk.HORIZONTAL,
                            command=lambda value, name=name: self.__updateParameter(name, value))
        trackbar.set(getattr(self.object, name))
        trackbar.pack(pady=5)

    def __toggleEnableButton(self) -> None:
        setattr(self.object, self.enableName, not bool(getattr(self.object, self.enableName)))
        self.enableButton.configure(text="Disable" if bool(getattr(self.object, self.enableName)) else "Enable")

    def __updateParameter(self, name: str, value: float) -> None:
        setattr(self.object, name, float(value))

        if name in self.labels.keys():
            self.labels[name].configure(text=f"{name}: {getattr(self.object, name):.3f}")

    def update(self) -> None:
        self.window.update_idletasks()
        self.window.update()

randomPulseEnabledButton = None

def createRandomPulseWindow():
    global pulseLabels
    global randomPulseEnabledButton

    window  = tk.Tk()
    window.title("Random Pulse Params")

    randomPulseEnabledButton = ttk.Button(window, text="Enable", command=toggleEnableRandomPulse)
    randomPulseEnabledButton.pack(padx=10, pady=10)

    for parameterName, (minVal, maxVal) in pulseRanges.items():
        label = ttk.Label(window, text=f"{parameterName}: {getattr(pulseGen, parameterName):.3f}")
        pulseLabels[parameterName] = label
        label.pack(pady=5)

        trackbar = ttk.Scale(window, from_=minVal, to=maxVal, length=200, orient=tk.HORIZONTAL,
                                 command=lambda value, name=parameterName: updatePulseGen(name, value))
        
        trackbar.set(getattr(pulseGen, parameterName))
        trackbar.pack(pady=5)

    return window

def toggleEnableRandomPulse():
    global enableRandomPulse
    global randomPulseEnabledButton

    enableRandomPulse = True
    randomPulseEnabledButton.configure(text="Disable" if enableRandomPulse else "Enable")

def updatePulseGen(paramName, value):
    global pulseLabels
    global pulseGen

    setattr(pulseGen, paramName, float(value) if "count" not in paramName else int(float(value)))
    if paramName in pulseLabels.keys():
        pulseLabels[paramName].configure(text=f"{paramName}: {getattr(pulseGen, paramName):.3f}")

    if "ampl" not in paramName:
        pulseGen.launch()


button_enabled = None
button_stab = None

def update_value(param_name, value):
    global params
    global labels
    setattr(params, param_name, float(value))
    if param_name in labels.keys():
        labels[param_name].configure(text=f"{param_name}: {float(value):.3f}")

def update_inputs(param_name, value):
    global inputs
    global labels
    setattr(inputs.stepTiming, param_name, float(value))
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

def createPIDStabWindow():
    global PIDStab
    global PIDStabLabels
    global PIDStabRanges

    window = tk.Tk()
    window.title("PID Stabilization")
    
    for attrName, (minValue, maxValue) in PIDStabRanges.items():
        label = ttk.Label(window, text=f"{attrName}: {getattr(PIDStab.params, attrName):.3f}")
        PIDStabLabels[attrName] = label
        label.pack(pady=5)

        trackbar = ttk.Scale(window, from_=minValue, to=maxValue, length=200, orient=tk.HORIZONTAL,
                                command=lambda value, paramName=attrName: updatePIDStabParams(paramName, value))
        trackbar.set(getattr(PIDStab.params, attrName))
        trackbar.pack(pady=5)

    return window

def updatePIDStabParams(paramName, value):
    global PIDStab
    global PIDStabLabels

    setattr(PIDStab.params, paramName, float(value))
    if paramName in PIDStabLabels.keys():
        PIDStabLabels[paramName].configure(text=f"{paramName}: {getattr(PIDStab.params, paramName):.3f}")


def createPressStabWindow():
    global pressStab
    global pressStabLabels
    global pressStabRanges

    window = tk.Tk()
    window.title("Pressure Stabilization")
    
    for attrName, (minValue, maxValue) in pressStabRanges.items():
        label = ttk.Label(window, text=f"{attrName}: {getattr(pressStab.params, attrName):.3f}")
        pressStabLabels[attrName] = label
        label.pack(pady=5)

        trackbar = ttk.Scale(window, from_=minValue, to=maxValue, length=200, orient=tk.HORIZONTAL,
                                command=lambda value, paramName=attrName: updatePressStabParams(paramName, value))
        trackbar.set(getattr(pressStab.params, attrName))
        trackbar.pack(pady=5)

    return window

def updatePressStabParams(paramName, value):
    global pressStab
    global pressStabLabels

    setattr(pressStab.params, paramName, float(value))
    if paramName in pressStabLabels.keys():
        pressStabLabels[paramName].configure(text=f"{paramName}: {getattr(pressStab.params, paramName):.3f}")

def toggle_enabled():
    global params
    global labels

    params.enabledGain = 1.0 if params.enabledGain == 0.0 else 0.0
    button_enabled.configure(text="Disable" if params.enabledGain == 1.0 else "Enable")

# w1 = create_window_1()
# w1.geometry("250x275+1525+0")
# w2 = create_window_2()
# w2.geometry("1450x130+0+0")

# PIDStabFusedRollWindow  = RobotSettingsWindow(PIDStab.params, fusedRollStabRanges,  "Fused Roll Stabilization")
# PIDStabFusedPitchWindow = RobotSettingsWindow(PIDStab.params, fusedPitchStabRanges, "Fused Pitch Stabilization")

# PIDStabStepTimingWindow     = RobotSettingsWindow(PIDStab.params, stepTimingRanges,         "Step Timing Stabilization",    enableStepTimingName)
# PIDStabRollCOMShiftWindow   = RobotSettingsWindow(PIDStab.params, fusedRollCOMShiftRanges,  "COM Shift Stabilization",      enableFusedRollCOMShift)
# PIDStabRollHipWindow        = RobotSettingsWindow(PIDStab.params, fusedRollHipRanges,       "Roll Hip Stabilization",       enableFusedRollHip)
# PIDStabPitchArmWindow       = RobotSettingsWindow(PIDStab.params, fusedPitchArmRanges,      "Pitch Arm Stabilization",      enableFusedPitchArm)
# PIDStabPitchHipWindow       = RobotSettingsWindow(PIDStab.params, fusedPitchHipRanges,      "Pitch Hip Stabilization",      enableFusedPitchHip)

# PIDStabFusedRollWindow.window.geometry("250x275+75+525")
# PIDStabFusedPitchWindow.window.geometry("250x275+325+525")

# PIDStabStepTimingWindow.window  .geometry("250x275+75+200")
# PIDStabRollCOMShiftWindow.window.geometry("250x275+325+200")
# PIDStabRollHipWindow.window     .geometry("250x275+575+200")
# PIDStabPitchHipWindow.window    .geometry("250x275+825+200")
# PIDStabPitchArmWindow.window    .geometry("250x335+1075+200")

# PIDStabWindow = RobotSettingsWindow(PIDStab.params, PIDStabRanges, "Arm Stabilization", "enableFusedPitchArm")
# PIDStabWindow.window.geometry("250x400+0+200")
# rpwind = createRandomPulseWindow()
# rpwind.geometry("250x275+1525+700")
# pressStabWindow = createPressStabWindow()
# pressStabWindow.geometry("250x300+1525+350")


def update_windows():
    global w1, w2, PIDStabWindow, rpwind
    w1.update_idletasks()
    w1.update()
    w2.update_idletasks()
    w2.update()

    # PIDStabFusedRollWindow.update()
    PIDStabFusedPitchWindow.update()

    # PIDStabStepTimingWindow.update()
    # PIDStabRollCOMShiftWindow.update()
    # PIDStabRollHipWindow.update()
    # PIDStabPitchArmWindow.update()
    PIDStabPitchHipWindow.update()

    rpwind.update_idletasks()
    rpwind.update()
    # pressStabWindow.update_idletasks()
    # pressStabWindow.update()