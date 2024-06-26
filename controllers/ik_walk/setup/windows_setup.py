import tkinter as tk
from tkinter import ttk

from setup.robot_params import attribute_ranges, labels, params, inputs, inputs_ranges

from setup.stabSetup import PIDStab, PIDStabLabels, PIDStabRanges
from setup.pulseSetup import pulseGen, pulseLabels, enableRandomPulse, pulseRanges

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

def create_window_3():
    global inputs
    global button_stab
    global 

    window = tk.Tk()
    window.title("Stabilization")
    
    for attr_name, (min_value, max_value) in inputs_ranges.items():
        label = ttk.Label(window, text=f"{attr_name}: {getattr(inputs.stepTiming, attr_name):.3f}")
        labels[attr_name] = label
        label.pack(pady=5)

        trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                command=lambda value, param_name=attr_name: update_inputs(param_name, value))
        trackbar.set(getattr(inputs.stepTiming, attr_name))
        trackbar.pack(pady=5)
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
w3 = create_window_3()
w3.geometry("+100+550")
rpwind = createRandomPulseWindow()
rpwind.geometry("+100+550")


def update_windows():
    global w1, w2, w3, rpwind
    w1.update_idletasks()
    w1.update()
    w2.update_idletasks()
    w2.update()
    w3.update_idletasks()
    w3.update()
    rpwind.update_idletasks()
    rpwind.update()