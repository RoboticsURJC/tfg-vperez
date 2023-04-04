import tkinter as tk
from tkinter import ttk
import time, threading
import robot

def moveJoint1Up():
    print("Moving joint 1 up...")

def moveJoint2Up():
    print("Moving joint 2 up...")

def moveJoint3Up():
    print("Moving joint 3 up...")   

def moveJoint1Down():
    print("Moving joint 1 down...")

def moveJoint2Down():
    print("Moving joint 2 down...")

def moveJoint3Down():
    print("Moving joint 3 down...") 
    

def moveXUp():
    print("Moving X up...")

def moveYUp():
    print("Moving Y up...")

def moveZUp():
    print("Moving Z up...")   

def moveXDown():
    print("Moving X down...")

def moveYDown():
    print("Moving Y down...")

def moveZDown():
    print("Moving Z down...")   

def increaseJointStep():
    jointStepSize.set(min(15, jointStepSize.get() + 1))

def decreaseJointStep():
    jointStepSize.set(max(1, jointStepSize.get() - 1))
    
def increaseXYZStep():
    XYZStepSize.set(min(30, XYZStepSize.get() + 1))

def decreaseXYZStep():
    XYZStepSize.set(max(1, XYZStepSize.get() - 1))

def statusThread():
    while True:
        if isConnected.get():
            machineStatus.set("[IDLE]")
        else:
            machineStatus.set("[OFF]")

def connectDisconnect():
    
    if isConnected.get():
        arm.stop()
        print("Disconnected!")
        canvas.itemconfig(statusLed, fill="red")
        isConnected.set(False)
        connectDisconnectText.set("Connect")
    else:
        print("Trying to connect to port: <" + selectedOption.get() + "> ...")
            
        if True:#arm.start(selectedOption.get()):
            canvas.itemconfig(statusLed, fill="#90EE90")
            isConnected.set(True)
            connectDisconnectText.set("Disconnect")
            print("Connected!")


arm = robot.Robot()

root = tk.Tk()
root.title("G-Arm Pendant")
root.resizable(False, False)

jointStepSize = tk.IntVar(value=1)
XYZStepSize = tk.IntVar(value=1)
machineStatus = tk.StringVar(value="[Undefined]")
isConnected = tk.BooleanVar(value=False)
connectDisconnectText = tk.StringVar(value="Connect")

frame = ttk.Frame(root, padding=10)
frame.pack()

# Buttons for first axis
ttk.Label(frame, text="Joint 1").grid(column=0, row=0, padx=5, pady=5)
ttk.Button(frame, text="Up", command=moveJoint1Up, width=10).grid(column=0, row=1, padx=5, pady=5)
ttk.Button(frame, text="Down", command=moveJoint1Down, width=10).grid(column=0, row=2, padx=5, pady=5)

# Buttons for second axis
ttk.Label(frame, text="Joint 2").grid(column=1, row=0, padx=5, pady=5)
ttk.Button(frame, text="Up", command=moveJoint2Up, width=10).grid(column=1, row=1, padx=5, pady=5)
ttk.Button(frame, text="Down", command=moveJoint2Down, width=10).grid(column=1, row=2, padx=5, pady=5)

# Buttons for third axis
ttk.Label(frame, text="Joint 3").grid(column=2, row=0, padx=5, pady=5)
ttk.Button(frame, text="Up", command=moveJoint3Up, width=10).grid(column=2, row=1, padx=5, pady=5)
ttk.Button(frame, text="Down", command=moveJoint3Down, width=10).grid(column=2, row=2, padx=5, pady=5)

# Add a separator between columns
ttk.Separator(frame, orient='vertical').grid(column=3, row=0, padx=0, pady=0, rowspan=7, sticky='nsew')

# Buttons for moving in X
ttk.Label(frame, text="Move in X").grid(column=3, row=0, padx=5, pady=5)
ttk.Button(frame, text="Up", command=moveXUp, width=10).grid(column=3, row=1, padx=5, pady=5)
ttk.Button(frame, text="Down", command=moveXDown, width=10).grid(column=3, row=2, padx=5, pady=5)

# Buttons for moving in Y
ttk.Label(frame, text="Move in Y").grid(column=4, row=0, padx=5, pady=5)
ttk.Button(frame, text="Up", command=moveYUp, width=10).grid(column=4, row=1, padx=5, pady=5)
ttk.Button(frame, text="Down", command=moveYDown, width=10).grid(column=4, row=2, padx=5, pady=5)

# Buttons for moving in Z
ttk.Label(frame, text="Move in Z").grid(column=5, row=0, padx=5, pady=5)
ttk.Button(frame, text="Up", command=moveZUp, width=10).grid(column=5, row=1, padx=5, pady=5)
ttk.Button(frame, text="Down", command=moveZDown, width=10).grid(column=5, row=2, padx=5, pady=5)

# Add a separator between columns
ttk.Separator(frame, orient='vertical').grid(column=6, row=0, padx=0, pady=0, rowspan=7, sticky='nsew')

# Indicator light
ttk.Label(frame, text="Connection Status").grid(column=6, row=0, padx=5, pady=5)

# Indicator machine status
ttk.Label(frame, text="Machine Status").grid(column=6, row=1, padx=5, pady=5)
ttk.Label(frame, textvariable=machineStatus).grid(column=7, row=1, padx=5, pady=5)

# Start thread
status = threading.Thread(target=statusThread)
status.start()

# Create a canvas widget to draw the indicator circle
canvas = tk.Canvas(frame, width=20, height=20, bd=0, highlightthickness=0)
canvas.grid(column=7, row=0, padx=5, pady=5)

# Draw a circle on the canvas to indicate connection status
statusLed = canvas.create_oval(2, 2, 18, 18, fill="red", outline="black")

# Increase the font size of all labels and buttons
style = ttk.Style()
style.configure('.', font=('TkDefaultFont', 12))

# Add a dropdown list
options = ["/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3"]
selectedOption = tk.StringVar()
options.insert(0, "/dev/ttyS0")
selectedOption.set(options[0])
dropdown = ttk.OptionMenu(frame, selectedOption, *options)
dropdown.grid(column=7, row=2, padx=5, pady=5)

# Label for serial port
ttk.Label(frame, text="Serial port:").grid(column=6, row=2, padx=5, pady=5)

# Button to connect/disconnect
ttk.Button(frame, textvariable=connectDisconnectText, command=connectDisconnect).grid(column=7, row=3, padx=5, pady=5)

# Add buttons to increase or decrease step size of the joint
ttk.Button(frame, text="+", command=increaseJointStep).grid(column=2, row=6, padx=5, pady=5)
ttk.Button(frame, text="-", command=decreaseJointStep).grid(column=0, row=6, padx=5, pady=5)
step_label = ttk.Label(frame, text="Step size (degree):")
step_label.grid(column=1, row=5, padx=5, pady=5)
step_text = ttk.Label(frame, textvariable=jointStepSize)
step_text.grid(column=1, row=6, padx=5, pady=5)

# Add buttons to increase or decrease step size of x, y, z movement
ttk.Button(frame, text="+", command=increaseXYZStep).grid(column=5, row=6, padx=5, pady=5)
ttk.Button(frame, text="-", command=decreaseXYZStep).grid(column=3, row=6, padx=5, pady=5)
step_label = ttk.Label(frame, text="Step size (mm):")
step_label.grid(column=4, row=5, padx=5, pady=5)
step_text = ttk.Label(frame, textvariable=XYZStepSize)
step_text.grid(column=4, row=6, padx=5, pady=5)

root.mainloop()
