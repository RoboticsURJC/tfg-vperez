### Code files

- **grblAPI.py** is a class to talk with the arm via USB Serial
- **robot.py** is a class to convert grbl linearity to rotary axes. Uses grblAPI

### Executable files
Execute them as:
```
python3 programName.py
```
- **robotNode.py** is a ROS 2 executable with a node that uses Moveit framework and robot class to controll the arm
- **pendant.py** is a tkinter application that uses robot class to bring the user a GUI to have manual control of the arm