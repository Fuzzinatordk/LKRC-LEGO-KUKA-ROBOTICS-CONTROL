The following documentation is for an OrangeApps Lego Kuka robot, to faciliate physcial simulation of robot kinematics of a real Kuka 6-DOF robot, using IK and FK from roboticstoolbox.

Requirements:
  - Python 3.xx
  - Pybricks firmware on hub
  - Bluetooth connectivity on device
    Python libs (can be installed with pip3 or any other package manager):
  - Roboticstoolbox-python
  - Numpy
  - Spartialmath-python
  - Keyboard
  - Time
  - pybricksdev
  - pybricks
  - pipx


Known issues:

When using the teach method, if using newer versions of matplotlib, an error can occur where the function is given more parameters than needed.
This can be fixed by navigating to Pyplot.py and go to line 575 and replace the line of code with the following:

                slider = Slider(
                  ax, "q" + str(j), qlim[0, j], qlim[1, j], valinit=np.degrees(q[j]), valfmt="%.1f°"
                


# KUKA 6-DOF Robot Control System

This repository contains a Python-based control system for a KUKA 6-DOF robot, using the **Robotics Toolbox** and **Pybricks**. The system allows for precise robot control, including homing, linear and circular Point-to-Point (PTP) motion, forward and inverse kinematics, and more. The code generates movement instructions and allows control over the robot's joints.

## Dependencies

To run the script, the following dependencies are required:
- **Python 3.8+**
- **numpy**: For matrix calculations and transformations
- **Robotics Toolbox for Python**: For defining robot models and performing kinematic computations
- **spatialmath**: For handling transformation matrices and rotations

Install these dependencies using:
```bash
pip install numpy roboticstoolbox-python spatialmath

