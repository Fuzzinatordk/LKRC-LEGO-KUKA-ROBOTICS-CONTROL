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
                )