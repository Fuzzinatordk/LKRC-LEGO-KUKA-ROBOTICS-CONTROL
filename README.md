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
                


                KUKA 6-DOF Robot Control with PyBricks Integration
This project provides an interface to control a KUKA 6-DOF robot arm using roboticstoolbox for simulation and PyBricks for real-world robot control. The script includes methods for forward and inverse kinematics, point-to-point (PTP) linear and circular movements, joint angle constraints, and more.

Dependencies
To run this project, the following libraries and dependencies are required:

Python 3.x
roboticstoolbox for simulating the robot
numpy for numerical calculations
spatialmath for spatial transformations
pybricks for controlling LEGO Mindstorms and Powered Up motors
You can install the necessary dependencies using:

bash
Copy code
pip install roboticstoolbox-python numpy spatialmath pybricks
Installing PyBricks
To install PyBricks for controlling LEGO motors, visit the official PyBricks website for instructions: PyBricks Installation Guide.

Methods
1. __DHParams()
Description: Initializes the Denavit-Hartenberg parameters for the robot.
Input: None.
2. homing()
Description: Resets the robot to its home state.
Input: None.
3. teach()
Description: Opens a GUI to manually control and configure the robot’s joints.
Input: None.
Output: List of chosen joint angles.
4. linearPTP(pose=[0,0,0])
Description: Moves the robot in a straight line to a given pose using point-to-point movement.
Input: A list of 3 pose values [x, y, z].
5. circularPTP(startPose, endPose, topPose, steps=100)
Description: Executes a circular PTP movement using Bezier curves.
Input:
startPose: List [x, y, z] for the starting position.
endPose: List [x, y, z] for the ending position.
topPose: List [x, y, z] for the highest point of the arc.
steps: Number of points to compute on the Bezier curve.
6. linearRun()
Description: Executes the list of poses added using linearPTP.
Input: None.
7. FK_solution(angles, type)
Description: Computes the forward kinematics based on provided joint angles.
Input:
angles: List of 6 joint angles.
type: Either 'deg' (degrees) or 'rad' (radians).
8. IK(pose, type, plot=False, PTP=False)
Description: Computes inverse kinematics to achieve the given pose.
Input:
pose: List of 6 pose values [x, y, z, roll, pitch, yaw].
type: Either 'deg' (degrees) or 'rad' (radians).
plot: Boolean to display the robot pose.
PTP: Boolean to run PTP motion after solving IK.
9. randomPose(plot=False, PTP=False)
Description: Generates and moves the robot to a random pose within its joint limits.
Input:
plot: Boolean to display the pose.
PTP: Boolean to run PTP motion after setting the pose.
10. limitsDeg()
Description: Prints the joint limits in degrees.
Input: None.
11. writeFile(sols)
Description: Writes the joint angle solutions to a Python file for execution on the PyBricks platform.
Input: sols - List of joint angle solutions.
